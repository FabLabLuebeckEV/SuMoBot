#include "poller_controller.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "comms/peer_config.h"
#include "common/espnow_link.h"
#include "hardware_config.h"

namespace poller {

PollerController* PollerController::instance_ = nullptr;

void PollerController::begin() {
  instance_ = this;

  pinMode(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR), INPUT);
  pollerSensorLatched_ = digitalRead(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR)) == LOW;

  stepper_.begin();
  leds_.begin();

  status_.state = comms::PollerState::kIdle;
  status_.activeAnimation = comms::AnimationId::kNone;
  status_.lastCommandId = 0;
  status_.statusFlags = 0;
  status_.lastRssi = -127;
  status_.emaRssi = -127;

  if (!comms::beginEspNow()) {
    Serial.println("ESP-NOW initialisation failed");
  }

  comms::setReceiveHandler(&PollerController::onEspNowReceive);
  comms::setSendHandler(nullptr);

  comms::addPeer(comms::PULT_MAC);
  memcpy(pultAddress_, comms::PULT_MAC, sizeof(pultAddress_));
  hasPeer_ = true;

  publishStatus(true);
}

void PollerController::loop() {
  stepper_.update();
  leds_.update();

  const bool endstopEvent = stepper_.consumeEndstopEvent();

  const bool sensorActive = digitalRead(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR)) == LOW;
  if (sensorActive != pollerSensorLatched_) {
    pollerSensorLatched_ = sensorActive;
    lastPollerSensorChangeMs_ = millis();
    publishStatus(true);
  }

  if (stepper_.isCalibrating()) {
    status_.state = comms::PollerState::kCalibrating;
  } else if (stepper_.isBusy()) {
    status_.state = comms::PollerState::kMoving;
  } else {
    status_.state = comms::PollerState::kIdle;
  }

  if (endstopEvent) {
    publishStatus(true);
  }

  publishStatus(false);
}

void PollerController::onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi) {
  if (!instance_ || !mac || !data || len <= 0) {
    return;
  }

  comms::PollerCommand command{};
  const size_t copyLen = len < static_cast<int>(sizeof(command)) ? len : sizeof(command);
  memcpy(&command, data, copyLen);
  instance_->handleCommand(command, rssi, mac);
}

void PollerController::handleCommand(const comms::PollerCommand& command, int8_t rssi, const uint8_t mac[6]) {
  if (!mac) {
    return;
  }

  if (!hasPeer_ || memcmp(pultAddress_, mac, 6) != 0) {
    memcpy(pultAddress_, mac, 6);
    comms::addPeer(pultAddress_);
    hasPeer_ = true;
  }

  lastRssi_ = rssi;
  if (isnan(emaRssi_)) {
    emaRssi_ = static_cast<float>(rssi);
  } else {
    emaRssi_ = 0.2f * static_cast<float>(rssi) + 0.8f * emaRssi_;
  }

  status_.lastCommandId = command.commandId;

  bool handled = true;

  switch (command.type) {
    case comms::CommandType::kNoop:
      handled = true;
      break;
    case comms::CommandType::kMoveAbsolute:
      stepper_.setTarget(command.value);
      status_.state = comms::PollerState::kMoving;
      break;
    case comms::CommandType::kMoveRelative:
      stepper_.moveBy(command.value);
      status_.state = comms::PollerState::kMoving;
      break;
    case comms::CommandType::kMoveToLimit:
      if (command.limit == comms::LimitDirection::kNone) {
        handled = false;
      } else {
        stepper_.moveToLimit(command.limit);
        status_.state = comms::PollerState::kMoving;
      }
      break;
    case comms::CommandType::kStopStepper:
      stepper_.stop();
      status_.state = comms::PollerState::kIdle;
      break;
    case comms::CommandType::kStartAnimation:
      leds_.startAnimation(command.animation);
      break;
    case comms::CommandType::kStopAnimation:
      leds_.stopAnimation();
      break;
    case comms::CommandType::kCalibrate:
      stepper_.startCalibration();
      status_.state = comms::PollerState::kCalibrating;
      break;
    default:
      handled = false;
      break;
  }

  if (!handled) {
    status_.statusFlags |= static_cast<uint16_t>(comms::StatusFlag::kCommandError);
  }

  publishStatus(true);
}

void PollerController::publishStatus(bool force) {
  const uint32_t now = millis();
  if (!force && (now - lastStatusSentMs_) < hardware::STATUS_INTERVAL_MS) {
    return;
  }

  status_.uptimeMs = now;
  status_.activeAnimation = leds_.activeAnimation();
  status_.currentPosition = stepper_.currentPosition();
  status_.targetPosition = stepper_.targetPosition();
  refreshStatusFlags();

  status_.lastRssi = lastRssi_;
  status_.emaRssi = isnan(emaRssi_) ? lastRssi_ : static_cast<int8_t>(roundf(emaRssi_));

  if (hasPeer_) {
    comms::sendTo(pultAddress_, reinterpret_cast<const uint8_t*>(&status_), sizeof(status_));
  } else {
    comms::sendTo(comms::PULT_MAC, reinterpret_cast<const uint8_t*>(&status_), sizeof(status_));
  }

  lastStatusSentMs_ = now;
}

void PollerController::refreshStatusFlags() {
  uint16_t flags = 0;
  if (stepper_.endstopActive()) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kEndstopActive);
  }
  if (pollerSensorLatched_) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kPollerSensorActive);
    flags |= static_cast<uint16_t>(comms::StatusFlag::kOverrunDetected);
  }
  if (!isnan(emaRssi_) && emaRssi_ < -90.0f) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kLinkLowQuality);
  }

  // Preserve sticky command error flag if already set.
  const bool hadCommandError = (status_.statusFlags & static_cast<uint16_t>(comms::StatusFlag::kCommandError)) != 0;
  status_.statusFlags = flags;
  if (hadCommandError) {
    status_.statusFlags |= static_cast<uint16_t>(comms::StatusFlag::kCommandError);
  }
}

}  // namespace poller
