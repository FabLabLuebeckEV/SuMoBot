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
  overrunArmed_ = isPollerLowered();
  overrunLatched_ = false;
  lastOverrunMs_ = 0;
  cooldownWasActive_ = false;

  stepper_.begin();
  leds_.begin();

  stepper_.startCalibration();
  status_.state = comms::PollerState::kCalibrating;

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
  const uint32_t now = millis();
  bool forcePublish = false;

  const bool sensorActive = digitalRead(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR)) == LOW;
  if (sensorActive != pollerSensorLatched_) {
    pollerSensorLatched_ = sensorActive;
    lastPollerSensorChangeMs_ = now;

    if (sensorActive) {
      if (overrunArmed_ && isPollerLowered()) {
        overrunLatched_ = true;
        overrunArmed_ = false;
      }
    }
    forcePublish = true;
  }

  if (overrunArmed_ && !isPollerLowered()) {
    overrunArmed_ = false;
    forcePublish = true;
  }

  if (stepper_.isCalibrating()) {
    status_.state = comms::PollerState::kCalibrating;
  } else if (stepper_.isBusy()) {
    status_.state = comms::PollerState::kMoving;
  } else {
    status_.state = comms::PollerState::kIdle;
  }

  const bool cooldownNow = cooldownActive(now);
  if (cooldownWasActive_ != cooldownNow) {
    cooldownWasActive_ = cooldownNow;
    forcePublish = true;
  }

  if (endstopEvent) {
    forcePublish = true;
  }

  publishStatus(forcePublish);
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
  const uint32_t now = millis();

  bool handled = true;

  switch (command.type) {
    case comms::CommandType::kNoop:
      handled = true;
      break;
    case comms::CommandType::kMoveAbsolute: {
      const int32_t target = command.value;
      const int32_t current = stepper_.currentPosition();
      if (!stepper_.isCalibrated() && target < current) {
        handled = false;
        break;
      }
      const int32_t threshold = hardware::POSITION_DOWN_TARGET + hardware::POLLER_DOWN_ARM_MARGIN;
      if (target > threshold) {
        if (!canInitiateOverrun(now)) {
          handled = false;
          break;
        }
        overrunArmed_ = false;
        lastOverrunMs_ = now;
        cooldownWasActive_ = true;
      }
      stepper_.setTarget(target);
      status_.state = comms::PollerState::kMoving;
      break;
    }
    case comms::CommandType::kMoveRelative: {
      const int32_t current = stepper_.currentPosition();
      const int32_t target = current + command.value;
      if (!stepper_.isCalibrated() && command.value < 0) {
        handled = false;
        break;
      }
      const int32_t threshold = hardware::POSITION_DOWN_TARGET + hardware::POLLER_DOWN_ARM_MARGIN;
      if (command.value > 0 && target > threshold) {
        if (!canInitiateOverrun(now)) {
          handled = false;
          break;
        }
        overrunArmed_ = false;
        lastOverrunMs_ = now;
        cooldownWasActive_ = true;
      }
      stepper_.moveBy(command.value);
      status_.state = comms::PollerState::kMoving;
      break;
    }
    case comms::CommandType::kMoveToLimit:
      if (command.limit == comms::LimitDirection::kNone) {
        handled = false;
      } else {
        if (command.limit == comms::LimitDirection::kDown && !stepper_.isCalibrated()) {
          handled = false;
          break;
        }
        if (command.limit == comms::LimitDirection::kUp) {
          if (!canInitiateOverrun(now)) {
            handled = false;
            break;
          }
          overrunArmed_ = false;
          lastOverrunMs_ = now;
          cooldownWasActive_ = true;
        }
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
    case comms::CommandType::kSetOverrunArmed:
      handled = setOverrunArmed(command.value != 0, now);
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
  refreshStatusFlags(now);

  status_.lastRssi = lastRssi_;
  status_.emaRssi = isnan(emaRssi_) ? lastRssi_ : static_cast<int8_t>(roundf(emaRssi_));

  if (hasPeer_) {
    comms::sendTo(pultAddress_, reinterpret_cast<const uint8_t*>(&status_), sizeof(status_));
  } else {
    comms::sendTo(comms::PULT_MAC, reinterpret_cast<const uint8_t*>(&status_), sizeof(status_));
  }

  lastStatusSentMs_ = now;
}

void PollerController::refreshStatusFlags(uint32_t now) {
  uint16_t flags = 0;
  if (stepper_.endstopActive()) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kEndstopActive);
  }
  if (pollerSensorLatched_) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kPollerSensorActive);
  }
  if (overrunLatched_) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kOverrunDetected);
  }
  const bool lowered = isPollerLowered();
  const bool cooldown = cooldownActive(now);
  if (overrunArmed_ && lowered && !cooldown) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kOverrunArmed);
  } else {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kCooldownActive);
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

bool PollerController::isPollerLowered() {
  const int32_t current = stepper_.currentPosition();
  const int32_t threshold = hardware::POSITION_DOWN_TARGET + hardware::POLLER_DOWN_ARM_MARGIN;
  return current <= threshold;
}

bool PollerController::cooldownActive(uint32_t now) const {
  if (lastOverrunMs_ == 0) {
    return false;
  }
  return static_cast<uint32_t>(now - lastOverrunMs_) < hardware::POLLER_COOLDOWN_MS;
}

bool PollerController::canInitiateOverrun(uint32_t now) {
  if (!isPollerLowered()) {
    return false;
  }
  if (cooldownActive(now)) {
    return false;
  }
  return true;
}

bool PollerController::setOverrunArmed(bool armed, uint32_t now) {
  if (armed) {
    if (cooldownActive(now)) {
      return false;
    }
    if (!isPollerLowered()) {
      return false;
    }

    const bool sensorNow = digitalRead(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR)) == LOW;
    if (sensorNow) {
      overrunLatched_ = true;
      overrunArmed_ = false;
      lastOverrunMs_ = now;
      cooldownWasActive_ = true;
      return false;
    }

    overrunLatched_ = false;
    overrunArmed_ = true;
    lastOverrunMs_ = 0;
    cooldownWasActive_ = false;
    return true;
  }

  overrunArmed_ = false;
  return true;
}

}  // namespace poller
