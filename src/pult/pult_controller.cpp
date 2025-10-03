#include "pult_controller.h"

#include <Arduino.h>
#include <string.h>

#include "comms/peer_config.h"
#include "common/espnow_link.h"

namespace pult {

namespace {
bool isMacConfigured(const uint8_t mac[6]) {
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != 0x00) {
      return true;
    }
  }
  return false;
}
}

PultController* PultController::instance_ = nullptr;

void PultController::begin() {
  instance_ = this;

  if (!comms::beginEspNow()) {
    Serial.println("ESP-NOW initialisation failed");
  }

  comms::setReceiveHandler(&PultController::onEspNowReceive);
  comms::setSendHandler(nullptr);

  if (isMacConfigured(comms::POLLER_MAC)) {
    memcpy(pollerAddress_, comms::POLLER_MAC, sizeof(pollerAddress_));
    comms::addPeer(pollerAddress_);
    pollerKnown_ = true;
  }

  if (isMacConfigured(comms::OBSERVER_MAC)) {
    memcpy(observerAddress_, comms::OBSERVER_MAC, sizeof(observerAddress_));
    comms::addPeer(observerAddress_);
    observerKnown_ = true;
  }
}

void PultController::loop() {
  const uint32_t now = millis();
  tickPing(now);
}

void PultController::onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi) {
  if (!instance_ || !mac || !data || len <= 0) {
    return;
  }

  comms::PollerStatus status{};
  const size_t toCopy = len < static_cast<int>(sizeof(status)) ? len : sizeof(status);
  memcpy(&status, data, toCopy);
  instance_->handleStatus(status, mac, rssi);
}

bool PultController::sendMoveAbsolute(int32_t position) {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kMoveAbsolute;
  command.value = position;
  return sendCommand(command);
}

bool PultController::sendMoveRelative(int32_t delta) {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kMoveRelative;
  command.value = delta;
  return sendCommand(command);
}

bool PultController::sendMoveToLimit(comms::LimitDirection direction) {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kMoveToLimit;
  command.limit = direction;
  return sendCommand(command);
}

bool PultController::sendStop() {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kStopStepper;
  return sendCommand(command);
}

bool PultController::sendStartAnimation(comms::AnimationId animation) {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kStartAnimation;
  command.animation = animation;
  return sendCommand(command);
}

bool PultController::sendStopAnimation() {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kStopAnimation;
  return sendCommand(command);
}

bool PultController::sendCalibrate() {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kCalibrate;
  return sendCommand(command);
}

bool PultController::sendSetOverrunArmed(bool armed) {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kSetOverrunArmed;
  command.value = armed ? 1 : 0;
  return sendCommand(command);
}

bool PultController::sendSetParameter(comms::PollerParameterId id, int32_t rawValue) {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kSetParameter;
  command.value = rawValue;
  command.reserved = static_cast<uint8_t>(id);
  return sendCommand(command);
}

uint32_t PultController::timeSinceLastPong(uint32_t now) const {
  if (lastPongMs_ == 0) {
    return UINT32_MAX;
  }
  return now - lastPongMs_;
}

bool PultController::hasPong() const {
  return lastPongMs_ != 0;
}

bool PultController::pingHealthy(uint32_t now, uint32_t timeoutMs) const {
  if (!hasPong()) {
    return false;
  }
  return timeSinceLastPong(now) <= timeoutMs;
}

bool PultController::sendObserverMessage(const char* text) {
  if (!text) {
    return false;
  }

  if (!observerKnown_) {
    Serial.println("Observer MAC unknown. Cannot send observer message.");
    return false;
  }

  struct TextPacket {
    char text[32];
  } packet{};
  strncpy(packet.text, text, sizeof(packet.text) - 1);
  packet.text[sizeof(packet.text) - 1] = '\0';

  bool sent = comms::sendTo(observerAddress_, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  if (!sent) {
    Serial.println("Failed to send observer ESP-NOW message");
  }
  return sent;
}

bool PultController::hasStatus() const {
  return statusReceived_;
}

const comms::PollerStatus& PultController::status() const {
  return lastStatus_;
}

uint32_t PultController::lastStatusTimestamp() const {
  return lastStatusMs_;
}

bool PultController::sendCommand(const comms::PollerCommand& commandTemplate, uint8_t* outCommandId) {
  if (!pollerKnown_) {
    Serial.println("Poller MAC unknown. Cannot send command.");
    return false;
  }

  comms::PollerCommand command = commandTemplate;
  if (++commandCounter_ == 0) {
    commandCounter_ = 1;
  }
  command.commandId = commandCounter_;
  if (outCommandId) {
    *outCommandId = command.commandId;
  }

  bool sent = comms::sendTo(pollerAddress_, reinterpret_cast<const uint8_t*>(&command), sizeof(command));
  if (!sent) {
    Serial.println("Failed to send ESP-NOW command");
  }
  return sent;
}

void PultController::handleStatus(const comms::PollerStatus& status, const uint8_t mac[6], int8_t /*rssi*/) {
  Serial.printf(
      "[Pult] Status recv state=%d anim=%d pos=%ld target=%ld flags=0x%04x overrunArmed=%s overrunDetected=%s cooldown=%s sensor=%s\n",
      static_cast<int>(status.state), static_cast<int>(status.activeAnimation), static_cast<long>(status.currentPosition),
      static_cast<long>(status.targetPosition), status.statusFlags,
      (status.statusFlags & static_cast<uint16_t>(comms::StatusFlag::kOverrunArmed)) ? "yes" : "no",
      (status.statusFlags & static_cast<uint16_t>(comms::StatusFlag::kOverrunDetected)) ? "yes" : "no",
      (status.statusFlags & static_cast<uint16_t>(comms::StatusFlag::kCooldownActive)) ? "yes" : "no",
      (status.statusFlags & static_cast<uint16_t>(comms::StatusFlag::kPollerSensorActive)) ? "yes" : "no");
  memcpy(&lastStatus_, &status, sizeof(lastStatus_));
  lastStatusMs_ = millis();
  statusReceived_ = true;

  if (lastPingId_ != 0 && status.lastCommandId == lastPingId_) {
    lastPongMs_ = lastStatusMs_;
  }

  if (!pollerKnown_ || memcmp(pollerAddress_, mac, 6) != 0) {
    memcpy(pollerAddress_, mac, 6);
    comms::addPeer(pollerAddress_);
    pollerKnown_ = true;
  }

  if (!observerKnown_ && isMacConfigured(comms::OBSERVER_MAC)) {
    memcpy(observerAddress_, comms::OBSERVER_MAC, sizeof(observerAddress_));
    comms::addPeer(observerAddress_);
    observerKnown_ = true;
  }
}

void PultController::tickPing(uint32_t now) {
  if (!pollerKnown_) {
    return;
  }
  if ((now - lastPingMs_) >= 5000) {
    sendPing(now);
  }
}

bool PultController::sendPing(uint32_t now) {
  comms::PollerCommand command{};
  command.type = comms::CommandType::kPing;
  uint8_t assignedId = 0;
  if (sendCommand(command, &assignedId)) {
    lastPingMs_ = now;
    lastPingId_ = assignedId;
    return true;
  }
  return false;
}

}  // namespace pult
