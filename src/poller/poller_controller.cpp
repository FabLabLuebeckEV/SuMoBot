#include "poller_controller.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "comms/peer_config.h"
#include "common/espnow_link.h"
#include "hardware_config.h"
#include "poller_settings.h"

namespace {

const char* commandTypeToString(comms::CommandType type) {
  switch (type) {
    case comms::CommandType::kNoop:
      return "noop";
    case comms::CommandType::kMoveAbsolute:
      return "move-absolute";
    case comms::CommandType::kMoveRelative:
      return "move-relative";
    case comms::CommandType::kMoveToLimit:
      return "move-limit";
    case comms::CommandType::kStopStepper:
      return "stop";
    case comms::CommandType::kStartAnimation:
      return "start-animation";
    case comms::CommandType::kStopAnimation:
      return "stop-animation";
    case comms::CommandType::kCalibrate:
      return "calibrate";
    case comms::CommandType::kSetOverrunArmed:
      return "set-overrun";
    case comms::CommandType::kSetParameter:
      return "set-parameter";
    case comms::CommandType::kPing:
      return "ping";
  }
  return "unknown";
}

const char* limitDirectionToString(comms::LimitDirection dir) {
  switch (dir) {
    case comms::LimitDirection::kDown:
      return "down";
    case comms::LimitDirection::kUp:
      return "up";
    case comms::LimitDirection::kNone:
    default:
      return "none";
  }
}

const char* animationIdToString(comms::AnimationId id) {
  switch (id) {
    case comms::AnimationId::kNone:
      return "none";
    case comms::AnimationId::kCountdown:
      return "countdown";
    case comms::AnimationId::kPollerOverrun:
      return "poller_overrun";
    case comms::AnimationId::kArenaStop:
      return "arena_stop";
    case comms::AnimationId::kArenaControl:
      return "arena_control";
  }
  return "unknown";
}

const char* parameterIdToString(comms::PollerParameterId id) {
  switch (id) {
    case comms::PollerParameterId::kPositionHome:
      return "positionHome";
    case comms::PollerParameterId::kPositionUpTarget:
      return "positionUpTarget";
    case comms::PollerParameterId::kPositionDownTarget:
      return "positionDownTarget";
    case comms::PollerParameterId::kDownArmMargin:
      return "downArmMargin";
    case comms::PollerParameterId::kStepperMaxSpeed:
      return "stepperMaxSpeed";
    case comms::PollerParameterId::kStepperAcceleration:
      return "stepperAcceleration";
    case comms::PollerParameterId::kStatusIntervalMs:
      return "statusIntervalMs";
    case comms::PollerParameterId::kOverrunCooldownMs:
      return "overrunCooldownMs";
    case comms::PollerParameterId::kCount:
    default:
      return "unknown";
  }
}

enum PublishReason : uint32_t {
  kReasonNone = 0,
  kReasonStartup = 1u << 0,
  kReasonSensorChange = 1u << 1,
  kReasonCooldownToggle = 1u << 2,
  kReasonEndstopEvent = 1u << 3,
  kReasonCommandHandled = 1u << 4,
  kReasonManualReleased = 1u << 5,
  kReasonArmingChange = 1u << 6,
  kReasonPositionChange = 1u << 7
};

void logPublishReasons(uint32_t mask) {
  if (mask == kReasonNone) {
    return;
  }
  Serial.print("[Poller] Publishing status (reasons: ");
  bool first = true;
  auto emit = [&](const char* text) {
    if (!first) {
      Serial.print(", ");
    }
    Serial.print(text);
    first = false;
  };
  if (mask & kReasonStartup) {
    emit("startup");
  }
  if (mask & kReasonSensorChange) {
    emit("sensor");
  }
  if (mask & kReasonCooldownToggle) {
    emit("cooldown");
  }
  if (mask & kReasonEndstopEvent) {
    emit("endstop");
  }
  if (mask & kReasonCommandHandled) {
    emit("command");
  }
  if (mask & kReasonManualReleased) {
    emit("manual");
  }
  if (mask & kReasonArmingChange) {
    emit("arming");
  }
  if (mask & kReasonPositionChange) {
    emit("position");
  }
  Serial.println(")");
}

}  // namespace

namespace poller {

PollerController* PollerController::instance_ = nullptr;

void PollerController::begin() {
  instance_ = this;

  Serial.println("[Poller] Initialising poller controller");

  pinMode(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR), INPUT);
  pollerSensorLatched_ = digitalRead(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR)) == LOW;
  overrunArmed_ = isPollerLowered();
  overrunLatched_ = false;
  lastOverrunMs_ = 0;
  cooldownWasActive_ = false;
  wasLowered_ = isPollerLowered();
  wasRaised_ = isPollerRaised();

  Serial.printf("[Poller] Sensor initial state: %s\n", pollerSensorLatched_ ? "active" : "idle");
  Serial.printf("[Poller] Initial overrun armed: %s\n", overrunArmed_ ? "yes" : "no");

  hardware::PollerParameters stored = hardware::DEFAULT_POLLER_PARAMETERS;
  if (settings::loadParameters(&stored)) {
    config_ = hardware::sanitized(stored);
    Serial.println("[Poller] Parameters loaded from NVS");
  } else {
    config_ = hardware::DEFAULT_POLLER_PARAMETERS;
    settings::saveParameters(config_);
    Serial.println("[Poller] Using default parameters");
  }
  Serial.printf("[Poller] Cooldown default %lu ms, down target %ld\n",
                static_cast<unsigned long>(config_.overrunCooldownMs),
                static_cast<long>(config_.positionDownTarget));

  stepper_.begin(config_);
  leds_.begin();

  stepper_.startCalibration();
  status_.state = comms::PollerState::kCalibrating;
  Serial.println("[Poller] Calibration initiated");

  status_.activeAnimation = comms::AnimationId::kNone;
  status_.lastCommandId = 0;
  status_.statusFlags = 0;
  status_.lastRssi = -127;
  status_.emaRssi = -127;
  status_.config = config_;

  if (!comms::beginEspNow()) {
    Serial.println("[Poller] ESP-NOW initialisation failed");
  } else {
    Serial.println("[Poller] ESP-NOW initialised");
  }

  comms::setReceiveHandler(&PollerController::onEspNowReceive);
  comms::setSendHandler(nullptr);

  comms::addPeer(comms::PULT_MAC);
  memcpy(pultAddress_, comms::PULT_MAC, sizeof(pultAddress_));
  hasPeer_ = true;
  Serial.println("[Poller] Default peer configured");

  publishStatus(true, kReasonStartup);
}

void PollerController::loop() {
  stepper_.update();

  const bool endstopEvent = stepper_.consumeEndstopEvent();
  const uint32_t now = millis();
  bool forcePublish = false;
  uint32_t publishMask = kReasonNone;

  const bool sensorActive = digitalRead(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR)) == LOW;
  if (sensorActive != pollerSensorLatched_) {
    pollerSensorLatched_ = sensorActive;
    lastPollerSensorChangeMs_ = now;

    if (sensorActive) {
      const bool lowered = isPollerLowered();
      Serial.printf("[Poller] Sensor triggered (armed=%s, lowered=%s)\n", overrunArmed_ ? "yes" : "no",
                    lowered ? "yes" : "no");
      if (lowered) {
        overrunLatched_ = true;
        if (overrunArmed_) {
          overrunArmed_ = false;
          publishMask |= kReasonArmingChange;
        }
        lastOverrunMs_ = now;
        cooldownWasActive_ = true;
        publishMask |= kReasonCooldownToggle;
        Serial.println("[Poller] Overrun latched; cooldown started");
      }
    } else {
      Serial.println("[Poller] Sensor released");
    }
    forcePublish = true;
    publishMask |= kReasonSensorChange;
  }

  if (overrunArmed_ && !isPollerLowered()) {
    overrunArmed_ = false;
    Serial.println("[Poller] Overrun disarmed because poller left lower zone");
    forcePublish = true;
    publishMask |= kReasonArmingChange;
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
    Serial.printf("[Poller] Cooldown %s\n", cooldownNow ? "active" : "cleared");
    forcePublish = true;
    publishMask |= kReasonCooldownToggle;
  }

  if (endstopEvent) {
    Serial.println("[Poller] Endstop event detected");
    forcePublish = true;
    publishMask |= kReasonEndstopEvent;
  }

  if (manualControlActive_) {
    if (!stepper_.isBusy() && (now - manualControlLastMs_) > 50U) {
      manualControlActive_ = false;
      Serial.println("[Poller] Manual control released");
      forcePublish = true;
      publishMask |= kReasonManualReleased;
    }
  }

  const bool loweredNow = isPollerLowered();
  if (loweredNow != wasLowered_) {
    wasLowered_ = loweredNow;
    Serial.printf("[Poller] Poller %s lower position\n", loweredNow ? "reached" : "left");
    forcePublish = true;
    publishMask |= kReasonPositionChange;
  }

  const bool raisedNow = isPollerRaised();
  if (raisedNow != wasRaised_) {
    wasRaised_ = raisedNow;
    Serial.printf("[Poller] Poller %s upper position\n", raisedNow ? "reached" : "left");
    forcePublish = true;
    publishMask |= kReasonPositionChange;
  }

  LedController::Inputs ledInputs{};
  ledInputs.pollerIsLowered = isPollerLowered();
  ledInputs.pollerIsUp = isPollerRaised();
  ledInputs.pollerMoving = stepper_.isBusy();
  ledInputs.manualControl = manualControlActive_;
  ledInputs.overrunArmed = overrunArmed_;
  ledInputs.cooldownActive = cooldownActive(now);
  ledInputs.sensorActive = pollerSensorLatched_;
  ledInputs.calibrating = stepper_.isCalibrating();
  leds_.applyInputs(ledInputs);
  leds_.update();

  publishStatus(forcePublish, publishMask);
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

  Serial.printf(
      "[Poller] Command #%u type=%s value=%ld limit=%s animation=%s param=%u rssi=%d\n",
      command.commandId, commandTypeToString(command.type), static_cast<long>(command.value),
      limitDirectionToString(command.limit), animationIdToString(command.animation),
      command.reserved, static_cast<int>(rssi));

  bool handled = true;
  const char* rejectReason = nullptr;

  switch (command.type) {
    case comms::CommandType::kNoop:
      handled = true;
      break;
    case comms::CommandType::kMoveAbsolute: {
      const int32_t target = command.value;
      const int32_t current = stepper_.currentPosition();
      if (!stepper_.isCalibrated() && target < current) {
        handled = false;
        rejectReason = "not calibrated for downward move";
        break;
      }
      const int32_t threshold = config_.positionDownTarget + config_.downArmMargin;
      if (target > threshold) {
        if (!canInitiateOverrun(now)) {
          handled = false;
          rejectReason = "overrun cooldown active";
          break;
        }
        overrunArmed_ = false;
        lastOverrunMs_ = now;
        cooldownWasActive_ = true;
      }
      stepper_.setTarget(target);
      status_.state = comms::PollerState::kMoving;
      manualControlActive_ = true;
      manualControlLastMs_ = now;
      Serial.printf("[Poller] Manual absolute move to %ld initiated\n", static_cast<long>(target));
      break;
    }
    case comms::CommandType::kMoveRelative: {
      const int32_t current = stepper_.currentPosition();
      const int32_t target = current + command.value;
      if (!stepper_.isCalibrated() && command.value < 0) {
        handled = false;
        rejectReason = "not calibrated for downward move";
        break;
      }
      const int32_t threshold = config_.positionDownTarget + config_.downArmMargin;
      if (command.value > 0 && target > threshold) {
        if (!canInitiateOverrun(now)) {
          handled = false;
          rejectReason = "overrun cooldown active";
          break;
        }
        overrunArmed_ = false;
        lastOverrunMs_ = now;
        cooldownWasActive_ = true;
      }
      stepper_.moveBy(command.value);
      status_.state = comms::PollerState::kMoving;
      manualControlActive_ = true;
      manualControlLastMs_ = now;
      Serial.printf("[Poller] Manual relative move delta=%ld initiated\n", static_cast<long>(command.value));
      break;
    }
    case comms::CommandType::kMoveToLimit:
      if (command.limit == comms::LimitDirection::kNone) {
        handled = false;
        rejectReason = "missing direction";
      } else {
        if (command.limit == comms::LimitDirection::kDown && !stepper_.isCalibrated()) {
          handled = false;
          rejectReason = "not calibrated for down";
          break;
        }
        if (command.limit == comms::LimitDirection::kUp) {
          if (!cooldownActive(now)) {
            lastOverrunMs_ = now;
            cooldownWasActive_ = true;
          }
          overrunArmed_ = false;
        }
        stepper_.moveToLimit(command.limit);
        status_.state = comms::PollerState::kMoving;
        const bool autoRaise = (command.limit == comms::LimitDirection::kUp) && pollerSensorLatched_;
        if (!autoRaise) {
          manualControlActive_ = true;
          manualControlLastMs_ = now;
          Serial.printf("[Poller] Manual limit move (%s) initiated\n",
                        limitDirectionToString(command.limit));
        }
      }
      break;
    case comms::CommandType::kStopStepper:
      stepper_.stop();
      status_.state = comms::PollerState::kIdle;
      Serial.println("[Poller] Stop stepper command handled");
      break;
    case comms::CommandType::kStartAnimation:
      Serial.printf("[Poller] Start animation %s\n", animationIdToString(command.animation));
      leds_.startAnimation(command.animation);
      break;
    case comms::CommandType::kStopAnimation:
      Serial.println("[Poller] Stop animation command");
      leds_.stopAnimation();
      break;
    case comms::CommandType::kCalibrate:
      Serial.println("[Poller] Calibration command received");
      stepper_.startCalibration();
      status_.state = comms::PollerState::kCalibrating;
      break;
    case comms::CommandType::kSetOverrunArmed:
      handled = setOverrunArmed(command.value != 0, now);
      break;
    case comms::CommandType::kPing:
      handled = true;
      break;
    case comms::CommandType::kSetParameter: {
      const auto parameter = static_cast<comms::PollerParameterId>(command.reserved);
      handled = handleParameterUpdate(parameter, command.value);
      break;
    }
    default:
      handled = false;
      rejectReason = "unsupported command";
      break;
  }

  if (!handled) {
    Serial.printf("[Poller] Command #%u rejected: %s\n", command.commandId,
                  rejectReason ? rejectReason : "unknown reason");
    status_.statusFlags |= static_cast<uint16_t>(comms::StatusFlag::kCommandError);
  } else {
    Serial.printf("[Poller] Command #%u completed\n", command.commandId);
  }

  publishStatus(true, kReasonCommandHandled);
}

void PollerController::publishStatus(bool force, uint32_t reasonMask) {
  const uint32_t now = millis();
  if (!force && (now - lastStatusSentMs_) < config_.statusIntervalMs) {
    return;
  }

  if (force) {
    if (reasonMask == kReasonNone) {
      Serial.println("[Poller] Publishing status (forced)");
    } else {
      logPublishReasons(reasonMask);
    }
  } else {
    Serial.println("[Poller] Publishing status (interval)");
  }

  status_.uptimeMs = now;
  status_.activeAnimation = leds_.activeAnimation();
  status_.currentPosition = stepper_.currentPosition();
  status_.targetPosition = stepper_.targetPosition();
  refreshStatusFlags(now);
  status_.config = config_;

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
  if (cooldown) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kCooldownActive);
  }
  if (overrunArmed_ && lowered && !cooldown) {
    flags |= static_cast<uint16_t>(comms::StatusFlag::kOverrunArmed);
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
  const int32_t threshold = config_.positionDownTarget + config_.downArmMargin;
  return current <= threshold;
}

bool PollerController::isPollerRaised() {
  const int32_t current = stepper_.currentPosition();
  const int32_t threshold = config_.positionUpTarget - 200;
  return current >= threshold;
}

bool PollerController::cooldownActive(uint32_t now) const {
  if (lastOverrunMs_ == 0) {
    return false;
  }
  return static_cast<uint32_t>(now - lastOverrunMs_) < config_.overrunCooldownMs;
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
      Serial.println("[Poller] Reject arming: cooldown active");
      return false;
    }
    if (!isPollerLowered()) {
      Serial.println("[Poller] Reject arming: poller not lowered");
      return false;
    }

    const bool sensorNow = digitalRead(static_cast<uint8_t>(hardware::PIN_POLLER_SENSOR)) == LOW;
    if (sensorNow) {
      Serial.println("[Poller] Reject arming: sensor already active");
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
    Serial.println("[Poller] Overrun armed");
    return true;
  }

  overrunArmed_ = false;
  Serial.println("[Poller] Overrun disarmed by command");
  return true;
}

bool PollerController::handleParameterUpdate(comms::PollerParameterId id, int32_t rawValue) {
  hardware::PollerParameters updated = config_;
  bool recognised = true;
  switch (id) {
    case comms::PollerParameterId::kPositionHome:
      updated.positionHome = rawValue;
      Serial.printf("[Poller] Parameter preview positionHome=%ld\n", static_cast<long>(rawValue));
      break;
    case comms::PollerParameterId::kPositionUpTarget:
      updated.positionUpTarget = rawValue;
      Serial.printf("[Poller] Parameter preview positionUpTarget=%ld\n", static_cast<long>(rawValue));
      break;
    case comms::PollerParameterId::kPositionDownTarget:
      updated.positionDownTarget = rawValue;
      Serial.printf("[Poller] Parameter preview positionDownTarget=%ld\n", static_cast<long>(rawValue));
      break;
    case comms::PollerParameterId::kDownArmMargin:
      updated.downArmMargin = rawValue;
      Serial.printf("[Poller] Parameter preview downArmMargin=%ld\n", static_cast<long>(rawValue));
      break;
    case comms::PollerParameterId::kStepperMaxSpeed: {
      float value = 0.0f;
      static_assert(sizeof(value) == sizeof(rawValue), "float and int32_t size mismatch");
      memcpy(&value, &rawValue, sizeof(value));
      if (!isfinite(value)) {
        return false;
      }
      updated.stepperMaxSpeed = value;
      Serial.printf("[Poller] Parameter preview stepperMaxSpeed=%0.2f\n", value);
      break;
    }
    case comms::PollerParameterId::kStepperAcceleration: {
      float value = 0.0f;
      static_assert(sizeof(value) == sizeof(rawValue), "float and int32_t size mismatch");
      memcpy(&value, &rawValue, sizeof(value));
      if (!isfinite(value)) {
        return false;
      }
      updated.stepperAcceleration = value;
      Serial.printf("[Poller] Parameter preview stepperAcceleration=%0.2f\n", value);
      break;
    }
    case comms::PollerParameterId::kStatusIntervalMs:
      if (rawValue <= 0) {
        return false;
      }
      updated.statusIntervalMs = static_cast<uint32_t>(rawValue);
      Serial.printf("[Poller] Parameter preview statusIntervalMs=%ld\n", static_cast<long>(rawValue));
      break;
    case comms::PollerParameterId::kOverrunCooldownMs:
      if (rawValue <= 0) {
        return false;
      }
      updated.overrunCooldownMs = static_cast<uint32_t>(rawValue);
      Serial.printf("[Poller] Parameter preview overrunCooldownMs=%ld\n", static_cast<long>(rawValue));
      break;
    default:
      recognised = false;
      break;
  }

  if (!recognised) {
    Serial.printf("[Poller] Parameter update rejected: %s (unrecognised)\n",
                  parameterIdToString(id));
    return false;
  }

  updated = hardware::sanitized(updated);
  config_ = updated;
  stepper_.applyConfig(config_);
  onConfigChanged();

  if (!settings::saveParameters(config_)) {
    Serial.println("[Poller] Parameter save failed");
    return false;
  }
  Serial.printf("[Poller] Parameter %s updated\n", parameterIdToString(id));
  return true;
}

void PollerController::onConfigChanged() {
  status_.config = config_;
  overrunArmed_ = isPollerLowered();
  const uint32_t now = millis();
  cooldownWasActive_ = cooldownActive(now);
}

}  // namespace poller
