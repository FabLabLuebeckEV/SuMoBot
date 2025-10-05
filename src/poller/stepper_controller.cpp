#include "stepper_controller.h"

#include <Arduino.h>

#include "poller/logging.h"

namespace poller {

void StepperController::begin(const hardware::PollerParameters& config) {
  configCache_ = hardware::sanitized(config);
  pinMode(static_cast<uint8_t>(hardware::PIN_POLLER_ENABLE), OUTPUT);
  digitalWrite(static_cast<uint8_t>(hardware::PIN_POLLER_ENABLE), HIGH);

  pinMode(static_cast<uint8_t>(hardware::PIN_ENDSTOP), INPUT_PULLUP);

  updateMotionProfile();
  stepper_.setCurrentPosition(configCache_.positionHome);

  mode_ = Mode::kIdle;
  lastReportedMode_ = mode_;
  endstopLatched_ = false;
  endstopEvent_ = false;
  calibrated_ = false;
  calibrationBackoffActive_ = false;

  POLLER_LOG_PRINTF(
      "[Stepper] Init complete (home=%ld, up=%ld, down=%ld, margin=%ld)\n",
      static_cast<long>(configCache_.positionHome), static_cast<long>(configCache_.positionUpTarget),
      static_cast<long>(configCache_.positionDownTarget), static_cast<long>(configCache_.downArmMargin));
  reportModeChange(mode_, "startup");
}

void StepperController::update() {
  const bool endstopNow = digitalRead(static_cast<uint8_t>(hardware::PIN_ENDSTOP)) == LOW;
  if (endstopNow && !endstopLatched_) {
    handleEndstopTriggered();
  }
  endstopLatched_ = endstopNow;

  stepper_.run();

  if (mode_ == Mode::kCalibrating) {
    if (calibrationBackoffActive_) {
      if (stepper_.distanceToGo() == 0) {
        calibrationBackoffActive_ = false;
        calibrated_ = true;
        mode_ = Mode::kIdle;
        POLLER_LOG_PRINTF("[Stepper] Calibration backoff complete at %ld\n",
                      static_cast<long>(stepper_.currentPosition()));
        reportModeChange(mode_, "calibration complete");
      }
    } else if (stepper_.distanceToGo() == 0) {
      mode_ = Mode::kIdle;
      POLLER_LOG_PRINTF("[Stepper] Calibration sweep ended at %ld\n",
                    static_cast<long>(stepper_.currentPosition()));
      reportModeChange(mode_, "calibration sweep done");
    }
  } else if (mode_ == Mode::kMoving && stepper_.distanceToGo() == 0) {
    mode_ = Mode::kIdle;
    POLLER_LOG_PRINTF("[Stepper] Movement complete at %ld\n", static_cast<long>(stepper_.currentPosition()));
    reportModeChange(mode_, "target reached");
  }

  updateEnablePin();
}

void StepperController::setTarget(int32_t position) {
  const int32_t current = stepper_.currentPosition();
  const int32_t clamped = clampTarget(position);
  if (clamped != position) {
    POLLER_LOG_PRINTF("[Stepper] Target request %ld clamped to %ld\n", static_cast<long>(position),
                  static_cast<long>(clamped));
  }
  stepper_.moveTo(clamped);
  mode_ = Mode::kMoving;
  POLLER_LOG_PRINTF("[Stepper] Move absolute -> %ld (current %ld)\n", static_cast<long>(clamped),
                static_cast<long>(current));
  reportModeChange(mode_, "absolute target");
}

void StepperController::moveBy(int32_t delta) {
  const int32_t current = stepper_.currentPosition();
  const int32_t desired = current + delta;
  const int32_t clamped = clampTarget(desired);
  if (clamped != desired) {
    POLLER_LOG_PRINTF("[Stepper] Relative move request %ld clamped to %ld\n", static_cast<long>(desired),
                  static_cast<long>(clamped));
  }
  stepper_.moveTo(clamped);
  mode_ = Mode::kMoving;
  const int32_t target = stepper_.targetPosition();
  POLLER_LOG_PRINTF("[Stepper] Move relative delta=%ld (target %ld)\n", static_cast<long>(delta),
                static_cast<long>(target));
  reportModeChange(mode_, "relative move");
}

void StepperController::moveToLimit(comms::LimitDirection direction) {
  switch (direction) {
    case comms::LimitDirection::kUp:
      POLLER_LOG_PRINTF("[Stepper] Move to limit: up (%ld)\n", static_cast<long>(config().positionUpTarget));
      setTarget(config().positionUpTarget);
      break;
    case comms::LimitDirection::kDown:
      POLLER_LOG_PRINTF("[Stepper] Move to limit: down (%ld)\n", static_cast<long>(config().positionDownTarget));
      setTarget(config().positionDownTarget);
      break;
    default:
      POLLER_LOG_PRINTLN("[Stepper] Move to limit: unsupported direction");
      break;
  }
}

void StepperController::stop() {
  stepper_.stop();
  mode_ = Mode::kIdle;
  calibrationBackoffActive_ = false;
  POLLER_LOG_PRINTF("[Stepper] Stop requested at %ld\n", static_cast<long>(stepper_.currentPosition()));
  reportModeChange(mode_, "stop command");
}

void StepperController::startCalibration() {
  if (mode_ == Mode::kCalibrating) {
    POLLER_LOG_PRINTLN("[Stepper] Calibration already running");
    return;
  }

  mode_ = Mode::kCalibrating;
  calibrated_ = false;
  calibrationBackoffActive_ = false;
  stepper_.stop();
  const int32_t delta = (config().positionUpTarget - stepper_.currentPosition()) + 2000;
  const int32_t sweep = (delta <= 0) ? 2000 : delta;
  stepper_.move(sweep);
  POLLER_LOG_PRINTF("[Stepper] Calibration started, sweep=%ld\n", static_cast<long>(sweep));
  reportModeChange(mode_, "calibration start");
}

bool StepperController::isBusy() {
  return stepper_.distanceToGo() != 0 || mode_ != Mode::kIdle;
}

bool StepperController::isCalibrating() const {
  return mode_ == Mode::kCalibrating;
}

bool StepperController::isCalibrated() const {
  return calibrated_;
}

bool StepperController::endstopActive() const {
  return endstopLatched_;
}

bool StepperController::consumeEndstopEvent() {
  const bool event = endstopEvent_;
  endstopEvent_ = false;
  return event;
}

void StepperController::handleEndstopTriggered() {
  endstopEvent_ = true;
  stepper_.stop();
  stepper_.setCurrentPosition(config().positionHome);
  POLLER_LOG_PRINTF("[Stepper] Endstop triggered, home reset to %ld\n",
                static_cast<long>(config().positionHome));

  if (mode_ == Mode::kCalibrating) {
    calibrationBackoffActive_ = true;
    const int32_t target = clampTarget(stepper_.currentPosition() - 200);
    stepper_.moveTo(target);
    POLLER_LOG_PRINTF("[Stepper] Calibration backoff engaged (target %ld)\n", static_cast<long>(target));
  } else {
    mode_ = Mode::kMoving;
    const int32_t target = clampTarget(stepper_.currentPosition() - 100);
    stepper_.moveTo(target);
    calibrated_ = true;
    POLLER_LOG_PRINTF("[Stepper] Endstop hit during move, backoff target %ld\n", static_cast<long>(target));
    reportModeChange(mode_, "endstop backoff");
  }
}

void StepperController::updateEnablePin() {
  const bool active = stepper_.distanceToGo() != 0;
  digitalWrite(static_cast<uint8_t>(hardware::PIN_POLLER_ENABLE), active ? LOW : HIGH);
}

void StepperController::applyConfig(const hardware::PollerParameters& config) {
  const hardware::PollerParameters sanitised = hardware::sanitized(config);
  const int32_t oldHome = configCache_.positionHome;
  configCache_ = sanitised;
  updateMotionProfile();

  const int32_t offset = configCache_.positionHome - oldHome;
  if (offset != 0) {
    const int32_t current = stepper_.currentPosition() + offset;
    const int32_t target = stepper_.targetPosition() + offset;
    stepper_.setCurrentPosition(current);
    stepper_.moveTo(target);
    POLLER_LOG_PRINTF("[Stepper] Config applied with home offset %ld (current %ld, target %ld)\n",
                  static_cast<long>(offset), static_cast<long>(current), static_cast<long>(target));
  } else {
    POLLER_LOG_PRINTLN("[Stepper] Config applied without home offset");
  }
}

int32_t StepperController::currentPosition() {
  return stepper_.currentPosition();
}

int32_t StepperController::targetPosition() {
  return stepper_.targetPosition();
}

void StepperController::updateMotionProfile() {
  stepper_.setMaxSpeed(configCache_.stepperMaxSpeed);
  stepper_.setAcceleration(configCache_.stepperAcceleration);
  POLLER_LOG_PRINTF("[Stepper] Motion profile updated (max=%0.2f, accel=%0.2f)\n",
                configCache_.stepperMaxSpeed, configCache_.stepperAcceleration);
}

const hardware::PollerParameters& StepperController::config() const {
  return configCache_;
}

int32_t StepperController::clampTarget(int32_t position) const {
  const int32_t lower = configCache_.positionDownTarget;
  const int32_t upper = configCache_.positionUpTarget;
  if (lower <= upper) {
    if (position < lower) {
      return lower;
    }
    if (position > upper) {
      return upper;
    }
    return position;
  }
  // Fallback in case of unexpected configuration ordering.
  const int32_t minTarget = upper;
  const int32_t maxTarget = lower;
  if (position < minTarget) {
    return minTarget;
  }
  if (position > maxTarget) {
    return maxTarget;
  }
  return position;
}

const char* StepperController::modeName(Mode mode) {
  switch (mode) {
    case Mode::kIdle:
      return "idle";
    case Mode::kMoving:
      return "moving";
    case Mode::kCalibrating:
      return "calibrating";
  }
  return "unknown";
}

void StepperController::reportModeChange(Mode newMode, const char* reason) {
  if (newMode == lastReportedMode_) {
    return;
  }
  lastReportedMode_ = newMode;
  POLLER_LOG_PRINTF("[Stepper] Mode -> %s", modeName(newMode));
  if (reason && reason[0] != '\0') {
    POLLER_LOG_PRINTF(" (%s)", reason);
  }
  POLLER_LOG_LINEBREAK();
}

}  // namespace poller
