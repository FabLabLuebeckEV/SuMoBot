#include "stepper_controller.h"

#include <Arduino.h>

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

  Serial.printf(
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
        Serial.printf("[Stepper] Calibration backoff complete at %ld\n",
                      static_cast<long>(stepper_.currentPosition()));
        reportModeChange(mode_, "calibration complete");
      }
    } else if (stepper_.distanceToGo() == 0) {
      mode_ = Mode::kIdle;
      Serial.printf("[Stepper] Calibration sweep ended at %ld\n",
                    static_cast<long>(stepper_.currentPosition()));
      reportModeChange(mode_, "calibration sweep done");
    }
  } else if (mode_ == Mode::kMoving && stepper_.distanceToGo() == 0) {
    mode_ = Mode::kIdle;
    Serial.printf("[Stepper] Movement complete at %ld\n", static_cast<long>(stepper_.currentPosition()));
    reportModeChange(mode_, "target reached");
  }

  updateEnablePin();
}

void StepperController::setTarget(int32_t position) {
  const int32_t current = stepper_.currentPosition();
  stepper_.moveTo(position);
  mode_ = Mode::kMoving;
  Serial.printf("[Stepper] Move absolute -> %ld (current %ld)\n", static_cast<long>(position),
                static_cast<long>(current));
  reportModeChange(mode_, "absolute target");
}

void StepperController::moveBy(int32_t delta) {
  stepper_.move(delta);
  mode_ = Mode::kMoving;
  Serial.printf("[Stepper] Move relative delta=%ld (target %ld)\n", static_cast<long>(delta),
                static_cast<long>(stepper_.targetPosition()));
  reportModeChange(mode_, "relative move");
}

void StepperController::moveToLimit(comms::LimitDirection direction) {
  switch (direction) {
    case comms::LimitDirection::kUp:
      Serial.printf("[Stepper] Move to limit: up (%ld)\n", static_cast<long>(config().positionUpTarget));
      setTarget(config().positionUpTarget);
      break;
    case comms::LimitDirection::kDown:
      Serial.printf("[Stepper] Move to limit: down (%ld)\n", static_cast<long>(config().positionDownTarget));
      setTarget(config().positionDownTarget);
      break;
    default:
      Serial.println("[Stepper] Move to limit: unsupported direction");
      break;
  }
}

void StepperController::stop() {
  stepper_.stop();
  mode_ = Mode::kIdle;
  calibrationBackoffActive_ = false;
  Serial.printf("[Stepper] Stop requested at %ld\n", static_cast<long>(stepper_.currentPosition()));
  reportModeChange(mode_, "stop command");
}

void StepperController::startCalibration() {
  if (mode_ == Mode::kCalibrating) {
    Serial.println("[Stepper] Calibration already running");
    return;
  }

  mode_ = Mode::kCalibrating;
  calibrated_ = false;
  calibrationBackoffActive_ = false;
  stepper_.stop();
  const int32_t delta = (config().positionUpTarget - stepper_.currentPosition()) + 2000;
  const int32_t sweep = (delta <= 0) ? 2000 : delta;
  stepper_.move(sweep);
  Serial.printf("[Stepper] Calibration started, sweep=%ld\n", static_cast<long>(sweep));
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
  Serial.printf("[Stepper] Endstop triggered, home reset to %ld\n",
                static_cast<long>(config().positionHome));

  if (mode_ == Mode::kCalibrating) {
    calibrationBackoffActive_ = true;
    stepper_.move(-200);
    Serial.println("[Stepper] Calibration backoff engaged (-200)");
  } else {
    mode_ = Mode::kMoving;
    stepper_.move(-100);
    calibrated_ = true;
    Serial.println("[Stepper] Endstop hit during move, backing off 100 steps");
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
    Serial.printf("[Stepper] Config applied with home offset %ld (current %ld, target %ld)\n",
                  static_cast<long>(offset), static_cast<long>(current), static_cast<long>(target));
  } else {
    Serial.println("[Stepper] Config applied without home offset");
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
  Serial.printf("[Stepper] Motion profile updated (max=%0.2f, accel=%0.2f)\n",
                configCache_.stepperMaxSpeed, configCache_.stepperAcceleration);
}

const hardware::PollerParameters& StepperController::config() const {
  return configCache_;
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
  Serial.printf("[Stepper] Mode -> %s", modeName(newMode));
  if (reason && reason[0] != '\0') {
    Serial.printf(" (%s)", reason);
  }
  Serial.println();
}

}  // namespace poller
