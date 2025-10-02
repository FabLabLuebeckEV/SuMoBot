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
  endstopLatched_ = false;
  endstopEvent_ = false;
  calibrated_ = false;
  calibrationBackoffActive_ = false;
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
      }
    } else if (stepper_.distanceToGo() == 0) {
      mode_ = Mode::kIdle;
    }
  } else if (mode_ == Mode::kMoving && stepper_.distanceToGo() == 0) {
    mode_ = Mode::kIdle;
  }

  updateEnablePin();
}

void StepperController::setTarget(int32_t position) {
  stepper_.moveTo(position);
  mode_ = Mode::kMoving;
}

void StepperController::moveBy(int32_t delta) {
  stepper_.move(delta);
  mode_ = Mode::kMoving;
}

void StepperController::moveToLimit(comms::LimitDirection direction) {
  switch (direction) {
    case comms::LimitDirection::kUp:
      setTarget(config().positionUpTarget);
      break;
    case comms::LimitDirection::kDown:
      setTarget(config().positionDownTarget);
      break;
    default:
      break;
  }
}

void StepperController::stop() {
  stepper_.stop();
  mode_ = Mode::kIdle;
  calibrationBackoffActive_ = false;
}

void StepperController::startCalibration() {
  if (mode_ == Mode::kCalibrating) {
    return;
  }

  mode_ = Mode::kCalibrating;
  calibrated_ = false;
  calibrationBackoffActive_ = false;
  stepper_.stop();
  const int32_t delta = (config().positionUpTarget - stepper_.currentPosition()) + 2000;
  stepper_.move(delta <= 0 ? 2000 : delta);
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

  if (mode_ == Mode::kCalibrating) {
    calibrationBackoffActive_ = true;
    stepper_.move(-200);
  } else {
    mode_ = Mode::kMoving;
    stepper_.move(-100);
    calibrated_ = true;
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
}

const hardware::PollerParameters& StepperController::config() const {
  return configCache_;
}

}  // namespace poller
