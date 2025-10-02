#include "stepper_controller.h"

#include <Arduino.h>

namespace poller {

void StepperController::begin() {
  pinMode(static_cast<uint8_t>(hardware::PIN_POLLER_ENABLE), OUTPUT);
  digitalWrite(static_cast<uint8_t>(hardware::PIN_POLLER_ENABLE), HIGH);

  pinMode(static_cast<uint8_t>(hardware::PIN_ENDSTOP), INPUT_PULLUP);

  stepper_.setMaxSpeed(hardware::STEPPER_MAX_SPEED);
  stepper_.setAcceleration(hardware::STEPPER_ACCELERATION);
  stepper_.setCurrentPosition(hardware::POSITION_HOME);

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
      setTarget(hardware::POSITION_UP_TARGET);
      break;
    case comms::LimitDirection::kDown:
      setTarget(hardware::POSITION_DOWN_TARGET);
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
  const int32_t delta = (hardware::POSITION_UP_TARGET - stepper_.currentPosition()) + 2000;
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

int32_t StepperController::currentPosition() {
  return stepper_.currentPosition();
}

int32_t StepperController::targetPosition() {
  return stepper_.targetPosition();
}

void StepperController::handleEndstopTriggered() {
  endstopEvent_ = true;
  stepper_.stop();
  stepper_.setCurrentPosition(hardware::POSITION_HOME);

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

}  // namespace poller
