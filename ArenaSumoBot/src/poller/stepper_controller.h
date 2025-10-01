#pragma once

#include <AccelStepper.h>

#include "comms/messages.h"
#include "hardware_config.h"

namespace poller {

class StepperController {
 public:
  void begin();
  void update();

  void setTarget(int32_t position);
  void moveBy(int32_t delta);
  void moveToLimit(comms::LimitDirection direction);
  void stop();
  void startCalibration();

  bool isBusy();
  bool isCalibrating() const;
  bool endstopActive() const;
  bool consumeEndstopEvent();

  int32_t currentPosition();
  int32_t targetPosition();

 private:
  enum class Mode : uint8_t {
    kIdle,
    kMoving,
    kCalibrating
  };

  void handleEndstopTriggered();
  void updateEnablePin();

  AccelStepper stepper_{AccelStepper::DRIVER, hardware::PIN_STEPPER_STEP, hardware::PIN_STEPPER_DIR};
  Mode mode_ = Mode::kIdle;
  bool endstopLatched_ = false;
  bool endstopEvent_ = false;
};

}  // namespace poller
