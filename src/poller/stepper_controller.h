#pragma once

#include <AccelStepper.h>

#include "comms/messages.h"
#include "hardware_config.h"

namespace poller {

class StepperController {
 public:
  void begin(const hardware::PollerParameters& config);
  void update();

  void setTarget(int32_t position);
  void moveBy(int32_t delta);
  void moveToLimit(comms::LimitDirection direction);
  void stop();
  void startCalibration();
  void applyConfig(const hardware::PollerParameters& config);

  bool isBusy();
  bool isCalibrating() const;
  bool isCalibrated() const;
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
  void updateMotionProfile();
  const hardware::PollerParameters& config() const;
  static const char* modeName(Mode mode);
  void reportModeChange(Mode newMode, const char* reason = nullptr);

  AccelStepper stepper_{AccelStepper::DRIVER, hardware::PIN_STEPPER_STEP, hardware::PIN_STEPPER_DIR};
  Mode mode_ = Mode::kIdle;
  bool endstopLatched_ = false;
  bool endstopEvent_ = false;
  bool calibrated_ = false;
  bool calibrationBackoffActive_ = false;
  hardware::PollerParameters configCache_ = hardware::DEFAULT_POLLER_PARAMETERS;
  Mode lastReportedMode_ = Mode::kIdle;
};

}  // namespace poller
