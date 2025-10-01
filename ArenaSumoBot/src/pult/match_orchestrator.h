#pragma once

#include <stdint.h>

#include "comms/messages.h"
#include "pult/hardware_config.h"
#include "pult/pult_controller.h"

namespace pult {

class MatchOrchestrator {
 public:
  enum class Phase : uint8_t {
    kIdle,
    kCountdown,
    kRunning
  };

  explicit MatchOrchestrator(PultController& controller);

  void begin();
  void update();

  void startMatch();
  void stopMatch();
  void triggerCountdown();
  void raisePoller();
  void lowerPoller();
  void calibrate();
  void stopStepper();
  void movePollerAbsolute(int32_t position);
  void movePollerRelative(int32_t delta);
  void movePollerToLimit(comms::LimitDirection direction);

  Phase phase() const { return phase_; }
  bool matchRunning() const { return phase_ == Phase::kRunning; }
  bool countdownActive() const { return phase_ == Phase::kCountdown; }
  uint32_t matchElapsedMs() const;
  uint32_t remainingMatchTimeMs() const;
  uint32_t countdownElapsedMs() const;
  const char* lastAction() const { return lastAction_; }
  bool pollerKnown() const { return controller_.pollerKnown(); }

 private:
  struct ButtonState {
    bool level = false;
    uint32_t lastChangeMs = 0;
    uint32_t lastTriggerMs = 0;
  };

  void updateButtons();
  void updateFromStatus();
  void updateTimers();
  void onCountdownFinished();
  void handlePollerSensorEvent();
  void setActionMessage(const char* msg);
  bool sendAnimation(comms::AnimationId animation);
  bool sendObserver(const char* text);

  PultController& controller_;
  Phase phase_ = Phase::kIdle;
  bool awaitingCountdown_ = false;
  bool countdownEngaged_ = false;
  uint32_t countdownStartMs_ = 0;
  uint32_t matchStartMs_ = 0;
  uint32_t lastProcessedStatusMs_ = 0;
  uint16_t lastStatusFlags_ = 0;
  bool pollerSensorHandled_ = false;
  char lastAction_[20] = "ready";

  ButtonState startStopButton_{};
  ButtonState pollerButton_{};
  ButtonState countdownButton_{};
};

}  // namespace pult
