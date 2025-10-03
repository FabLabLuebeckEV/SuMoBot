#pragma once

#include <stdint.h>

#include "comms/messages.h"
#include "pult/hardware_config.h"
#include "pult/pult_controller.h"
#include "pult/status_notifier.h"

namespace pult {

class MatchOrchestrator {
 public:
  enum class Phase : uint8_t {
    kIdle,
    kCountdown,
    kRunning
  };

  enum class MatchType : uint8_t {
    kUnknown,
    kNormal,
    kDeath
  };

  explicit MatchOrchestrator(PultController& controller, StatusNotifier& notifier);

  void begin();
  void update();

  void startMatch();
  void stopMatch();
  void triggerCountdown();
  void raisePoller();
  void lowerPoller();
  void togglePoller();
  void calibrate();
  void stopStepper();
  void movePollerAbsolute(int32_t position);
  void movePollerRelative(int32_t delta);
  void movePollerToLimit(comms::LimitDirection direction);
  void armPollerOverrun(bool announce = true);
  void disarmPollerOverrun(bool announce = true);

  Phase phase() const { return phase_; }
  bool matchRunning() const { return phase_ == Phase::kRunning; }
  bool countdownActive() const { return phase_ == Phase::kCountdown; }
  MatchType matchType() const { return matchType_; }
  uint32_t matchElapsedMs() const;
  uint32_t remainingMatchTimeMs() const;
  uint32_t countdownElapsedMs() const;
  uint32_t countdownRemainingMs() const;
  const char* lastAction() const { return lastAction_; }
  bool pollerKnown() const { return controller_.pollerKnown(); }
  bool pollerOverrunArmed() const;
  bool pollerCooldownActive() const;
  bool pollerOverrunDetected() const;
  void setAutoLowerDelayMs(uint32_t delay);
  uint32_t autoLowerDelayMs() const { return autoLowerDelayMs_; }

 private:
  struct ButtonState {
    bool level = false;
    uint32_t lastChangeMs = 0;
    uint32_t lastTriggerMs = 0;
  };

  void updateButtons();
  void updateFromStatus();
  void updateTimers();
  void finishCountdown();
  void handlePollerSensorEvent();
  void setActionMessage(const char* msg);
  bool sendAnimation(comms::AnimationId animation);
  bool sendObserver(const char* text);
  void scheduleAutoLower(uint32_t now);
  void resetAutoLower();
  bool pollerIsLowered(const comms::PollerStatus& status) const;
  void notify(StatusNotifier::Action action);

  PultController& controller_;
  StatusNotifier& notifier_;
  Phase phase_ = Phase::kIdle;
  MatchType matchType_ = MatchType::kUnknown;
  bool countdownForMatch_ = false;
  Phase countdownReturnPhase_ = Phase::kIdle;
  uint32_t countdownStartMs_ = 0;
  uint32_t matchStartMs_ = 0;
  uint32_t lastProcessedStatusMs_ = 0;
  uint16_t lastStatusFlags_ = 0;
  bool overrunHandled_ = false;
  char lastAction_[20] = "ready";

  uint32_t autoLowerDelayMs_ = 3000;
  uint32_t autoLowerScheduledMs_ = 0;
  bool autoLowerTriggered_ = false;
  uint32_t overrunRaiseScheduledMs_ = 0;
  bool overrunRaisePending_ = false;

  ButtonState startStopButton_{};
  ButtonState pollerButton_{};
  ButtonState countdownButton_{};
};

}  // namespace pult
