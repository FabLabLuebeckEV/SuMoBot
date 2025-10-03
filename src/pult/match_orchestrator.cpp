#include "match_orchestrator.h"

#include <Arduino.h>
#include <functional>
#include <string.h>

namespace pult {

namespace {
constexpr uint32_t kStartStopCooldownMs = 1000;
constexpr uint32_t kAuxButtonCooldownMs = 3000;
constexpr uint32_t kCountdownDurationMs = 5000;
}

MatchOrchestrator::MatchOrchestrator(PultController& controller, StatusNotifier& notifier)
    : controller_(controller), notifier_(notifier) {
  setActionMessage("ready");
}

void MatchOrchestrator::begin() {
  pinMode(static_cast<uint8_t>(pult_hw::PIN_BUTTON_START_STOP), INPUT);
  pinMode(static_cast<uint8_t>(pult_hw::PIN_BUTTON_POLLER), INPUT);
  pinMode(static_cast<uint8_t>(pult_hw::PIN_BUTTON_COUNTDOWN), INPUT);

  startStopButton_.level = digitalRead(static_cast<uint8_t>(pult_hw::PIN_BUTTON_START_STOP)) == LOW;
  pollerButton_.level = digitalRead(static_cast<uint8_t>(pult_hw::PIN_BUTTON_POLLER)) == LOW;
  countdownButton_.level = digitalRead(static_cast<uint8_t>(pult_hw::PIN_BUTTON_COUNTDOWN)) == LOW;
}

void MatchOrchestrator::update() {
  updateButtons();
  updateFromStatus();
  updateTimers();
}

void MatchOrchestrator::startMatch() {
  if (phase_ == Phase::kCountdown || phase_ == Phase::kRunning) {
    return;
  }

  const uint32_t now = millis();
  countdownStartMs_ = now;
  matchStartMs_ = 0;
  countdownForMatch_ = true;
  countdownReturnPhase_ = Phase::kRunning;
  phase_ = Phase::kCountdown;
  overrunHandled_ = false;
  resetAutoLower();

  matchType_ = MatchType::kNormal;
  if (controller_.hasStatus()) {
    const auto& status = controller_.status();
    if (pollerIsLowered(status)) {
      matchType_ = MatchType::kDeath;
    }
  }

  disarmPollerOverrun(false);
  sendAnimation(comms::AnimationId::kCountdown);

  if (matchType_ == MatchType::kDeath) {
    sendObserver("deathmatch countdown");
    setActionMessage("death countdown");
  } else {
    sendObserver("countdown");
    setActionMessage("countdown");
  }
  notify(StatusNotifier::Action::kCountdownStart);
}

void MatchOrchestrator::stopMatch() {
  if (phase_ == Phase::kIdle && !countdownActive()) {
    return;
  }

  if (phase_ == Phase::kCountdown && !countdownForMatch_ && countdownReturnPhase_ == Phase::kIdle) {
    controller_.sendStopAnimation();
    countdownStartMs_ = 0;
    countdownForMatch_ = false;
    phase_ = Phase::kIdle;
    countdownReturnPhase_ = Phase::kIdle;
    setActionMessage("countdown stop");
    return;
  }

  countdownForMatch_ = false;
  phase_ = Phase::kIdle;
  countdownReturnPhase_ = Phase::kIdle;
  matchStartMs_ = 0;
  countdownStartMs_ = 0;
  matchType_ = MatchType::kUnknown;
  resetAutoLower();

  sendAnimation(comms::AnimationId::kArenaStop);
  controller_.sendStop();
  sendObserver("stop");
  setActionMessage("stop");
  notify(StatusNotifier::Action::kMatchStop);
}

void MatchOrchestrator::triggerCountdown() {
  const uint32_t now = millis();
  countdownStartMs_ = now;
  countdownForMatch_ = false;
  countdownReturnPhase_ = phase_ == Phase::kCountdown ? countdownReturnPhase_ : phase_;
  phase_ = Phase::kCountdown;

  sendAnimation(comms::AnimationId::kCountdown);
  sendObserver("countdown");
  setActionMessage("countdown");
  notify(StatusNotifier::Action::kCountdownStart);
}

void MatchOrchestrator::raisePoller() {
  disarmPollerOverrun(false);
  controller_.sendMoveToLimit(comms::LimitDirection::kUp);
  sendAnimation(comms::AnimationId::kPollerOverrun);
  sendObserver("poller");
  setActionMessage("poller up");
  if (phase_ == Phase::kRunning && matchType_ == MatchType::kNormal && autoLowerDelayMs_ > 0) {
    scheduleAutoLower(millis());
  } else {
    resetAutoLower();
  }
  notify(StatusNotifier::Action::kPollerUp);
}

void MatchOrchestrator::lowerPoller() {
  controller_.sendMoveToLimit(comms::LimitDirection::kDown);
  armPollerOverrun(false);
  setActionMessage("poller down");
  if (phase_ == Phase::kRunning && matchType_ == MatchType::kNormal) {
    autoLowerScheduledMs_ = 0;
    autoLowerTriggered_ = true;
  } else {
    resetAutoLower();
  }
  notify(StatusNotifier::Action::kPollerDown);
}

void MatchOrchestrator::togglePoller() {
  if (!controller_.hasStatus()) {
    raisePoller();
    return;
  }

  const auto& status = controller_.status();
  if (pollerIsLowered(status)) {
    raisePoller();
  } else {
    lowerPoller();
  }
}

void MatchOrchestrator::calibrate() {
  controller_.sendCalibrate();
  setActionMessage("calibrate");
}

void MatchOrchestrator::stopStepper() {
  controller_.sendStop();
  setActionMessage("motor stop");
}

void MatchOrchestrator::movePollerAbsolute(int32_t position) {
  controller_.sendMoveAbsolute(position);
  char buf[20];
  snprintf(buf, sizeof(buf), "abs %ld", static_cast<long>(position));
  setActionMessage(buf);
}

void MatchOrchestrator::movePollerRelative(int32_t delta) {
  controller_.sendMoveRelative(delta);
  char buf[20];
  snprintf(buf, sizeof(buf), "rel %ld", static_cast<long>(delta));
  setActionMessage(buf);
}

void MatchOrchestrator::movePollerToLimit(comms::LimitDirection direction) {
  controller_.sendMoveToLimit(direction);
  const char* msg = direction == comms::LimitDirection::kUp ? "limit up" : "limit down";
  setActionMessage(msg);
}

void MatchOrchestrator::armPollerOverrun(bool announce) {
  controller_.sendSetOverrunArmed(true);
  if (announce) {
    setActionMessage("overrun arm");
  }
}

void MatchOrchestrator::disarmPollerOverrun(bool announce) {
  controller_.sendSetOverrunArmed(false);
  if (announce) {
    setActionMessage("overrun off");
  }
}

uint32_t MatchOrchestrator::matchElapsedMs() const {
  if (matchStartMs_ == 0 || !matchRunning()) {
    return 0;
  }
  return millis() - matchStartMs_;
}

uint32_t MatchOrchestrator::remainingMatchTimeMs() const {
  if (!matchRunning() || matchStartMs_ == 0) {
    return pult_hw::MATCH_DURATION_MS;
  }

  const uint32_t elapsed = millis() - matchStartMs_;
  if (matchType_ == MatchType::kDeath) {
    return elapsed;
  }
  if (elapsed >= pult_hw::MATCH_DURATION_MS) {
    return 0;
  }
  return pult_hw::MATCH_DURATION_MS - elapsed;
}

uint32_t MatchOrchestrator::countdownElapsedMs() const {
  if (phase_ != Phase::kCountdown || countdownStartMs_ == 0) {
    return 0;
  }
  return millis() - countdownStartMs_;
}

uint32_t MatchOrchestrator::countdownRemainingMs() const {
  if (phase_ != Phase::kCountdown || countdownStartMs_ == 0) {
    return 0;
  }
  const uint32_t elapsed = millis() - countdownStartMs_;
  if (elapsed >= kCountdownDurationMs) {
    return 0;
  }
  return kCountdownDurationMs - elapsed;
}

bool MatchOrchestrator::pollerOverrunArmed() const {
  return (lastStatusFlags_ & static_cast<uint16_t>(comms::StatusFlag::kOverrunArmed)) != 0;
}

bool MatchOrchestrator::pollerCooldownActive() const {
  return (lastStatusFlags_ & static_cast<uint16_t>(comms::StatusFlag::kCooldownActive)) != 0;
}

bool MatchOrchestrator::pollerOverrunDetected() const {
  return (lastStatusFlags_ & static_cast<uint16_t>(comms::StatusFlag::kOverrunDetected)) != 0;
}

void MatchOrchestrator::setAutoLowerDelayMs(uint32_t delay) {
  autoLowerDelayMs_ = delay;
  if (phase_ == Phase::kRunning && matchType_ == MatchType::kNormal && autoLowerDelayMs_ > 0) {
    scheduleAutoLower(millis());
  } else if (autoLowerDelayMs_ == 0) {
    resetAutoLower();
  }
}

void MatchOrchestrator::updateButtons() {
  const uint32_t now = millis();

  auto handleButton = [&](ButtonState& state, gpio_num_t pin, uint32_t cooldownMs, const std::function<void()>& onPress) {
    const bool level = digitalRead(static_cast<uint8_t>(pin)) == LOW;
    if (level != state.level) {
      if (now - state.lastChangeMs >= pult_hw::BUTTON_DEBOUNCE_MS) {
        state.level = level;
        state.lastChangeMs = now;
        if (level && (now - state.lastTriggerMs >= cooldownMs)) {
          state.lastTriggerMs = now;
          onPress();
        }
      }
    }
  };

  handleButton(startStopButton_, pult_hw::PIN_BUTTON_START_STOP, kStartStopCooldownMs, [&]() {
    if (phase_ == Phase::kIdle) {
      startMatch();
    } else {
      stopMatch();
    }
  });

  handleButton(pollerButton_, pult_hw::PIN_BUTTON_POLLER, kAuxButtonCooldownMs, [&]() {
    togglePoller();
  });

  handleButton(countdownButton_, pult_hw::PIN_BUTTON_COUNTDOWN, kAuxButtonCooldownMs, [&]() {
    triggerCountdown();
  });
}

void MatchOrchestrator::updateFromStatus() {
  if (!controller_.hasStatus()) {
    return;
  }

  const uint32_t stamp = controller_.lastStatusTimestamp();
  if (stamp == 0 || stamp == lastProcessedStatusMs_) {
    return;
  }

  lastProcessedStatusMs_ = stamp;
  const comms::PollerStatus& status = controller_.status();
  const uint16_t flags = status.statusFlags;
  lastStatusFlags_ = flags;

  const bool lowered = pollerIsLowered(status);
  const uint32_t now = millis();

  if (phase_ == Phase::kCountdown && countdownForMatch_ && matchType_ == MatchType::kUnknown) {
    matchType_ = lowered ? MatchType::kDeath : MatchType::kNormal;
  }

  if (phase_ == Phase::kRunning && matchType_ == MatchType::kUnknown) {
    matchType_ = lowered ? MatchType::kDeath : MatchType::kNormal;
  }

  if (phase_ == Phase::kRunning && matchType_ == MatchType::kNormal && autoLowerDelayMs_ > 0) {
    if (lowered) {
      autoLowerTriggered_ = true;
      autoLowerScheduledMs_ = 0;
    } else if (autoLowerScheduledMs_ == 0 || autoLowerTriggered_) {
      autoLowerScheduledMs_ = now + autoLowerDelayMs_;
      autoLowerTriggered_ = false;
    }
  }

  if (phase_ == Phase::kRunning && matchType_ == MatchType::kNormal && lowered) {
    const bool overrunArmed = (flags & static_cast<uint16_t>(comms::StatusFlag::kOverrunArmed)) != 0;
    const bool cooldown = (flags & static_cast<uint16_t>(comms::StatusFlag::kCooldownActive)) != 0;
    const bool sensorActive = (flags & static_cast<uint16_t>(comms::StatusFlag::kPollerSensorActive)) != 0;
    if (!overrunArmed && !cooldown && !sensorActive) {
      armPollerOverrun(false);
    }
  }

  const bool overrunDetected = (flags & static_cast<uint16_t>(comms::StatusFlag::kOverrunDetected)) != 0;
  if (overrunDetected && !overrunHandled_) {
    handlePollerSensorEvent();
    overrunHandled_ = true;
  } else if (!overrunDetected) {
    overrunHandled_ = false;
  }
}

void MatchOrchestrator::updateTimers() {
  const uint32_t now = millis();

  if (phase_ == Phase::kCountdown && countdownStartMs_ != 0) {
    if (now - countdownStartMs_ >= kCountdownDurationMs) {
      finishCountdown();
    }
  }

  if (phase_ == Phase::kRunning && matchStartMs_ != 0 && matchType_ != MatchType::kDeath) {
    if (now - matchStartMs_ >= pult_hw::MATCH_DURATION_MS) {
      stopMatch();
      return;
    }
  }

  if (phase_ == Phase::kRunning && autoLowerDelayMs_ > 0 && !autoLowerTriggered_ && autoLowerScheduledMs_ != 0) {
    if (now >= autoLowerScheduledMs_) {
      bool lowered = false;
      if (controller_.hasStatus()) {
        lowered = pollerIsLowered(controller_.status());
      }
      if (!lowered) {
        controller_.sendMoveToLimit(comms::LimitDirection::kDown);
        armPollerOverrun(false);
        sendObserver("auto lower");
        setActionMessage("auto lower");
        notify(StatusNotifier::Action::kPollerDown);
      }
      autoLowerTriggered_ = true;
    }
  }

  if (overrunRaisePending_ && now >= overrunRaiseScheduledMs_) {
    overrunRaisePending_ = false;
    overrunRaiseScheduledMs_ = 0;
    controller_.sendMoveToLimit(comms::LimitDirection::kUp);
    notify(StatusNotifier::Action::kPollerUp);
  }
}

void MatchOrchestrator::finishCountdown() {
  const uint32_t now = millis();
  const bool forMatch = countdownForMatch_;
  const Phase resume = countdownReturnPhase_;

  countdownForMatch_ = false;
  countdownReturnPhase_ = Phase::kIdle;
  countdownStartMs_ = 0;

  if (forMatch) {
    phase_ = Phase::kRunning;
    matchStartMs_ = now;
    const bool death = (matchType_ == MatchType::kDeath);
    sendAnimation(comms::AnimationId::kArenaControl);
    sendObserver(death ? "death start" : "start");
    setActionMessage(death ? "death start" : "start");
    scheduleAutoLower(now);
    if (!death) {
      armPollerOverrun(false);
    }
    notify(StatusNotifier::Action::kMatchStart);
  } else {
    phase_ = resume;
    if (phase_ == Phase::kRunning) {
      sendAnimation(comms::AnimationId::kArenaControl);
      setActionMessage("countdown done");
    } else {
      controller_.sendStopAnimation();
      setActionMessage("ready");
    }
  }
}

void MatchOrchestrator::handlePollerSensorEvent() {
  if (phase_ != Phase::kRunning || matchType_ != MatchType::kNormal) {
    return;
  }
  disarmPollerOverrun(false);
  sendAnimation(comms::AnimationId::kPollerOverrun);
  sendObserver("poller");
  setActionMessage("poller");
  const uint32_t now = millis();
  notify(StatusNotifier::Action::kPollerOverrun);
  if (autoLowerDelayMs_ > 0) {
    overrunRaiseScheduledMs_ = now + autoLowerDelayMs_;
    overrunRaisePending_ = true;
  } else {
    controller_.sendMoveToLimit(comms::LimitDirection::kUp);
  }
  autoLowerScheduledMs_ = 0;
  autoLowerTriggered_ = true;
}

void MatchOrchestrator::resetAutoLower() {
  autoLowerScheduledMs_ = 0;
  autoLowerTriggered_ = false;
  overrunRaisePending_ = false;
  overrunRaiseScheduledMs_ = 0;
}

void MatchOrchestrator::scheduleAutoLower(uint32_t now) {
  if (autoLowerDelayMs_ == 0 || matchType_ != MatchType::kNormal) {
    resetAutoLower();
    return;
  }

  if (controller_.hasStatus()) {
    const auto& status = controller_.status();
    if (pollerIsLowered(status)) {
      autoLowerScheduledMs_ = 0;
      autoLowerTriggered_ = true;
      return;
    }
  }

  autoLowerScheduledMs_ = now + autoLowerDelayMs_;
  autoLowerTriggered_ = false;
}

bool MatchOrchestrator::pollerIsLowered(const comms::PollerStatus& status) const {
  const int32_t threshold = status.config.positionDownTarget + status.config.downArmMargin;
  return status.currentPosition <= threshold;
}

void MatchOrchestrator::setActionMessage(const char* msg) {
  if (!msg) {
    return;
  }
  strncpy(lastAction_, msg, sizeof(lastAction_) - 1);
  lastAction_[sizeof(lastAction_) - 1] = '\0';
}

bool MatchOrchestrator::sendAnimation(comms::AnimationId animation) {
  return controller_.sendStartAnimation(animation);
}

bool MatchOrchestrator::sendObserver(const char* text) {
  return controller_.sendObserverMessage(text);
}

void MatchOrchestrator::notify(StatusNotifier::Action action) {
  notifier_.notify(action);
}

}  // namespace pult
