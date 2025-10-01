#include "match_orchestrator.h"

#include <Arduino.h>
#include <functional>
#include <string.h>

namespace pult {

namespace {
constexpr uint32_t kStartStopCooldownMs = 1000;
constexpr uint32_t kAuxButtonCooldownMs = 3000;
constexpr uint32_t kMinRunBeforeStopMs = 3000;
}

MatchOrchestrator::MatchOrchestrator(PultController& controller)
    : controller_(controller) {
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
  awaitingCountdown_ = true;
  countdownEngaged_ = false;
  countdownStartMs_ = now;
  matchStartMs_ = 0;
  phase_ = Phase::kCountdown;

  sendAnimation(comms::AnimationId::kCountdown);
  movePollerToLimit(comms::LimitDirection::kUp);
  sendObserver("countdown");
  setActionMessage("countdown");
}

void MatchOrchestrator::stopMatch() {
  if (phase_ == Phase::kIdle && !awaitingCountdown_) {
    return;
  }

  awaitingCountdown_ = false;
  countdownEngaged_ = false;
  phase_ = Phase::kIdle;
  matchStartMs_ = 0;
  countdownStartMs_ = 0;

  sendAnimation(comms::AnimationId::kArenaStop);
  controller_.sendStop();
  controller_.sendMoveToLimit(comms::LimitDirection::kDown);
  sendObserver("stop");
  setActionMessage("stop");
}

void MatchOrchestrator::triggerCountdown() {
  sendAnimation(comms::AnimationId::kCountdown);
  sendObserver("countdown");
  setActionMessage("countdown");
}

void MatchOrchestrator::raisePoller() {
  controller_.sendMoveToLimit(comms::LimitDirection::kUp);
  sendAnimation(comms::AnimationId::kPollerOverrun);
  sendObserver("poller");
  setActionMessage("poller up");
}

void MatchOrchestrator::lowerPoller() {
  controller_.sendMoveToLimit(comms::LimitDirection::kDown);
  setActionMessage("poller down");
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
    if (phase_ == Phase::kIdle && !awaitingCountdown_) {
      startMatch();
    } else if (phase_ == Phase::kCountdown) {
      stopMatch();
    } else if (phase_ == Phase::kRunning) {
      const uint32_t elapsed = matchElapsedMs();
      if (elapsed >= kMinRunBeforeStopMs) {
        stopMatch();
      }
    }
  });

  handleButton(pollerButton_, pult_hw::PIN_BUTTON_POLLER, kAuxButtonCooldownMs, [&]() {
    raisePoller();
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

  if (phase_ == Phase::kCountdown) {
    if (status.activeAnimation == comms::AnimationId::kCountdown) {
      countdownEngaged_ = true;
    } else if (countdownEngaged_ && awaitingCountdown_) {
      onCountdownFinished();
    }
  }

  const bool sensorActive = (flags & static_cast<uint16_t>(comms::StatusFlag::kPollerSensorActive)) != 0;
  if (sensorActive && !pollerSensorHandled_) {
    handlePollerSensorEvent();
    pollerSensorHandled_ = true;
  } else if (!sensorActive) {
    pollerSensorHandled_ = false;
  }

  lastStatusFlags_ = flags;
}

void MatchOrchestrator::updateTimers() {
  if (phase_ == Phase::kRunning && matchStartMs_ != 0) {
    const uint32_t elapsed = millis() - matchStartMs_;
    if (elapsed >= pult_hw::MATCH_DURATION_MS) {
      stopMatch();
    }
  }
}

void MatchOrchestrator::onCountdownFinished() {
  awaitingCountdown_ = false;
  countdownEngaged_ = false;
  phase_ = Phase::kRunning;
  matchStartMs_ = millis();
  countdownStartMs_ = 0;
  sendObserver("start");
  sendAnimation(comms::AnimationId::kArenaControl);
  setActionMessage("start");
}

void MatchOrchestrator::handlePollerSensorEvent() {
  controller_.sendMoveToLimit(comms::LimitDirection::kUp);
  sendAnimation(comms::AnimationId::kPollerOverrun);
  sendObserver("poller");
  setActionMessage("poller");
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

}  // namespace pult
