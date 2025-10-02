#include "led_controller.h"

#include <Arduino.h>

namespace poller {
constexpr uint8_t LedController::kRingSizes[LedController::kRingCount];
constexpr uint16_t LedController::kRingOffsets[LedController::kRingCount];
namespace {
constexpr uint32_t kInitBlueHoldMs = 2000;
constexpr uint32_t kRainbowStepMs = 30;
constexpr uint8_t kRainbowSaturation = 200;
constexpr uint8_t kRainbowValue = 150;
constexpr uint8_t kIdlePollerBrightness = 40;
constexpr uint8_t kIdleRundumBrightness = 25;
constexpr uint8_t kMatchGreenValue = 180;
constexpr uint32_t kArmedStepMs = 220;
constexpr uint32_t kOverrunWaveStepMs = 45;
constexpr uint32_t kStopModeHoldMs = 10000;
constexpr uint32_t kCountdownAmberPhaseMs = 3000;
constexpr uint32_t kCountdownBlinkAmberMs = 500;
constexpr uint32_t kCountdownBlinkRedMs = 250;
constexpr uint32_t kOverrunTriggeredResetDelayMs = 500;

CRGB makeDim(const CRGB& color, uint8_t value) {
  CRGB result = color;
  result.nscale8_video(value);
  return result;
}

uint8_t clampBrightness(int value) {
  if (value <= 0) {
    return 0;
  }
  if (value >= 255) {
    return 255;
  }
  return static_cast<uint8_t>(value);
}

}  // namespace

void LedController::begin() {
  FastLED.addLeds<WS2812B, static_cast<int>(hardware::PIN_LED_RUNDUM), RGB>(ledsRundum_, hardware::LEDS_RUNDUM);
  FastLED.addLeds<WS2812B, static_cast<int>(hardware::PIN_LED_ARENA), RGB>(ledsArena_, hardware::LEDS_ARENA);
  FastLED.addLeds<WS2812B, static_cast<int>(hardware::PIN_LED_POLLER), GRB>(ledsPoller_, hardware::LEDS_POLLER);

  resetStrips(CRGB::Blue);
  FastLED.show();

  mode_ = Mode::kInit;
  previousMode_ = Mode::kInit;
  modeStartMs_ = millis();
  lastUpdateMs_ = modeStartMs_;
  rainbowLastStepMs_ = modeStartMs_;
  lastInputs_ = inputs_;
  lastArmedStepMs_ = modeStartMs_;
  lastWaveStepMs_ = modeStartMs_;
  overrunTriggerMs_ = 0;
}

void LedController::applyInputs(const Inputs& inputs) {
  inputs_ = inputs;
}

void LedController::startAnimation(comms::AnimationId id) {
  switch (id) {
    case comms::AnimationId::kCountdown:
      setMode(Mode::kCountdown);
      break;
    case comms::AnimationId::kArenaControl:
      setMode(Mode::kMatch);
      break;
    case comms::AnimationId::kArenaStop:
      setMode(Mode::kStop);
      break;
    case comms::AnimationId::kPollerOverrun:
      overrunTriggered_ = true;
      overrunTriggerMs_ = millis();
      break;
    case comms::AnimationId::kNone:
    default:
      break;
  }

}

void LedController::stopAnimation() {
  setMode(Mode::kIdle);
  overrunTriggered_ = false;
}

comms::AnimationId LedController::activeAnimation() const {
  if (overrunTriggered_) {
    return comms::AnimationId::kPollerOverrun;
  }
  switch (mode_) {
    case Mode::kCountdown:
      return comms::AnimationId::kCountdown;
    case Mode::kMatch:
      return comms::AnimationId::kArenaControl;
    case Mode::kStop:
      return comms::AnimationId::kArenaStop;
    case Mode::kIdle:
    case Mode::kInit:
    default:
      return comms::AnimationId::kNone;
  }
}

void LedController::update() {
  const uint32_t now = millis();

  if (!lastInputs_.sensorActive && inputs_.sensorActive) {
    overrunTriggered_ = true;
    overrunTriggerMs_ = now;
  }

  if (overrunTriggered_ && inputs_.pollerIsUp && !inputs_.sensorActive && inputs_.overrunArmed) {
    // keep a small delay before clearing to avoid flicker when motor still moving
    if ((now - overrunTriggerMs_) > kOverrunTriggeredResetDelayMs) {
      overrunTriggered_ = false;
    }
  }

  if (mode_ == Mode::kStop && (now - modeStartMs_) > kStopModeHoldMs) {
    setMode(Mode::kIdle);
  }

  render(now);
  applyShow();

  lastInputs_ = inputs_;
  lastUpdateMs_ = now;
}

void LedController::setMode(Mode mode) {
  if (mode_ == mode) {
    return;
  }
  previousMode_ = mode_;
  mode_ = mode;
  modeStartMs_ = millis();
  rainbowLastStepMs_ = modeStartMs_;
  blinkAnchorMs_ = modeStartMs_;
  countdownBlinkPhase_ = false;
  dirty_ = true;

  if (mode_ == Mode::kCountdown) {
    overrunTriggered_ = false;
  }

  if (mode_ == Mode::kIdle && previousMode_ == Mode::kInit) {
    resetStrips(CRGB::Blue);
  }
}

void LedController::render(uint32_t nowMs) {
  switch (mode_) {
    case Mode::kInit:
      renderInit(nowMs);
      break;
    case Mode::kIdle:
      renderIdle(nowMs);
      break;
    case Mode::kCountdown:
      renderCountdown(nowMs);
      break;
    case Mode::kMatch:
      renderMatch(nowMs);
      break;
    case Mode::kStop:
      renderStop(nowMs);
      break;
  }
}

void LedController::renderInit(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - modeStartMs_;
  if (elapsed <= kInitBlueHoldMs) {
    resetStrips(CRGB::Blue);
    return;
  }
  setMode(Mode::kIdle);
  renderIdle(nowMs);
}

void LedController::renderIdle(uint32_t nowMs) {
  renderArenaRainbow(nowMs);

  // Poller ring softly blue if up, otherwise off.
  const bool pollerUp = inputs_.pollerIsUp;
  const CRGB pollerBase = pollerUp ? CRGB(0, 0, 80) : CRGB::Black;
  fill_solid(ledsPoller_, hardware::LEDS_POLLER, pollerBase);
  dirty_ = true;

  // Rundum gently lit when poller above down threshold.
  const bool pollerLowered = inputs_.pollerIsLowered;
  const CRGB rundumColor = pollerLowered ? CRGB::Black : makeDim(CRGB::Blue, kIdleRundumBrightness);
  fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, rundumColor);
  dirty_ = true;

  renderPoller(nowMs, pollerBase);
}

void LedController::renderCountdown(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - modeStartMs_;
  const bool finalPhase = elapsed >= kCountdownAmberPhaseMs;
  const uint32_t period = finalPhase ? kCountdownBlinkRedMs : kCountdownBlinkAmberMs;
  const uint32_t phase = (elapsed / period) & 0x1u;
  const bool on = (phase == 0u);
  const CRGB color = finalPhase ? CRGB::Red : CRGB::Orange;

  if (on) {
    renderArenaSolid(color);
  } else {
    resetStrips(CRGB::Black);
  }

  fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, on ? color : CRGB::Black);
  fill_solid(ledsPoller_, hardware::LEDS_POLLER, on ? color : CRGB::Black);
  dirty_ = true;
}

void LedController::renderMatch(uint32_t nowMs) {
  const CRGB arenaColor = CRGB(0, clampBrightness(kMatchGreenValue), 0);
  renderArenaSolid(arenaColor);

  renderRundum(arenaColor);
  renderPoller(nowMs, arenaColor);
}

void LedController::renderStop(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - modeStartMs_;
  const bool blinkPhase = ((elapsed / 300u) & 0x1u) == 0u;
  const CRGB color = blinkPhase ? CRGB::Red : CRGB::Black;

  renderArenaSolid(color);
  fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, color);
  fill_solid(ledsPoller_, hardware::LEDS_POLLER, color);
  dirty_ = true;
}

void LedController::renderPoller(uint32_t nowMs, const CRGB& arenaColor) {
  if (inputs_.manualControl) {
    fill_solid(ledsPoller_, hardware::LEDS_POLLER, arenaColor);
    dirty_ = true;
    return;
  }

  if (overrunTriggered_) {
    renderPollerOverrunRaw(nowMs);
    return;
  }

  if (inputs_.overrunArmed && !inputs_.cooldownActive && !inputs_.pollerMoving && !inputs_.sensorActive && !inputs_.pollerIsLowered) {
    renderPollerArmed(nowMs);
    return;
  }

  if (mode_ == Mode::kMatch) {
    const CRGB color = inputs_.pollerIsLowered ? CRGB::Black : arenaColor;
    fill_solid(ledsPoller_, hardware::LEDS_POLLER, color);
    dirty_ = true;
    return;
  }

  if (mode_ == Mode::kIdle) {
    const CRGB color = inputs_.pollerIsUp ? CRGB(0, 0, 90) : CRGB::Black;
    fill_solid(ledsPoller_, hardware::LEDS_POLLER, color);
    dirty_ = true;
    return;
  }

  if (mode_ == Mode::kStop) {
    // Already set in renderStop; nothing additional.
    return;
  }

  // Fallback: mirror arena color.
  fill_solid(ledsPoller_, hardware::LEDS_POLLER, arenaColor);
  dirty_ = true;
}

void LedController::renderPollerOverrunRaw(uint32_t nowMs) {
  if (nowMs - lastWaveStepMs_ >= kOverrunWaveStepMs) {
    lastWaveStepMs_ = nowMs;
    fill_solid(ledsPoller_, hardware::LEDS_POLLER, CRGB::Black);

    for (uint8_t ring = 0; ring < kRingCount; ++ring) {
      const uint16_t start = kRingOffsets[ring];
      const uint8_t size = kRingSizes[ring];
      const uint8_t pos = static_cast<uint8_t>((wavePosition_ + ring) % size);
      ledsPoller_[start + pos] = (ring == 0) ? CRGB::Blue : CRGB::Cyan;
      const uint8_t prev = (pos + size - 1) % size;
      ledsPoller_[start + prev] = CRGB(0, 0, 40);
    }

    ++wavePosition_;
    dirty_ = true;
  }
}

void LedController::renderPollerArmed(uint32_t nowMs) {
  if (nowMs - lastArmedStepMs_ >= kArmedStepMs) {
    lastArmedStepMs_ = nowMs;
    armedRingIndex_ = (armedRingIndex_ + 1) % kRingCount;
    dirty_ = true;
  }

  fill_solid(ledsPoller_, hardware::LEDS_POLLER, CRGB::Black);
  for (uint8_t ring = 0; ring < kRingCount; ++ring) {
    const uint16_t start = kRingOffsets[ring];
    const uint8_t size = kRingSizes[ring];
    const bool active = (ring == armedRingIndex_);
    const CRGB color = active ? CRGB::Blue : CRGB(0, 0, 25);
    for (uint8_t i = 0; i < size; ++i) {
      ledsPoller_[start + i] = color;
    }
  }
  dirty_ = true;
}

void LedController::renderRundum(const CRGB& arenaColor) {
  if (inputs_.pollerIsLowered) {
    fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, CRGB::Black);
  } else {
    fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, makeDim(arenaColor, 140));
  }
  dirty_ = true;
}

void LedController::renderArenaRainbow(uint32_t nowMs) {
  if (nowMs - rainbowLastStepMs_ < kRainbowStepMs) {
    return;
  }
  rainbowLastStepMs_ = nowMs;
  rainbowHue_ += 1;

  for (uint16_t i = 0; i < hardware::LEDS_ARENA; ++i) {
    const uint8_t hue = static_cast<uint8_t>(rainbowHue_ + (i * 2));
    ledsArena_[i] = CHSV(hue, kRainbowSaturation, kRainbowValue);
  }
  dirty_ = true;
}

void LedController::renderArenaSolid(const CRGB& color) {
  fill_solid(ledsArena_, hardware::LEDS_ARENA, color);
  dirty_ = true;
}

void LedController::renderArenaBlink(uint32_t nowMs, const CRGB& color, uint16_t periodMs) {
  const uint32_t elapsed = nowMs - modeStartMs_;
  const bool on = ((elapsed / periodMs) & 0x1u) == 0u;
  fill_solid(ledsArena_, hardware::LEDS_ARENA, on ? color : CRGB::Black);
  dirty_ = true;
}

void LedController::resetStrips(const CRGB& color) {
  fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, color);
  fill_solid(ledsPoller_, hardware::LEDS_POLLER, color);
  fill_solid(ledsArena_, hardware::LEDS_ARENA, color);
  dirty_ = true;
}

void LedController::applyShow() {
  if (!dirty_) {
    return;
  }
  FastLED.show();
  dirty_ = false;
}

}  // namespace poller
