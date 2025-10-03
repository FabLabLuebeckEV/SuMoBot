#include "led_controller.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

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

const char* animationName(comms::AnimationId id) {
  switch (id) {
    case comms::AnimationId::kNone:
      return "none";
    case comms::AnimationId::kCountdown:
      return "countdown";
    case comms::AnimationId::kPollerOverrun:
      return "poller_overrun";
    case comms::AnimationId::kArenaStop:
      return "arena_stop";
    case comms::AnimationId::kArenaControl:
      return "arena_control";
  }
  return "unknown";
}

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
  lastShowMs_ = modeStartMs_;

  requestedMode_ = Mode::kInit;
  modeChangeRequested_ = false;
  overrunTriggerRequested_ = false;
  inputsPending_ = false;

#if CONFIG_FREERTOS_UNICORE
  const BaseType_t ledCore = 0;
#else
  const BaseType_t ledCore = 0;
#endif
  if (xTaskCreatePinnedToCore(&LedController::ledTaskTrampoline, "PollerLED", 4096, this, 2, &taskHandle_, ledCore) != pdPASS) {
    taskHandle_ = nullptr;
    Serial.println("[LedController] Failed to create LED task");
  } else {
    Serial.println("[LedController] LED task started");
  }

  Serial.println("[LedController] Controller initialised");
}

void LedController::applyInputs(const Inputs& inputs) {
  taskENTER_CRITICAL(&inputsMux_);
  pendingInputs_ = inputs;
  inputsPending_ = true;
  taskEXIT_CRITICAL(&inputsMux_);
}

void LedController::startAnimation(comms::AnimationId id) {
  Serial.printf("[LedController] Start animation: %s\n", animationName(id));
  switch (id) {
    case comms::AnimationId::kCountdown:
      requestMode(Mode::kCountdown);
      break;
    case comms::AnimationId::kArenaControl:
      requestMode(Mode::kMatch);
      break;
    case comms::AnimationId::kArenaStop:
      requestMode(Mode::kStop);
      break;
    case comms::AnimationId::kPollerOverrun:
      taskENTER_CRITICAL(&stateMux_);
      overrunTriggerRequested_ = true;
      taskEXIT_CRITICAL(&stateMux_);
      break;
    case comms::AnimationId::kNone:
    default:
      break;
  }

}

void LedController::stopAnimation() {
  Serial.println("[LedController] Stop animation");
  requestMode(Mode::kIdle);
  taskENTER_CRITICAL(&stateMux_);
  overrunTriggered_ = false;
  overrunTriggerRequested_ = false;
  taskEXIT_CRITICAL(&stateMux_);
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
  // LED updates are handled asynchronously by the FreeRTOS task.
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
  logModeChange(mode_, "request");

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

void LedController::applyShow(uint32_t nowMs) {
  if (!dirty_) {
    return;
  }
  if ((nowMs - lastShowMs_) < kMinShowIntervalMs) {
    return;
  }
  FastLED.show();
  lastShowMs_ = nowMs;
  dirty_ = false;
}

void LedController::requestMode(Mode mode) {
  Serial.printf("[LedController] Mode request -> %s\n", modeName(mode));
  taskENTER_CRITICAL(&stateMux_);
  requestedMode_ = mode;
  modeChangeRequested_ = true;
  taskEXIT_CRITICAL(&stateMux_);
}

void LedController::ledTaskTrampoline(void* param) {
  auto* self = static_cast<LedController*>(param);
  self->taskLoop();
}

void LedController::taskLoop() {
  const TickType_t delayTicks = pdMS_TO_TICKS(8);
  while (true) {
    const uint32_t now = millis();

    taskENTER_CRITICAL(&inputsMux_);
    if (inputsPending_) {
      inputs_ = pendingInputs_;
      inputsPending_ = false;
    }
    taskEXIT_CRITICAL(&inputsMux_);

    bool applyMode = false;
    Mode newMode = mode_;
    bool triggerOverrun = false;
    taskENTER_CRITICAL(&stateMux_);
    if (modeChangeRequested_) {
      newMode = requestedMode_;
      modeChangeRequested_ = false;
      applyMode = true;
    }
    if (overrunTriggerRequested_) {
      triggerOverrun = true;
      overrunTriggerRequested_ = false;
    }
    taskEXIT_CRITICAL(&stateMux_);

    if (applyMode) {
      Serial.printf("[LedController] Applying mode -> %s\n", modeName(newMode));
      setMode(newMode);
    }
    if (triggerOverrun) {
      overrunTriggered_ = true;
      overrunTriggerMs_ = now;
      Serial.println("[LedController] Overrun animation triggered (manual)");
    }

    if (!lastInputs_.sensorActive && inputs_.sensorActive) {
      overrunTriggered_ = true;
      overrunTriggerMs_ = now;
      Serial.println("[LedController] Overrun animation triggered (sensor)");
    }

    if (overrunTriggered_ && inputs_.pollerIsUp && !inputs_.sensorActive && inputs_.overrunArmed) {
      if ((now - overrunTriggerMs_) > kOverrunTriggeredResetDelayMs) {
        overrunTriggered_ = false;
        Serial.println("[LedController] Overrun animation reset");
      }
    }

    if (mode_ == Mode::kStop && (now - modeStartMs_) > kStopModeHoldMs) {
      setMode(Mode::kIdle);
    }

    render(now);
    applyShow(now);

    lastInputs_ = inputs_;
    lastUpdateMs_ = now;

    vTaskDelay(delayTicks);
  }
}

const char* LedController::modeName(Mode mode) {
  switch (mode) {
    case Mode::kInit:
      return "init";
    case Mode::kIdle:
      return "idle";
    case Mode::kCountdown:
      return "countdown";
    case Mode::kMatch:
      return "match";
    case Mode::kStop:
      return "stop";
  }
  return "unknown";
}

void LedController::logModeChange(Mode mode, const char* reason) {
  Serial.printf("[LedController] Mode -> %s", modeName(mode));
  if (reason && reason[0] != '\0') {
    Serial.printf(" (%s)", reason);
  }
  Serial.println();
}

}  // namespace poller
