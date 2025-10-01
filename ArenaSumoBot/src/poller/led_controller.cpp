#include "led_controller.h"

#include <Arduino.h>

namespace poller {

void LedController::begin() {
  FastLED.addLeds<WS2812B, static_cast<int>(hardware::PIN_LED_RUNDUM), RGB>(ledsRundum_, hardware::LEDS_RUNDUM);
  FastLED.addLeds<WS2812B, static_cast<int>(hardware::PIN_LED_ARENA), RGB>(ledsArena_, hardware::LEDS_ARENA);
  FastLED.addLeds<WS2812B, static_cast<int>(hardware::PIN_LED_POLLER), GRB>(ledsPoller_, hardware::LEDS_POLLER);

  resetStrips(CRGB::Blue);
  FastLED.show();
  active_ = comms::AnimationId::kNone;
  phase_ = 0;
  phaseStartMs_ = millis();
  phaseStepMs_ = phaseStartMs_;
}

void LedController::update() {
  if (active_ == comms::AnimationId::kNone) {
    return;
  }

  const uint32_t now = millis();
  switch (active_) {
    case comms::AnimationId::kCountdown:
      updateCountdown(now);
      break;
    case comms::AnimationId::kPollerOverrun:
      updateOverrun(now);
      break;
    case comms::AnimationId::kArenaStop:
      updateStop(now);
      break;
    case comms::AnimationId::kArenaControl:
      updateControl(now);
      break;
    default:
      break;
  }

  applyShow();
}

void LedController::startAnimation(comms::AnimationId id) {
  active_ = id;
  phase_ = 0;
  phaseStartMs_ = millis();
  phaseStepMs_ = 0;
  wavePosition_ = 0;
  spinnerPosition_ = 0;
  dirty_ = true;

  if (id == comms::AnimationId::kNone) {
    resetStrips();
    applyShow();
  }
}

void LedController::stopAnimation() {
  if (active_ == comms::AnimationId::kNone) {
    return;
  }
  active_ = comms::AnimationId::kNone;
  phase_ = 0;
  phaseStepMs_ = 0;
  wavePosition_ = 0;
  spinnerPosition_ = 0;
  resetStrips();
  applyShow();
}

comms::AnimationId LedController::activeAnimation() const {
  return active_;
}

void LedController::resetStrips(const CRGB& color) {
  fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, color);
  fill_solid(ledsPoller_, hardware::LEDS_POLLER, color);
  fill_solid(ledsArena_, hardware::LEDS_ARENA, color);
  dirty_ = true;
}

void LedController::updateCountdown(uint32_t nowMs) {
  static const uint16_t phaseDurations[] = {500, 500, 500, 500, 500, 500, 2000};

  if (phaseStepMs_ == 0) {
    phaseStepMs_ = nowMs;
    dirty_ = true;
  }

  if (phase_ >= static_cast<uint8_t>(sizeof(phaseDurations))) {
    stopAnimation();
    return;
  }

  if (nowMs - phaseStepMs_ >= phaseDurations[phase_]) {
    ++phase_;
    phaseStepMs_ = nowMs;
    dirty_ = true;
  }

  if (!dirty_) {
    return;
  }

  if (phase_ < 6) {
    const bool on = (phase_ % 2) == 0;
    if (on) {
      fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, CRGB::Orange);
      fill_solid(ledsPoller_, hardware::LEDS_POLLER, CRGB::Orange);
      fill_solid(ledsArena_, hardware::LEDS_ARENA, CRGB::Orange);
    } else {
      resetStrips(CRGB::Black);
      return;
    }
  } else if (phase_ == 6) {
    fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, CRGB::Green);
    fill_solid(ledsPoller_, hardware::LEDS_POLLER, CRGB::Green);
    fill_solid(ledsArena_, hardware::LEDS_ARENA, CRGB::Green);
  }
}

void LedController::updateOverrun(uint32_t nowMs) {
  if (phase_ == 0) {
    if (phaseStepMs_ == 0) {
      phaseStepMs_ = nowMs;
      phaseStartMs_ = nowMs;
    }

    if (nowMs - phaseStepMs_ >= 50) {
      phaseStepMs_ = nowMs;
      fill_solid(ledsPoller_, hardware::LEDS_POLLER, CRGB::Black);

      constexpr int ringSizes[] = {8, 12, 16, 14};
      constexpr int ringStarts[] = {0, 8, 20, 36};
      constexpr int ringCount = sizeof(ringSizes) / sizeof(ringSizes[0]);
      for (int ring = 0; ring < ringCount; ++ring) {
        const int size = ringSizes[ring];
        const int start = ringStarts[ring];
        const int pos = (wavePosition_ + ring) % size;
        ledsPoller_[start + pos] = CRGB::Blue;
      }

      ++wavePosition_;
      dirty_ = true;
    }

    if (nowMs - phaseStartMs_ >= 3000) {
      phase_ = 1;
      phaseStepMs_ = nowMs;
      spinnerPosition_ = 0;
      fill_solid(ledsPoller_, hardware::LEDS_POLLER, CRGB::Red);
      fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, CRGB::Red);
      dirty_ = true;
    }
  } else {
    if (nowMs - phaseStepMs_ >= 80) {
      phaseStepMs_ = nowMs;
      fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, CRGB::Black);

      const int prev = (spinnerPosition_ - 1 + hardware::LEDS_RUNDUM) % hardware::LEDS_RUNDUM;
      const int next = (spinnerPosition_ + 1) % hardware::LEDS_RUNDUM;
      ledsRundum_[prev] = CRGB(64, 0, 0);
      ledsRundum_[spinnerPosition_] = CRGB::Red;
      ledsRundum_[next] = CRGB(64, 0, 0);

      spinnerPosition_ = (spinnerPosition_ + 1) % hardware::LEDS_RUNDUM;
      dirty_ = true;
    }
  }
}

void LedController::updateStop(uint32_t nowMs) {
  if (phaseStepMs_ == 0) {
    phaseStepMs_ = nowMs;
    dirty_ = true;
  }

  if (phase_ >= 12) {
    stopAnimation();
    return;
  }

  if (nowMs - phaseStepMs_ >= 150) {
    phaseStepMs_ = nowMs;
    ++phase_;
    dirty_ = true;
  }

  if (!dirty_) {
    return;
  }

  if ((phase_ & 0x1) == 0) {
    fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, CRGB::Red);
    fill_solid(ledsPoller_, hardware::LEDS_POLLER, CRGB::Red);
    fill_solid(ledsArena_, hardware::LEDS_ARENA, CRGB::Red);
  } else {
    resetStrips(CRGB::Black);
    dirty_ = true;
  }
}

void LedController::updateControl(uint32_t nowMs) {
  if (nowMs - phaseStepMs_ < 50) {
    return;
  }
  phaseStepMs_ = nowMs;
  dirty_ = true;

  fill_solid(ledsRundum_, hardware::LEDS_RUNDUM, CRGB(0, 0, 32));
  fill_solid(ledsPoller_, hardware::LEDS_POLLER, CRGB(0, 0, 64));

  for (uint16_t i = 0; i < hardware::LEDS_ARENA; ++i) {
    const uint8_t hue = static_cast<uint8_t>((spinnerPosition_ + i) & 0xFF);
    ledsArena_[i] = CHSV(hue, 200, 128);
  }

  spinnerPosition_ = (spinnerPosition_ + 3) & 0xFF;
}

void LedController::applyShow() {
  if (!dirty_) {
    return;
  }
  FastLED.show();
  dirty_ = false;
}

}  // namespace poller
