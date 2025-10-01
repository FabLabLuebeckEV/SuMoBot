#pragma once

#include <FastLED.h>

#include "comms/messages.h"
#include "hardware_config.h"

namespace poller {

class LedController {
 public:
  void begin();
  void update();

  void startAnimation(comms::AnimationId id);
  void stopAnimation();
  comms::AnimationId activeAnimation() const;

 private:
  void resetStrips(const CRGB& color = CRGB::Black);
  void updateCountdown(uint32_t nowMs);
  void updateOverrun(uint32_t nowMs);
  void updateStop(uint32_t nowMs);
  void updateControl(uint32_t nowMs);
  void applyShow();

  comms::AnimationId active_ = comms::AnimationId::kNone;
  uint32_t phaseStartMs_ = 0;
  uint32_t phaseStepMs_ = 0;
  uint8_t phase_ = 0;
  bool dirty_ = false;
  int wavePosition_ = 0;
  int spinnerPosition_ = 0;

  CRGB ledsRundum_[hardware::LEDS_RUNDUM];
  CRGB ledsPoller_[hardware::LEDS_POLLER];
  CRGB ledsArena_[hardware::LEDS_ARENA];
};

}  // namespace poller
