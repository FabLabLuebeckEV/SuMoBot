#pragma once

#include <FastLED.h>

#include "comms/messages.h"
#include "hardware_config.h"

namespace poller {

class LedController {
 public:
  enum class Mode : uint8_t {
    kInit,
    kIdle,
    kCountdown,
    kMatch,
    kStop
  };

  struct Inputs {
    bool pollerIsUp = false;
    bool pollerIsLowered = true;
    bool pollerMoving = false;
    bool manualControl = false;
    bool overrunArmed = false;
    bool cooldownActive = false;
    bool sensorActive = false;
  };

  void begin();
  void update();
  void applyInputs(const Inputs& inputs);

  void startAnimation(comms::AnimationId id);
  void stopAnimation();
  comms::AnimationId activeAnimation() const;

 private:
  void setMode(Mode mode);
  void render(uint32_t nowMs);
  void renderInit(uint32_t nowMs);
  void renderIdle(uint32_t nowMs);
  void renderCountdown(uint32_t nowMs);
  void renderMatch(uint32_t nowMs);
  void renderStop(uint32_t nowMs);
  void renderPoller(uint32_t nowMs, const CRGB& arenaColor);
  void renderPollerOverrunRaw(uint32_t nowMs);
  void renderPollerArmed(uint32_t nowMs);
  void renderRundum(const CRGB& arenaColor);
  void renderArenaRainbow(uint32_t nowMs);
  void renderArenaSolid(const CRGB& color);
  void renderArenaBlink(uint32_t nowMs, const CRGB& color, uint16_t periodMs);
  void resetStrips(const CRGB& color = CRGB::Black);
  void applyShow();

  Mode mode_ = Mode::kInit;
  Mode previousMode_ = Mode::kInit;
  Inputs inputs_{};
  Inputs lastInputs_{};
  uint32_t modeStartMs_ = 0;
  uint32_t lastUpdateMs_ = 0;
  uint32_t rainbowLastStepMs_ = 0;
  uint8_t rainbowHue_ = 0;
  uint8_t armedRingIndex_ = 0;
  uint32_t lastArmedStepMs_ = 0;
  uint32_t overrunTriggerMs_ = 0;
  uint32_t blinkAnchorMs_ = 0;
  int wavePosition_ = 0;
  uint32_t lastWaveStepMs_ = 0;
  uint8_t spinnerPosition_ = 0;
  bool dirty_ = false;
  bool overrunTriggered_ = false;
  bool countdownBlinkPhase_ = false;

  static constexpr uint8_t kRingCount = 4;
  static constexpr uint8_t kRingSizes[kRingCount] = {8, 12, 16, 24};
  static constexpr uint16_t kRingOffsets[kRingCount] = {0, 8, 20, 36};
  CRGB ledsRundum_[hardware::LEDS_RUNDUM];
  CRGB ledsPoller_[hardware::LEDS_POLLER];
  CRGB ledsArena_[hardware::LEDS_ARENA];
};

}  // namespace poller
