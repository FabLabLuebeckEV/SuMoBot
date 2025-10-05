#pragma once

#include <Arduino.h>

namespace hardware {

// Stepper driver pins
constexpr gpio_num_t PIN_POLLER_ENABLE = GPIO_NUM_13;
constexpr gpio_num_t PIN_STEPPER_STEP  = GPIO_NUM_17;
constexpr gpio_num_t PIN_STEPPER_DIR   = GPIO_NUM_16;
constexpr gpio_num_t PIN_ENDSTOP       = GPIO_NUM_14;  // active low
constexpr gpio_num_t PIN_POLLER_SENSOR = GPIO_NUM_36;  // digital sensor input

// LED strip pins and sizes
constexpr gpio_num_t PIN_LED_RUNDUM = GPIO_NUM_25;
constexpr uint16_t   LEDS_RUNDUM    = 8;

constexpr gpio_num_t PIN_LED_POLLER = GPIO_NUM_4;
constexpr uint16_t   LEDS_POLLER    = 60;

constexpr gpio_num_t PIN_LED_ARENA  = GPIO_NUM_26;
constexpr uint16_t   LEDS_ARENA     = 300;

struct PollerParameters {
  int32_t positionHome = 0;
  int32_t positionUpTarget = 7500;
  int32_t positionDownTarget = -7350;
  int32_t downArmMargin = 400;
  float   stepperMaxSpeed = 45000.0f;
  float   stepperAcceleration = 2000.0f;
  uint32_t statusIntervalMs = 250;
  uint32_t overrunCooldownMs = 10000;
};

constexpr PollerParameters DEFAULT_POLLER_PARAMETERS{};

constexpr int32_t POLLER_POSITION_MIN = DEFAULT_POLLER_PARAMETERS.positionDownTarget - 500;
constexpr int32_t POLLER_POSITION_MAX = DEFAULT_POLLER_PARAMETERS.positionUpTarget + 500;
constexpr int32_t POLLER_MIN_POSITION_GUARD = 400;
constexpr int32_t POLLER_MIN_ARM_MARGIN = 100;

inline bool isValid(const PollerParameters& params) {
  return params.stepperMaxSpeed > 0.0f && params.stepperAcceleration > 0.0f &&
         params.statusIntervalMs > 0 && params.overrunCooldownMs > 0;
}

inline PollerParameters sanitized(const PollerParameters& params) {
  PollerParameters result = params;

  auto clamp = [](int32_t value, int32_t low, int32_t high) {
    if (value < low) {
      return low;
    }
    if (value > high) {
      return high;
    }
    return value;
  };

  if (result.stepperMaxSpeed <= 0.0f) {
    result.stepperMaxSpeed = DEFAULT_POLLER_PARAMETERS.stepperMaxSpeed;
  }
  if (result.stepperAcceleration <= 0.0f) {
    result.stepperAcceleration = DEFAULT_POLLER_PARAMETERS.stepperAcceleration;
  }
  if (result.statusIntervalMs == 0) {
    result.statusIntervalMs = DEFAULT_POLLER_PARAMETERS.statusIntervalMs;
  }
  if (result.overrunCooldownMs == 0) {
    result.overrunCooldownMs = DEFAULT_POLLER_PARAMETERS.overrunCooldownMs;
  }
  if (result.downArmMargin < 0) {
    result.downArmMargin = DEFAULT_POLLER_PARAMETERS.downArmMargin;
  }

  result.positionHome = clamp(result.positionHome, POLLER_POSITION_MIN + POLLER_MIN_POSITION_GUARD,
                              POLLER_POSITION_MAX - POLLER_MIN_POSITION_GUARD);
  result.positionUpTarget = clamp(result.positionUpTarget, result.positionHome + POLLER_MIN_POSITION_GUARD,
                                  POLLER_POSITION_MAX);
  result.positionDownTarget = clamp(result.positionDownTarget, POLLER_POSITION_MIN,
                                    result.positionHome - POLLER_MIN_POSITION_GUARD);

  const bool positionsValid = result.positionDownTarget < result.positionHome &&
                              result.positionHome < result.positionUpTarget;
  if (!positionsValid) {
    result.positionHome = DEFAULT_POLLER_PARAMETERS.positionHome;
    result.positionUpTarget = DEFAULT_POLLER_PARAMETERS.positionUpTarget;
    result.positionDownTarget = DEFAULT_POLLER_PARAMETERS.positionDownTarget;
  }

  const int32_t downRange = result.positionHome - result.positionDownTarget;
  const int32_t maxMargin = (downRange > POLLER_MIN_POSITION_GUARD)
                                ? (downRange - POLLER_MIN_POSITION_GUARD)
                                : POLLER_MIN_ARM_MARGIN;
  if (result.downArmMargin < POLLER_MIN_ARM_MARGIN) {
    result.downArmMargin = POLLER_MIN_ARM_MARGIN;
  }
  if (result.downArmMargin > maxMargin) {
    result.downArmMargin = maxMargin;
  }

  return result;
}

}  // namespace hardware
