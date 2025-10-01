#pragma once

#include <Arduino.h>

namespace hardware {

// Stepper driver pins
constexpr gpio_num_t PIN_POLLER_ENABLE = GPIO_NUM_13;
constexpr gpio_num_t PIN_STEPPER_STEP  = GPIO_NUM_17;
constexpr gpio_num_t PIN_STEPPER_DIR   = GPIO_NUM_16;
constexpr gpio_num_t PIN_ENDSTOP       = GPIO_NUM_14;  // active low
constexpr gpio_num_t PIN_POLLER_SENSOR = GPIO_NUM_36;  // analog capable, used digital

// LED strip pins and sizes
constexpr gpio_num_t PIN_LED_RUNDUM = GPIO_NUM_25;
constexpr uint16_t   LEDS_RUNDUM    = 8;

constexpr gpio_num_t PIN_LED_POLLER = GPIO_NUM_4;
constexpr uint16_t   LEDS_POLLER    = 60;

constexpr gpio_num_t PIN_LED_ARENA  = GPIO_NUM_26;
constexpr uint16_t   LEDS_ARENA     = 300;

// Stepper motion limits
constexpr int32_t POSITION_HOME          = 0;
constexpr int32_t POSITION_UP_TARGET     = 7500;
constexpr int32_t POSITION_DOWN_TARGET   = -7350;
constexpr float   STEPPER_MAX_SPEED      = 45000.0f;
constexpr float   STEPPER_ACCELERATION   = 2000.0f;

// Status publish interval
constexpr uint32_t STATUS_INTERVAL_MS = 250;

}  // namespace hardware
