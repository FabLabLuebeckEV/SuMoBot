#pragma once

#include <Arduino.h>

namespace pult_hw {

// Digital inputs (active LOW push buttons)
constexpr gpio_num_t PIN_BUTTON_START_STOP = GPIO_NUM_4;
constexpr gpio_num_t PIN_BUTTON_POLLER     = GPIO_NUM_39;
constexpr gpio_num_t PIN_BUTTON_COUNTDOWN  = GPIO_NUM_34;

// I2C LCD
constexpr uint8_t LCD_ADDRESS = 0x27;
constexpr int LCD_COLUMNS = 20;
constexpr int LCD_ROWS = 4;
constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_21;
constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_22;

// Match timing
constexpr uint32_t MATCH_DURATION_MS = 180000;  // 3 minutes
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
constexpr uint32_t BUTTON_REPEAT_MS = 1000;

}  // namespace pult_hw
