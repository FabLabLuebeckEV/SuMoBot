#include "lcd_display.h"

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

namespace pult {

LcdDisplay::LcdDisplay()
    : lcd_(pult_hw::LCD_ADDRESS, pult_hw::LCD_COLUMNS, pult_hw::LCD_ROWS) {}

void LcdDisplay::begin() {
  Wire.begin(static_cast<int>(pult_hw::PIN_I2C_SDA), static_cast<int>(pult_hw::PIN_I2C_SCL));

  lcd_.init();
  lcd_.backlight();
  lcd_.clear();
  printLine(0, "Verbleibende Zeit:");
  ready_ = true;
}

void LcdDisplay::update(const MatchOrchestrator& orchestrator, const PultController& controller) {
  if (!ready_) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastRenderMs_ < 200) {
    return;
  }
  lastRenderMs_ = now;

  char line[32];

  // Line 1: time / countdown status
  if (orchestrator.matchRunning()) {
    if (orchestrator.matchType() == MatchOrchestrator::MatchType::kDeath) {
      const uint32_t elapsed = orchestrator.matchElapsedMs();
      const uint32_t minutes = elapsed / 60000;
      const uint32_t seconds = (elapsed % 60000) / 1000;
      snprintf(line, sizeof(line), "Death %02lu:%02lu", static_cast<unsigned long>(minutes), static_cast<unsigned long>(seconds));
    } else {
      const uint32_t remaining = orchestrator.remainingMatchTimeMs();
      const uint32_t minutes = remaining / 60000;
      const uint32_t seconds = (remaining % 60000) / 1000;
      snprintf(line, sizeof(line), "%02lu:%02lu", static_cast<unsigned long>(minutes), static_cast<unsigned long>(seconds));
    }
  } else if (orchestrator.countdownActive()) {
    const uint32_t remaining = orchestrator.countdownRemainingMs();
    const uint32_t seconds = (remaining + 999) / 1000;
    snprintf(line, sizeof(line), "Countdown %lus", static_cast<unsigned long>(seconds));
  } else {
    snprintf(line, sizeof(line), "Bereit");
  }
  printLine(1, line);

  // Line 2: last action text
  snprintf(line, sizeof(line), "Aktion: %s", orchestrator.lastAction());
  printLine(2, line);

  // Line 3: link status
  if (controller.hasStatus()) {
    const comms::PollerStatus& status = controller.status();
    snprintf(line, sizeof(line), "RSSI %d/%d dBm", status.lastRssi, status.emaRssi);
  } else {
    snprintf(line, sizeof(line), "Warte auf Poller");
  }
  printLine(3, line);
}

void LcdDisplay::printLine(uint8_t row, const char* text) {
  char buffer[pult_hw::LCD_COLUMNS + 1];
  memset(buffer, ' ', sizeof(buffer));
  buffer[pult_hw::LCD_COLUMNS] = '\0';

  if (text) {
    strncpy(buffer, text, pult_hw::LCD_COLUMNS);
  }

  lcd_.setCursor(0, row);
  lcd_.print(F("                    "));
  lcd_.setCursor(0, row);
  lcd_.print(buffer);
}

}  // namespace pult
