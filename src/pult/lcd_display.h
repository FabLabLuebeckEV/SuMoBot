#pragma once

#include <LiquidCrystal_I2C.h>

#include "pult/hardware_config.h"
#include "pult/match_orchestrator.h"
#include "pult/pult_controller.h"

namespace pult {

class LcdDisplay {
 public:
  LcdDisplay();

  void begin();
  void update(const MatchOrchestrator& orchestrator, const PultController& controller);

 private:
  void printLine(uint8_t row, const char* text);

  LiquidCrystal_I2C lcd_;
  bool ready_ = false;
  uint32_t lastRenderMs_ = 0;
};

}  // namespace pult
