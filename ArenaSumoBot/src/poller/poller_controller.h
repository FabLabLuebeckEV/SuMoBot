#pragma once

#include <stdint.h>

#include "comms/messages.h"
#include "poller/led_controller.h"
#include "poller/stepper_controller.h"

namespace poller {

class PollerController {
 public:
  void begin();
  void loop();

  static void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi);

 private:
  void handleCommand(const comms::PollerCommand& command, int8_t rssi, const uint8_t mac[6]);
  void publishStatus(bool force = false);
  void refreshStatusFlags();

  StepperController stepper_;
  LedController leds_;

  comms::PollerStatus status_{};
  uint32_t lastStatusSentMs_ = 0;
  uint32_t lastPollerSensorChangeMs_ = 0;
  bool pollerSensorLatched_ = false;
  bool hasPeer_ = false;
  uint8_t pultAddress_[6] = {0};
  int8_t lastRssi_ = -127;
  float emaRssi_ = NAN;

  static PollerController* instance_;
};

}  // namespace poller
