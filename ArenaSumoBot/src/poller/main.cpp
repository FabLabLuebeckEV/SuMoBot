#include <Arduino.h>

#include "poller/poller_controller.h"

namespace {
poller::PollerController controller;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  controller.begin();
}

void loop() {
  controller.loop();
  delay(1);
}
