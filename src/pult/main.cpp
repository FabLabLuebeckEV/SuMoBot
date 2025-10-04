#include <Arduino.h>
#include <WiFi.h>

#include "network_config.h"
#include "pult/lcd_display.h"
#include "pult/match_orchestrator.h"
#include "pult/pult_controller.h"
#include "pult/status_notifier.h"
#include "pult/pult_web_server.h"

namespace {
pult::StatusNotifier notifier;
pult::PultController controller;
pult::MatchOrchestrator orchestrator(controller, notifier);
pult::LcdDisplay lcd;
pult::PultWebServer webServer(controller, orchestrator, notifier);
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(network::WIFI_SSID, network::WIFI_PASSWORD);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection failed");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  connectWifi();

  notifier.begin();
  controller.begin();
  orchestrator.begin();
  lcd.begin();
  webServer.begin();
}

void loop() {
  notifier.loop();
  controller.loop();
  orchestrator.update();
  webServer.loop();
  lcd.update(orchestrator, controller);
  delay(10);
}
