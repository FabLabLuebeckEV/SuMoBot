#include <Arduino.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
// WLED FastLED
#include <FastLED.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Pin-Definitionen
#define POLLER_EN 13        // Enable Pin for the Poller
#define STEP_PIN 17         // Step-Pin
#define DIR_PIN 16          // Richtungs-Pin
#define ENDSTOP_PIN 14      // Endstop Pin, with external Pullup
#define STEPS_TO_DOWN -7350 // -7300 //-29200
#define POSITION_UP 7500
#define POLLER_SESNOR_PIN 36 // Sensor für den Poller
#define STEPPER_SPEED 45000.0f  // Geschwindigkeit des Steppers
#define STEPPER_ACCELERATION 2000 // Beschleunigung des Steppers

// Definition der Pins und LED-Anzahlen
#define WLED_PIN_RUNDUM_LEUCHTE 25
#define NUM_LEDS_RUNDUM_LEUCHTE 8

#define WLED_PIN_POLLER_STATUS 4
#define NUM_LEDS_POLLER_STATUS 60

#define WLED_PIN_ARENA 26
#define NUM_LEDS_ARENA 300

// Schalte WIFI für Debugging ein/aus
#define WIFI_ON_STARTUP false

// REPLACE WITH YOUR RECEIVER MAC Address ac:15:18:e9:7e:78
uint8_t broadcastAddress[] = {0xac, 0x15, 0x18, 0xe9, 0x7e, 0x78};

// Structure example to receive data
typedef struct struct_message {
    char message[32];
} struct_message;

// Create a struct_message called myData
struct_message myData;
struct_message myDataSend;

esp_now_peer_info_t peerInfo;

// LED-Arrays
CRGB leds_rundum[NUM_LEDS_RUNDUM_LEUCHTE];
CRGB leds_poller[NUM_LEDS_POLLER_STATUS];
CRGB leds_arena[NUM_LEDS_ARENA];

// MAC-Adresse des eigenen Geräts
static uint8_t SELF_MAC[6];

// Globale Variable zur Steuerung der Animationen
// ENUM LED_ANIMATION
enum LED_ANIMATION {
    KEINE_ANIMATION,
    COUNTDOWN_ANIMATION,
    POLLER_UEBERFAHRUNG_ANIMATION,
    ARENA_STOP_ANIMATION,
    ARENA_CONTROL_ANIMATION
};
volatile LED_ANIMATION animationToRun = KEINE_ANIMATION;

// Task-Handles
TaskHandle_t TaskLEDAnimation;
TaskHandle_t TaskArenaControl;

// AccelStepper-Objekt
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// WLAN-Credentials
const char *ssid = "fablab";
const char *password = "fablabfdm";

// Erstelle einen Webserver auf Port 80
AsyncWebServer server(80);

// Aktuelle Geschwindigkeit
int currentSpeed = 0;

// Zeitsteuerung für Poller Überfahrung
unsigned long pollerStartTime = 0;
bool pollerTriggered = false;

// Interrupt-Flagge
volatile bool pollerUpFlag = false;

// Letzte Link-Qualität zum zuletzt gesehenen Peer
static uint8_t lastPeer[6] = {0};
static int8_t  lastRSSI = -127;
static float   emaRSSI  = NAN;    // gleitender Mittelwert
static uint32_t lastSeenMs = 0;

// ISR: Setzt nur die Flagge
void IRAM_ATTR pollerUp() {
    pollerUpFlag = true;
}

// Start Match
void startMatch() {
    // Starte die Animation
    animationToRun = COUNTDOWN_ANIMATION;
    stepper.moveTo(POSITION_UP);
    Serial.println("Match gestartet");
}

// Start DathMatch
void startDMatch() {
    // Starte die Animation
    animationToRun = COUNTDOWN_ANIMATION;
    stepper.moveTo(STEPS_TO_DOWN);
    Serial.println("Dath Match gestartet");
}

// Stop Match
void stopMatch() {
    // Starte die Animation
    animationToRun = ARENA_STOP_ANIMATION;
    stepper.moveTo(STEPS_TO_DOWN);
    Serial.println("Match gestoppt");
}

// Hilfsfunktion: MAC hübsch ausgeben
String macToString(const uint8_t m[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           m[0], m[1], m[2], m[3], m[4], m[5]);
  return String(buf);
}

// dBm → grobe Qualitäts-% (0…100). Kein Standard, aber praxisnah.
int rssiToQuality(int8_t rssi) {
  if (rssi <= -100) return 0;
  if (rssi >=  -50) return 100;
  return 2 * (rssi + 100);
}

// Callback function that will be executed when data is received
// Neuer ESP-NOW Receive Callback (Arduino-ESP32 v3 / IDF v5)
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (!info || !incomingData || len <= 0) return;

  // Absender-MAC (optional, falls du sie brauchst)
  const uint8_t *mac = info->src_addr;   // 6 Bytes
  // int recv_channel = info->recv_channel; // optional: Kanal

  // RSSI in dBm aus dem RX-Control-Block
  int8_t rssi = info->rx_ctrl->rssi;
  lastRSSI = rssi;
  if (isnan(emaRSSI)) emaRSSI = rssi;
  else                emaRSSI = 0.2f * rssi + 0.8f * emaRSSI;
  lastSeenMs = millis();

  // Sichere Kopie (begrenzen auf Größe von myData)
  int copyLen = len < (int)sizeof(myData) ? len : (int)sizeof(myData);
  memcpy(&myData, incomingData, copyLen);

  Serial.printf("RX %dB von %s, RSSI %d dBm\n", len, macToString(lastPeer).c_str(), rssi);

  Serial.print("Bytes received: ");
  Serial.println(len);

  Serial.print("From: ");
  for (int i = 0; i < 6; ++i) {
    if (i) Serial.print(':');
    Serial.print(mac[i], HEX);
  }
  Serial.println();

  Serial.print("Message: ");
  Serial.println(myData.message);

  String msg = String(myData.message);
  if (msg == "start") {
    startMatch();
  } else if (msg == "startdm") {
    startDMatch(); // Starte Dath Match
  } else if (msg == "stop") {
    stopMatch();
  } else if (msg == "up") {
    stepper.moveTo(POSITION_UP);
  } else if (msg == "poller") {
    animationToRun = POLLER_UEBERFAHRUNG_ANIMATION;
    pollerStartTime = millis();
    pollerTriggered = true;
  } else if (msg == "down") {
    stepper.moveTo(STEPS_TO_DOWN);
  } else if (msg == "stopM") {
    stepper.stop();
  } else if (msg == "callibrate") {
    stepper.move(5000);
    while (digitalRead(ENDSTOP_PIN) == HIGH) {
      stepper.run();
    }
    stepper.stop();
    stepper.setCurrentPosition(0);
  }
}


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Send data using ESP-NOW
void sendEspNow(const char* data) {
  // Set values to send
  strcpy(myDataSend.message, data);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDataSend, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void runStopAnimation() {
    for (int i = 0; i < 10; i++) {
    // Alle LEDs rot aufleuchten lassen
    fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Red);
    fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Red);
    fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Red);
    FastLED.show();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // LEDs ausschalten
    fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Black);
    fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Black);
    fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Black);
    FastLED.show();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Funktion für die Countdown-Animation
void runCountdownAnimation(bool end) {
  if (!end) {
    for (int i = 0; i < 3; i++) {
      // Alle LEDs rot aufleuchten lassen
      fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Orange);
      fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Orange);
      fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Orange);
      FastLED.show();
      vTaskDelay(500 / portTICK_PERIOD_MS);

      // LEDs ausschalten
      fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Black);
      fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Black);
      fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Black);
      FastLED.show();
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }

  if (end) { 
    for (int i = 0; i < 3; i++) {
      // Alle LEDs rot aufleuchten lassen
      fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Red);
      fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Red);
      fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Red);
      FastLED.show();
      vTaskDelay(500 / portTICK_PERIOD_MS);

      // LEDs ausschalten
      fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Black);
      fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Black);
      fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Black);
      FastLED.show();
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    sendEspNow("matchReady");

    // Alles schlagartig grün für 2 Sekunden
    fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Green);
    fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Green);
    fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Green);
    FastLED.show();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // LEDs ausschalten
    fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Black);
    fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Black);
    fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Black);
    FastLED.show();
  }
}

// Hilfsfunktion für die blaue Wellenanimation
void animateBlueWaves() {
  static int wavePosition = 0;
  fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Black);

  // Simulation der Ringe mit unterschiedlichen LED-Anzahlen
  int ringSizes[] = {8, 12, 16, 14};  // Summe sollte 50 ergeben
  int ringStarts[] = {0, 8, 20, 36};
  int numRings = sizeof(ringSizes) / sizeof(ringSizes[0]);

  for (int ring = 0; ring < numRings; ring++) {
    int start = ringStarts[ring];
    int size = ringSizes[ring];
    int pos = (wavePosition + ring) % size;
    leds_poller[start + pos] = CRGB::Blue;
  }

  FastLED.show();
  wavePosition++;
}

// Hilfsfunktion für das rote Rundumlicht
void animateRotatingRedLight() {
  static int position = 0;

  fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Black);

  int prevPos = (position - 1 + NUM_LEDS_RUNDUM_LEUCHTE) % NUM_LEDS_RUNDUM_LEUCHTE;
  int nextPos = (position + 1) % NUM_LEDS_RUNDUM_LEUCHTE;

  leds_rundum[prevPos] = CRGB::Red / 2;
  leds_rundum[position] = CRGB::Red;
  leds_rundum[nextPos] = CRGB::Red / 2;

  FastLED.show();
  position = (position + 1) % NUM_LEDS_RUNDUM_LEUCHTE;
}

// Funktion für die PollerÜberfahrung-Animation
void runPollerUeberfahrungAnimation() {
  uint32_t startTime = millis();
  bool isRedPhase = false;

  while (millis() - startTime < 3000 || !isRedPhase) {
    if (millis() - startTime < 3000) {
      // Blaue Wellenanimation von außen nach innen
      animateBlueWaves();
    } else {
      if (!isRedPhase) {
        // Wechsel zur Rotphase
        fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Red);
        FastLED.show();
        isRedPhase = true;
      }

      // Rundumlicht in Rot
      animateRotatingRedLight();
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

  // LEDs ausschalten
  fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Black);
  fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Black);
  FastLED.show();
}

// Task für die LED-Animationen
void LEDAnimationTask(void *pvParameters) {
  while (1) {
    switch (animationToRun) {
      case COUNTDOWN_ANIMATION:
        runCountdownAnimation(false);
        if (!stepper.isRunning()) {
          runCountdownAnimation(true);
          animationToRun = KEINE_ANIMATION;  // Reset der Animation
        }
        break;
      case POLLER_UEBERFAHRUNG_ANIMATION:
        runPollerUeberfahrungAnimation();
        if (!stepper.isRunning()) {
          animationToRun = KEINE_ANIMATION;
        }
        break;
      case ARENA_STOP_ANIMATION:
        runStopAnimation();
        if (!stepper.isRunning()) {
          animationToRun = KEINE_ANIMATION;
        }
        break;
      default:
        vTaskDelay(100 / portTICK_PERIOD_MS);
            // LEDs ausschalten
            //fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Black);
            //fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Black);
            //fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Black);
            //FastLED.show();
        break;
    }
  }
}

// Arena-Steuerung Task (hier kannst du weitere Animationen implementieren)
void ArenaControlTask(void *pvParameters) {
    while (true) {
      stepper.run();

      // Überprüfe, ob der Poller oben ist
      if (pollerUpFlag) {
          pollerUpFlag = false;
          stepper.stop();
          stepper.setCurrentPosition(0);
          stepper.moveTo(POSITION_UP);
          Serial.println("Endstop reached (Interrupt)");
      }

      // Steuere den Enable-Pin des Steppers
      if (stepper.currentPosition() == stepper.targetPosition()) {
          digitalWrite(POLLER_EN, HIGH);
      } else {
          digitalWrite(POLLER_EN, LOW);
      }

      // Überprüfe den Endstop
      if (digitalRead(ENDSTOP_PIN) == LOW) {
          stepper.stop();
          stepper.setCurrentPosition(0);
          stepper.move(-100); // Sichere Entfernung vom Endstop
          Serial.println("Endstop reached");
          Serial.println(stepper.maxSpeed());
      }

      // Überprüfe Poller Überfahrung
      if (digitalRead(POLLER_SESNOR_PIN) == LOW && !pollerTriggered && stepper.currentPosition() == stepper.targetPosition()) {
          // Animation auslösen und Timer starten
          animationToRun = POLLER_UEBERFAHRUNG_ANIMATION;
          pollerStartTime = millis();
          pollerTriggered = true;
          Serial.println("Poller Überfahrung erkannt");
      }

      // Wenn Poller Überfahrung erkannt wurde, 3 Sekunden warten und dann Stepper bewegen
      if (pollerTriggered && (millis() - pollerStartTime >= 3000)) {
          // Poller hochfahren
          stepper.moveTo(POSITION_UP);
          stepper.run(); // Dies muss in jedem Loop aufgerufen werden, damit der Motor sich bewegt
          pollerTriggered = false; // Rücksetzen, um zukünftige Aktionen zu ermöglichen
          Serial.println("Poller hoch, da überfahren");
      }
      delay(1);
    }
}

void loop() {
  vTaskDelete(NULL);
  // Periodisch Link-Qualität loggen, falls zuletzt Frames empfangen wurden
}


void setup() {
    pinMode(ENDSTOP_PIN, INPUT_PULLUP);
    pinMode(POLLER_SESNOR_PIN, INPUT);
    pinMode(POLLER_EN, OUTPUT);
    digitalWrite(POLLER_EN, LOW);

    // Seriellen Monitor starten
    Serial.begin(115200);
    while (!Serial) {
      delay(10);
    }

    // WLED starten
    FastLED.addLeds<WS2812B, WLED_PIN_RUNDUM_LEUCHTE, RGB>(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE); //,  RGB>(leds_poller, NUM_LEDS_POLLER_STATUS, NUM_LEDS_RUNDUM_LEUCHTE)
    FastLED.addLeds<WS2812B, WLED_PIN_ARENA, RGB>(leds_arena, NUM_LEDS_ARENA);
    FastLED.addLeds<WS2812B, WLED_PIN_POLLER_STATUS, GRB>(leds_poller, NUM_LEDS_POLLER_STATUS);
    // Alle streifen an auf blau
    fill_solid(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE, CRGB::Blue);
    fill_solid(leds_poller, NUM_LEDS_POLLER_STATUS, CRGB::Blue);
    fill_solid(leds_arena, NUM_LEDS_ARENA, CRGB::Blue);
    FastLED.show();

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    //WiFi.disconnect();

    // Eigene MAC (STA) ausgeben
    esp_wifi_get_mac(WIFI_IF_STA, SELF_MAC);
    Serial.printf("ESP-NOW MAC (STA): %s\n", macToString(SELF_MAC).c_str());


    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        ESP.restart();
        return;
    }

    // WLAN starten
    if (WIFI_ON_STARTUP) {
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
          delay(1000);
          Serial.println("Verbindung zum WLAN wird hergestellt...");
      }
      Serial.println("Mit dem WLAN verbunden!");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("WLAN deaktiviert."); 
    }

    // LittleFS starten
    if (!LittleFS.begin()) {
        Serial.println("LittleFS konnte nicht gestartet werden.");
        return;
    }

    // Konfiguration des Steppers
    stepper.setMaxSpeed(STEPPER_SPEED);
    stepper.setAcceleration(STEPPER_ACCELERATION);

    // Interrupt für Poller
    attachInterrupt(digitalPinToInterrupt(ENDSTOP_PIN), pollerUp, FALLING);

    // Routen für Webserver definieren
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/style.css", "text/css");
    });

    // API-Routen
    server.on("/up", HTTP_GET, [](AsyncWebServerRequest *request) {
        stepper.moveTo(POSITION_UP);
        request->send(200, "text/plain", "Motor nach oben");
    });

    server.on("/callibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
        stepper.move(100000);
        request->send(200, "text/plain", "Motor kalibriert");
    });

    server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request) {
        stepper.moveTo(STEPS_TO_DOWN);
        request->send(200, "text/plain", "Motor nach unten");
    });

    server.on("/stopM", HTTP_GET, [](AsyncWebServerRequest *request) {
        stepper.stop();
        request->send(200, "text/plain", "Motor gestoppt");
    });

    server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request) {
        startMatch();
        request->send(200, "text/plain", "Geschwindigkeit gesetzt");
    });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
        stopMatch();
        request->send(200, "text/plain", "Geschwindigkeit gesetzt");
    });

    // Webserver starten
    server.begin();
    // Erstelle Task für LED-Animationen
    // Erstellung der Tasks
  xTaskCreatePinnedToCore(
    LEDAnimationTask,    // Task-Funktion
    "LEDAnimationTask",  // Name des Tasks
    4096,                // Stack-Größe
    NULL,                // Task-Parameter
    1,                   // Priorität
    &TaskLEDAnimation,   // Task-Handle
    0                    // Core-ID
  );

  xTaskCreatePinnedToCore(
    ArenaControlTask,
    "ArenaControlTask",
    4096,
    NULL,
    1,
    &TaskArenaControl,
    1
  );

  stepper.moveTo(POSITION_UP);
}