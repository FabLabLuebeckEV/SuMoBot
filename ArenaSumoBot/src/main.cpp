#include <Arduino.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
// WLED FastLED
#include <FastLED.h>
#include <esp_now.h>

/*27    Out     5    UART TX
32    In    5    UART RX*/
#define RX2 32
#define TX2 27

// Pin-Definitionen
#define POLLER_EN 13        // Enable Pin for the Poller
#define STEP_PIN 17         // Step-Pin
#define DIR_PIN 16          // Richtungs-Pin
#define ENDSTOP_PIN 14      // Endstop Pin, with external Pullup
#define STEPS_TO_DOWN -7200 // -7300 //-29200
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

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xec, 0x64, 0xc9, 0x90, 0xf9, 0x54};

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

// Stop Match
void stopMatch() {
    // Starte die Animation
    animationToRun = ARENA_STOP_ANIMATION;
    stepper.moveTo(STEPS_TO_DOWN);
    Serial.println("Match gestoppt");
}

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Message: ");
  Serial.println(myData.message);
  String msg = String(myData.message);
  if (msg == "start") {
    startMatch();
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
          Serial.println("Poller hich, da überfahren");
      }

      // Serielle Kommunikation
      if (Serial2.available()) {
          Serial.println("Available");
          String receivedString = Serial2.readString();
          Serial.println(receivedString);
          if (receivedString == "up") {
              stepper.moveTo(POSITION_UP);
          } else if (receivedString == "down") {
              stepper.moveTo(STEPS_TO_DOWN);
          } else if (receivedString == "stopM") {
              stepper.stop();
          } else if (receivedString.startsWith("speed")) {
              currentSpeed = receivedString.substring(6).toInt();
              stepper.setSpeed(currentSpeed);
          } else if (receivedString == "callibrate") {
              stepper.move(5000);
              while (digitalRead(ENDSTOP_PIN) == HIGH) {
                  stepper.run();
              }
              stepper.stop();
              stepper.setCurrentPosition(0);
          } else if (receivedString == "start") {
              startMatch();
          } else if (receivedString == "stop") {
              stopMatch();
          }
      }
      delay(1);
    }
}

void loop() {
  vTaskDelete(NULL);
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
    
    Serial2.begin(9600, SERIAL_8N1, RX2, TX2);

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
    //WiFi.disconnect();

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
    //WiFi.begin(ssid, password);
    /*while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Verbindung zum WLAN wird hergestellt...");
    }*/
    //Serial.println("Mit dem WLAN verbunden!");
    //Serial.println(WiFi.localIP());

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
    Serial2.println("Verbunden");
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