#include <Arduino.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
// WLED FastLED
#include <FastLED.h>

// Pin-Definitionen
#define POLLER_EN 13        // Enable Pin for the Poller
#define STEP_PIN 17         // Step-Pin
#define DIR_PIN 16          // Richtungs-Pin
#define ENDSTOP_PIN 14      // Endstop Pin, with external Pullup
#define POLLER_SESNOR_PIN 33       // Poller-Sensor-Pin
#define STEPS_TO_DOWN -25000
#define POSITION_UP -100
#define POLLER_SENSOR_PIN 36 // Sensor für den Poller
#define STEPPER_SPEED 25000  // Geschwindigkeit des Steppers
#define STEPPER_ACCELERATION 2000 // Beschleunigung des Steppers

// Definition der Pins und LED-Anzahlen
#define WLED_PIN_RUNDUM_LEUCHTE 25
#define NUM_LEDS_RUNDUM_LEUCHTE 8

#define WLED_PIN_POLLER_STATUS 27
#define NUM_LEDS_POLLER_STATUS 50

#define WLED_PIN_ARENA 26
#define NUM_LEDS_ARENA 150

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
}

// Stop Match
void stopMatch() {
    // Starte die Animation
    animationToRun = ARENA_STOP_ANIMATION;
    stepper.moveTo(STEPS_TO_DOWN);
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
void runCountdownAnimation() {
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
        runCountdownAnimation();
        animationToRun = KEINE_ANIMATION;  // Reset der Animation
        break;
      case POLLER_UEBERFAHRUNG_ANIMATION:
        runPollerUeberfahrungAnimation();
        animationToRun = KEINE_ANIMATION;
        break;
      case ARENA_STOP_ANIMATION:
        runStopAnimation();
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
        stepper.moveTo(-100);
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
    }

    // Überprüfe Poller Überfahrung
    if (digitalRead(POLLER_SESNOR_PIN) == LOW && !pollerTriggered && stepper.currentPosition() == stepper.targetPosition()) {
        // Animation auslösen und Timer starten
        animationToRun = POLLER_UEBERFAHRUNG_ANIMATION;
        pollerStartTime = millis();
        pollerTriggered = true;
    }

    // Wenn Poller Überfahrung erkannt wurde, 3 Sekunden warten und dann Stepper bewegen
    if (pollerTriggered && (millis() - pollerStartTime >= 3000)) {
        // Poller hochfahren
        stepper.moveTo(100);
        stepper.run(); // Dies muss in jedem Loop aufgerufen werden, damit der Motor sich bewegt
        pollerTriggered = false; // Rücksetzen, um zukünftige Aktionen zu ermöglichen
    }

    // Serielle Kommunikation
    if (Serial2.available()) {
        String receivedString = Serial2.readString();
        Serial.println(receivedString);
        if (receivedString == "up") {
            stepper.moveTo(100);
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
    pinMode(POLLER_SENSOR_PIN, INPUT);
    pinMode(POLLER_EN, OUTPUT);
    digitalWrite(POLLER_EN, LOW);
    // Setup Poller Sensor Pin
    pinMode(POLLER_SESNOR_PIN, INPUT);

    // Seriellen Monitor starten
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 9, 10);

    // WLED starten
    FastLED.addLeds<WS2812B, WLED_PIN_RUNDUM_LEUCHTE, GRB>(leds_rundum, NUM_LEDS_RUNDUM_LEUCHTE);
    FastLED.addLeds<WS2812B, WLED_PIN_ARENA, GRB>(leds_arena, NUM_LEDS_ARENA);
    FastLED.addLeds<WS2812B, WLED_PIN_POLLER_STATUS, GRB>(leds_poller, NUM_LEDS_POLLER_STATUS);

    // WLAN starten
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Verbindung zum WLAN wird hergestellt...");
    }
    Serial.println("Mit dem WLAN verbunden!");
    Serial.println(WiFi.localIP());

    // LittleFS starten
    if (!LittleFS.begin()) {
        Serial.println("LittleFS konnte nicht gestartet werden.");
        return;
    }

    // Konfiguration des Steppers
    stepper.setMaxSpeed(STEPPER_SPEED);
    stepper.setAcceleration(STEPPER_ACCELERATION);

    // Interrupt für Poller
    attachInterrupt(digitalPinToInterrupt(POLLER_SENSOR_PIN), pollerUp, FALLING);

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

    server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            currentSpeed = request->getParam("value")->value().toInt();
            stepper.setSpeed(currentSpeed);
        }
        request->send(200, "text/plain", "Geschwindigkeit gesetzt");
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
}