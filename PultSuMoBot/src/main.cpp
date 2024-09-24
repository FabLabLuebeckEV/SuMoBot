#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

// Pin-Definitionen
#define START_STOP_PIN 13        // Enable Pin for the Poller
// HD44780 LCD über i2c


// Definition für Arena
#define FIGHT_DURATION 180000    // Dauer eines Kampfes in ms

// WLAN-Credentials
const char *ssid = "fablab";
const char *password = "fablabfdm";

// Erstelle einen Webserver auf Port 80
AsyncWebServer server(80);

// Aktuelle Zeit
int currentFightStartTime = 0;

// Start Match
void startMatch() {
    Serial2.println("start");
    currentFightStartTime = millis();
}

// Stop Match
void stopMatch() {
    Serial2.println("stop");
    currentFightStartTime = 0;
}


void loop() {
    if (digitalRead(START_STOP_PIN) == LOW && currentFightStartTime == 0) {
        startMatch();
    } else if (digitalRead(START_STOP_PIN) == LOW && currentFightStartTime != 0 && millis() - currentFightStartTime >= 3000) { // 3 Sekunden delay zwischen start und ende
        stopMatch();
    }
    if (currentFightStartTime != 0 && millis() - currentFightStartTime >= FIGHT_DURATION) {
        stopMatch();
    }
    // Ausgabe in Minuten und Sekunden
    Serial.println((FIGHT_DURATION - (millis() - currentFightStartTime)) / 60000 + ":" + ((FIGHT_DURATION - (millis() - currentFightStartTime)) % 60000) / 1000);
    delay(100);
}


void setup() {
    pinMode(START_STOP_PIN, INPUT_PULLUP);
    
    // Seriellen  Monitor starten
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 9, 10);

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

    // Routen für Webserver definieren
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/style.css", "text/css");
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
}