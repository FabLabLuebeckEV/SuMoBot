#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

/*hier ist die liste des arena-bedienpults:
16 out Button Led
04 in Button Switch 
21 i2c sda
22 i2c scl
26 Eth RST 
19 Eth miso
18 Eth sck
23 Eth mosi
05 Eth CS 
13 Eth int (W5500)
35 in rs232 RX
33 out rs232 TX
34 in Button Der Poller
36 in Button The Count*/

// Pin-Definitionen
//INputs
#define START_STOP_PIN 4        // Enable Pin for the Poller (in Button Switch )
#define BUTTON_POLLER 34        // Button Der Poller
#define BUTTON_COUNT 36         // Button The Count

// HD44780 LCD über i2c
#define addr 0x27
#define I2C_SDA 21
#define I2C_SCL 22
LiquidCrystal_I2C lcd(addr, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display


// W5500 Ethernet
#define ETH_RST 26
#define ETH_MISO 19
#define ETH_SCK 18
#define ETH_MOSI 23
#define ETH_CS 5
#define ETH_INT 13

// Definition für Arena
#define FIGHT_DURATION 180000    // Dauer eines Kampfes in ms

// WLAN-Credentials
const char *ssid = "fablab";
const char *password = "fablabfdm";

// Erstelle einen Webserver auf Port 80
AsyncWebServer server(80);

// Aktuelle Zeit
long currentFightStartTime = 0;
// Status Buttons Poller und Count
long buttonPollerChanges = 0;
long buttonCountChange = 0;

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

// count down
void countDown() {
    Serial2.println("count");
}

// Poller bewegen
void movePoller() {
    Serial2.println("poller");
}

// Check Buttons
void checkButtons() {
    // Großer Roter Knopf
    if (digitalRead(START_STOP_PIN) == LOW && currentFightStartTime == 0) {
        startMatch();
    } else if (digitalRead(START_STOP_PIN) == LOW && currentFightStartTime != 0 && millis() - currentFightStartTime >= 3000) { // 3 Sekunden delay zwischen start und ende
        stopMatch();
    }
    // Der Poller
    if (digitalRead(BUTTON_POLLER) == LOW && millis() - buttonPollerChanges >= 3000) {
        buttonCountChange = millis();
        movePoller();
    }
    // The Count
    if (digitalRead(BUTTON_COUNT) == LOW && millis() - buttonCountChange >= 3000) {
        buttonPollerChanges = millis();
        countDown();
    }
}

void loop() {
    
    if (currentFightStartTime != 0 && millis() - currentFightStartTime >= FIGHT_DURATION) {
        stopMatch();
    }

    // Ausgabe in Minuten und Sekunden
    Serial.println((FIGHT_DURATION - (millis() - currentFightStartTime)) / 60000 + ":" + ((FIGHT_DURATION - (millis() - currentFightStartTime)) % 60000) / 1000);
    // Ausgabe auf dem display
    lcd.setCursor(0, 0);
    lcd.print("Verbleibende Zeit:");
    lcd.setCursor(0, 1);
    lcd.print((FIGHT_DURATION - (millis() - currentFightStartTime)) / 60000 + ":" + ((FIGHT_DURATION - (millis() - currentFightStartTime)) % 60000) / 1000);
    // Buttons prüfen
    checkButtons();
    delay(100);
}


void setup() {
    pinMode(START_STOP_PIN, INPUT_PULLUP);
    // initialize LCD
    lcd.init();
    // turn on LCD backlight                      
    lcd.backlight();
    
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