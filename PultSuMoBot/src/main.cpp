#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>

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

// Pin-Definitionen Serial
#define RXD2 16
#define TXD2 17

// Pin-Definitionen
//INputs
#define START_STOP_PIN 4        // Enable Pin for the Poller (in Button Switch )
#define BUTTON_POLLER 39        // Button Der Poller
#define BUTTON_COUNT 34         // Button The Count

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

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x34, 0x86, 0x5d, 0xfb, 0xe7, 0xe8};
uint8_t broadcastAddressObs[] = {0x08, 0x3a, 0xf2, 0x37, 0x3c, 0xfc};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char message[32];
} struct_message;

// Create a struct_message called myData
struct_message myData;
struct_message myDataRecv;
struct_message myDataObsSend;

esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfoObs;

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

bool initMatch = false;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Send data using ESP-NOW
void sendEspNow(const char* data) {
  // Set values to send
  strcpy(myData.message, data);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

// Send data using ESP-NOW
void sendEspNowObs(const char* data) {
  // Set values to send
  strcpy(myDataObsSend.message, data);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddressObs, (uint8_t *) &myDataObsSend, sizeof(myDataObsSend));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myDataRecv, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Message: ");
  Serial.println(myDataRecv.message);
  String msg = String(myDataRecv.message);
  if (msg == "matchReady") {
    Serial.println("Match ready");
    currentFightStartTime = millis();
    initMatch = false;
    sendEspNowObs("start");
  } else if (msg == "matchCountdown") {
    Serial.println("Match countdown");
    sendEspNowObs("countdown");
    // TODO implement
  }
}

// Init ESP-NOW
void initESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

// init W5500
void initW5500() {
    pinMode(ETH_RST, OUTPUT);
    digitalWrite(ETH_RST, LOW);
    delay(100);
    digitalWrite(ETH_RST, HIGH);
    delay(100);
}

// Start Match
void startMatch() {
    Serial2.println("start");
    Serial.println("start");
    lcd.setCursor(0, 2);
    lcd.print("start");
    initMatch = true;
    sendEspNow("start");
}

// Stop Match
void stopMatch() {
    Serial2.println("stop");
    Serial.println("stop");
    lcd.setCursor(0, 2);
    lcd.print("stop");
    currentFightStartTime = 0;
    initMatch = false;
    sendEspNow("stop");
    sendEspNowObs("stop");
}

// count down
void countDown() {
    lcd.setCursor(0, 2);
    lcd.print("count");
    Serial2.println("count");
    Serial.println("count");
    sendEspNow("count");
    sendEspNow("down");
}

// Poller bewegen
void movePoller() {
    lcd.setCursor(0, 2);
    lcd.print("poller");
    Serial2.println("poller");
    Serial.println("poller");
    sendEspNow("poller");
}

// Check Buttons
void checkButtons() {
    // Großer Roter Knopf
    if (digitalRead(START_STOP_PIN) == LOW && currentFightStartTime == 0 && !initMatch && millis() - buttonPollerChanges >= 1000) {// entprellen
        startMatch();
        delay(100);
        buttonCountChange = millis();
    } else if (digitalRead(START_STOP_PIN) == LOW && ((currentFightStartTime != 0 && millis() - currentFightStartTime >= 3000) || initMatch) && millis() - buttonPollerChanges >= 1000) { // 3 Sekunden delay zwischen start und ende
        stopMatch();
        buttonCountChange = millis();
    }
    // Der Poller
    if (digitalRead(BUTTON_POLLER) == LOW && millis() - buttonPollerChanges >= 3000) {
        buttonPollerChanges = millis();
        movePoller();
    }
    // The Count
    if (digitalRead(BUTTON_COUNT) == LOW && millis() - buttonCountChange >= 3000) {
        buttonCountChange = millis();
        countDown();
    }
}

void loop() {
    if (currentFightStartTime != 0 && millis() - currentFightStartTime >= FIGHT_DURATION) {
        stopMatch();
    }

    // Ausgabe in Minuten und Sekunden
    Serial.print((FIGHT_DURATION - (millis() - currentFightStartTime)) / 60000);
    Serial.print(":");
    Serial.println(((FIGHT_DURATION - (millis() - currentFightStartTime)) % 60000) / 1000);
    // Ausgabe auf dem display
    lcd.setCursor(0, 0);
    lcd.print("Verbleibende Zeit:");
    lcd.setCursor(0, 1);
    if (currentFightStartTime == 0) {
        lcd.print("00:00");
    } else {
        // clear the line
        lcd.print("                ");
        lcd.setCursor(0, 1);
    lcd.print((FIGHT_DURATION - (millis() - currentFightStartTime)) / 60000);
    lcd.print(":");
    lcd.print(((FIGHT_DURATION - (millis() - currentFightStartTime)) % 60000) / 1000);
    }
    // Buttons prüfen
    checkButtons();
    delay(100);
}

void setup() {
    pinMode(START_STOP_PIN, INPUT);
    pinMode(BUTTON_POLLER, INPUT);
    pinMode(BUTTON_COUNT, INPUT);
    // initialize LCD
    lcd.init();
    // turn on LCD backlight                      
    lcd.backlight();
    
    // Seriellen  Monitor starten
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    initESPNow();

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
    //WiFI.begin(ssid, password);
    /*while (//WiFI.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Verbindung zum WLAN wird hergestellt...");
    }*/
    Serial.println("Mit dem WLAN verbunden!");
    //Serial.println(//WiFI.localIP());

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