# Arena Sumo Bot Steuerung

## Projektueberblick
Dieses Projekt steuert die Arena eines Sumo-Robotik-Wettbewerbs mit einem ESP32. Ein Schrittmotor faehrt den Poller hoch und runter, mehrere WS2812-LED-Streifen visualisieren den Arena-Status und ESP-NOW dient als drahtloses Interface zu externen Controllern. Optional kann ein WLAN-Access-Point fuer ein kleines Web-Interface aktiviert werden.

## Hardware und Bibliotheken
- ESP32 Development Board
- Schrittmotor mit Endschalter (`POLLER_EN`, `STEP_PIN`, `DIR_PIN`, `ENDSTOP_PIN`)
- Sensorsignal fuer Poller-Ueberfahrung (`POLLER_SESNOR_PIN`)
- WS2812B LED-Streifen fuer Rundumlicht, Poller-Status und Arena-Beleuchtung
- Bibliotheken: `AccelStepper`, `FastLED`, `esp_now`, `ESPAsyncWebServer`, `LittleFS`, `WiFi`

## Ablauf im Code (`src/main.cpp`)
### Globale Steuerung
- `AccelStepper stepper` verwaltet den Poller, inkl. Zielpositionen `POSITION_UP` und `STEPS_TO_DOWN`.
- Globale LED-Puffer `leds_rundum`, `leds_poller`, `leds_arena` werden gemeinsam ueber `FastLED` aktualisiert.
- `animationToRun` vom Typ `LED_ANIMATION` entscheidet, welche LED-Sequenz die Task ausfuehrt.
- ESP-NOW-Callbacks verarbeiten eingehende Funkbefehle und schicken Rueckmeldungen.

### Zentrale Funktionen
- `IRAM_ATTR pollerUp()`: Interrupt-Routine setzt eine Flagge, sobald der Endstop betaetigt wird.
- `startMatch()`, `startDMatch()`, `stopMatch()`: Steuern Stepper-Zielpositionen und stoßen die passenden LED-Animationen an.
- `macToString()` und `rssiToQuality()`: Helfer zur Darstellung von MAC-Adressen bzw. Signalqualitaet.
- `OnDataRecv(...)`: ESP-NOW Receive Callback; wertet Nachrichten wie `start`, `stop`, `poller`, `callibrate` aus.
- `OnDataSent(...)`: Gibt den Erfolg eines ESP-NOW-Sendeversuchs auf der seriellen Konsole aus.
- `sendEspNow()`: Verpackt Textbefehle in `myDataSend` und verschickt sie an den hinterlegten Peer.
- `runStopAnimation()`, `runCountdownAnimation()`, `runPollerUeberfahrungAnimation()`: Sequenzen fuer Stopp-, Countdown- und Poller-Ueberfahrungs-Szenarien.
- `animateBlueWaves()` und `animateRotatingRedLight()`: Hilfsanimationen fuer Poller-LEDs und Rundumlicht.
- `LEDAnimationTask(void*)`: FreeRTOS-Task, die je nach `animationToRun` die passende Animation abspielt.
- `ArenaControlTask(void*)`: FreeRTOS-Task, die den Stepper zyklisch bewegt, Endstop/Sensor auswertet und LED-Aktionen ausloest.
- `setup()`: Initialisiert Pins, Serielle Ausgabe, LED-Streifen, ESP-NOW, optionales WLAN, LittleFS, Webserver-Routen und startet die Tasks.
- `loop()`: Wird nach Task-Start beendet (`vTaskDelete(NULL)`), da Logik in den Tasks laeuft.

### Web-API (AsyncWebServer)
- `GET /`: liefert `data/index.html` aus dem LittleFS.
- `GET /style.css`: liefert zugehoerige CSS-Datei.
- `GET /up`: setzt die Zielposition `POSITION_UP`.
- `GET /down`: faehrt den Poller auf `STEPS_TO_DOWN`.
- `GET /stopM`: stoppt den Schrittmotor.
- `GET /callibrate`: laesst den Stepper gegen den Endstop fahren und setzt die Referenzposition.
- `GET /start`: startet Match-Countdown und faehrt anschliessend hoch.
- `GET /stop`: loest Stop-Animation aus und faehrt den Poller herunter.

## Betriebsablauf
1. Flashen Sie das Programm via PlatformIO auf den ESP32.
2. Optional: Setzen Sie `WIFI_ON_STARTUP` auf `true`, um das Web-Interface zu aktivieren.
3. Stellen Sie sicher, dass der ESP-NOW-Peer (MAC `ac:15:18:e9:7e:78`) erreichbar ist oder passen Sie `broadcastAddress` an.
4. Nutzen Sie ESP-NOW oder die HTTP-Endpunkte, um Matches zu starten, zu stoppen oder den Poller zu kalibrieren.
