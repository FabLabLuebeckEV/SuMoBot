# Arena Sumo Bot Steuerung

## Überblick
Dieses Repository bündelt die Firmware für Arena-Poller und Bedienpult des SuMoBot-Projekts in einem gemeinsamen PlatformIO-Projekt. Beide Firmware-Ziele teilen sich Hardware- und Kommunikations-Module, kommunizieren per ESP-NOW und nutzen einen ESP32 als Plattform.

- **Poller (env:poller)** – verwaltet Stepper, Endschalter, Poller-Sensor und LED-Animationen direkt in der Arena. Im Fokus stehen deterministische Bewegungen, saubere Statusmeldungen und ein schlanker ESP-NOW-Befehlsempfänger.
- **Pult (env:pult)** – übernimmt die Match-Orchestrierung. Es liest Bedientaster, steuert ein I²C-LCD, verwaltet den ESP-NOW-Link, stellt eine Weboberfläche (LittleFS) bereit und gibt Observer-Meldungen weiter.

## Verzeichnisstruktur
```
├── data/                 # Weboberfläche für das Pult (LittleFS)
├── include/
│   ├── hardware_config.h # Pin-/Timing-Definitionen Poller
│   ├── network_config.h  # WLAN-Zugangsdaten Pult
│   ├── comms/            # ESP-NOW Nachrichten & Peers
│   └── pult/             # Pult-spezifische Header (LCD, Buttons)
├── src/
│   ├── common/           # ESP-NOW Infrastruktur
│   ├── poller/           # Poller-Controller, Stepper & LEDs
│   └── pult/             # Pult-Controller, Match-Orchestrator, Webserver
└── platformio.ini        # Zwei Build-Targets (poller/pult)
```

## Bauen & Flashen
1. Stelle sicher, dass `pio` (PlatformIO) auf dem Pfad liegt (`~/.local/bin/pio`).
2. Poller-Firmware bauen/flashen:
   ```bash
   pio run -e poller
   pio run -e poller -t upload
   ```
3. Pult-Firmware bauen/flashen:
   ```bash
   pio run -e pult
   pio run -e pult -t upload
   ```

## Kommunikation
- **ESP-NOW**: Nachrichtenstruktur steht in `include/comms/messages.h`. Das Pult sendet `PollerCommand`, der Poller antwortet mit `PollerStatus` (inkl. RSSI, Flags, Positionen).
- **Webserver (Pult)**: `/api/command` nimmt Steuerbefehle (POST/GET), `/api/status` liefert aktuelle Poller-/Match-Daten. Statische Dateien kommen aus `data/` (LittleFS).
- **Observer-Link**: Optionaler MAC (`OBSERVER_MAC`) für Info-Panels.

## Hardware-Hinweise
- **Poller**: Poller-Endschalter auf `GPIO 14` (LOW aktiv), Stepper-Pins `GPIO 16/17`, Enable `GPIO 13`, Sensor `GPIO 36`. LED-Streifen an `GPIO 25/4/26`.
- **Pult**: Taster (`GPIO 4`, `39`, `34`), I²C-LCD (`SDA 21`, `SCL 22`), optional Ethernet (W5500) vorbereitet.

## Weitere Infos
- `include/hardware_config.h` & `include/pult/hardware_config.h` sind zentrale Anlaufstellen für Pinmaps und Timing.
- WLAN-Zugangsdaten für das Pult liegen in `include/network_config.h` (Standard: `fablab` / `fablabfdm`).
- Beide Targets laufen auf Arduino-ESP32 (IDF5); die Poller-LEDs nutzen FastLED 3.10, das Pult setzt auf ESPAsyncWebServer 3.6.
