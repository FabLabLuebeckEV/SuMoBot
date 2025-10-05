# SuMoBot Firmware

Firmware für Arena-Poller und Bedienpult des SuMoBot-Projekts. Beide Geräte basieren auf ESP32, teilen sich eine gemeinsame Codebasis und werden mit PlatformIO gebaut.

- **env:poller** – steuert Stepper, Sensorik und LED-Choreografie in der Arena.
- **env:pult** – orchestriert Matches, stellt das Web-UI bereit, spricht das LCD an und verwaltet die Kommunikation.

Die Geräte entdecken sich gegenseitig über UDP-Broadcast (Port `42142`). Status- und Kommandopakete sind in `include/comms/messages.h` definiert. Standardmäßig wird keine feste IP benötigt – optional können in `include/comms/peer_config.h` statische Ziele hinterlegt werden (IPv4 + Port).

## Voraussetzungen

- Python ≥ 3.8
- PlatformIO Core (`pio`) über `pipx`

Empfohlenes Setup unter Ubuntu/WSL:

```bash
sudo apt update
sudo apt install python3 python3-venv python3-pip pipx
pipx ensurepath    # einmalig, danach neues Terminal
pipx install platformio
```

`pio --version` sollte anschließend funktionieren.

## Repository-Struktur

```
├── agents.md          # aktuelle Projekt-/Laufzeitnotizen
├── data/              # LittleFS-Inhalte für das Pult-Web-UI
├── include/           # geteilte Header (Hardware, Netzwerk, Protokolle)
├── lib/               # Platz für zusätzliche Bibliotheken (leer)
├── platformio.ini     # Build-Konfiguration für poller/pult
├── src/
│   ├── common/        # UDP-Link, Utility-Code, gemeinsame Module
│   ├── poller/        # Poller-Firmware (Stepper, LEDs, Status)
│   └── pult/          # Pult-Firmware (Match-Orchestrator, Webserver, LCD)
├── test/              # Tests (derzeit leer)
└── README.arena.md    # Hardware-/Ablaufdetails zur Arena
```

## Bauen & Flashen

Poller:
```bash
pio run -e poller
pio run -e poller -t upload        # optional: flashen
```

Pult:
```bash
pio run -e pult
pio run -e pult -t upload
pio run -t uploadfs -e pult        # LittleFS/Webinhalte aktualisieren
```

Serielle Ausgaben laufen standardmäßig mit 115200 Baud. Für alternative Upload-Tools kann die erzeugte Firmware unter `.pio/build/<env>/firmware.bin` verwendet werden.

## Wichtige Laufzeitdetails

- **Kalibrierung:** Der Poller führt nach jedem Start ein Homing nach oben aus. Abwärtsbewegungen werden gesperrt, solange keine gültige Referenz vorliegt.
- **Bewegungsgrenzen:** `hardware::sanitized()` klemmt alle Poller-Parameter; zusätzliche Clamp-Logik verhindert, dass der Stepper über `positionDownTarget` hinaus fährt.
- **Kommunikation:** UDP-Statuspakete enthalten aktuelle Parameter, Ping-Informationen und Flags (`kOverrunArmed`, `kCooldownActive`, `kPollerSensorActive`, …). Das Pult sendet alle 5 s Pings und wertet `PollerStatus.lastCommandId` aus.
- **LEDs:** Der Poller rendert LEDs in einem FreeRTOS-Task (Core 0) mit FastLEDs ESP32-I2S-Treiber (`FASTLED_ESP32_I2S`, 4 DMA-Buffers). `FastLED.show()` wird auf ≥16 ms getaktet, um Stepper-Updates nicht zu blockieren.
- **Webhooks:** Das Pult kann Statusereignisse an eine frei wählbare Basis-URL posten (`/queues/queue?name=SuMo:<Action>`). Konfiguration über das Web-UI.

## Konfiguration

- Netzwerkeinstellungen: `include/network_config.h` (SSID/Passwort für das Pult-WLAN).
- UDP-Targets: `include/comms/peer_config.h` (IPv4 + Port, Standard ist Broadcast 255.255.255.255:42142).
- Poller-Parameter: `hardware::PollerParameters` (EEPROM, editierbar über das Web-UI unter „Poller Parameter“).
- LED-/Match-Logik: `src/poller/led_controller.cpp` bzw. `src/pult/match_orchestrator.cpp`.

## Troubleshooting

- **Keine Kommunikation:** Prüfen, ob beide Geräte im selben WLAN sind und Port 42142 nicht gefiltert wird. Bei Bedarf feste IPs in `peer_config.h` eintragen.
- **LED-Flackern:** Sicherstellen, dass beide Targets mit aktuellem I2S-Build geflasht wurden. Bei hoher Last kann `FASTLED_ESP32_I2S_NUM_DMA_BUFFERS` in `platformio.ini` erhöht werden.
- **Parameteränderungen:** Nach Updates der Poller-Parameter Homing erneut auslösen, damit die neue Home-Position übernommen wird.
- **Web-UI veraltet:** `pio run -t uploadfs -e pult` ausführen und das Pult neu starten.

Weitere Detailinfos (z. B. LED-Zustände oder Matchphasen) findest du in `agents.md`.

