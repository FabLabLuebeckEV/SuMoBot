# Agent Notes

- Branch: `feature/readme-funktionen`
- PlatformIO project hosts zwei Firmware-Targets (`env:poller`, `env:pult`) mit gemeinsamer Modulbasis.
- Struktur:
  - `src/common/` ESP-NOW Hilfen.
  - `src/poller/` Poller-Firmware (Stepper, LEDs, Statuscontroller).
  - `src/pult/` Pult-Firmware (Controller, Match-Orchestrator, LCD, Webserver).
  - `include/` Hardware-/Comms-Header (`hardware_config.h`, `comms/`, `pult/`).
  - `data/` Weboberfläche für das Pult.
- PlatformIO CLI liegt unter `~/.local/bin/pio`; `~/.local/bin` ist im PATH.
- Beide Targets bauen mit `pio run -e poller` bzw. `pio run -e pult` (Arduino-ESP32 IDF5, AccelStepper/FastLED/WebServer).
- Poller fährt beim Start automatisch zur oberen Endlage (Kalibrierung) und ignoriert Abwärtsbefehle, solange keine gültige Referenz vorliegt.
- Bewegungs-/Timing-Parameter (`PollerParameters`) werden im EEPROM persistiert; sie lassen sich per Pult-Weboberfläche (Sektion „Poller Parameter“) ändern und werden via ESP-NOW (`kSetParameter`) an den Poller übertragen.
- `/api/status` liefert die aktuell verwendeten Parameter zurück, damit das UI die Felder befüllen kann.
- Pult sendet alle 5 s ein ESP-NOW `kPing`; der Poller antwortet sofort über den Status-Stream. Das Web-UI zeigt Alter und Zustand des letzten Ping-Pong an.
- Poller-LEDs: Init blau, Idle mit Arena-Regenbogen; Matchphase grün, Stopp rot. Überfahrt animiert den Poller-Ring (außen→innen) und hält Sensor-Events, Rundum-Licht folgt Poller-Position.
