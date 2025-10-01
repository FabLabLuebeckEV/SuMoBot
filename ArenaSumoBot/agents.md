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
