# SuMoBot – Firmware Übersicht

Dieses Repository bündelt alle Firmware-Komponenten des SuMoBot-Arena-Projekts.

- `ArenaSumoBot/` – gemeinsames PlatformIO-Projekt mit zwei Targets:
  - `env:poller`: Firmware für den Arena-Poller (Stepper, LEDs, Sensoren, ESP-NOW).
  - `env:pult`: Firmware für das Bedienpult (Buttons, LCD, Webserver, ESP-NOW-Orchestrierung).
- `PultSuMoBot/` – älteres, ausgemustertes Pult-Projekt (nur Referenz; Logik wurde in das gemeinsame Projekt migriert).

Zum Bauen/Flashen siehe `ArenaSumoBot/README.md`. PlatformIO (`pio`) ist im Home-Verzeichnis unter `~/.local/bin` installiert.
