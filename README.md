# SuMoBot – Firmware Übersicht

Dieses Repository bündelt die Firmware-Komponenten des SuMoBot-Arena-Projekts in einem gemeinsamen PlatformIO-Setup. Es existieren zwei buildbare Targets, die sich Module und Konfigurationen teilen:

- `env:poller` – Steuerung des Arena-Pollers (Stepper, Sensoren, LEDs, ESP-NOW-Empfang)
- `env:pult` – Bedienpult inklusive Match-Orchestrierung, LCD, ESP-NOW und Weboberfläche

## Projektaufbau
```
├── agents.md             # Aufgaben-/Teamnotizen
├── data/                 # LittleFS-Dateien für das Pult (Web-Oberfläche)
├── include/              # Gemeinsame Header: Hardware-, Netzwerk- und Comms-Definitionen
├── lib/                  # Platz für begleitende Bibliotheken (leer, README)
├── platformio.ini        # Build-Konfiguration für Poller und Pult
├── src/
│   ├── common/           # ESP-NOW Basis & geteilte Module
│   ├── poller/           # Poller-spezifische Firmware
│   └── pult/             # Pult-spezifische Firmware
├── test/                 # PlatformIO-Tests (noch leer)
├── .vscode/              # Editor-Empfehlungen
└── legacy/
    └── pult/             # Altes Einzelprojekt (historische Referenz)
```

Weitere technische Details zu den Modulen und Hardware-Pins findest du in `README.arena.md`.

## Laufzeitverhalten (Kurzüberblick)
- **Kalibrierung:** Nach dem Boot fährt der Poller automatisch gegen den oberen Anschlag und akzeptiert erst danach Abwärts-Kommandos.
- **Ping-Pong-Monitoring:** Das Pult sendet alle 5 s einen ESP-NOW-`kPing`; der Poller bestätigt implizit im Status. `/api/status` liefert unter `ping` Alter & Health.
- **Konfigurierbare Parameter:** Bewegungs-/Timing-Werte (`PollerParameters`) liegen im EEPROM und lassen sich im Web-UI des Pults anpassen (Endpoint `/api/command?type=updateConfig`).
- **LED-Orchestrierung:**
  - Eigenes FreeRTOS-Task (Core 0, ~125 Hz) steuert Arena-/Poller-Licht, `FastLED.show()` wird throttled (≥16 ms Abstand), damit Stepper-Takte nicht geblockt werden.
  - **Init:** Komplettes System blau (~2 s).
  - **Idle:** Arena zeigt Regenbogenlauf; Rundum dezent blau, solange Poller nicht unten; Poller-Ringe blau wenn oben, ansonsten aus.
  - **Countdown:** Orange → Rot Blinksequenz über alle Segmente.
  - **Match:** Arena konstant grün; Rundum grün solange Poller oben; Poller-Ringe übernehmen Match-Farbe.
  - **Stop:** Komplettes System blinkt rot.
  - **Überfahrt armed:** Poller-Ringe laufen außen→innen (vier Ringe: 8/12/16/24 LEDs); Sensor-Trigger hält Animation bis Poller wieder oben (danach blau).
  - **Überfahrt ausgelöst:** Standardmäßig wartet das Pult 3000 ms, hebt den Poller, lässt ihn nach gleicher Verzögerung wieder absenken und blockiert erneutes Arming für 20000 ms Cooldown.
- **Manuelle Fahrten:** Spezialeffekte pausieren, Poller leuchtet in Match-Farbe bzw. bleibt aus.

## Status-Webhooks
- Im Pult-Web-UI kann optional eine Basis-URL hinterlegt werden. Ab Werk ist `http://10.124.42.5:5000/` eingetragen; bei jeder Statusänderung sendet das Pult einen HTTP `POST` nach `BASE_URL/queues/queue?name=SuMo:<Aktion>`.
- Sobald eine URL gesetzt ist, werden die folgenden Aktionsnamen verwendet:
  - `MatchStart`
  - `MatchStop`
  - `CountdownStart`
  - `PollerUp`
  - `PollerDown`
  - `PollerOverrun`
- Der POST-Body ist ein leeres JSON-Objekt (`{}`); die Aktion kann über den Query-Parameter ausgelesen werden.

## Bauen & Flashen
1. Stelle sicher, dass `pio` (PlatformIO) über den Pfad erreichbar ist (`~/.local/bin/pio`).
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

Serielle Debug-Ausgaben laufen standardmäßig auf 115200 Baud. Der ESP32-NVS bzw. LittleFS-Inhalt kann mit `pio run -t uploadfs` pro Environment geschrieben werden.

## Legacy-Code
Der frühere Stand des separaten Pult-Projekts liegt nun unter `legacy/pult/`. Die aktiven Targets verwenden ausschließlich die Module aus `src/` und `include/`.

Bei Bedarf lassen sich die Legacy-Quellen weiterhin mit PlatformIO inspizieren; sie werden jedoch nicht mehr gepflegt.
