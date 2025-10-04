# Agent Notes

## Repository Snapshot
- Branch: `feature/readme-funktionen`
- PlatformIO project mit zwei Targets (`env:poller`, `env:pult`) auf gemeinsamer Codebasis.
- Layout:
  - `src/common/` ESP-NOW-/Utility-Code.
  - `src/poller/` Poller-Firmware (Stepper, LED-Orchestrierung, Status).
  - `src/pult/` Pult-Firmware (MatchOrchestrator, Webserver, LCD).
  - `include/` Hardware- und Protokoll-Header (`hardware_config.h`, `comms/`, `pult/`).
  - `data/` LittleFS-Weboberfläche des Pults.
- PlatformIO CLI liegt unter `~/.local/bin/pio` (Pfad vorhanden).

## Build & Deploy
- Poller bauen: `pio run -e poller`
- Pult bauen: `pio run -e pult`
- Webassets hochladen: `pio run -t uploadfs -e pult`
- Firmware flashen via Standard-PlatformIO-Upload oder externes Tool.

## Runtime Essentials
- Poller kalibriert beim Start automatisch (fährt hoch, setzt HOME) und verweigert Abwärtskommando solange keine gültige Referenz vorliegt.
- Bewegungs-/Timing-Parameter liegen in `hardware::PollerParameters`, werden im EEPROM persistiert und sind über das Pult-Web-UI (Sektion „Poller Parameter“) editierbar; Übertragung via ESP-NOW `kSetParameter`.
- `/api/status` liefert unter `config` die aktuellen Parameter sowie unter `ping` Latenz/Health der Verbindung.
- Pult sendet alle 5 s einen `kPing`; Poller antwortet implizit im Status. UI zeigt Alter/Status.

## LED-Choreografie (Poller)
- Läuft in `LedController` auf eigenem FreeRTOS-Task (Core 0, ~8 ms Zyklus). `FastLED.show()` wird max. alle 16 ms ausgeführt, sodass die Stepper-Zyklen unbeeinflusst bleiben.
- **Init:** Komplette Beleuchtung blau (~2 s), anschließend Übergang zu Idle.
- **Idle:** Arena mit Regenbogenlauf; Rundum schwach blau solange Poller nicht vollständig unten. Poller-Ringe: blau, wenn Poller oben, sonst dunkel.
- **Countdown:** Orange/rote Blinksequenz über alle Segmente bis Countdown-Ende.
- **Match:** Arena bleibt grün; Rundum folgt (grün aktiv, solange Poller nicht unten); Poller-Ringe spiegeln die Arenafarbe.
- **Stop/Timeout:** Gesamtes System blinkt rot, bis wieder Idle erreicht wird.
- **Überfahrt armed:** Poller zeigt Lauflicht außen→innen über die vier Ringe (8/12/16/24 LEDs), solange Überfahrt aktiv und kein Cooldown läuft.
- **Überfahrt ausgelöst:** Sensor-Low triggert Overrun-Animation außen→innen; bleibt aktiv, bis Poller wieder ganz oben und Sensor frei → danach Poller-Ringe blau.
- **Manuelle Fahrten:** Während manueller `Move*`/`Limit`-Kommandos werden Spezialanimationen ausgesetzt, Poller zeigt Match-Farbe bzw. bleibt aus; nach Bewegung kehrt passender Zustand zurück.

## Kommunikation / Matchsteuerung
- `MatchOrchestrator` bedient Countdown, Match-Start, Poller-Überfahrt (automatisches Hochfahren, Animation trigger).
- PollerCommander validiert Kommandos gegen Kalibrier-/Cooldown-Status; `kMove*` Kommandos setzen `manualControlActive_` zur LED-Synchronisation.

## Zustands-/Flag-Hinweise
- Status-Flags: `kOverrunArmed` nur aktiv, wenn Poller unten und kein Cooldown läuft; `kCooldownActive` ansonsten gesetzt.
- Poller-Sensor latched hängt, bis Poller wieder oben und Sensor frei.
- Pult-Webstatus zeigt `ageMs` (Zeit seit letztem Status) sowie Ping-Infos.

## Test / Troubleshooting
- Nach Parameteränderungen Kalibrierung triggern, damit Stepper neue Home-Basis übernimmt.
- Bei LED-Regressionen: `LedController::applyInputs` befüllt die State-Maschine; prüfen, ob `manualControlActive_` korrekt gesetzt wird.
- Für Netzwerktests: Ping-Anzeige im Web-UI beobachten; sollte sich alle ~5 s aktualisieren.

## Aktueller Stand (Webhook & Delay)
- Web-UI: neues Feld **Status Endpoint (Basis-URL)** plus Checkbox **Poller Delay aktiv**. Poller Delay default 3000 ms, Webhooks initial auf `http://10.124.42.5:5000/` gesetzt.
- Events senden optional `POST` auf `/queues/queue?name=SuMo:<Action>` mit Actions `MatchStart`, `MatchStop`, `CountdownStart`, `PollerUp`, `PollerDown`, `PollerOverrun`.
- Delay-Logik: Überfahrt hält Puls-Animation bis `Poller Delay` abläuft, dann Raise. Kein automatisches Re-Arming im Poller-Loop – das übernimmt die Orchestrierung nach dem Hochfahren.
- Defaults: Cooldown 20000 ms, Poller Delay 3000 ms (Checkbox aktiv). Werte werden über `/api/command?type=updateConfig` gespeichert.

Happy hacking!
