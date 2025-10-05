# Agent Notes

## Repository Snapshot
- Branch: `feature/readme-funktionen`
- PlatformIO-Projekt mit zwei Targets auf gemeinsamer Codebasis (`env:poller`, `env:pult`).
- Wichtige Verzeichnisse:
  - `src/common/` transport-/utility-Code (UDP-Link, shared helpers)
  - `src/poller/` Poller-Firmware (Stepper, LED-Controller, Status)
  - `src/pult/` Pult-Firmware (MatchOrchestrator, Webserver, LCD)
  - `include/` Hardware-/Protokoll-Header
  - `data/` LittleFS-Weboberfläche für das Pult
- Historische Verzeichnisse (`legacy/`, `ArenaSumoBot/`, `PultSuMoBot/`) wurden entfernt.

## Build & Tooling
- PlatformIO CLI via `pipx install platformio` verfügbar (`~/.local/bin/pio` liegt auf dem PATH).
- Builds:
  - Poller: `pio run -e poller`
  - Pult: `pio run -e pult`
  - Pult-Webassets: `pio run -t uploadfs -e pult`
- Standard-Upload über PlatformIO oder externes Tool; nach größeren LED-Änderungen beide Targets flashen.

## Runtime Essentials
- Poller kalibriert beim Start (Homing nach oben) und blockiert Down-Kommandos bis eine Referenz gesetzt ist.
- Bewegungs-/Timing-Parameter in `hardware::PollerParameters`; Änderungen via Web-UI (`/api/command?type=updateConfig`) → im EEPROM persistiert.
- Pult↔Poller-Kommunikation läuft über UDP (Port `42142`) mit Broadcast-Autodiscovery; Status enthält aktuelle Parameter + Ping-Antworten.
- Pult sendet alle 5 s `kPing`; Poller spiegelt das in `PollerStatus.lastCommandId` für Latenz-Monitoring.

## LED Controller Highlights (Poller)
- Läuft als FreeRTOS-Task auf Core 0 (~8 ms Loop). `FastLED.show()` throttelt auf ≥16 ms.
- FastLED nutzt den ESP32 I2S/DMA-Treiber (`FASTLED_ESP32_I2S`, 4 DMA-Buffers, RMT4 erzwingen) und deaktivierte Dithering.
- Szenen:
  - Init: alles blau (~2 s)
  - Idle: Arena-Regenbogen, Rundum blau (solange Poller nicht unten)
  - Countdown: Orange/Rot-Blinken
  - Match: Arena grün, Rundum spiegelt Status
  - Stop/Timeout: Rotblink global
  - Überfahrt armed/triggered: Lauflicht bzw. Wellenanimation auf Poller-Ringen
  - Manuelle Moves: Spezialanimationen pausieren

## Kommunikation & Matchsteuerung
- `MatchOrchestrator` steuert Countdown, Matchphasen, Auto-Lower und Webhooks.
- Poller akzeptiert Kommandos nur, wenn Kalibrier- und Cooldown-Status es erlauben.
- Status-Flags: `kOverrunArmed`, `kCooldownActive`, `kPollerSensorActive`, `kOverrunDetected`, `kLinkLowQuality`, `kCommandError` (sticky).

## Troubleshooting
- Nach Parameteränderungen Kalibrierung erneut anstoßen.
- Poller fährt nie unter `positionDownTarget`; Werte werden in `hardware::sanitized` geklemmt, Stepper-Targets werden im Controller geclamped.
- LED-Fehler: prüfen, ob Task läuft (`LedController`-Logs) und ob `manualControlActive_` korrekt gesetzt wird.
- Netzwerk: UDP-Status (`ageMs`) im Web-UI beobachten; Broadcast greift, falls keine statischen IPs konfiguriert.
- Bei LED-Flackern: sicherstellen, dass beide Targets mit aktuellem FastLED-I2S-Build geflasht sind.
