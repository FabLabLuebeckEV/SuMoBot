# Arena Sumo Bot – Detaildokumentation

## Komponenten

- **Poller (env:poller)**
  - Stepper-gesteuerter Hubmechanismus mit Endschalter (GPIO 14, LOW aktiv)
  - Poller-Sensor (GPIO 36)
  - LED-Streifen: Rundum (GPIO 25), Poller-Ringe (GPIO 4), Arena (GPIO 26)
  - UDP-Status/Command-Link, FastLED (ESP32 I2S) für LED-Streaming
- **Pult (env:pult)**
  - Match-Orchestrierung, Auto-Lower, Web-UI (LittleFS)
  - LCD via I²C (GPIO 21/22), Taster (GPIO 4/39/34)
  - UDP-Kommandos an den Poller, Webhooks für externe Systeme

## Kommunikation

| Kanal          | Protokoll | Port  | Beschreibung                                             |
|----------------|-----------|-------|----------------------------------------------------------|
| Poller ↔ Pult  | UDP       | 42142 | Broadcast-Autodiscovery, `PollerCommand` / `PollerStatus`|
| Web-UI         | HTTP      | 80    | Pult stellt API/Frontend über LittleFS bereit            |
| Webhooks       | HTTP POST | frei  | Optional, konfigurierbar im Web-UI (`/queues/queue?...`) |

`include/comms/messages.h` enthält die Paketstrukturen. `include/comms/peer_config.h` definiert Standardziele (Broadcast). Bei festen IPs: Bytes 0–3 = IPv4, Bytes 4–5 = UDP-Port (Big Endian).

## Poller-Ablauf

1. **Startup**: FastLED-Init, alle Streifen ca. 2 s blau.
2. **Homing**: Stepper fährt nach oben, Endschalter setzt `positionHome`. Erst danach werden Abwärtskommandos akzeptiert.
3. **Status-Publish**: `PollerStatus` enthält Positionen, Flags, Parameter-Kopie, Ping-IDs.
4. **LED-Task**: FreeRTOS-Task (`core 0`, ~8 ms Loop) rendert Animationen und ruft `FastLED.show()` mit ≥16 ms Abstand.

Sicherheitsmechanismen:
- `hardware::sanitized()` klemmt `position*`-Werte; `StepperController::clampTarget()` verhindert das Verlassen des erlaubten Bewegungsfensters.
- Sensortrigger (`kOverrunDetected`) löst Poller-Wellenanimation und Auto-Raise aus.
- Cooldown (`overrunCooldownMs`) blockiert erneutes Arming, bis der Poller wieder oben und freigegeben ist.

## LED-Zustände (Kurzfassung)

| Phase                | Arena                      | Rundum                         | Poller-Ringe                              |
|----------------------|----------------------------|--------------------------------|-------------------------------------------|
| Init                 | Blau                       | Blau                            | Blau                                     |
| Idle                 | Regenbogenlauf             | Schwach blau (solange nicht unten)| Blau wenn Poller oben, sonst aus      |
| Countdown            | Orange/Rot-Blink (300/500 ms) | Orange/Rot synchron            | folgt Arena                              |
| Match                | Grün                       | Grün (solange Poller oben)      | Grün (oben) / Aus (unten)               |
| Stop/Timeout         | Rot-Blink                  | Rot-Blink                       | Rot-Blink                                |
| Überfahrt „armed“    | Statusabhängig             | Wie Idle                        | Lauflicht außen→innen                    |
| Überfahrt „triggered"| Statusabhängig             | Wie Idle                        | Wellenanimation, danach wieder Blau      |
| Manuelle Fahrten     | Spiegeln Matchfarbe        | Animation pausiert              | Matchfarbe oder aus                     |

## Pult-Funktionen

- Drei Taster (Start/Stop, Poller, Countdown) mit Debounce & Cooldowns.
- LCD-Anzeige für Status (Matchphase, Ping, Pollerinformationen).
- Web-UI (LittleFS `data/`): Status-Seite, Parameter-Formulare, Webhook-/Delay-Konfiguration.
- Auto-Lower (konfigurierbarer Delay) & Webhook-Ereignisse (`MatchStart`, `PollerOverrun`, ...).

## Konfiguration & Dateien

- **Poller-Parameter** (`hardware::PollerParameters`): EEPROM, via Web-UI veränderbar. Änderungen erfordern erneutes Homing.
- **Netzwerk** (`include/network_config.h`): Wi-Fi-SSID/-Passwort des Pults.
- **UDP-Ziele** (`include/comms/peer_config.h`): Standard Broadcast 255.255.255.255:42142; bei Bedarf IP + Port setzen.
- **LED-Setup** (`src/poller/led_controller.cpp`): FastLED I2S, DMA-Puffer, Dithering-Off, Stromlimit 6 A bei 5 V.

## Build/Flash Kurzanleitung

```bash
# Poller
pio run -e poller
pio run -e poller -t upload

# Pult
pio run -e pult
pio run -e pult -t upload
pio run -t uploadfs -e pult   # Webinhalte
```

Serielle Konsole: 115200 Baud. Firmware liegt nach dem Build unter `.pio/build/<env>/firmware.bin`.

## Fehlersuche

- **Keine UDP-Verbindung**: WLAN-Abdeckung prüfen, ggf. feste IPs konfigurieren oder Firewall anpassen.
- **LED-Flackern bei Wi-Fi**: Sicherstellen, dass Poller mit aktuellem FastLED-I2S-Build läuft; DMA-Puffer ggf. erhöhen (`FASTLED_ESP32_I2S_NUM_DMA_BUFFERS`).
- **Poller fährt zu weit**: Parameter prüfen; Logs zeigen geklemmte Targets (`[Stepper] Target request … clamped …`).
- **Webhooks**: Basis-URL im Web-UI setzen; Aktionen erscheinen als Query-Parameter (`SuMo:<Action>`).

Weitere Ablauf- und Troubleshooting-Notizen sind in `agents.md` hinterlegt.

