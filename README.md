# CYD CO2 Monitor

An indoor air quality monitor built on the ESP32-2432S028R ("Cheap Yellow Display") with a Sensirion SCD41 CO2 sensor. Shows live CO2, temperature, and humidity on the touchscreen, logs samples to flash for ~400 days of history, and serves a web dashboard with CSV export over WiFi.

## Hardware

- **MCU board**: ESP32-2432S028R (CYD), 2.8" 240x320 ILI9341 LCD with XPT2046 resistive touch
- **Sensor**: Sensirion SCD41 (NDIR CO2 + temp/RH on one I2C device, address 0x62)
- **Connection**: 4-wire I2C from the CYD's "Temp/Humidity" connector (silkscreened with 3V3/GND/27/22) to the SCD41's GND/VDD/SCL/SDA pads
- **Enclosure**: parametric OpenSCAD desk stand with mounting standoffs and an external sensor mount on the rear of the foot

See `cyd.md` for board-level programming notes and `user_manual.md` for end-user operation.

## What it does

The sketch runs an event loop that:

1. Polls the SCD41 every 5 seconds via I2C.
2. Updates on-screen readings on each successful read.
3. Every 5 minutes, captures a sample to a 60-entry in-RAM ring buffer (drives the on-device graph) and appends a fully-timestamped record to a flash log.
4. Renders a touch-driven UI: settings (altitude, temp offset, calibration), time settings (timezone, 12/24h, NTP), forced-calibration confirmation, WiFi setup via captive portal.
5. When connected to WiFi, runs a small web server with a Chart.js dashboard, JSON endpoint, CSV download, and OTA firmware update at `/update`.

## On-device UI

The screen is divided top-to-bottom:

- **Top**: the date and a large clock. Tap the clock area to open Time Settings.
- **Middle**: the current CO2 reading. The number itself is tappable — tap it to open the main Settings screen.
- **Below CO2**: GOOD / MODERATE / POOR / VERY POOR status banner, color-coded.
- **Lower**: temperature and humidity. Tap the temperature to switch between °F and °C.
- **Bottom**: a 5-hour CO2 trend graph.

The Settings screen has a brightness toggle (top-right of the header, cycles 100/75/50/25%), altitude, temp offset, calibration controls, and WiFi setup. Time Settings has timezone, 12/24h format, NTP toggle, and manual sync.

The UI is dark-mode-only — no theme toggle.

When calibration is more than 90 days old, an orange "Calibration overdue - tap CO2" alert appears at the bottom of the main screen.

## Persistence

Settings, the live graph buffer, and uptime accumulate to the ESP32's NVS via Arduino `Preferences`. The long-term sample log lives on a LittleFS partition.

| Storage | Lives in | Holds |
|---|---|---|
| Settings (units, altitude, temp offset, timezone, etc.) | NVS namespace `co2mon` | small typed values |
| Live 60-sample graph buffer | NVS bytes `hist` | written every 5 min |
| Total powered-on time | NVS uint `uptime` | hourly + on FRC |
| Last calibration time | NVS uint `frc_at` | uptime in seconds |
| Long-term log (CO2 + T + RH + UTC timestamp) | LittleFS `/co2log.bin` | up to 120,000 records |
| WiFi credentials | ESP32 system NVS (managed by WiFiManager) | SSID + password |

The log uses a fixed-size header followed by a ring buffer of 10-byte records. When full, oldest records are overwritten; the CSV export emits oldest-first across the wraparound.

## Calibration policy

The SCD41 ships with Automatic Self-Calibration (ASC) enabled. ASC takes the lowest CO2 reading in a multi-day window and assumes it equals fresh outdoor air at ~400 ppm. **For an indoor desk monitor, this is wrong** — rooms that never see fresh air get re-anchored downward by 100-300 ppm over weeks. The firmware disables ASC at boot and tracks calibration age instead. After `FRC_REMINDER_DAYS` (default 90) the screen shows an "overdue" alert and the in-page settings shows the cal-age in orange.

The `frc <ppm>` serial command and the on-screen "Force Recalibrate" button both call `performForcedRecalibration(420, ...)`. Both update `lastFrcUptimeSec` so the reminder timer resets.

ASC remains accessible via the `asc on` / `asc off` serial commands. It is intentionally not exposed in the touch UI to avoid foot-guns.

## NTP behavior

On a fresh boot it can take 5-15 seconds for NTP to sync after WiFi associates — DNS resolution and the first SNTP UDP roundtrip are slow on freshly-joined networks. The firmware:

1. Issues a blocking `getLocalTime(15000)` immediately after WiFi connects.
2. If that fails, schedules a retry every 30s in the main loop.
3. Also opportunistically detects asynchronous late completion: if `time(nullptr)` returns a sane value (post-2023) at any point, marks NTP as synced even if our explicit retry hasn't fired yet.

The clock and date update with one-second resolution and redraw the date string only when it changes.

## Color handling and the "magenta" issue

CYDs ship in two revisions with different LCD panel polarity. The single-USB variant displays colors normally; the dual-USB (USB-C + micro-USB) variant has inverted colors out of the box. We unconditionally call `tft.invertDisplay(INVERT_DISPLAY_COLORS)` (default true). If green looks magenta or red looks cyan on a future board, flip that compile-time constant.

## Touch axis quirk

The XPT2046 touch controller's native axes don't track the display rotation set in TFT_eSPI. For our portrait orientation (`tft.setRotation(0)`) on the test unit:

- Touch chip `p.x` actually tracks vertical motion on screen.
- Touch chip `p.y` actually tracks horizontal motion, but inverted.

So we set `TS_SWAP_XY = 1` and `TS_X_MIN > TS_X_MAX` (the swap-mapped X axis is reversed). The four `TS_*_MIN`/`MAX` constants are calibrated to specific corner taps on the test unit. New units may need re-tuning; tap each corner, watch the `TAP raw=...` lines on serial, and update the constants so each corner maps to its expected `(sx, sy)`.

See `cyd.md` for a more thorough walkthrough.

## Backlight PWM

Brightness is controlled via LEDC PWM on GPIO 21. The frequency had to be set to **5 kHz** (lower than you'd expect) because the on-board MOSFET driver has a slow turn-off characteristic — at 10+ kHz the gate doesn't discharge between cycles and the duty cycle has no visible effect. 5 kHz is well above the human flicker threshold (~80 Hz) but slow enough for the MOSFET to actually modulate. Some CYD revisions reportedly have a hardware pull-up that prevents PWM control entirely; on those boards, brightness is fixed at 100% regardless of duty cycle.

If you observe **continuous flicker even at full brightness**, that's a separate issue — almost always a marginal USB power supply. See `cyd.md` for mitigations.

## Web interface

When WiFi is connected:

| Endpoint | Returns |
|---|---|
| `/` | HTML status page with a Chart.js plot of the last 24h, refreshes every 60s |
| `/data.json` | Current readings + settings as JSON |
| `/history.json?n=288` | Recent N samples for the chart (default 288 = 24h) |
| `/history.csv` | Full log as CSV (timestamp, CO2, temp, RH), streamed in chunks |
| `/update` | ElegantOTA firmware update form |

mDNS is published as `co2monitor.local` so the device is reachable without knowing the IP.

## Build

Arduino IDE 1.8.x or 2.x with the **esp32 by Espressif Systems** core (2.x series) installed. Required libraries from Library Manager:

- TFT_eSPI by Bodmer
- XPT2046_Touchscreen by Paul Stoffregen
- Sensirion I2C SCD4x by Sensirion (1.0.0+)
- Sensirion Core (auto-installed)
- WiFiManager by tzapu (tablatronix)
- ElegantOTA by Ayush Sharma (3.x)

TFT_eSPI is configured globally via its `User_Setup.h`. Paste the block in the sketch header into `Documents/Arduino/libraries/TFT_eSPI/User_Setup.h`, replacing whatever's there.

Tools menu:

- Board: **ESP32 Dev Module**
- Partition Scheme: **Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)** — the SPIFFS partition is what LittleFS uses
- Flash Size: **4MB (32Mb)**
- Flash Frequency: 80 MHz
- Flash Mode: QIO (or DIO if QIO won't boot)
- Upload Speed: 921600

After the first flash, OTA updates work via `http://co2monitor.local/update`. Sketch → Export Compiled Binary in the Arduino IDE produces a `.bin` to upload.

## Files in this project

| File | Purpose |
|---|---|
| `cyd_scd41_co2_monitor_portrait.ino` | Main sketch |
| `cyd_desk_stand.scad` | Parametric OpenSCAD enclosure |
| `README.md` | This file |
| `cyd.md` | Notes on programming the CYD board |
| `user_manual.md` | End-user operation guide |

## License

This project is for personal use. The libraries it depends on retain their respective licenses (TFT_eSPI, XPT2046_Touchscreen, etc. — mostly MIT / FreeBSD). The Sensirion drivers are BSD-3-Clause.
