# CYD CO2 Monitor

An indoor air quality monitor built on the ESP32-2432S028R ("Cheap Yellow Display") with a Sensirion SCD41 CO2 sensor. Shows live CO2, temperature, and humidity on the touchscreen, logs samples to flash for ~9 months of history, serves a web dashboard with Chart.js graphs and CSV export, and optionally publishes to MQTT for Home Assistant / Node-RED. Can also passively sniff an **Aranet Rn** radon detector over Bluetooth and surface radon as a fourth metric.

## ⚡ Flash it from your browser

No toolchain needed — flash a pre-built binary straight from Chrome/Edge/Opera with the web installer:

### 👉 **https://jamesdavid.github.io/cyd-scd41-co2-monitor/**

Plug the CYD into your computer with a data USB cable, click **Connect & Flash**, and pick the serial port. (Web Serial is desktop Chrome / Edge / Opera only; Firefox and Safari can't do USB flashing.) After this one USB flash, future updates go over the air at `http://<device>.local/update`.

> **Heads-up:** this build changes the flash partition table (to make room for Bluetooth), so it must be flashed over USB once and it **resets the on-device history log**. If you're upgrading an existing unit, download `http://<device>.local/history.csv` first. Settings survive.

To build from source instead, see [Build](#build) below.

## Bill of Materials

### Amazon

- **CYD board (ESP32-2432S028R, 2.8" 240×320 touchscreen)** — about $30 for a 2-pack. Either the single-USB or dual-USB variant works; firmware auto-handles the color-inversion difference between them.
  https://www.amazon.com/dp/B0CLR7MQ91 (AITRIP 2-pack)
- **Sensirion SCD41 CO2 + temp + humidity sensor module** — true NDIR sensor, not a cheaper VOC equivalent. ~$30.
  https://www.amazon.com/dp/B0C622SS34 (Teyleten Robot SCD41)
- **Right-angle USB-C adapter** — the case routes the cable behind the stand, so a 90° adapter keeps the cable from sticking straight out the back. ~$10 for a 4-pack.
  https://www.amazon.com/dp/B0BDZCMYZG

You'll also need 4 × M3 self-tapping screws (any length 6–10 mm works) for the PCB-to-standoff mount, plus an existing USB-C cable and 5 V power supply (any phone charger).

### 3D-printed enclosure

This project uses a remixed version of the **Aura** weather display case, modified for the CYD's specific PCB dimensions and the SCD41's mount. Print on any FDM printer; the case prints in two pieces (front frame + L-shaped snap-fit back) and the SCD41 mounts to the rear of the foot.

Original Aura case: https://makerworld.com/en/models/1382304-aura-smart-weather-forecast-display

The OpenSCAD source for the modified version is `cyd_desk_stand.scad` in this repo. Open it in OpenSCAD, set `render_part` to `"front"`, export STL, then set to `"back"` and export again. Print each piece in opaque filament — black or dark colors work best, since the LDR shroud relies on the case being opaque to block the LCD backlight from leaking onto the ambient light sensor.

### Arduino libraries

Install via the Arduino IDE Library Manager:

| Library | Author | Notes |
|---|---|---|
| TFT_eSPI | Bodmer | Display driver. Needs `User_Setup.h` patched (see Build section). |
| XPT2046_Touchscreen | Paul Stoffregen | Touch driver. |
| Sensirion I2C SCD4x | Sensirion | 1.0.0 or later. |
| Sensirion Core | Sensirion | Auto-installed as a dependency. |
| WiFiManager | tzapu (tablatronix) | 2.0.17 or later. Captive-portal WiFi setup. |
| ElegantOTA | Ayush Sharma | 3.x series. Web-form firmware update at `/update`. |
| PubSubClient | Nick O'Leary | MQTT client. |
| NimBLE-Arduino | h2zero | 1.4.x. Passive BLE sniffing of the Aranet Rn radon detector. |

The ESP32 board package itself ships with `WiFi.h`, `WebServer.h`, `ESPmDNS.h`, `Preferences.h`, `LittleFS.h` — no separate install needed.

> This repo is now a **PlatformIO** project (`platformio.ini` + `partitions.csv`); the library versions and the TFT_eSPI configuration are pinned there, so the Arduino-IDE Library-Manager steps above are only needed if you build in the IDE instead. Build with `pio run -e usb`, flash with `pio run -e usb -t upload`.

## Hardware wiring

The CYD has a "Temp/Humidity" connector with the silkscreen labels **3V3 / GND / 27 / 22**, which map to the SCD41 module's **VDD / GND / SDA / SCL** pins (I2C bus on GPIO 27 SDA, GPIO 22 SCL, address 0x62). One JST connection, no soldering required if your CYD came with the matching cable.

## What it does

1. Polls the SCD41 every 5 seconds via I2C.
2. Updates on-screen readings on each successful read.
3. Every 5 minutes, captures a sample to a 60-entry in-RAM ring buffer (drives the on-device graph) and appends a fully-timestamped record to a flash log.
4. Renders a touch-driven UI: settings (altitude, temp offset, calibration), time settings (timezone, 12/24h, NTP), 24-hour stats screen, forced-calibration confirmation, WiFi setup via captive portal.
5. When connected to WiFi, runs a web server with Chart.js plots (CO2, temperature, humidity, and radon when present), a JSON endpoint, CSV download, MQTT configuration page, and OTA firmware update at `/update`.
6. Optionally publishes to MQTT every 30 seconds with Home Assistant auto-discovery.
7. Optionally sniffs an **Aranet Rn** radon detector over BLE (see below).

## Radon (Aranet Rn over Bluetooth)

If you own an [Aranet Rn](https://aranet.com) radon detector and enable **Smart Home Integration** mode in the Aranet app, it broadcasts its latest reading in a BLE advertisement. This firmware passively sniffs that advertisement (no pairing) and treats radon as a fourth metric:

- The bottom row of the main screen becomes **Temp | RH | Radon** while a fresh reading is present, and falls back to **Temp | RH** when the Aranet is out of range or the reading goes stale.
- Tap the radon number to toggle **Bq/m³ ↔ pCi/L**. Radon joins the tappable graph cycle (CO2 → Temp → RH → Radon).
- A low-battery marker appears by the radon value when the Aranet battery is low.
- Radon flows into `/data.json`, `/history.csv` (`radon_bqm3` column), `/history.json`, the web dashboard chart, and MQTT (a `Radon` and a `Radon Battery` entity auto-discover in Home Assistant).
- Only the **current** reading is broadcast — the 7d/30d/90d averages in the Aranet app are computed in the cloud and are not available over BLE.
- Configure the staleness timeout, default units, and (if you have several Aranet Rn units) pin a specific one by MAC at **`/radon`**. The `ble-status` serial command lists every Aranet Rn it can hear. Radon samples are stored in Bq/m³; only the display/JSON honor the unit toggle (the CSV stays Bq/m³, mirroring temperature staying Celsius).

No extra wiring — the ESP32's built-in radio does both WiFi and BLE (a short passive/active scan runs about once a minute).

## On-device UI

The screen is divided top-to-bottom:

- **Top**: the date and a large clock. Tap the **date** to open the 24-hour stats screen. Tap the **clock** to open Time Settings.
- **Middle**: the current CO2 reading. The number itself is tappable — tap it to open the main Settings screen.
- **Below CO2**: GOOD / MODERATE / POOR / VERY POOR status banner, color-coded.
- **Lower**: temperature and humidity. Tap the temperature to switch between °F and °C.
- **Bottom**: a 5-hour CO2 trend graph. Tap to cycle CO2 → Temperature → Humidity.

The Settings screen has a brightness toggle (top-right of the header, cycles 100 / 75 / 50 / 25% / Auto), altitude, temp offset, calibration controls, and WiFi setup. Time Settings has timezone, 12/24h format, NTP toggle, and manual sync.

The 24-hour stats screen shows min/avg/max CO2 over the last day, a colored bar with proportional time spent in each CO2 zone, the best and worst hours of day (lowest and highest average CO2 by hour, requires NTP sync), and the temperature and humidity ranges. Tap anywhere to return to the main screen.

The UI is dark-mode-only — no theme toggle.

When calibration is more than 90 days old, an orange "Calibration overdue - tap CO2" alert appears at the bottom of the main screen.

## Auto-brightness

The CYD has an LDR (silkscreened R21 — actually a photoresistor, not a regular resistor) on GPIO 34. The Auto brightness mode reads it every 2 seconds, applies exponential smoothing, and maps the result to a backlight duty cycle between 15% (dark room) and 100% (bright room). The case includes an internal opaque shroud around the LDR position to block the LCD backlight from leaking sideways onto the sensor — without it, the LDR reads "lit" even in a pitch-dark room.

## Persistence

Settings, the live graph buffer, and uptime accumulate to the ESP32's NVS via Arduino `Preferences`. The long-term sample log lives on a LittleFS partition.

| Storage | Lives in | Holds |
|---|---|---|
| Settings (units, altitude, temp offset, timezone, brightness, MQTT, etc.) | NVS namespace `co2mon` | small typed values |
| Live 60-sample graph buffer | NVS bytes `hist`, `histT`, `histH` | written every 5 min |
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
| `/` | HTML status page with Chart.js plots (CO2, temperature, humidity, and radon when present) and live readings; refreshes every 60s |
| `/data.json` | Current readings + settings as JSON (radon fields included when a fresh reading exists) |
| `/history.json?n=288` | Recent N samples for the charts (default 288 = 24h); includes CO2, temp in current unit, RH, and radon (`null` where absent) |
| `/history.csv` | Full log as CSV (`timestamp_utc,co2_ppm,temp_c,rh_pct,radon_bqm3`), streamed in chunks |
| `/mqtt` | MQTT configuration form with live connection status |
| `/setmqtt` (POST) | Saves MQTT settings |
| `/radon` | Radon (Aranet Rn) settings: units, staleness timeout, target-device picker; lists every Aranet Rn heard |
| `/setradon` (POST) | Saves radon settings |
| `/sethostname` (POST) | Updates the mDNS hostname |
| `/update` | ElegantOTA firmware update form |

mDNS is published as `co2monitor-XXXXXX.local` (where XXXXXX is a 6-character ID derived from the MAC) so multiple devices on the same network don't collide. The hostname is changeable via the web form.

## MQTT and Home Assistant

The MQTT client (PubSubClient under the hood) publishes every 30 seconds when enabled and connected. Topics under `{prefix}` (default = device hostname):

| Topic | Payload |
|---|---|
| `{prefix}/co2` | integer ppm |
| `{prefix}/temp_c` | float Celsius |
| `{prefix}/temp_f` | float Fahrenheit |
| `{prefix}/rh` | float % humidity |
| `{prefix}/status` | `good` / `moderate` / `poor` / `verypoor` |
| `{prefix}/radon` | integer Bq/m³ (only when a fresh Aranet Rn reading exists) |
| `{prefix}/radon_battery` | Aranet battery % (only when fresh) |
| `{prefix}/state` | JSON with all of the above (for HA) |

If Home Assistant auto-discovery is enabled (it is by default), the device publishes its config to `homeassistant/sensor/{node}/{co2,temp,rh,radon,radon_batt}/config` on connect. HA picks up the device automatically, exposing one device with up to five sensor entities. Temperature units track the on-device F/C setting; radon is always reported in Bq/m³ over MQTT.

Configure via the `/mqtt` page on the web dashboard.

## Build

### PlatformIO (recommended)

```
pio run -e usb            # compile
pio run -e usb -t upload  # flash over USB
pio device monitor        # serial console @115200
```

Everything is pinned in `platformio.ini`: the `espressif32 @ 6.9.0` platform (arduino-esp32 2.0.x), all library versions, the TFT_eSPI configuration (via `build_flags`, so no global `User_Setup.h` edit is needed), and the custom partition table (`partitions.csv`).

**The radon build needs a custom partition table.** Adding the BLE (NimBLE) stack pushes the firmware past the stock 1.25 MB app slot, so `partitions.csv` grows each OTA app slot to 1.5 MB and shrinks LittleFS to ~0.94 MB (still ~277 days / ~9 months of 5-minute samples). Consequences:

- **A partition-table change can only be applied over USB.** Flashing this version onto a device running an older build requires one USB flash; `/history.csv` should be downloaded first because the new layout resets the on-flash log.
- **OTA still works for every future update.** Dual-OTA is preserved (two 1.5 MB slots) and the ~1.4 MB firmware fits, so `http://co2monitor-XXXXXX.local/update` behaves exactly as before once the new partitioning is in place.

### Updating the web flasher

The browser flasher under `docs/` serves pre-built binaries. After a release build, refresh them and bump the version in `docs/manifest.json`:

```
pio run -e usb
cp .pio/build/usb/{bootloader,partitions,firmware}.bin docs/firmware/
cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin docs/firmware/
```

The ESP32 flash offsets in `docs/manifest.json` are `0x1000` bootloader, `0x8000` partitions, `0xe000` boot_app0, `0x10000` app.

### Arduino IDE (alternative)

Still buildable as a single `.cpp`/`.ino` in the IDE 1.8.x/2.x with the esp32 2.x core, but you must do it the manual way: paste the TFT_eSPI block from the sketch header into `Documents/Arduino/libraries/TFT_eSPI/User_Setup.h`, install NimBLE-Arduino, set Board **ESP32 Dev Module**, and select a custom partition CSV matching `partitions.csv` (the stock "Default 4MB with spiffs" is too small).

## Files in this project

| File | Purpose |
|---|---|
| `src/cyd_scd41_co2_monitor_portrait.cpp` | Main firmware (single file) |
| `platformio.ini` | PlatformIO build config (platform, libraries, TFT_eSPI flags) |
| `partitions.csv` | Custom 4 MB partition table (1.5 MB dual-OTA app + ~0.94 MB LittleFS) |
| `docs/` | GitHub Pages web flasher: `index.html` + `manifest.json` + `firmware/*.bin` (ESP Web Tools) |
| `cyd_desk_stand.scad` | Parametric OpenSCAD enclosure (two-piece: front + L-shaped back) |
| `README.md` | This file |
| `cyd.md` | Notes on programming the CYD board |
| `scd41.md` | SCD41 sensor reference (gotchas, ASC trap, FRC procedure) |
| `user_manual.md` | End-user operation guide |

## License

This project is for personal use. The libraries it depends on retain their respective licenses (TFT_eSPI, XPT2046_Touchscreen, etc. — mostly MIT / FreeBSD). The Sensirion drivers are BSD-3-Clause.
