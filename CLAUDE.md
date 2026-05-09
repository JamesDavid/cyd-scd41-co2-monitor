# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-file Arduino sketch (~3,400 lines of C++) plus an OpenSCAD enclosure and Markdown reference docs. The sketch (`cyd_scd41_co2_monitor_portrait/cyd_scd41_co2_monitor_portrait.ino`) is firmware for an ESP32-2432S028R "CYD" board with a Sensirion SCD41 sensor that drives a touchscreen UI, a captive-portal-configured WiFi web dashboard, MQTT, and OTA updates. There is **no host-side test or build runner** — everything is built and flashed via the Arduino IDE.

## Build & flash

There is no `make`, `cmake`, npm, or test runner here. Builds happen in the Arduino IDE (1.8.x or 2.x) targeting **ESP32 Dev Module**, partition scheme **Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)** (LittleFS lives in the SPIFFS partition).

Two non-obvious build steps that are easy to forget:

1. **TFT_eSPI is configured globally**, not per-sketch. The `User_Setup.h` block to paste into `Documents/Arduino/libraries/TFT_eSPI/User_Setup.h` is documented in `cyd.md` (under "TFT_eSPI configuration") and `README.md` (under "Build"). Without that, the display is blank or wrong colors.
2. **Sensirion library v1.0.0+ renamed the header** to lowercase `<SensirionI2cScd4x.h>`. The sketch uses the new name; older 0.4.x installs will fail to compile.

Required libraries are listed in the sketch header comment and in `README.md`. After the first USB flash, subsequent updates can use OTA at `http://<hostname>.local/update` (Sketch → Export Compiled Binary in the IDE produces the `.bin`).

There is no automated test suite. "Testing" means flashing and watching serial output at 115200 baud — the `info` serial command dumps current chip + NVS state and is the primary debugging tool. Other serial commands: `wifi-setup`, `wifi-reset`, `wifi-status`, `frc <ppm>`, `asc on|off`, `reset` (clear NVS), `erase-log`.

## Versioning rule (embedded in the sketch header)

Lines ~69–83 of the .ino contain a "NOTE TO LLMs / FUTURE EDITORS" block: **bump `FIRMWARE_VERSION` on any non-trivial code change** (patch for tiny fixes, minor for new features/behavior, major for breaking schema changes). Leave `FIRMWARE_AUTHOR` alone unless explicitly asked. This is a project convention; respect it.

## Architecture

Single `loop()` runs a fixed set of cooperative "service" routines without blocking — never add `delay()` to the main loop or you'll stall touch and the web server. The structure is:

- **Time-driven state machine in `loop()`**: `handleSerialCommands` → `handleTouch` → either `serviceWifiPortal` or `server.handleClient` + `ElegantOTA.loop` → `saveUptimePeriodic` → `serviceNtp` → `serviceAutoBrightness` → `serviceMqtt` → 1-second clock/date redraw → 5-second sensor read → 5-minute graph sample + log append.
- **Two independent persistence layers**: settings, the 60-sample in-RAM graph buffer, and uptime live in NVS via `Preferences` (namespace `co2mon`). The long-term sample log lives on LittleFS at `/co2log.bin` as a fixed-header + ring-buffer of 10-byte records (CO2/temp/RH/UTC). WiFi credentials live in a separate ESP32 system NVS namespace owned by WiFiManager.
- **Screen state machine**: `currentScreen` (enum `Screen`) selects which `draw*Screen()` and which touch hit-rects are active. Hit-rects are `const Rect` globals named `btn*`. Touch is a debounced single-tap model (no drag/swipe). Most redraws are partial (`fillRect` over the changed region) — full `fillScreen()` causes visible flicker, so don't rebuild the whole UI in response to a single value change.
- **Web server endpoints** are wired in `startWebServer()` and split across `handleRoot/handleData/handleHistoryCsv/handleHistoryJson/handleSetHostname/handleMqttPage/handleSetMqtt`. The CSV is streamed in chunks because the full log can be ~1 MB and won't fit in RAM.
- **MQTT** uses PubSubClient with Home Assistant auto-discovery. Topics are built from `mqttTopicPrefix` (defaults to mDNS hostname).

The four documentation files are tightly cross-referenced and worth reading before non-trivial changes:

- `README.md` — bill of materials, hardware wiring, what the device does, web/MQTT endpoint reference, build setup.
- `cyd.md` — board-level reference (pin map, TFT_eSPI config, touch axis quirk, backlight PWM gotcha, LDR wiring + backlight-bleed mitigation, RGB LED gotcha).
- `scd41.md` — sensor-level reference (the 500 ms-after-stop rule, ASC trap for indoor monitors, FRC procedure, error codes).
- `user_manual.md` — end-user guide (CO2 zone colors, settings UI flows, calibration walkthrough). Match its language when changing user-visible strings.

## Hardware-driven gotchas you will hit

These are the things that make this codebase non-obvious. Each is documented in depth in `cyd.md` or `scd41.md`; treat those files as the authoritative reference.

- **`stopPeriodicMeasurement()` requires a 500 ms delay before any config write**, or the chip silently ignores the write while returning success. The sketch wraps this in a `withChipIdle(...)` helper — use it for `setSensorAltitude`, `setTemperatureOffset`, `setAutomaticSelfCalibrationEnabled`, `performForcedRecalibration`, etc. Never bypass it.
- **ASC is intentionally disabled at every boot** and is *not* loaded from NVS even if a previous version stored it. The SCD41's own EEPROM remembers the setting independently, so `loadPrefs()` explicitly forces `ascEnabled = false` and `applySensorSettings()` writes 0 to the chip. Don't "fix" this by restoring NVS load — read the comment in `loadPrefs` and `scd41.md` first.
- **Touch chip axes don't follow `tft.setRotation()`**. The XPT2046's raw `(p.x, p.y)` are remapped via `TS_SWAP_XY` and the four `TS_*_MIN/MAX` constants, with `TS_X_MIN > TS_X_MAX` to invert one axis. New units may need recalibration — tap each corner, watch the `TAP raw=...` serial output, update the constants.
- **`setupBacklight()` must run after `tft.init()`**, otherwise TFT_eSPI's pinMode call clobbers the LEDC attach and brightness is stuck at 100%. The sketch comments this in `setup()`; preserve the ordering.
- **Backlight PWM frequency is 5 kHz, not the more common 10–20 kHz.** The on-board MOSFET driver doesn't switch off cleanly at higher frequencies, so duty cycle has no visible effect. Do not "modernize" this constant.
- **CYD comes in two LCD revisions** with opposite color polarity. The compile-time flag `INVERT_DISPLAY_COLORS` (default `true`) handles this — if green looks magenta on a future board, flip it; don't add runtime detection.
- **LDR auto-brightness depends on an opaque case** (the 3D-printed shroud). Without it the LDR reads the LCD backlight bleed instead of room light. Behavior changes that look fine on the bench may misbehave in the closed enclosure.
