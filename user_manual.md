# CO2 Monitor User Manual

## What this device measures

This is a desktop CO2 monitor. The big number is the carbon dioxide concentration in your air, measured in parts per million (ppm). The device also reports temperature and humidity.

CO2 in fresh outdoor air sits around 420 ppm. Indoor CO2 climbs as people breathe — the higher it goes, the stuffier the room, and at high enough levels you start to feel sluggish, lose focus, or get a headache. The color and status text on the screen tell you what zone you're in:

| Reading | Color | Status | What it means |
|---|---|---|---|
| Below 800 ppm | Green | GOOD AIR | Well-ventilated; comparable to outdoor air |
| 800-1199 ppm | Blue | MODERATE | Acceptable but watch the trend |
| 1200-1999 ppm | Orange | POOR - VENTILATE | Open a window or door |
| 2000+ ppm | Red | VERY POOR | Get fresh air now |

A small arrow next to the number shows whether CO2 is rising or falling over the last several samples.

### Radon (optional, needs an Aranet Rn)

If you have an **Aranet Rn** radon detector with "Smart Home Integration" turned on in the Aranet app, this monitor listens for it over Bluetooth and shows radon as a third reading at the bottom of the screen — no pairing or wiring needed. Radon is a colorless radioactive gas; long-term exposure to high levels raises lung-cancer risk. The radon number is color-coded:

| Reading | Color | What it means |
|---|---|---|
| Below 100 Bq/m³ (2.7 pCi/L) | Green | Low |
| 100-147 Bq/m³ | Orange | Elevated — WHO reference level |
| 148+ Bq/m³ (4 pCi/L) | Red | High — US EPA action level; consider mitigation |

Radon only appears when a recent reading has been received. If the Aranet goes out of range or is switched off, the radon column disappears after a few minutes and the bottom row returns to just Temp and RH. The Aranet measures radon every 10 minutes, so the number updates slowly. A small battery icon appears next to "RADON" when the Aranet's battery is low.

## The screen

```
       Sunday, May 3 2026               <- date

           10:42 AM                     <- big clock (tap to set up)

            CO2 (ppm)
              760                       <- big reading + trend arrow
            GOOD AIR                    <- color-coded status

   TEMP        RH      RADON            <- RADON column only when in range
   72.4 F      48 %    96 Bq            <- tap TEMP=F/C, tap RADON=Bq/pCi

      [---small trend graph---]         <- tap to cycle CO2/Temp/RH/Radon
```

**What's tappable:**

- **Clock**: opens Time Settings (timezone, 12/24-hour format, NTP).
- **CO2 number**: opens Settings (calibration, altitude, temp offset, WiFi).
- **Temperature value**: switches between Fahrenheit and Celsius.
- **Radon value** (when shown): switches between Bq/m³ and pCi/L.
- **Graph**: cycles through CO2, Temperature, Humidity, and Radon (Radon is included only when a reading is present).

If a calibration is more than 90 days old, an orange "Calibration overdue - tap CO2" message appears at the bottom of the screen.

## First-time setup

1. Plug into power via the micro-USB or USB-C port. The screen lights up and shows "Initializing..." for a few seconds.
2. The CO2 number appears as "warming up..." for the first 5-10 seconds while the sensor takes its first reading.
3. Once a reading appears, the device is functional. You can use it stand-alone right now.
4. **Set up WiFi (optional)**: tap the CO2 number to enter Settings, then tap the WiFi tile at the bottom. Connect your phone to the AP called `CO2-Monitor-Setup-XXXXXX` (where XXXXXX is a 6-character ID unique to your device — the exact name appears on the WiFi setup screen so you can read it off). Pick your home network from the captive portal, enter the password. The device saves it and reconnects automatically on every future boot.
5. **Set the clock (optional)**: with WiFi working, tap the time at the top of the screen, set your timezone, choose 12 or 24-hour format. Time auto-syncs from internet time servers.
6. **Calibrate (recommended after a few days)**: see the calibration section below.

## Settings (tap the CO2 number)

- **Brightness button** (top-right of the Settings header): cycles 100% → 75% → 50% → 25% → 100%. Setting persists across reboots. Useful for dimming the display at night or in a dark room.
- **Altitude**: adjust in 10-meter steps. Atmospheric pressure changes with altitude and affects CO2 ppm accuracy by a few percent. Set this to roughly your elevation. (Tempe, AZ ≈ 340m. Sea-level cities = 0. Denver ≈ 1600m.)
- **Temp Off.**: temperature offset. The sensor reads a few degrees higher than ambient because it self-heats from the electronics. Use the +/- buttons to compensate. The default of +5.7°C / +10.2°F is a good starting point. Fine-tune by comparing with a separate, known-accurate thermometer after the device has been on for at least an hour.
- **Last cal: N days ago**: shows when you last calibrated. Turns orange and suggests recalibration after 90 days.
- **Force Recalibrate**: opens the calibration flow.
- **WiFi tile**: shows the IP address and the device's `.local` hostname when connected. The default hostname is `co2monitor-XXXXXX` (where XXXXXX is unique to your device); you can change it to something more memorable from the web interface — see below. Tap the tile to reconfigure or initially set up WiFi.

## Time settings (tap the clock)

- **Timezone**: ±15 minute steps from -12:00 to +14:00. Set to your local UTC offset. For Arizona, UTC-7:00 (no daylight saving). For US Pacific, UTC-7:00 in summer / UTC-8:00 in winter; the device doesn't auto-handle DST so you'll need to nudge it twice a year if you're in a DST-observing zone.
- **Format**: 12-hour (with AM/PM) or 24-hour.
- **NTP**: Network Time Protocol — automatic time sync over the internet. Leave on unless you have a reason to disable it.
- **Sync Now**: forces an immediate time sync. Only available with WiFi connected.

If the date and clock show "(awaiting time sync)" or "--:--" for more than a minute or two after WiFi connects, tap "Sync Now" — sometimes the first NTP attempt times out and a manual retry fixes it.

## Calibration: when, why, and how

The CO2 sensor will drift a small amount over time, typically reading slightly higher than reality after months of use. To correct this, you "calibrate" by exposing the sensor to fresh outdoor air (which is reliably ~420 ppm worldwide) and telling it to call that reading the truth.

**When to calibrate:**

- Once when you first set up the device, after a day or two of normal use.
- Whenever the on-screen alert appears (every ~90 days).
- If you notice readings that seem implausibly high or low.

**How to calibrate:**

1. Pick a calm day with normal weather. Avoid:
   - Rush-hour times near roads (vehicle exhaust raises local CO2 by 100+ ppm)
   - Crowds or groups of people standing nearby
   - Confined outdoor spaces like courtyards or covered porches (they can trap exhaled air)
2. Take the device outside on USB power and let it run for **at least 5 minutes** in open air, away from people. A small USB power bank works well for this.
3. After 5 minutes, the reading will be stable in the 400-450 ppm range.
4. Tap the CO2 number → Force Recalibrate → read the warning → tap Calibrate.
5. The screen says "Calibrating..." for ~10 seconds, then either "Success" with a delta correction, or "Failed" with a reason.

If calibration fails with "Sensor was not stable enough," wait another 5 minutes outdoors and try again.

After a successful calibration the device returns to the main screen. The "Last cal" age in Settings resets to zero, and any overdue alert disappears.

## Web interface

Once WiFi is set up, the device runs a small web server. From any computer or phone on the same network, visit:

**http://co2monitor-XXXXXX.local/** (where XXXXXX is the unique device suffix shown on the WiFi tile in Settings, or whatever custom name you've set in the web interface)

(If `.local` doesn't work, use the IP address shown in the WiFi tile in Settings.)

The page shows:

- Current readings, color-coded
- 1-hour minimum / maximum / average
- Calibration status alerts
- Live CO2 chart of the last 24 hours
- Download buttons for CSV (full history) and JSON (current reading)
- A link to **Firmware Update** for over-the-air updates (advanced)

The page auto-refreshes every minute.

The CSV download contains all stored samples as `timestamp,CO2 ppm,temp °C,humidity %` rows, suitable for opening in Excel, Numbers, Google Sheets, etc. With the device running 24/7, you'll get about a year of history before old samples start being overwritten.

## Common situations

**Readings won't drop below 600 ppm even with windows open**: probably uncalibrated drift. Take it outside and run a Force Recalibrate.

**Readings spike when you're nearby**: that's working as intended — exhaling near the sensor sends CO2 to 1500+ ppm in seconds. Move it to a spot a meter or so from where you sit.

**Temperature reads 5°F higher than my thermostat**: that's the self-heating effect. Adjust the Temp Off. setting to compensate. The default of +5.7°C / +10.2°F is the empirically-tuned offset for this build; yours may be slightly different.

**Backlight flickers**: usually a cheap USB cable or weak USB port — the brief current spikes during screen redraws cause the 5V rail to dip if the cable can't supply them cleanly. Try a thicker cable or a wall adapter rated for 1A or more. As a workaround, lowering the Brightness setting reduces average current draw and may eliminate flicker on a marginal supply.

**Touch buttons don't respond to taps**: the CYD uses a resistive touchscreen — press firmly with a fingertip rather than swiping or tapping with a fingernail.

**Clock shows `--:-- (no time)` or date shows "(awaiting time sync)"**: NTP hasn't synced yet. The device retries automatically every 30 seconds in the background. If it stays unsynced for more than a few minutes, check that WiFi is connected (Settings → WiFi tile shows an IP). You can also tap the clock → Sync Now to force a sync.

**Forgotten WiFi password / changed networks**: tap the CO2 number → tap the WiFi tile → repeat the captive-portal setup.

## Specifications

| Item | Value |
|---|---|
| CO2 sensor | Sensirion SCD41, NDIR |
| CO2 range | 400 - 5000 ppm (accuracy ±(50 ppm + 5%) of reading) |
| Temperature range | 0 - 50 °C, accuracy ±0.8°C after offset tuning |
| Humidity range | 0 - 100 %RH, accuracy ±6%RH |
| Sample rate | every 5 seconds |
| Logging interval | every 5 minutes |
| Storage capacity | ~120,000 samples (~415 days) |
| Power | 5V via USB, ~150-200 mA |
| Physical | 86 × 50 × 30 mm including stand |

## Things this device deliberately does not do

- **Automatic Self-Calibration (ASC)** is disabled. The SCD41 supports an automatic mode that re-zeroes the sensor based on the lowest reading seen over a multi-day window, assuming that minimum represents fresh outdoor air at 400 ppm. Indoors, this assumption is wrong — your room rarely drops to 400 ppm — and over time ASC will silently shift readings downward by several hundred ppm. This device uses manual Forced Recalibration instead, with a 90-day reminder. (If you genuinely do open windows enough that your indoor minimum reaches outdoor levels, you can re-enable ASC via the serial command `asc on`.)
- **Cloud connectivity**: this device does not phone home, does not have an account, and does not send data to any third-party service. The web interface runs locally on the device itself.
- **Sound or alarms**: there's an audio amp on the board but no buzzer alarm at high CO2 levels. Visual + color-coded indication only.
- **Light mode**: removed. The device only operates in dark mode.
- **Daylight Saving Time**: not auto-handled. Adjust the timezone setting twice a year if you're in a DST-observing region.
