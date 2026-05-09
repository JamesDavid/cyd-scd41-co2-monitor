# Programming the Sensirion SCD41

Practical notes accumulated from building the CO2 monitor. The SCD41 is a great sensor — small, low-power, single I2C device for CO2 + temperature + humidity — but its API has sharp edges that don't match the casual "just call the function" expectation Arduino libraries usually create.

## The chip

NDIR (non-dispersive infrared) CO2 sensor with an integrated SHT4x-class temperature/humidity die. One I2C address (`0x62`), 3.3V supply, ~18 mA peak / ~3 mA average in periodic mode. Range 400-5000 ppm CO2, accuracy ±(50 ppm + 5%) of reading. Reports a fresh sample every 5 seconds in continuous mode.

The SCD40 is the slightly less-accurate sibling: same package, same API, ±(50 ppm + 5%) becomes ±(50 ppm + 5%) above 1000 ppm but ±100 ppm below. Same code works for both.

## Library

We use **Sensirion I2C SCD4x by Sensirion**, version 1.0.0+. Important note for older code:

- v0.4.x: header is `<SensirionI2CScd4x.h>` (capital C), class is `SensirionI2CScd4x`, method names use `getDataReadyFlag`.
- v1.0.0+: header is `<SensirionI2cScd4x.h>` (lowercase c), class is `SensirionI2cScd4x`, method `getDataReadyFlag` was renamed to `getDataReadyStatus`, and `begin()` now takes the I2C address as a second parameter (`SCD41_I2C_ADDR_62`).

The migration is mechanical but easy to miss. Use 1.0.0+ for new code.

## Wiring

Pull-ups already on the breakout board for most modules; if you're connecting bare, add 4.7k or 10k pull-ups on SDA and SCL. **Use 3.3V, not 5V.** The chip is not 5V-tolerant. We use the CYD's "Temp/Humidity" silkscreened JST connector which provides 3V3, GND, GPIO 27 (SDA), and GPIO 22 (SCL).

```cpp
#include <Wire.h>
#include <SensirionI2cScd4x.h>

#define I2C_SDA_PIN 27
#define I2C_SCL_PIN 22

SensirionI2cScd4x scd4x;

Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
scd4x.begin(Wire, SCD41_I2C_ADDR_62);
```

## The 500ms-after-stop rule (THE big gotcha)

This is the single most important thing in this document.

**After `stopPeriodicMeasurement()`, you MUST wait 500 ms before issuing any other command.** The datasheet says so but examples in the wild often skip it. If you don't wait, the next command (e.g., `setSensorAltitude`, `setTemperatureOffset`, `performForcedRecalibration`, `setAutomaticSelfCalibrationEnabled`) will silently fail. The function returns success, but the chip ignores the write.

Symptom: you change a setting through your UI, your shadow variable updates, your NVS persists, but the chip behaves as if nothing changed. Hard to spot because everything *looks* like it worked.

The right pattern:

```cpp
scd4x.stopPeriodicMeasurement();
delay(500);                              // <-- THIS LINE IS NOT OPTIONAL
int16_t err = scd4x.setSensorAltitude(myAltitude);
if (err) { /* handle error */ }
scd4x.startPeriodicMeasurement();
```

In our code we wrap this with a helper template so it's hard to forget:

```cpp
template <typename Fn>
int16_t withChipIdle(Fn fn) {
  scd4x.stopPeriodicMeasurement();
  delay(500);
  int16_t err = fn();
  scd4x.startPeriodicMeasurement();
  return err;
}

// Usage:
int16_t err = withChipIdle([]() {
  return scd4x.setSensorAltitude(altitude);
});
```

The 500 ms is per the datasheet's "stop_periodic_measurement" command timing — the sensor needs that long to actually stop sampling and enter idle mode before it'll accept config writes.

## Configuration commands need idle mode

These all require the "stop periodic measurement, wait 500 ms" dance:

- `setSensorAltitude` / `getSensorAltitude`
- `setTemperatureOffset` / `getTemperatureOffset`
- `setAutomaticSelfCalibrationEnabled` / `getAutomaticSelfCalibrationEnabled`
- `performForcedRecalibration`
- `persistSettings`
- `factoryReset`
- `getSerialNumber`

These can be called during periodic measurement (no stop needed):

- `getDataReadyStatus` (v1.0.0+) / `getDataReadyFlag` (v0.4.x)
- `readMeasurement`
- `setAmbientPressure` (this is the runtime-friendly alternative to `setSensorAltitude`)
- `measureSingleShot` and `measureSingleShotRhtOnly` (single-shot mode is a separate workflow)

If you have a real pressure sensor, prefer `setAmbientPressure` over altitude. It's more accurate and doesn't require entering idle mode.

## Auto-Calibration: a trap for indoor monitors

ASC (Automatic Self-Calibration) is enabled by default. The sensor watches your readings over a multi-day window and assumes the lowest sustained value equals fresh outdoor air at 400 ppm. It then re-zeros the calibration to make that minimum read 400.

**This is wrong for an indoor desk monitor.** Indoor spaces rarely drop to outdoor CO2 levels. Over weeks, ASC keeps re-zeroing downward, and your readings drift 100-300 ppm low. We've seen units settling into the 200-300 ppm range — physically impossible outdoors, but the sensor doesn't know that.

When ASC is appropriate:
- Devices that genuinely see outdoor air daily (HVAC sensors near intakes, outdoor stations).
- Greenhouses with regular ventilation cycles.
- Battery-conscious portable devices where users go outside often.

When ASC should be disabled:
- Indoor desk monitors (this build).
- Closed offices, sealed rooms, server closets.
- Anywhere the lowest-CO2 minute of any given day is still well above 400 ppm.

We disable ASC at every boot and use **Forced Recalibration (FRC)** outdoors instead, with a 90-day reminder.

```cpp
// At boot, in your sensor init:
withChipIdle([]() {
  return scd4x.setAutomaticSelfCalibrationEnabled(0);
});
```

Importantly, the SCD41 stores ASC state in its own EEPROM (persists across power cycles independently of your code). So if a previous firmware version had ASC on, the chip remembers that. Always write 0 explicitly at boot if you want it off, regardless of what NVS or your shadow variable says.

## Forced Recalibration (FRC)

FRC overwrites the chip's internal calibration baseline. You expose the sensor to a known reference concentration, tell the chip "this reading is exactly N ppm," and it computes a correction.

Standard outdoor reference is **420 ppm** as of the mid-2020s. Don't use 400 — that's outdated.

```cpp
scd4x.stopPeriodicMeasurement();
delay(500);

uint16_t correction = 0;
int16_t err = scd4x.performForcedRecalibration(420, correction);

scd4x.startPeriodicMeasurement();

if (err) {
  // Communication error
} else if (correction == 0xFFFF) {
  // Sensor wasn't stable enough; try again after letting it settle
} else {
  // Success; (correction - 0x8000) is the delta in ppm
  int16_t deltaPpm = (int16_t)(correction - 0x8000);
}
```

The `0xFFFF` return is critical — it means the sensor refused to calibrate because the reading wasn't stable for the required window (~3 minutes). If you ignore this and treat it as success, you've done nothing while thinking you've calibrated.

**FRC procedure that actually works:**

1. Take the device to a calm outdoor location, away from people, vehicles, building exhaust.
2. Power it from a USB battery bank.
3. Let it run for **at least 5 minutes** in the new environment. This is for two things: the chip needs to thermally settle (it self-heats), and the gas in the sensor cavity needs to equilibrate with outdoor air.
4. Verify the reading has stabilized (changes of less than 5-10 ppm minute-over-minute).
5. Then issue FRC.

Common failure mode: doing FRC indoors after just walking outside. The reading hasn't equilibrated yet, FRC succeeds with whatever the indoor-tainted reading was, and now your calibration is *worse* than before.

## Temperature offset

The chip self-heats — typically reads 4-6°C above ambient when mounted on a PCB inside an enclosure. The temp offset setting subtracts a fixed value from the raw temperature reading before reporting it. **It only affects the displayed temperature, not the CO2 calculation** (the chip uses raw temperature internally for IR-absorption compensation, which is correct because the raw value is the chip's actual temperature, which equals the temperature of the gas inside the sensor cavity).

Tuning the offset:

1. Run the device in its final mounting/enclosure for at least 1 hour to thermally settle.
2. Compare its temperature reading against a separate, known-accurate thermometer placed nearby.
3. Set the offset to (chip-reading) − (true-temperature). Default 4.0°C. Mounting position matters — sensor in free air on the rear of a stand reads lower offset; sensor sandwiched inside an enclosure with WiFi heat reads higher.

For this build, +5.7°C / +10.2°F is the empirically-tuned offset.

## Altitude / pressure compensation

Atmospheric pressure varies with altitude, and CO2 ppm depends on molecule count per volume. The chip can compensate if you tell it your altitude or current barometric pressure. The effect is small (~1-2% per 100m altitude error), so it matters more in Denver than Boston.

```cpp
withChipIdle([]() {
  return scd4x.setSensorAltitude(340);  // meters above sea level
});
```

Altitude can only be changed in idle mode. If you have a real barometer, `setAmbientPressure(uint16_t pressure_pa_div_100)` accepts pressure in hPa and works during periodic measurement.

## Reading data

```cpp
bool ready = false;
int16_t err = scd4x.getDataReadyStatus(ready);
if (err || !ready) return;       // call this before readMeasurement

uint16_t co2;
float t_celsius, rh_percent;
err = scd4x.readMeasurement(co2, t_celsius, rh_percent);
if (err) { /* handle */ }
if (co2 == 0) {
  // Some library versions / firmware revisions return 0 in the
  // first 1-2 readings after wakeUp(). Just skip until non-zero.
  return;
}
```

Sample interval in periodic mode is 5 seconds. Polling more often than that just returns the same sample with `ready` false — wasteful but harmless. Polling exactly every 5 seconds drifts in/out of phase with the chip's internal clock; we found polling every 5 seconds with a `getDataReadyStatus` check works reliably in practice.

## State persistence

The SCD41 persists these in its own EEPROM across power cycles:

- ASC enabled/disabled
- Sensor altitude
- Temperature offset
- The internal calibration baseline (modified by FRC and ASC)

But there's a wrinkle: changes you make are kept in volatile RAM until you call `persistSettings()` (separate command). The library may or may not call this for you depending on which method. Best practice: call `persistSettings()` once after you've tuned everything, or never (and just re-apply settings on every boot from your own NVS-saved values, which is what we do — simpler and avoids excess EEPROM writes).

## Single-shot mode

Useful for battery applications. The chip wakes, takes one sample, returns to sleep. Drawback: ~5 second blocking call per sample, ~3 seconds of which is settling time. We use periodic mode because the device is USB-powered and wants live readings.

```cpp
scd4x.measureSingleShot();   // takes ~5 seconds
delay(5000);
scd4x.readMeasurement(co2, t, rh);
```

There's also `measureSingleShotRhtOnly()` which skips the CO2 measurement entirely (for sub-millisecond temp/humidity reads).

## Common error codes

The library returns a non-zero `int16_t` on failure. Useful ones:

- `errorToString(err, buf, len)` converts to a human string.
- I2C NACK errors usually mean: chip not yet awake (need `wakeUp()` first), no pull-ups, wrong bus pins, wrong address, or 5V applied (chip dead).
- "Frame received with bad CRC" usually means SDA/SCL got crosstalk on a long wire or you forgot pull-ups.

The chip needs ~30ms after `wakeUp()` before responding. Our init sequence:

```cpp
scd4x.begin(Wire, SCD41_I2C_ADDR_62);
scd4x.wakeUp();
delay(30);                          // wakeUp settling time
scd4x.stopPeriodicMeasurement();    // in case it was still running from a previous session
delay(500);                          // mandatory after stop
// ... config writes ...
scd4x.startPeriodicMeasurement();
```

## Verifying chip state

When debugging, never trust your shadow variables. Read back from the chip:

```cpp
withChipIdle([]() {
  uint16_t asc, alt;
  float toff;
  scd4x.getAutomaticSelfCalibrationEnabled(asc);
  scd4x.getSensorAltitude(alt);
  scd4x.getTemperatureOffset(toff);
  Serial.printf("Chip: ASC=%d alt=%u toff=%.1fC\n", asc, alt, toff);
  return 0;
});
```

If your code thinks ASC is off but the chip says on, you've got a write that silently failed (probably the 500ms issue). Our `info` serial command does this readback and flags mismatches.

## What works well

- Periodic mode at 5-second intervals is rock-solid. Years of continuous use without lockups.
- `setAmbientPressure` is the right move for any device with a barometer; works during measurement.
- The chip's self-heating is consistent enough that a fixed temp offset is sufficient — no need for dynamic compensation.
- FRC is forgiving once you get the procedure right; the chip clearly tells you when its conditions aren't met (the 0xFFFF return).

## What to be wary of

- **The 500ms post-stop delay** is non-negotiable. Skipping it fails silently.
- ASC will silently degrade your calibration over weeks if your environment isn't right for it. Default to off for indoor monitors.
- The chip stores state in its own EEPROM. Code changes alone don't reset it; either explicitly write the settings you want at every boot, or do a `factoryReset` if you suspect it's wedged.
- Library API changed at 1.0.0 (header rename, method rename, begin() signature). Match your code to your installed version.
- After `wakeUp()`, give the chip 30ms before issuing commands.
- `co2 == 0` returned from `readMeasurement` can happen briefly after wake; just skip and try again.
- `correction == 0xFFFF` from FRC means it failed silently — don't treat that as success.
