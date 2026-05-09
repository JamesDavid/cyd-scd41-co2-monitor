# Programming the CYD (ESP32-2432S028R)

Practical notes accumulated from building the CO2 monitor on this board. The CYD is cheap and works well, but it's a hodgepodge of Chinese clone variants with peripheral wiring choices that fight several common Arduino libraries. This file is a cheat-sheet for the next project on this board.

## The board

ESP32-WROOM-32 (4MB flash, 240MHz dual-core) glued to a 2.8" 240x320 ILI9341 LCD with XPT2046 resistive touch, plus an SD card slot, an LDR, an RGB LED, an audio amp, and several JST connectors for I/O. Sold as "ESP32-2432S028R" or simply "Cheap Yellow Display."

There are at least two hardware revisions in the wild:

- **Single-USB (older, micro-USB only)**: colors display normally without inversion.
- **Dual-USB (newer, USB-C + micro-USB)**: ILI9341 is wired with reversed RGB polarity, so colors display inverted unless you call `tft.invertDisplay(true)`. Other minor pinout differences exist on the connectors.

There is also a "S028" (no R) variant with a different touch IC. The notes here apply to the resistive `R` variants.

## Pin reference (the ones you will actually use)

```
Display (ILI9341, HSPI on the WROOM):
  TFT_MISO   GPIO 12
  TFT_MOSI   GPIO 13
  TFT_SCLK   GPIO 14
  TFT_CS     GPIO 15
  TFT_DC     GPIO  2
  TFT_RST    -1   (tied to system reset)
  TFT_BL     GPIO 21    backlight, active HIGH

Touch (XPT2046, separate VSPI):
  T_IRQ      GPIO 36    input only, no pullup
  T_DIN      GPIO 32
  T_OUT      GPIO 39    input only
  T_CLK      GPIO 25
  T_CS       GPIO 33

Onboard RGB LED (active LOW):
  RED        GPIO  4
  GREEN      GPIO 16
  BLUE       GPIO 17

LDR (light sensor, ADC):
  LDR        GPIO 34    input only

SD card (HSPI, shared with display):
  SS         GPIO  5
  SCK        GPIO 18
  MOSI       GPIO 23
  MISO       GPIO 19

Speaker amp:
  Audio out  GPIO 26    (DAC2)

Boot/serial:
  USB serial via CH340 (older) or CP2102 (newer)
```

### Connectors for external I/O

The CYD has multiple JST 1.25mm connectors on the bottom edge with different pinouts. Identify by the silkscreen labels.

**P3 / "Temp humidity" connector** (the one you want for I2C):
```
GND - 3V3 - GPIO 27 - GPIO 22
```
This is the connector with `3V3` printed on it. GPIO 27 and 22 are completely free, so they're routed here for I2C use. Note the chip's hardware-default I2C pins are GPIO 21 (SDA) and 22 (SCL), but GPIO 21 is hijacked for the backlight, so the CYD designers picked 27 instead and you tell `Wire.begin(27, 22)` in software.

**Extended IO connector**:
```
GND - GPIO 35 - GPIO 22 - GPIO 21
```
Don't confuse this one with the I2C connector. GPIO 35 is input-only and GPIO 21 is the backlight.

**Speaker connector**: 2-pin for an external speaker driven by the audio amp.

**SD card slot**: works as a normal SD card filesystem if you initialize it with `SD.begin(5, ...)` on the HSPI bus. Be aware it shares SCK/MOSI/MISO with the display, so if you hammer the SD card you may see display glitches.

## TFT_eSPI configuration

TFT_eSPI is the de-facto LCD library for this kind of board, but it's configured globally via a single header in the library folder, not per-sketch. Paste this into `Documents/Arduino/libraries/TFT_eSPI/User_Setup.h`, replacing whatever's there:

```cpp
#define ILI9341_2_DRIVER     // try ILI9341_DRIVER if colors look off
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1
#define TFT_BL   21
#define TFT_BACKLIGHT_ON HIGH

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define SMOOTH_FONT

#define SPI_FREQUENCY 55000000
```

The two driver constants matter. `ILI9341_2_DRIVER` has slightly different gamma/init compared to `ILI9341_DRIVER` and one variant of the CYD wants one, the other wants the other. Try one; if colors look weird, swap.

After the driver is right, `tft.invertDisplay(true)` may still be needed on the dual-USB variant. Make this a compile-time flag in the sketch.

## Touch quirks (read this twice)

The XPT2046's raw axis orientation does **not** follow the display rotation. When you call `tft.setRotation(0)` for portrait mode, the display's `(x, y)` coordinates are remapped, but `TS_Point::x` and `TS_Point::y` from `XPT2046_Touchscreen::getPoint()` keep returning chip-native values.

For portrait (`setRotation(0)`) on this board, empirically:

- Touch chip's `p.x` increases as you tap **down** the screen (so `p.x` ≈ display `y`).
- Touch chip's `p.y` increases as you tap **right-to-left** (so `p.y` ≈ inverted display `x`).

The code handles this with a swap and an inverted X mapping. If a new sketch needs touch, the right move is to:

1. Add a debug log of every `getPoint()` result.
2. Tap each screen corner.
3. Set the four calibration constants based on what you see.

```cpp
TS_Point p = ts.getPoint();
// Swap and remap:
int16_t rx = TS_SWAP_XY ? p.y : p.x;
int16_t ry = TS_SWAP_XY ? p.x : p.y;
sx = map(rx, TS_X_MIN, TS_X_MAX, 0, SCREEN_W);
sy = map(ry, TS_Y_MIN, TS_Y_MAX, 0, SCREEN_H);
```

`map()` handles inverted ranges fine: setting `TS_X_MIN > TS_X_MAX` reverses the axis without extra logic.

The library's default `Z_THRESHOLD` of 400 is too high for many CYD panels — light taps don't register. Read `p.z` directly and use a lower threshold like 50.

The XPT2046 lives on its own SPI bus (VSPI) separate from the display (HSPI). Initialize it explicitly:

```cpp
SPIClass touchSPI(VSPI);
touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
ts.begin(touchSPI);
```

## Backlight

GPIO 21, active high. `digitalWrite(21, HIGH)` turns it on; `LOW` turns it off.

PWM dimming via the LEDC peripheral works, but is finicky in a way that took empirical testing to figure out. **Counter-intuitively, you want a *low* PWM frequency, not a high one.** The on-board MOSFET driver has a slow turn-off characteristic: at frequencies above about 10 kHz the gate doesn't have time to discharge between cycles, so the MOSFET is effectively always on regardless of duty cycle. Symptom: changing the duty cycle has no visible effect on brightness, even though `ledcWrite()` is being called correctly.

Working configuration on this build:

```cpp
#define BACKLIGHT_LEDC_FREQ  5000      // Hz - 5 kHz works
#define BACKLIGHT_LEDC_BITS  8         // 0-255 duty range

// Core 2.x:
ledcSetup(0, BACKLIGHT_LEDC_FREQ, BACKLIGHT_LEDC_BITS);
ledcAttachPin(21, 0);
ledcWrite(0, duty);

// Core 3.x:
ledcAttach(21, BACKLIGHT_LEDC_FREQ, BACKLIGHT_LEDC_BITS);
ledcWrite(21, duty);
```

5 kHz gives the MOSFET ~100 µs to turn off per cycle, plenty of margin, and is well above the human-visible-flicker threshold (~80 Hz) so dimming looks smooth. If 5 kHz still doesn't dim on a particular board, drop to 2 kHz or 1 kHz — those are still flicker-free to the eye but give the MOSFET even more switch-off time.

If you observe **continuous backlight flicker even at full brightness**, that's *not* the PWM issue — it's the USB power supply. The WROOM module + backlight pulls ~150-200 mA steady with transient current spikes during display updates. A thin USB cable from a laptop port can't hold the 5V rail steady through the spikes, and the brief sag dims the backlight perceptibly. Try a wall adapter or a thicker cable before suspecting the hardware. A 100-470 µF cap across the board's 5V rail also fixes it.

Some CYD revisions reportedly have a hardware pull-up on the backlight pin that overrides PWM control entirely. If duty cycle changes have no visible effect even at 1 kHz, you're probably on one of those boards and brightness control just isn't possible without removing that pull-up.

## LDR (ambient light sensor)

The component on the front of the PCB silkscreened **R21** is *not* a regular resistor — it's a GT36516 photoconductive cell. Sunton's schematic numbers all two-terminal passive components in a single R-series, regardless of whether they're fixed resistors or photoresistors, which is confusing.

**Physical location**: looking at the front of the board with the USB ports on the right short edge (landscape, screen right-side-up), the LDR sits near the **left short edge, close to the bottom long edge** — roughly 8-10 mm in from the left short edge, 6-8 mm up from the bottom long edge. It is *next to* the active display, not above or below it.

Wiring on the v2 / v3 boards:

```
 3.3V ----[ R15 ]----+---- GPIO 34 (ADC1_CH6, input-only)
                     |
                     +----[ R19 ]----+---- GND
                                     |
                              [ LDR (R21) ]----GND
```

R15 (pull-up to 3.3V) and R19 (pull-down to GND, in parallel with the LDR) form a voltage divider against the LDR's variable resistance. On older boards both were 1MΩ; on 2024+ revisions they're around 10kΩ, which gives a more useful range on the ADC because the LDR's typical illuminated resistance is in the 5-10kΩ neighborhood.

LDR resistance behavior:
- **Bright daylight**: ~5kΩ
- **Normal indoor lighting**: 10-30kΩ
- **Dim room**: 50-200kΩ
- **Pitch dark** (after ~10s for the cell to relax): >300kΩ

Because the LDR is on the *low* side of the divider, GPIO 34 reads **high in the dark** and **low in bright light**:

```cpp
int raw = analogRead(34);   // 0..4095
// On the test unit, raw ≈ 200 in bright light, ≈ 3500 in pitch dark.
// Calibrate your specific board with `info` debug output once.
```

**Auto-brightness implementation**: We use the LDR to drive backlight PWM duty cycle as an "Auto" mode option. Workflow: read `analogRead(34)` every 2 seconds, map raw → 0-100% via the calibrated dark/bright endpoints, exponentially smooth (alpha 0.15) to absorb transient shadows, then map the smoothed value to a PWM duty cycle between 15% (very dim) and 100% (full bright). Smoothing is essential — without it, a passing hand or a flickering desk lamp causes constant backlight changes that are themselves distracting.

Calibration tip: enable Auto mode, run the `info` serial command in different lighting conditions, note the raw ADC values, and tune `LDR_DARK_RAW` / `LDR_BRIGHT_RAW` in the firmware.

**Caveats**:
- GPIO 34 is input-only (no pull-ups can be enabled, no output mode). Fine for ADC use, but you can't repurpose it for output if you decide you don't want the LDR.
- ADC1 readings on the ESP32 are noisy and have a non-linear response curve (especially below 100mV). For a 0-100 brightness mapping this doesn't matter, but if you wanted to log "lux" you'd need a real light sensor.
- The LDR has **slow recovery from dark** — it can take 10+ seconds to climb its resistance after going from a lit room to total darkness. The smoothing in firmware happens to mask this, but be aware.
- Older boards with 1MΩ dividers will saturate the ADC at the high end (reading ~4095 / dark) much more easily, so the usable dynamic range is smaller. The 2024+ 10kΩ divider is much better.
- **Backlight bleed contaminates the LDR reading.** This is the big one. The LDR sits very close to the LCD module, and without optical isolation it picks up sideways-leaking backlight light instead of room light. Symptom: in a pitch-dark room with no bezel hole at all, the LDR still reads as if the room is brightly lit (we measured 79-82%). In bright rooms the room light dominates so the problem is hidden, but in dim conditions you get a positive feedback loop where the LDR sees mostly its own backlight. Mitigations: a printed shroud around the LDR inside the case (the SCAD file has this as `ldr_shroud = true`), a black tape patch over the LDR with a pinhole through it for the bezel hole alignment, or printing the case in opaque black filament so the bezel itself doesn't conduct light from the LCD edge.

**Physical mounting**: if you put the CYD in an enclosure, leave a small opening (3-4mm diameter) over the LDR position. Note that the LDR is *not* in the middle of the bezel — it's offset to one corner of the board near the short edge opposite the USB ports. Don't make a centered hole expecting it to align.

**Critical: light isolation from the LCD backlight.** The LDR sits directly adjacent to the LCD module on the same side of the PCB. With the case closed and the backlight on, edge-light from the LCD leaks across the case interior and reaches the LDR — enough that a pitch-dark room reads ~80% on the LDR scale (i.e., "well-lit"). Auto-brightness becomes useless because the LDR mostly sees the backlight, not the room.

The fix is a small opaque shroud — a square or round tube on the inside of the front bezel that protrudes down to within ~1 mm of the LDR surface, blocking light from the LCD-side from reaching the sensor while leaving the front-facing optical path through the bezel hole clean. Our 3D-printed stand has this built in (parameters `ldr_shroud`, `ldr_shroud_drop`, etc. in the SCAD file). Without the shroud, auto-brightness simply does not work in a closed case. Black foam tape between the LCD and LDR is a reasonable hand-built alternative.

## RGB LED gotcha

GPIO 4, 16, 17 drive the on-board RGB LED, active LOW. They float low at boot, which makes the red LED come on dimly until something explicitly drives them HIGH. Always do this in `setup()`:

```cpp
pinMode(4, OUTPUT);  digitalWrite(4, HIGH);
pinMode(16, OUTPUT); digitalWrite(16, HIGH);
pinMode(17, OUTPUT); digitalWrite(17, HIGH);
```

These pins are otherwise free for repurposing if you don't want the LED.

## Mounting holes

Per DIYmall datasheet, the four mounting holes are inset 4.0mm from each PCB edge. Centre-to-centre pitch is therefore 78mm × 42mm. Holes accept M3 self-tapping screws cleanly into PLA/PETG with a 2.5mm pilot.

## Common build errors

**"WiFi.h: multiple libraries found"** — harmless. Arduino IDE picks the right one (the ESP32 core's). The Windows-store install of Arduino IDE bundles a stub `WiFi.h` that gets shadowed.

**"No DFU capable USB device available"** at upload time — wrong board selected. The CYD wants `Tools → Board → ESP32 Arduino → ESP32 Dev Module`, not the Arduino-branded "Nano ESP32" board.

**Upload hangs at "Connecting......_____......"** — auto-reset isn't pulsing the BOOT pin. Hold the BOOT button on the back of the board, then click upload, release BOOT once you see "Writing at 0x..." on the IDE console.

**Sensirion library `SensirionI2CScd4x.h: no such file`** — version 1.0.0 of the library renamed the header (lowercase 'c'). Either install 1.0.0+ and use `<SensirionI2cScd4x.h>` and the new class name, or roll back to 0.4.0 in Library Manager.

## What works well

- TFT_eSPI is fast and the font rendering is clean. Font 6 is a tall numeric-only font perfect for clocks. Font 7 is the gigantic 7-segment-style font for big readouts. Font 2 and 4 are normal proportional fonts.
- Drawing partial rectangles for redraw-only-what-changed is essential; you can't `fillScreen()` and rebuild the whole UI on every frame without visible flicker.
- `LittleFS` is a far better choice than `Preferences` for any data set bigger than a few hundred bytes. NVS gets slow and wasteful past that scale.
- WiFiManager (tzapu/tablatronix) handles the captive portal correctly and persists credentials in the system NVS, separate from your app's Preferences namespace.
- ElegantOTA gives you `/update` for free with maybe 3 lines of integration code. Worth it.

## What to be wary of

- The CYD's documentation is wiki-quality at best and varies between sellers. Trust the silkscreen labels on connectors over anything else.
- Don't use 5V on the I2C connector. The 3V3 pin on the connector is your supply for sensors.
- Touch coordinates do **not** auto-rotate with the display.
- Backlight PWM at "normal" frequencies (10+ kHz) does nothing visible because the on-board MOSFET can't switch off fast enough. Use 5 kHz or lower — counter-intuitive but real. See the Backlight section.
- The LDR (light sensor) above the screen is silkscreened **R21**. Sunton labels it as if it were a regular resistor; it's actually a photoresistor on GPIO 34. Easy to miss when reading the schematic.
- The LDR is mounted right next to the LCD module. Without an internal light shroud, LCD backlight leakage drowns out room light at the sensor and auto-brightness reads "lit" even in a pitch-dark case. See the LDR section.
- ASC on the SCD41 will silently degrade your readings if the device is indoors. Disable it explicitly.
- The graph on a 240x320 portrait screen has limited vertical room. Keep it short and use color rather than fine resolution to convey range.
