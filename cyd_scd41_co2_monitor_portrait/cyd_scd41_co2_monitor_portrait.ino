/*
 * CO2 Air Quality Monitor - PORTRAIT layout (240 x 320)
 *   Board:  ESP32-2432S028R ("CYD" - Cheap Yellow Display)
 *   Sensor: Sensirion SCD41 (I2C, address 0x62)
 *
 * UI (touch):
 *   - Top-left "CAL" button   -> Settings
 *   - Top-right sun/moon icon -> dark / light theme
 *   - Tap the clock           -> Time Settings (NTP, timezone, 12/24h)
 *   - Tap the temperature     -> toggle F <-> C
 *
 * Long-term logging:
 *   Samples (CO2 + temp + RH + UTC timestamp) are written to LittleFS
 *   every 5 minutes. Capacity is ~120,000 samples (~400 days). Ring-
 *   buffer overwrites oldest when full. Download via web /history.csv.
 *
 * Web (when WiFi connected):
 *   /              status page with live chart (Chart.js)
 *   /data.json     current readings
 *   /history.json  full sample history
 *   /history.csv   CSV download
 *   /update        ElegantOTA firmware update page
 *
 *   Reachable at http://co2monitor.local/ (mDNS) or by IP.
 *
 * Wiring (SCD41 -> CYD I2C connector with 3V3 silkscreen):
 *   VDD -> 3.3V    SDA -> GPIO 27
 *   GND -> GND     SCL -> GPIO 22
 *
 * Required libraries:
 *   - TFT_eSPI                 by Bodmer
 *   - XPT2046_Touchscreen      by Paul Stoffregen
 *   - Sensirion I2C SCD4x      by Sensirion (v1.0.0+)
 *   - Sensirion Core           (auto-installed dependency)
 *   - WiFiManager              by tzapu (tablatronix)  v2.0.17+
 *   - ElegantOTA               by Ayush Sharma         v3.x
 *
 * Serial commands (115200 baud):
 *   wifi-setup     start the WiFi config portal
 *   wifi-reset     forget WiFi credentials
 *   wifi-status    print connection state and IP
 *   frc <ppm>      manual Forced Recalibration
 *   asc on / off   toggle Automatic Self-Calibration (default OFF)
 *   info           print all current settings
 *   reset          clear NVS preferences
 *   erase-log      wipe LittleFS sample log
 */

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <ElegantOTA.h>
#include <LittleFS.h>
#include <time.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <SensirionI2cScd4x.h>

// ============================================================
//                     CONFIGURATION
// ============================================================

// ----------------- Pins -----------------
#define I2C_SDA_PIN     27
#define I2C_SCL_PIN     22
#define BACKLIGHT_PIN   21

#define XPT2046_IRQ     36
#define XPT2046_MOSI    32
#define XPT2046_MISO    39
#define XPT2046_CLK     25
#define XPT2046_CS      33

#define LED_R           4    // active LOW
#define LED_G           16
#define LED_B           17

// ----------------- Behaviour -----------------
#define READ_INTERVAL_MS    5000UL
#define SAMPLE_INTERVAL_MS  (5UL * 60UL * 1000UL)
#define UPTIME_SAVE_MS      (60UL * 60UL * 1000UL)
#define CLOCK_REDRAW_MS     1000UL
#define MAX_SAMPLES         60          // in-RAM ring (used for graph)
#define TAP_DEBOUNCE_MS     250
#define WIFI_CONNECT_MS     10000UL
#define WIFI_PORTAL_TIMEOUT 180
#define WIFI_AP_NAME        "CO2-Monitor-Setup"
#define MDNS_HOSTNAME       "co2monitor"
#define FRC_REMINDER_DAYS   90
#define FRC_TARGET_PPM      420

// SCD41 first-boot defaults
#define DEFAULT_ASC          false
#define DEFAULT_ALTITUDE_M   340       // Tempe AZ
#define DEFAULT_TEMP_OFFSET  5.7f      // tuned empirically on this build
#define DEFAULT_TEMP_F       true

// Backlight PWM. The CYD's on-board MOSFET has a slow turn-off time;
// at high PWM frequencies (>10 kHz) it never fully switches off
// between cycles and the duty cycle has no visible effect. 5 kHz
// gives the MOSFET ~100 us to turn off, which is enough margin on
// every CYD revision tested. Still well above human flicker threshold
// (~80 Hz) so dimming looks smooth.
#define BACKLIGHT_LEDC_CHAN     0
#define BACKLIGHT_LEDC_FREQ     5000
#define BACKLIGHT_LEDC_BITS     8
#define BACKLIGHT_LEDC_MAX      255
#define DEFAULT_BRIGHTNESS_PCT  100

// Time settings defaults
#define DEFAULT_TZ_OFFSET_MIN   (-7 * 60)    // UTC-7 (MST/Arizona, no DST)
#define DEFAULT_USE_24H         false
#define DEFAULT_NTP_SERVER      "pool.ntp.org"

// Setting ranges
#define ALT_MIN              0
#define ALT_MAX              5000          // covers all populated areas
#define ALT_STEP             10
#define TEMP_OFF_MIN         -5.0f
#define TEMP_OFF_MAX         15.0f
#define TEMP_OFF_STEP_C      0.5f
#define TEMP_OFF_STEP_F_IN_C (5.0f / 9.0f)
#define TZ_MIN_OFFSET        (-12 * 60)
#define TZ_MAX_OFFSET        (+14 * 60)
#define TZ_STEP_MIN          15

// CO2 thresholds (good -> bad: green / blue / orange / red)
#define CO2_GOOD_MAX        800
#define CO2_MODERATE_MAX    1200
#define CO2_POOR_MAX        2000

// Chart axes auto-fit to the data, but never shrink below this
// floor/ceiling. 400 ppm is roughly outdoor baseline; 2000 ppm
// covers the common "very poor" range. If samples ever exceed
// these we expand the axis (rounded to the nearest 200 ppm) so
// nothing gets clipped against the top or bottom of the graph.
#define CO2_CHART_DEFAULT_MIN   400
#define CO2_CHART_DEFAULT_MAX   2000

// Trend arrow needs at least this many samples and this much delta.
// At 5 min/sample, MIN=2 means the arrow appears after 10 minutes
// of uptime. The arrow always shows once we have enough samples;
// when the trend is flat (within DELTA) we draw a horizontal arrow.
#define TREND_MIN_SAMPLES   2
#define TREND_DELTA_PPM     15

// Flip if green looks magenta or red looks cyan
#define INVERT_DISPLAY_COLORS true

// LittleFS log
#define LOG_PATH            "/co2log.bin"
#define LOG_MAGIC           0x4332474F    // "CO2L"
#define LOG_VERSION         1
#define LOG_MAX_RECORDS     120000UL      // ~415 days at 5 min/sample

// ----------------- Touch calibration -----------------
// Calibrated from corner taps. The CYD's touch chip axes are swapped
// vs the display, and one axis is inverted.
//   top-left  -> (0, 0)
//   top-right -> (240, 0)
//   bot-left  -> (0, 320)
//   bot-right -> (240, 320)
#define TOUCH_PRESSURE_MIN   50
#define TS_X_MIN             3700   // post-swap raw at LEFT edge (high)
#define TS_X_MAX             350    // post-swap raw at RIGHT edge (low)
#define TS_Y_MIN             300    // post-swap raw at TOP edge
#define TS_Y_MAX             3700   // post-swap raw at BOTTOM edge
#define TS_SWAP_XY           1

// ============================================================
//                      LAYOUT
// ============================================================

#define SCREEN_W        240
#define SCREEN_H        320

// All "upper" sections (date through TEMP/RH values) use a uniform
// 8 px gap between them. The bottom (caption + graph) is unchanged
// from earlier so the graph still reaches y=307 with a 12 px
// reserve below for the cal-overdue alert.
//
// Layout summary (visible glyph spans, 8 px gaps):
//   y=  4..19   date          (Font 2, 16 px)
//   y= 28..63   clock         (Font 6, 36 px)
//   y= 72..87   "CO2 (ppm)"   (Font 2)
//   y= 96..143  CO2 value     (Font 7, 48 px)
//   y=152..177  status banner (Font 4, 26 px)
//   y=186..201  TEMP / RH     (Font 2)
//   y=210..235  temp/RH vals  (Font 4)
//   ---- bottom area, unchanged from previous version ----
//   y=242..249  graph caption (Font 1)
//   y=256..307  graph
//   y=308..319  cal-overdue alert area
#define DATE_Y          4
#define DATE_H          16
#define CLOCK_Y         28
#define CLOCK_H         36               // Font 6 visible height; redraw fillRect
                                         // ends at y=63 so it can't bleed into label

#define CO2_LABEL_Y     80               // MC datum: label centered here
#define CO2_VALUE_Y     96               // TC datum: top of big number
#define STATUS_Y        152              // TC datum

#define TR_TOP          186              // TC datum: TEMP / RH small labels
#define TR_VALUE_Y      210              // TC datum: temperature / RH values

#define GRAPH_X         30
#define GRAPH_CAPTION_Y 242
#define GRAPH_Y         256
#define GRAPH_W         205
#define GRAPH_H         52               // bottom 12px reserved for cal-overdue alert

// Settings screen
#define SET_HEADER_H    32
#define SET_ROW_H       60
#define SET_ROW1_Y      40
#define SET_ROW2_Y      104
#define SET_LASTCAL_Y   172
#define SET_FRC_Y       196
#define SET_FRC_H       50
#define SET_WIFI_Y      258
#define SET_WIFI_H      54

// Time settings screen
#define TS_HEADER_H     32
#define TS_ROW1_Y       44
#define TS_ROW2_Y       108
#define TS_ROW3_Y       172
#define TS_NTP_Y        236
#define TS_BTN_Y        272

// ============================================================
//                       COLORS
// ============================================================
// Dark theme only. Light mode was removed for simplicity and
// because it always looked worse on this LCD.
#define COLOR_BG          TFT_BLACK
#define COLOR_TEXT        TFT_WHITE
#define COLOR_TEXT_DIM    TFT_LIGHTGREY
#define COLOR_TITLE_BG    TFT_NAVY
#define COLOR_TITLE_TEXT  TFT_WHITE
#define COLOR_GRAPH_AXIS  TFT_WHITE
#define COLOR_BTN_BG      0x10A2
#define COLOR_BTN_TEXT    TFT_WHITE
#define COLOR_DIALOG_BG   0x18C3
#define COLOR_ACCENT      TFT_CYAN

// ============================================================
//                      RECTS
// ============================================================

struct Rect {
  int16_t x, y, w, h;
  bool contains(int16_t px, int16_t py) const {
    return px >= x && px < x + w && py >= y && py < y + h;
  }
};

// Main screen
const Rect btnDate      = {0, 0, SCREEN_W, DATE_H + 6};                    // (no action; reserved for future)
const Rect btnClock     = {0, CLOCK_Y - 4, SCREEN_W, CLOCK_H + 8};
// Tap on the CO2 number opens Settings (replaces the old CAL button).
const Rect btnCO2       = {0, CO2_LABEL_Y - 4, SCREEN_W, 64};
const Rect btnTempUnit  = {0, TR_TOP - 6, SCREEN_W / 2, 60};

// Settings screen
const Rect btnSetBack    = {0, 0, 70, SET_HEADER_H + 6};
const Rect btnAltMinus   = {SCREEN_W - 116, SET_ROW1_Y + 14, 36, 32};
const Rect btnAltPlus    = {SCREEN_W - 44,  SET_ROW1_Y + 14, 36, 32};
const Rect btnTempMinus  = {SCREEN_W - 116, SET_ROW2_Y + 14, 36, 32};
const Rect btnTempPlus   = {SCREEN_W - 44,  SET_ROW2_Y + 14, 36, 32};
const Rect btnForceRecal = {12, SET_FRC_Y, SCREEN_W - 24, SET_FRC_H};
// Brightness toggle in the Settings header (right side, mirroring "< Back" on the left).
// Sized just wide enough for "100%" with minimal padding so it does not crowd
// the centered "Settings" title.
const Rect btnBrightness = {SCREEN_W - 52, 2, 48, SET_HEADER_H - 4};
const Rect btnWifiTile   = {12, SET_WIFI_Y, SCREEN_W - 24, SET_WIFI_H};

// Time settings
const Rect btnTsBack     = {0, 0, 70, TS_HEADER_H + 6};
const Rect btnTzMinus    = {SCREEN_W - 116, TS_ROW1_Y + 14, 36, 32};
const Rect btnTzPlus     = {SCREEN_W - 44,  TS_ROW1_Y + 14, 36, 32};
const Rect btnFmtToggle  = {SCREEN_W - 92,  TS_ROW2_Y + 14, 80, 32};
const Rect btnNtpToggle  = {SCREEN_W - 92,  TS_ROW3_Y + 14, 80, 32};
const Rect btnSyncNow    = {12, TS_BTN_Y, SCREEN_W - 24, 36};

// Dialog
const Rect btnDlgCancel  = {12,             SCREEN_H - 64, 102, 52};
const Rect btnDlgConfirm = {SCREEN_W - 114, SCREEN_H - 64, 102, 52};

const Rect btnWifiBack   = {0, 0, 70, SET_HEADER_H + 6};

// ============================================================
//                      SCREEN STATE
// ============================================================

enum Screen {
  SCREEN_MAIN,
  SCREEN_SETTINGS,
  SCREEN_TIME_SETTINGS,
  SCREEN_CONFIRM_CAL,
  SCREEN_CALIBRATING,
  SCREEN_CAL_RESULT,
  SCREEN_WIFI_SETUP,
};

// ============================================================
//                      GLOBALS
// ============================================================

TFT_eSPI            tft = TFT_eSPI();
SPIClass            touchSPI(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
SensirionI2cScd4x   scd4x;
Preferences         prefs;
WebServer           server(80);
WiFiManager         wm;

// Settings (persisted)
bool     ascEnabled       = DEFAULT_ASC;
uint16_t sensorAltitudeM  = DEFAULT_ALTITUDE_M;
float    tempOffsetC      = DEFAULT_TEMP_OFFSET;
// (theme is fixed: dark only)
bool     tempInFahrenheit = DEFAULT_TEMP_F;
uint8_t  brightnessPct    = DEFAULT_BRIGHTNESS_PCT;
int16_t  tzOffsetMin      = DEFAULT_TZ_OFFSET_MIN;
bool     use24hClock      = DEFAULT_USE_24H;
bool     ntpEnabled       = true;
String   ntpServer        = DEFAULT_NTP_SERVER;

// Uptime tracking
uint32_t baseUptimeSec    = 0;
uint32_t lastFrcUptimeSec = 0;
unsigned long lastUptimeSaveMs = 0;

// WiFi state
bool     wifiConnected      = false;
bool     ntpSynced          = false;
bool     webServerRunning   = false;
bool     wifiPortalActive   = false;
bool     mdnsRunning        = false;
unsigned long lastWifiUiUpdate = 0;

// Screen state
Screen   currentScreen    = SCREEN_MAIN;
bool     calSuccess       = false;
char     calResultMsg[80] = "";
unsigned long lastClockDraw = 0;
char     lastClockText[12] = "";

// In-RAM history (for graph)
uint16_t co2History[MAX_SAMPLES];
int      historyCount = 0;
int      historyHead  = 0;

// Auto-fit graph y-axis bounds. Recomputed whenever the buffer
// changes; expand only - they shrink back to the defaults on a
// full screen redraw so transient spikes don't lock the axis high.
uint16_t chartMin = CO2_CHART_DEFAULT_MIN;
uint16_t chartMax = CO2_CHART_DEFAULT_MAX;

// LittleFS log header (kept in RAM, written through to flash).
// We force tight packing with #pragma pack because the natural
// layout has a 4-byte alignment that the compiler may pad to 36.
#pragma pack(push, 1)
struct LogHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t recordSize;
  uint32_t capacity;     // max records
  uint32_t count;        // records written so far (saturates at capacity)
  uint32_t head;         // next write position
  uint32_t reserved[2];
};
#pragma pack(pop)
LogHeader logHdr;
bool      fsReady = false;

#pragma pack(push, 1)
struct LogRecord {
  uint32_t ts_utc;       // unix time, 0 if not synced
  uint16_t co2_ppm;
  int16_t  temp_c100;    // Celsius * 100
  uint16_t rh_x10;       // RH * 10
};
#pragma pack(pop)
static_assert(sizeof(LogRecord) == 10, "LogRecord must be 10 bytes");
static_assert(sizeof(LogHeader) == 28, "LogHeader must be 28 bytes");

// Current readings
unsigned long lastReadTime   = 0;
unsigned long lastSampleTime = 0;
unsigned long lastTapTime    = 0;
bool          wasTouched     = false;

uint16_t currentCO2  = 0;
float    currentTemp = 0.0f;
float    currentRH   = 0.0f;
bool     sensorOK    = false;
bool     dataValid   = false;

static char errorMessage[64];

// ============================================================
//                  FORWARD DECLARATIONS
// ============================================================
void drawDialogButton(const Rect& r, const char* label,
                      uint16_t bg, uint16_t fg);
void drawSmallButton(const Rect& r, const char* label,
                     uint16_t bg, uint16_t fg, uint8_t font);
void drawWifiTile(const Rect& r);

void drawMainScreen();
void drawSettingsScreen();
void drawTimeSettingsScreen();
void drawConfirmCalScreen();
void drawCalibratingScreen();
void drawCalResultScreen();
void drawWifiSetupScreen();
void drawTopArea();
void drawDateRow();
void formatDate(char* buf, size_t bufsize);
void drawClockRow(bool force = false);
void drawReadings();
void drawGraph();
void runForcedCalibration();
bool applySensorSettings();
void tryWifiConnect();
void startWifiPortal();
void stopWifiPortal();
void startWebServer();
void stopWebServer();
void startMdns();
void handleRoot();
void handleData();
void handleHistoryCsv();
void handleHistoryJson();
void syncNtp(bool blocking);
void initLog();
void appendLog(uint16_t co2, float t, float rh);
void eraseLog();

// ============================================================
//                     HELPERS
// ============================================================

void logError(const char* context, int16_t err) {
  Serial.print(context); Serial.print(": ");
  errorToString(err, errorMessage, sizeof errorMessage);
  Serial.println(errorMessage);
}

uint16_t co2Color(uint16_t co2) {
  if (co2 < CO2_GOOD_MAX)     return TFT_GREEN;
  if (co2 < CO2_MODERATE_MAX) return TFT_SKYBLUE;
  if (co2 < CO2_POOR_MAX)     return TFT_ORANGE;
  return TFT_RED;
}

const char* co2StatusText(uint16_t co2) {
  if (co2 < CO2_GOOD_MAX)     return "GOOD AIR";
  if (co2 < CO2_MODERATE_MAX) return "MODERATE";
  if (co2 < CO2_POOR_MAX)     return "POOR - VENTILATE";
  return "VERY POOR";
}

const char* co2StatusKey(uint16_t co2) {
  if (co2 < CO2_GOOD_MAX)     return "good";
  if (co2 < CO2_MODERATE_MAX) return "moderate";
  if (co2 < CO2_POOR_MAX)     return "poor";
  return "verypoor";
}

float displayTemp() {
  return tempInFahrenheit ? (currentTemp * 9.0f / 5.0f + 32.0f) : currentTemp;
}
const char* tempUnit() { return tempInFahrenheit ? "F" : "C"; }
float tempOffsetDisplay() {
  return tempInFahrenheit ? (tempOffsetC * 9.0f / 5.0f) : tempOffsetC;
}

// Round x down/up to the next multiple of `step`.
uint16_t roundDownTo(uint16_t v, uint16_t step) { return (v / step) * step; }
uint16_t roundUpTo  (uint16_t v, uint16_t step) {
  return ((v + step - 1) / step) * step;
}

// Recompute graph y-axis bounds from the live history. Returns true
// if either bound changed (so the caller can redraw the frame).
bool recomputeChartBounds() {
  uint16_t newMin = CO2_CHART_DEFAULT_MIN;
  uint16_t newMax = CO2_CHART_DEFAULT_MAX;

  // Include the current live reading so the axis adjusts even before
  // the first sample is committed to history.
  if (dataValid) {
    if (currentCO2 < newMin) newMin = currentCO2;
    if (currentCO2 > newMax) newMax = currentCO2;
  }
  for (int i = 0; i < historyCount; i++) {
    int idx = (historyHead - historyCount + i + MAX_SAMPLES) % MAX_SAMPLES;
    uint16_t v = co2History[idx];
    if (v < newMin) newMin = v;
    if (v > newMax) newMax = v;
  }

  // Snap to nearest 200 ppm so axis labels stay tidy
  newMin = roundDownTo(newMin, 200);
  newMax = roundUpTo  (newMax, 200);
  // Guarantee at least 200 ppm of range
  if (newMax - newMin < 200) newMax = newMin + 200;

  bool changed = (newMin != chartMin) || (newMax != chartMax);
  chartMin = newMin;
  chartMax = newMax;
  return changed;
}

int co2ToY(uint16_t co2) {
  if (co2 < chartMin) co2 = chartMin;
  if (co2 > chartMax) co2 = chartMax;
  long range = chartMax - chartMin;
  return GRAPH_Y + GRAPH_H - (int)(((long)(co2 - chartMin) * GRAPH_H) / range);
}

uint32_t getTotalUptimeSec() {
  return baseUptimeSec + (uint32_t)(millis() / 1000UL);
}

uint32_t daysSinceLastFrc() {
  if (lastFrcUptimeSec == 0) return UINT32_MAX;
  uint32_t now = getTotalUptimeSec();
  if (now <= lastFrcUptimeSec) return 0;
  return (now - lastFrcUptimeSec) / 86400UL;
}

bool calibrationOverdue() {
  uint32_t d = daysSinceLastFrc();
  return (d != UINT32_MAX) && (d >= FRC_REMINDER_DAYS);
}

// Trend over last few samples: -1 falling, 0 flat, +1 rising
int co2Trend() {
  if (historyCount < TREND_MIN_SAMPLES) return 0;
  int n = historyCount < 6 ? historyCount : 6;
  int firstIdx = (historyHead - n + MAX_SAMPLES) % MAX_SAMPLES;
  int lastIdx  = (historyHead - 1 + MAX_SAMPLES) % MAX_SAMPLES;
  int diff = (int)co2History[lastIdx] - (int)co2History[firstIdx];
  if (diff >  TREND_DELTA_PPM) return  1;
  if (diff < -TREND_DELTA_PPM) return -1;
  return 0;
}

// Min / max / avg over last hour (last 12 samples)
void hourStats(uint16_t& mn, uint16_t& mx, uint16_t& avg) {
  if (historyCount == 0) { mn = mx = avg = 0; return; }
  int n = historyCount < 12 ? historyCount : 12;
  uint32_t sum = 0;
  mn = UINT16_MAX; mx = 0;
  for (int i = 0; i < n; i++) {
    int idx = (historyHead - 1 - i + MAX_SAMPLES) % MAX_SAMPLES;
    uint16_t v = co2History[idx];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v;
  }
  avg = (uint16_t)(sum / n);
}

void disableOnboardLeds() {
  pinMode(LED_R, OUTPUT); digitalWrite(LED_R, HIGH);
  pinMode(LED_G, OUTPUT); digitalWrite(LED_G, HIGH);
  pinMode(LED_B, OUTPUT); digitalWrite(LED_B, HIGH);
}

// Backlight is driven via LEDC PWM so brightness is configurable.
// Note: the CYD's MOSFET driver is marginal at low duty cycles. PWM
// frequency is set high (20 kHz) to push any flicker out of the
// visible band, but very low brightness levels may still flicker on
// some boards. If you observe flicker even at 100% brightness, the
// cause is almost always the USB power supply / cable, not PWM.
//
// The LEDC API changed between ESP32 Arduino core 2.x and 3.x:
//   2.x: ledcSetup(channel, freq, bits) + ledcAttachPin(pin, channel)
//        + ledcWrite(channel, duty)
//   3.x: ledcAttach(pin, freq, bits) + ledcWrite(pin, duty)
// We auto-detect via the core version macro and use whichever is
// available so the sketch builds on either.
void setupBacklight() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  bool ok = ledcAttach(BACKLIGHT_PIN, BACKLIGHT_LEDC_FREQ, BACKLIGHT_LEDC_BITS);
  Serial.printf("Backlight: LEDC v3 attach pin=%d freq=%d bits=%d -> %s\n",
                BACKLIGHT_PIN, BACKLIGHT_LEDC_FREQ, BACKLIGHT_LEDC_BITS,
                ok ? "ok" : "FAILED");
#else
  double actualFreq = ledcSetup(BACKLIGHT_LEDC_CHAN,
                                BACKLIGHT_LEDC_FREQ, BACKLIGHT_LEDC_BITS);
  ledcAttachPin(BACKLIGHT_PIN, BACKLIGHT_LEDC_CHAN);
  Serial.printf("Backlight: LEDC v2 chan=%d pin=%d req=%d Hz actual=%.1f Hz bits=%d\n",
                BACKLIGHT_LEDC_CHAN, BACKLIGHT_PIN,
                BACKLIGHT_LEDC_FREQ, actualFreq, BACKLIGHT_LEDC_BITS);
#endif
}

void applyBrightness() {
  uint32_t duty = ((uint32_t)brightnessPct * BACKLIGHT_LEDC_MAX) / 100;
  if (duty > BACKLIGHT_LEDC_MAX) duty = BACKLIGHT_LEDC_MAX;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(BACKLIGHT_PIN, duty);
#else
  ledcWrite(BACKLIGHT_LEDC_CHAN, duty);
#endif
  Serial.printf("Backlight: %u%% (duty %lu/%d)\n",
                (unsigned)brightnessPct, (unsigned long)duty,
                BACKLIGHT_LEDC_MAX);
}

void changeBrightness() {
  // Cycle: 100 -> 75 -> 50 -> 25 -> 100
  if      (brightnessPct > 75) brightnessPct = 75;
  else if (brightnessPct > 50) brightnessPct = 50;
  else if (brightnessPct > 25) brightnessPct = 25;
  else                          brightnessPct = 100;
  savePref("bright", brightnessPct);
  applyBrightness();
}

// ============================================================
//                   PREFERENCES (NVS)
// ============================================================

void loadPrefs() {
  prefs.begin("co2mon", false);
  ascEnabled       = prefs.getBool ("asc",     DEFAULT_ASC);
  sensorAltitudeM  = prefs.getUShort("alt",    DEFAULT_ALTITUDE_M);
  tempOffsetC      = prefs.getFloat ("tempoff", DEFAULT_TEMP_OFFSET);
  tempInFahrenheit = prefs.getBool ("tempf",   DEFAULT_TEMP_F);
  brightnessPct    = prefs.getUChar("bright",  DEFAULT_BRIGHTNESS_PCT);
  if (brightnessPct > 100) brightnessPct = 100;
  if (brightnessPct < 10)  brightnessPct = 10;
  baseUptimeSec    = prefs.getUInt ("uptime",  0);
  lastFrcUptimeSec = prefs.getUInt ("frc_at",  0);
  tzOffsetMin      = prefs.getShort("tzmin",   DEFAULT_TZ_OFFSET_MIN);
  use24hClock      = prefs.getBool ("use24h",  DEFAULT_USE_24H);
  ntpEnabled       = prefs.getBool ("ntpon",   true);
  ntpServer        = prefs.getString("ntpsrv", DEFAULT_NTP_SERVER);

  // In-RAM history copy (for graph at boot if we still have it cached)
  size_t bytesAvail = prefs.getBytesLength("hist");
  if (bytesAvail == sizeof(co2History)) {
    prefs.getBytes("hist", co2History, sizeof(co2History));
    historyCount = prefs.getUShort("histcnt", 0);
    historyHead  = prefs.getUShort("histhd",  0);
    if (historyCount > MAX_SAMPLES) historyCount = MAX_SAMPLES;
    if (historyHead  >= MAX_SAMPLES) historyHead  = 0;
  }
  prefs.end();

  Serial.printf("Loaded: alt=%u toff=%.1f tempF=%d tz=%d 24h=%d hist=%d uptime=%lu frc_at=%lu\n",
                sensorAltitudeM, tempOffsetC, tempInFahrenheit,
                tzOffsetMin, use24hClock, historyCount,
                (unsigned long)baseUptimeSec, (unsigned long)lastFrcUptimeSec);
}

void savePref(const char* key, bool v)     { prefs.begin("co2mon", false); prefs.putBool   (key, v); prefs.end(); }
void savePref(const char* key, uint8_t v)  { prefs.begin("co2mon", false); prefs.putUChar  (key, v); prefs.end(); }
void savePref(const char* key, uint16_t v) { prefs.begin("co2mon", false); prefs.putUShort (key, v); prefs.end(); }
void savePref(const char* key, int16_t v)  { prefs.begin("co2mon", false); prefs.putShort  (key, v); prefs.end(); }
void savePref(const char* key, uint32_t v) { prefs.begin("co2mon", false); prefs.putUInt   (key, v); prefs.end(); }
void savePref(const char* key, float v)    { prefs.begin("co2mon", false); prefs.putFloat  (key, v); prefs.end(); }
void savePref(const char* key, const String& v) { prefs.begin("co2mon", false); prefs.putString(key, v); prefs.end(); }

void saveHistory() {
  prefs.begin("co2mon", false);
  prefs.putBytes("hist", co2History, sizeof(co2History));
  prefs.putUShort("histcnt", historyCount);
  prefs.putUShort("histhd",  historyHead);
  prefs.end();
}

void saveUptime() { savePref("uptime", getTotalUptimeSec()); }

void saveUptimePeriodic() {
  if (millis() - lastUptimeSaveMs > UPTIME_SAVE_MS) {
    lastUptimeSaveMs = millis();
    saveUptime();
  }
}

void resetPrefs() {
  prefs.begin("co2mon", false);
  prefs.clear();
  prefs.end();
  Serial.println("Preferences cleared. Reboot to use defaults.");
}

// ============================================================
//                LITTLEFS LONG-TERM LOG
// ============================================================

void writeLogHeader() {
  if (!fsReady) return;
  File f = LittleFS.open(LOG_PATH, "r+");
  if (!f) {
    f = LittleFS.open(LOG_PATH, "w");
    if (!f) { Serial.println("LOG: cannot open for header write"); return; }
  }
  f.seek(0);
  f.write((const uint8_t*)&logHdr, sizeof(logHdr));
  f.close();
}

void initLog() {
  if (!LittleFS.begin(true)) {
    Serial.println("LOG: LittleFS mount failed");
    fsReady = false;
    return;
  }
  fsReady = true;

  // Try to load existing header
  if (LittleFS.exists(LOG_PATH)) {
    File f = LittleFS.open(LOG_PATH, "r");
    if (f && f.size() >= sizeof(LogHeader)) {
      f.read((uint8_t*)&logHdr, sizeof(logHdr));
      f.close();
      if (logHdr.magic == LOG_MAGIC && logHdr.version == LOG_VERSION
          && logHdr.recordSize == sizeof(LogRecord)
          && logHdr.capacity == LOG_MAX_RECORDS
          && logHdr.head < logHdr.capacity) {
        Serial.printf("LOG: existing log %lu records (head=%lu cap=%lu)\n",
                      (unsigned long)logHdr.count,
                      (unsigned long)logHdr.head,
                      (unsigned long)logHdr.capacity);
        return;
      }
      Serial.println("LOG: header mismatch, recreating");
      f.close();
    } else if (f) f.close();
    LittleFS.remove(LOG_PATH);
  }

  // Create fresh log
  memset(&logHdr, 0, sizeof(logHdr));
  logHdr.magic      = LOG_MAGIC;
  logHdr.version    = LOG_VERSION;
  logHdr.recordSize = sizeof(LogRecord);
  logHdr.capacity   = LOG_MAX_RECORDS;
  logHdr.count      = 0;
  logHdr.head       = 0;
  writeLogHeader();
  Serial.printf("LOG: created new log (capacity %lu records)\n",
                (unsigned long)LOG_MAX_RECORDS);
}

void appendLog(uint16_t co2, float t, float rh) {
  if (!fsReady) return;

  LogRecord rec;
  rec.ts_utc   = ntpSynced ? (uint32_t)time(nullptr) : 0;
  rec.co2_ppm  = co2;
  rec.temp_c100 = (int16_t)(t * 100.0f);
  rec.rh_x10    = (uint16_t)(rh * 10.0f);

  File f = LittleFS.open(LOG_PATH, "r+");
  if (!f) { Serial.println("LOG: append open failed"); return; }

  uint32_t offset = sizeof(LogHeader) + (uint32_t)logHdr.head * sizeof(LogRecord);
  f.seek(offset);
  f.write((const uint8_t*)&rec, sizeof(rec));

  // Advance head, saturate count at capacity
  logHdr.head = (logHdr.head + 1) % logHdr.capacity;
  if (logHdr.count < logHdr.capacity) logHdr.count++;

  // Write updated header (cheap - 32 bytes)
  f.seek(0);
  f.write((const uint8_t*)&logHdr, sizeof(logHdr));
  f.close();
}

void eraseLog() {
  if (!fsReady) return;
  LittleFS.remove(LOG_PATH);
  initLog();
  Serial.println("LOG: erased");
}

// Read log records into a callback, oldest first.
// `callback(idx, rec)` returns false to stop iteration.
template <typename F>
void iterateLog(F callback) {
  if (!fsReady || logHdr.count == 0) return;
  File f = LittleFS.open(LOG_PATH, "r");
  if (!f) return;

  uint32_t start;
  if (logHdr.count < logHdr.capacity) {
    start = 0;
  } else {
    // Buffer is full - oldest is at head
    start = logHdr.head;
  }

  for (uint32_t i = 0; i < logHdr.count; i++) {
    uint32_t pos = (start + i) % logHdr.capacity;
    f.seek(sizeof(LogHeader) + pos * sizeof(LogRecord));
    LogRecord rec;
    if (f.read((uint8_t*)&rec, sizeof(rec)) != sizeof(rec)) break;
    if (!callback(i, rec)) break;
  }
  f.close();
}

// ============================================================
//                       TOUCH
// ============================================================

bool readTap(int16_t& sx, int16_t& sy) {
  TS_Point p = ts.getPoint();
  bool nowTouched = (p.z >= TOUCH_PRESSURE_MIN);

  unsigned long now = millis();
  bool fired = false;

  if (nowTouched && !wasTouched && (now - lastTapTime > TAP_DEBOUNCE_MS)) {
    lastTapTime = now;
    int16_t rx = TS_SWAP_XY ? p.y : p.x;
    int16_t ry = TS_SWAP_XY ? p.x : p.y;
    sx = map(rx, TS_X_MIN, TS_X_MAX, 0, SCREEN_W);
    sy = map(ry, TS_Y_MIN, TS_Y_MAX, 0, SCREEN_H);
    sx = constrain(sx, 0, SCREEN_W - 1);
    sy = constrain(sy, 0, SCREEN_H - 1);
    Serial.printf("TAP raw=(%d,%d,z=%d) screen=(%d,%d)\n",
                  p.x, p.y, p.z, sx, sy);
    fired = true;
  }
  wasTouched = nowTouched;
  return fired;
}

// ============================================================
//                       ICONS
// ============================================================

// Trend arrow: -1 = down (red-ish on screen), +1 = up, 0 = steady.
// All three variants are roughly the same visual weight so the
// indicator stays present once we have enough samples to compute it.
void drawTrendArrow(int cx, int cy, int trend, uint16_t color) {
  if (trend > 0) {
    // up arrow: triangle pointing up
    tft.fillTriangle(cx, cy - 7, cx - 6, cy + 4, cx + 6, cy + 4, color);
  } else if (trend < 0) {
    // down arrow: triangle pointing down
    tft.fillTriangle(cx - 6, cy - 4, cx + 6, cy - 4, cx, cy + 7, color);
  } else {
    // steady: horizontal arrow pointing right
    tft.fillRect(cx - 6, cy - 2, 8, 4, color);
    tft.fillTriangle(cx + 1, cy - 5, cx + 1, cy + 5, cx + 6, cy, color);
  }
}

// ============================================================
//                     DATE + CLOCK
// ============================================================

void formatDate(char* buf, size_t bufsize) {
  if (!ntpSynced) {
    snprintf(buf, bufsize, "(awaiting time sync)");
    return;
  }
  time_t now = time(nullptr) + tzOffsetMin * 60;
  struct tm tmv;
  gmtime_r(&now, &tmv);

  // We avoid strftime's "%-d" (no-pad day) here because that is a
  // GNU extension that newlib on the ESP32 implements buggily,
  // producing day-1 instead of day. Build the string manually.
  static const char* const dayNames[] = {
    "Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday"
  };
  static const char* const monthNames[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };
  int wday = tmv.tm_wday;  if (wday < 0 || wday > 6)  wday = 0;
  int mon  = tmv.tm_mon;   if (mon  < 0 || mon  > 11) mon  = 0;
  snprintf(buf, bufsize, "%s, %s %d %d",
           dayNames[wday], monthNames[mon],
           tmv.tm_mday, tmv.tm_year + 1900);
}

void drawDateRow() {
  char buf[40];
  formatDate(buf, sizeof(buf));
  tft.fillRect(0, DATE_Y, SCREEN_W, DATE_H, COLOR_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(ntpSynced ? COLOR_TEXT : COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString(buf, SCREEN_W / 2, DATE_Y + DATE_H / 2, 2);
}

// Call this whenever entering main screen so date refreshes.
void drawTopArea() {
  drawDateRow();
}

void formatClock(char* buf, size_t bufsize) {
  if (!ntpSynced) {
    snprintf(buf, bufsize, "--:-- (no time)");
    return;
  }
  time_t now = time(nullptr);
  now += tzOffsetMin * 60;
  struct tm tmv;
  gmtime_r(&now, &tmv);

  if (use24hClock) {
    snprintf(buf, bufsize, "%02d:%02d", tmv.tm_hour, tmv.tm_min);
  } else {
    int h = tmv.tm_hour;
    const char* ampm = h < 12 ? "AM" : "PM";
    h = h % 12; if (h == 0) h = 12;
    snprintf(buf, bufsize, "%d:%02d %s", h, tmv.tm_min, ampm);
  }
}

void drawClockRow(bool force) {
  char buf[16];
  formatClock(buf, sizeof(buf));
  if (!force && strcmp(buf, lastClockText) == 0) return;
  strncpy(lastClockText, buf, sizeof(lastClockText));

  tft.fillRect(0, CLOCK_Y, SCREEN_W, CLOCK_H, COLOR_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(ntpSynced ? COLOR_TEXT : COLOR_TEXT_DIM, COLOR_BG);
  // Font 6 is a tall numeric font that includes ":" - perfect for a clock.
  // It doesn't include "AM"/"PM" letters, so for 12-hour mode we draw the
  // time in Font 6 and append AM/PM in a smaller font next to it.
  if (use24hClock || !ntpSynced) {
    tft.drawString(buf, SCREEN_W / 2, CLOCK_Y + CLOCK_H / 2, 6);
  } else {
    // Split "H:MM AM" -> "H:MM" + "AM"
    char* sp = strchr(buf, ' ');
    if (!sp) {
      tft.drawString(buf, SCREEN_W / 2, CLOCK_Y + CLOCK_H / 2, 6);
    } else {
      *sp = '\0';
      const char* ampm = sp + 1;
      int16_t timeW = tft.textWidth(buf, 6);
      int16_t totalW = timeW + 6 + tft.textWidth(ampm, 4);
      int16_t startX = (SCREEN_W - totalW) / 2;
      tft.setTextDatum(ML_DATUM);
      tft.drawString(buf, startX, CLOCK_Y + CLOCK_H / 2, 6);
      tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
      tft.drawString(ampm, startX + timeW + 6,
                     CLOCK_Y + CLOCK_H / 2 + 6, 4);
    }
  }
}

// ============================================================
//                       GRAPH
// ============================================================

void drawThresholdLines() {
  uint16_t cGood = TFT_DARKGREEN;
  uint16_t cWarn = 0x4225;
  // Only draw threshold lines that fall within the current y-axis range
  if (CO2_GOOD_MAX > chartMin && CO2_GOOD_MAX < chartMax) {
    int y = co2ToY(CO2_GOOD_MAX);
    for (int x = GRAPH_X + 2; x < GRAPH_X + GRAPH_W; x += 4)
      tft.drawPixel(x, y, cGood);
  }
  if (CO2_MODERATE_MAX > chartMin && CO2_MODERATE_MAX < chartMax) {
    int y = co2ToY(CO2_MODERATE_MAX);
    for (int x = GRAPH_X + 2; x < GRAPH_X + GRAPH_W; x += 4)
      tft.drawPixel(x, y, cWarn);
  }
}

void drawGraphFrame() {
  // Clear the label gutter and the frame itself so old labels from a
  // prior axis range don't linger when bounds change.
  tft.fillRect(0, GRAPH_Y - 4, GRAPH_X, GRAPH_H + 8, COLOR_BG);
  tft.drawFastVLine(GRAPH_X,           GRAPH_Y, GRAPH_H, COLOR_GRAPH_AXIS);
  tft.drawFastHLine(GRAPH_X, GRAPH_Y + GRAPH_H, GRAPH_W, COLOR_GRAPH_AXIS);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(MR_DATUM);

  // Top and bottom of the visible range, always.
  char buf[8];
  snprintf(buf, sizeof(buf), "%u", chartMax);
  tft.drawString(buf, GRAPH_X - 2, GRAPH_Y,            1);
  snprintf(buf, sizeof(buf), "%u", chartMin);
  tft.drawString(buf, GRAPH_X - 2, GRAPH_Y + GRAPH_H,  1);

  // Threshold-line labels at 800/1200 if they are in range.
  if (CO2_MODERATE_MAX > chartMin && CO2_MODERATE_MAX < chartMax) {
    tft.drawString("1200", GRAPH_X - 2, co2ToY(CO2_MODERATE_MAX), 1);
  }
  if (CO2_GOOD_MAX > chartMin && CO2_GOOD_MAX < chartMax) {
    tft.drawString("800",  GRAPH_X - 2, co2ToY(CO2_GOOD_MAX), 1);
  }

  tft.setTextDatum(TR_DATUM);
  tft.drawString("CO2 ppm  -  5 min/sample", SCREEN_W - 4, GRAPH_CAPTION_Y, 1);
}

void drawGraph() {
  // Adjust y-axis bounds to fit the current data; if they changed,
  // we need to redraw the gutter labels too.
  bool boundsChanged = recomputeChartBounds();
  if (boundsChanged) drawGraphFrame();

  tft.fillRect(GRAPH_X + 1, GRAPH_Y, GRAPH_W - 1, GRAPH_H, COLOR_BG);
  drawThresholdLines();
  if (historyCount == 0) return;

  float dx = (float)(GRAPH_W - 4) / (float)(MAX_SAMPLES - 1);
  int prevX = -1, prevY = -1;
  for (int i = 0; i < historyCount; i++) {
    int idx = (historyHead - historyCount + i + MAX_SAMPLES) % MAX_SAMPLES;
    uint16_t v = co2History[idx];
    int x = GRAPH_X + 2 + (int)(i * dx);
    int y = co2ToY(v);
    uint16_t c = co2Color(v);
    if (prevX >= 0) tft.drawLine(prevX, prevY, x, y, c);
    tft.fillCircle(x, y, 1, c);
    prevX = x; prevY = y;
  }
}

// ============================================================
//                      READINGS
// ============================================================

void drawReadings() {
  char buf[16];

  if (!dataValid) {
    tft.fillRect(0, CO2_VALUE_Y - 2, SCREEN_W, 60, COLOR_BG);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("warming up...", SCREEN_W / 2, CO2_VALUE_Y + 24, 4);
    return;
  }

  // CO2 number (Font 7)
  tft.fillRect(0, CO2_VALUE_Y, SCREEN_W, 56, COLOR_BG);
  tft.setTextColor(co2Color(currentCO2), COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  snprintf(buf, sizeof(buf), "%4u", currentCO2);
  tft.drawString(buf, SCREEN_W / 2, CO2_VALUE_Y, 7);

  // Trend arrow to the right of the number. Once we have enough
  // samples, the arrow is always drawn (horizontal when steady).
  if (historyCount >= TREND_MIN_SAMPLES) {
    int trend = co2Trend();
    drawTrendArrow(SCREEN_W - 22, CO2_VALUE_Y + 28, trend,
                   co2Color(currentCO2));
  }

  // Status banner
  tft.fillRect(0, STATUS_Y - 4, SCREEN_W, 30, COLOR_BG);
  tft.setTextColor(co2Color(currentCO2), COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(co2StatusText(currentCO2), SCREEN_W / 2, STATUS_Y, 4);

  // Temp
  tft.fillRect(0, TR_VALUE_Y, SCREEN_W / 2, 30, COLOR_BG);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  snprintf(buf, sizeof(buf), "%.1f", displayTemp());
  tft.drawString(buf, SCREEN_W / 4 - 6, TR_VALUE_Y, 4);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(tempUnit(), SCREEN_W / 4 + 30, TR_VALUE_Y + 4, 2);

  // RH
  tft.fillRect(SCREEN_W / 2, TR_VALUE_Y, SCREEN_W / 2, 30, COLOR_BG);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  snprintf(buf, sizeof(buf), "%.0f", currentRH);
  tft.drawString(buf, SCREEN_W * 3 / 4 - 6, TR_VALUE_Y, 4);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("%", SCREEN_W * 3 / 4 + 18, TR_VALUE_Y + 4, 2);
}

// ============================================================
//                     MAIN SCREEN
// ============================================================

void drawMainScreen() {
  tft.fillScreen(COLOR_BG);
  drawTopArea();
  lastClockText[0] = '\0';
  drawClockRow(true);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("CO2 (ppm)", SCREEN_W / 2, CO2_LABEL_Y, 2);

  tft.setTextDatum(TC_DATUM);
  tft.drawString("TEMP", SCREEN_W / 4,     TR_TOP, 2);
  tft.drawString("RH",   SCREEN_W * 3 / 4, TR_TOP, 2);

  // Compute bounds before the frame so the labels match the data
  // we are about to plot.
  recomputeChartBounds();
  drawGraphFrame();
  drawThresholdLines();
  drawReadings();
  drawGraph();

  // If calibration is overdue, show a small orange alert under the graph.
  if (calibrationOverdue()) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_ORANGE, COLOR_BG);
    // Draw it inline near the bottom edge so it doesn't compete with readings
    tft.fillRect(0, SCREEN_H - 12, SCREEN_W, 12, COLOR_BG);
    tft.drawString("Calibration overdue - tap CO2", SCREEN_W / 2, SCREEN_H - 6, 1);
  }
}

// ============================================================
//                    BUTTON HELPERS
// ============================================================

void drawDialogButton(const Rect& r, const char* label,
                      uint16_t bg, uint16_t fg) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 8, bg);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 8, fg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(fg, bg);
  tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2 + 1, 4);
}

void drawSmallButton(const Rect& r, const char* label,
                     uint16_t bg, uint16_t fg, uint8_t font) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, bg);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, fg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(fg, bg);
  tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2 + 1, font);
}

void drawWifiTile(const Rect& r) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 8, COLOR_BTN_BG);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 8, COLOR_BTN_TEXT);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_BTN_TEXT, COLOR_BTN_BG);
  tft.drawString("WiFi", r.x + 12, r.y + 8, 2);

  tft.setTextDatum(TR_DATUM);
  if (wifiConnected) {
    String ip = WiFi.localIP().toString();
    tft.setTextColor(TFT_GREEN, COLOR_BTN_BG);
    tft.drawString(ip.c_str(), r.x + r.w - 12, r.y + 8, 2);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BTN_BG);
    tft.drawString("co2monitor.local", r.x + r.w - 12, r.y + 26, 1);
    tft.setTextDatum(BR_DATUM);
    tft.drawString("Tap to reconfigure", r.x + r.w - 12, r.y + r.h - 8, 1);
  } else {
    tft.setTextColor(TFT_ORANGE, COLOR_BTN_BG);
    tft.drawString("Not connected", r.x + r.w - 12, r.y + 8, 2);
    tft.setTextDatum(BR_DATUM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BTN_BG);
    tft.drawString("Tap to set up", r.x + r.w - 12, r.y + r.h - 8, 1);
  }
}

// ============================================================
//                   SETTINGS SCREEN
// ============================================================

void drawAdjusterRow(int y, const char* label, const char* valueText,
                     const Rect& minusRect, const Rect& plusRect) {
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(label, 16, y + SET_ROW_H / 2, 4);

  drawSmallButton(minusRect, "-", COLOR_BTN_BG, COLOR_BTN_TEXT, 4);
  drawSmallButton(plusRect,  "+", COLOR_BTN_BG, COLOR_BTN_TEXT, 4);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  int cx = (minusRect.x + minusRect.w + plusRect.x) / 2;
  tft.drawString(valueText, cx, y + SET_ROW_H / 2, 2);

  tft.drawFastHLine(8, y + SET_ROW_H, SCREEN_W - 16, COLOR_TEXT_DIM);
}

void drawSettingsScreen() {
  tft.fillScreen(COLOR_BG);

  tft.fillRect(0, 0, SCREEN_W, SET_HEADER_H, COLOR_TITLE_BG);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(COLOR_TITLE_TEXT, COLOR_TITLE_BG);
  tft.drawString("< Back", 8, SET_HEADER_H / 2, 2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Settings", SCREEN_W / 2, SET_HEADER_H / 2, 4);

  // Brightness toggle on the right side of the header.
  // Tap to cycle 100 -> 75 -> 50 -> 25 -> 100.
  char bbuf[8];
  snprintf(bbuf, sizeof(bbuf), "%u%%", (unsigned)brightnessPct);
  drawSmallButton(btnBrightness, bbuf, COLOR_BTN_BG, COLOR_BTN_TEXT, 2);

  char buf[24];
  snprintf(buf, sizeof(buf), "%u m", sensorAltitudeM);
  drawAdjusterRow(SET_ROW1_Y, "Altitude", buf, btnAltMinus, btnAltPlus);

  snprintf(buf, sizeof(buf), "%+.1f %s", tempOffsetDisplay(), tempUnit());
  drawAdjusterRow(SET_ROW2_Y, "Temp Off.", buf, btnTempMinus, btnTempPlus);

  tft.setTextDatum(MC_DATUM);
  uint32_t days = daysSinceLastFrc();
  if (days == UINT32_MAX) {
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("Never calibrated", SCREEN_W / 2, SET_LASTCAL_Y, 2);
  } else {
    bool overdue = days >= FRC_REMINDER_DAYS;
    tft.setTextColor(overdue ? TFT_ORANGE : COLOR_TEXT, COLOR_BG);
    snprintf(buf, sizeof(buf), "Last cal: %lu day%s ago",
             (unsigned long)days, days == 1 ? "" : "s");
    tft.drawString(buf, SCREEN_W / 2, SET_LASTCAL_Y, 2);
  }

  drawDialogButton(btnForceRecal, "Force Recalibrate",
                   TFT_DARKGREEN, TFT_WHITE);
  drawWifiTile(btnWifiTile);
}

// ============================================================
//                TIME SETTINGS SCREEN
// ============================================================

void drawTimeSettingsScreen() {
  tft.fillScreen(COLOR_BG);

  tft.fillRect(0, 0, SCREEN_W, TS_HEADER_H, COLOR_TITLE_BG);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(COLOR_TITLE_TEXT, COLOR_TITLE_BG);
  tft.drawString("< Back", 8, TS_HEADER_H / 2, 2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Time", SCREEN_W / 2, TS_HEADER_H / 2, 4);

  char buf[32];

  // Timezone row
  int hours = tzOffsetMin / 60;
  int mins  = abs(tzOffsetMin) % 60;
  if (mins == 0) snprintf(buf, sizeof(buf), "UTC%+d", hours);
  else snprintf(buf, sizeof(buf), "UTC%+d:%02d", hours, mins);
  drawAdjusterRow(TS_ROW1_Y, "Timezone", buf, btnTzMinus, btnTzPlus);

  // Format toggle row
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Format", 16, TS_ROW2_Y + SET_ROW_H / 2, 4);
  drawSmallButton(btnFmtToggle, use24hClock ? "24-hour" : "12-hour",
                  COLOR_BTN_BG, COLOR_BTN_TEXT, 2);
  tft.drawFastHLine(8, TS_ROW2_Y + SET_ROW_H, SCREEN_W - 16, COLOR_TEXT_DIM);

  // NTP toggle row
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("NTP", 16, TS_ROW3_Y + SET_ROW_H / 2, 4);
  drawSmallButton(btnNtpToggle, ntpEnabled ? "Enabled" : "Disabled",
                  COLOR_BTN_BG, COLOR_BTN_TEXT, 2);
  tft.drawFastHLine(8, TS_ROW3_Y + SET_ROW_H, SCREEN_W - 16, COLOR_TEXT_DIM);

  // NTP server display
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  snprintf(buf, sizeof(buf), "Server: %s", ntpServer.c_str());
  tft.drawString(buf, SCREEN_W / 2, TS_NTP_Y, 1);
  if (ntpSynced) tft.setTextColor(TFT_GREEN, COLOR_BG);
  else           tft.setTextColor(TFT_ORANGE, COLOR_BG);
  tft.drawString(ntpSynced ? "Synced" : "Not synced",
                 SCREEN_W / 2, TS_NTP_Y + 12, 1);

  // Sync now button (only useful if WiFi is connected)
  if (wifiConnected && ntpEnabled) {
    drawSmallButton(btnSyncNow, "Sync Now", TFT_DARKGREEN, TFT_WHITE, 2);
  } else {
    drawSmallButton(btnSyncNow, wifiConnected ? "(NTP off)" : "(no WiFi)",
                    COLOR_BTN_BG, COLOR_TEXT_DIM, 2);
  }
}

// ============================================================
//                  WORD-WRAPPED TEXT
// ============================================================

void drawWrappedText(const char* text, int16_t x, int16_t y,
                     int16_t w, uint8_t font,
                     uint16_t fg, uint16_t bg, uint8_t lineHeight) {
  tft.setTextColor(fg, bg);
  tft.setTextDatum(TL_DATUM);

  char line[64] = "";
  const char* p = text;
  int16_t lineY = y;

  while (*p) {
    const char* wordStart = p;
    while (*p && *p != ' ') p++;
    int wordLen = p - wordStart;
    char word[32];
    if (wordLen >= (int)sizeof(word)) wordLen = sizeof(word) - 1;
    memcpy(word, wordStart, wordLen);
    word[wordLen] = '\0';
    if (*p == ' ') p++;

    char trial[80];
    if (line[0] == '\0') strncpy(trial, word, sizeof(trial));
    else snprintf(trial, sizeof(trial), "%s %s", line, word);

    if (tft.textWidth(trial, font) <= w) {
      strncpy(line, trial, sizeof(line));
    } else {
      tft.drawString(line, x, lineY, font);
      lineY += lineHeight;
      strncpy(line, word, sizeof(line));
    }
  }
  if (line[0]) tft.drawString(line, x, lineY, font);
}

// ============================================================
//                  CALIBRATION SCREENS
// ============================================================

void drawConfirmCalScreen() {
  tft.fillScreen(COLOR_DIALOG_BG);
  tft.fillRect(0, 0, SCREEN_W, 36, COLOR_TITLE_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_TITLE_TEXT, COLOR_TITLE_BG);
  tft.drawString("Force Calibration", SCREEN_W / 2, 18, 4);

  drawWrappedText(
    "Take the device OUTSIDE in fresh air, away from people and "
    "exhaust. Wait 3+ minutes for the reading to stabilise. Then "
    "tap CALIBRATE to set the current reading to 420 ppm "
    "(typical outdoor CO2).",
    12, 50, SCREEN_W - 24, 2, COLOR_TEXT, COLOR_DIALOG_BG, 18);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_ACCENT, COLOR_DIALOG_BG);
  tft.drawString("Cancel if you are still indoors.",
                 SCREEN_W / 2, 220, 2);

  drawDialogButton(btnDlgCancel,  "Cancel", COLOR_BTN_BG,    COLOR_BTN_TEXT);
  drawDialogButton(btnDlgConfirm, "Calib.", TFT_DARKGREEN, TFT_WHITE);
}

void drawCalibratingScreen() {
  tft.fillScreen(COLOR_DIALOG_BG);
  tft.fillRect(0, 0, SCREEN_W, 36, COLOR_TITLE_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_TITLE_TEXT, COLOR_TITLE_BG);
  tft.drawString("Calibrating...", SCREEN_W / 2, 18, 4);
  tft.setTextColor(COLOR_TEXT, COLOR_DIALOG_BG);
  tft.drawString("Holding sensor steady", SCREEN_W / 2, 140, 2);
  tft.drawString("and writing reference", SCREEN_W / 2, 162, 2);
  tft.drawString("to the SCD41.",         SCREEN_W / 2, 184, 2);
}

void drawCalResultScreen() {
  tft.fillScreen(COLOR_DIALOG_BG);
  tft.fillRect(0, 0, SCREEN_W, 36, COLOR_TITLE_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_TITLE_TEXT, COLOR_TITLE_BG);
  tft.drawString(calSuccess ? "Calibration Done" : "Calibration Failed",
                 SCREEN_W / 2, 18, 4);

  uint16_t resultColor = calSuccess ? TFT_GREEN : TFT_RED;
  tft.setTextColor(resultColor, COLOR_DIALOG_BG);
  tft.drawString(calSuccess ? "Success" : "Failed",
                 SCREEN_W / 2, 100, 4);

  drawWrappedText(calResultMsg, 16, 150, SCREEN_W - 32, 2,
                  COLOR_TEXT, COLOR_DIALOG_BG, 20);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_DIALOG_BG);
  tft.drawString("Tap anywhere to return", SCREEN_W / 2, SCREEN_H - 30, 2);
}

void drawWifiSetupHeader() {
  tft.fillRect(0, 0, SCREEN_W, SET_HEADER_H, COLOR_TITLE_BG);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(COLOR_TITLE_TEXT, COLOR_TITLE_BG);
  tft.drawString("< Back", 8, SET_HEADER_H / 2, 2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("WiFi Setup", SCREEN_W / 2, SET_HEADER_H / 2, 4);
}

void drawWifiSetupStatus(const char* state, uint16_t color) {
  tft.fillRect(0, 250, SCREEN_W, 40, COLOR_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Status:", SCREEN_W / 2, 254, 2);
  tft.setTextColor(color, COLOR_BG);
  tft.drawString(state, SCREEN_W / 2, 274, 4);
}

void drawWifiSetupScreen() {
  tft.fillScreen(COLOR_BG);
  drawWifiSetupHeader();

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.drawString("On your phone or PC:", 12, 48, 2);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("1. Connect to WiFi:", 16, 76, 2);
  tft.setTextColor(COLOR_ACCENT, COLOR_BG);
  tft.drawString(WIFI_AP_NAME, 16, 96, 4);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("2. A page should open.", 16, 134, 2);
  tft.drawString("   Or visit:", 16, 154, 2);
  tft.setTextColor(COLOR_ACCENT, COLOR_BG);
  tft.drawString("192.168.4.1", 16, 174, 4);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("3. Pick your network", 16, 212, 2);
  tft.drawString("   and enter password.", 16, 230, 2);

  drawWifiSetupStatus("Active", TFT_GREEN);
}

// ============================================================
//                FORCED CALIBRATION
// ============================================================

void runForcedCalibration() {
  currentScreen = SCREEN_CALIBRATING;
  drawCalibratingScreen();

  scd4x.stopPeriodicMeasurement();
  delay(500);

  uint16_t correction = 0;
  int16_t err = scd4x.performForcedRecalibration(FRC_TARGET_PPM, correction);

  scd4x.startPeriodicMeasurement();
  lastReadTime = millis();
  dataValid = false;

  if (err) {
    calSuccess = false;
    snprintf(calResultMsg, sizeof(calResultMsg),
             "Sensor error during FRC. Check wiring and try again.");
  } else if (correction == 0xFFFF) {
    calSuccess = false;
    snprintf(calResultMsg, sizeof(calResultMsg),
             "Sensor was not stable enough. Stay outside for 3+ minutes "
             "before retrying.");
  } else {
    int16_t deltaPpm = (int16_t)(correction - 0x8000);
    calSuccess = true;
    snprintf(calResultMsg, sizeof(calResultMsg),
             "Reference set to %u ppm. Correction applied: %+d ppm.",
             FRC_TARGET_PPM, deltaPpm);
    lastFrcUptimeSec = getTotalUptimeSec();
    savePref("frc_at", lastFrcUptimeSec);
    saveUptime();
  }

  currentScreen = SCREEN_CAL_RESULT;
  drawCalResultScreen();
}

// ============================================================
//                SETTINGS CHANGE HANDLERS
// ============================================================

void changeAltitude(int16_t delta) {
  int32_t v = (int32_t)sensorAltitudeM + delta;
  if (v < ALT_MIN) v = ALT_MIN;
  if (v > ALT_MAX) v = ALT_MAX;
  sensorAltitudeM = (uint16_t)v;
  savePref("alt", sensorAltitudeM);
  scd4x.stopPeriodicMeasurement(); delay(20);
  scd4x.setSensorAltitude(sensorAltitudeM);
  scd4x.startPeriodicMeasurement();
  lastReadTime = millis();
}

void changeTempOffset(int direction) {
  float deltaC = tempInFahrenheit
               ? (direction * TEMP_OFF_STEP_F_IN_C)
               : (direction * TEMP_OFF_STEP_C);
  float v = tempOffsetC + deltaC;
  if (v < TEMP_OFF_MIN) v = TEMP_OFF_MIN;
  if (v > TEMP_OFF_MAX) v = TEMP_OFF_MAX;
  tempOffsetC = v;
  savePref("tempoff", tempOffsetC);
  scd4x.stopPeriodicMeasurement(); delay(20);
  scd4x.setTemperatureOffset(tempOffsetC);
  scd4x.startPeriodicMeasurement();
  lastReadTime = millis();
}

void changeTimezone(int delta) {
  int v = tzOffsetMin + delta;
  if (v < TZ_MIN_OFFSET) v = TZ_MIN_OFFSET;
  if (v > TZ_MAX_OFFSET) v = TZ_MAX_OFFSET;
  tzOffsetMin = (int16_t)v;
  savePref("tzmin", tzOffsetMin);
  lastClockText[0] = '\0';
}

// ============================================================
//                  TOUCH DISPATCH
// ============================================================

void handleTouch() {
  int16_t x, y;
  if (!readTap(x, y)) return;

  if (currentScreen == SCREEN_MAIN) {
    if (btnCO2.contains(x, y)) {
      currentScreen = SCREEN_SETTINGS;
      drawSettingsScreen();
    } else if (btnClock.contains(x, y)) {
      currentScreen = SCREEN_TIME_SETTINGS;
      drawTimeSettingsScreen();
    } else if (btnTempUnit.contains(x, y) && dataValid) {
      tempInFahrenheit = !tempInFahrenheit;
      savePref("tempf", tempInFahrenheit);
      drawReadings();
    }
  } else if (currentScreen == SCREEN_SETTINGS) {
    if (btnSetBack.contains(x, y))      { currentScreen = SCREEN_MAIN; drawMainScreen(); }
    else if (btnAltMinus.contains(x, y))   { changeAltitude(-ALT_STEP); drawSettingsScreen(); }
    else if (btnAltPlus.contains(x, y))    { changeAltitude(+ALT_STEP); drawSettingsScreen(); }
    else if (btnTempMinus.contains(x, y))  { changeTempOffset(-1); drawSettingsScreen(); }
    else if (btnTempPlus.contains(x, y))   { changeTempOffset(+1); drawSettingsScreen(); }
    else if (btnBrightness.contains(x, y)) { changeBrightness(); drawSettingsScreen(); }
    else if (btnForceRecal.contains(x, y)) { currentScreen = SCREEN_CONFIRM_CAL; drawConfirmCalScreen(); }
    else if (btnWifiTile.contains(x, y))   { startWifiPortal(); }
  } else if (currentScreen == SCREEN_TIME_SETTINGS) {
    if (btnTsBack.contains(x, y))       { currentScreen = SCREEN_MAIN; drawMainScreen(); }
    else if (btnTzMinus.contains(x, y))    { changeTimezone(-TZ_STEP_MIN); drawTimeSettingsScreen(); }
    else if (btnTzPlus.contains(x, y))     { changeTimezone(+TZ_STEP_MIN); drawTimeSettingsScreen(); }
    else if (btnFmtToggle.contains(x, y))  { use24hClock = !use24hClock; savePref("use24h", use24hClock); drawTimeSettingsScreen(); }
    else if (btnNtpToggle.contains(x, y))  { ntpEnabled = !ntpEnabled; savePref("ntpon", ntpEnabled); drawTimeSettingsScreen(); }
    else if (btnSyncNow.contains(x, y) && wifiConnected && ntpEnabled) {
      syncNtp(true);
      drawTimeSettingsScreen();
    }
  } else if (currentScreen == SCREEN_CONFIRM_CAL) {
    if (btnDlgCancel.contains(x, y)) {
      currentScreen = SCREEN_SETTINGS;
      drawSettingsScreen();
    } else if (btnDlgConfirm.contains(x, y)) {
      runForcedCalibration();
    }
  } else if (currentScreen == SCREEN_CAL_RESULT) {
    currentScreen = SCREEN_MAIN;
    drawMainScreen();
  } else if (currentScreen == SCREEN_WIFI_SETUP) {
    if (btnWifiBack.contains(x, y)) {
      stopWifiPortal();
      currentScreen = SCREEN_SETTINGS;
      drawSettingsScreen();
    }
  }
}

// ============================================================
//                       SENSOR
// ============================================================

bool applySensorSettings() {
  int16_t err;
  err = scd4x.setAutomaticSelfCalibrationEnabled(ascEnabled ? 1 : 0);
  if (err) { logError("setASC", err); return false; }
  err = scd4x.setSensorAltitude(sensorAltitudeM);
  if (err) { logError("setSensorAltitude", err); return false; }
  err = scd4x.setTemperatureOffset(tempOffsetC);
  if (err) { logError("setTemperatureOffset", err); return false; }
  return true;
}

void initSensor() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  scd4x.begin(Wire, SCD41_I2C_ADDR_62);
  scd4x.wakeUp();
  delay(30);
  scd4x.stopPeriodicMeasurement();
  delay(500);
  if (!applySensorSettings()) { sensorOK = false; return; }
  Serial.printf("Sensor: ASC=%d alt=%u m T-offset=%.1f C\n",
                ascEnabled, sensorAltitudeM, tempOffsetC);
  int16_t err = scd4x.startPeriodicMeasurement();
  if (err) { logError("startPeriodicMeasurement", err); sensorOK = false; return; }
  sensorOK = true;
  Serial.println("SCD41 started");
}

bool readSensor() {
  if (!sensorOK) return false;
  bool ready = false;
  int16_t err = scd4x.getDataReadyStatus(ready);
  if (err || !ready) return false;
  uint16_t co2;
  float t, h;
  err = scd4x.readMeasurement(co2, t, h);
  if (err) { logError("readMeasurement", err); return false; }
  if (co2 == 0) return false;
  currentCO2  = co2;
  currentTemp = t;
  currentRH   = h;
  dataValid   = true;
  Serial.printf("CO2 %u ppm  T %.2f C  RH %.1f %%\n", co2, t, h);
  return true;
}

void addGraphSample(uint16_t co2) {
  co2History[historyHead] = co2;
  historyHead = (historyHead + 1) % MAX_SAMPLES;
  if (historyCount < MAX_SAMPLES) historyCount++;
  saveHistory();
}

// ============================================================
//                       WIFI / NTP
// ============================================================

// Time of the most recent successful sync attempt (ms since boot),
// and the next time to retry if the previous sync failed.
unsigned long lastNtpAttemptMs = 0;
unsigned long nextNtpRetryMs   = 0;

void syncNtp(bool blocking) {
  if (!wifiConnected || !ntpEnabled) return;
  Serial.printf("NTP: syncing with %s\n", ntpServer.c_str());
  configTime(0, 0, ntpServer.c_str());   // UTC; tz applied at display time
  lastNtpAttemptMs = millis();

  if (blocking) {
    // First-boot sync can take 10+ seconds (DNS + UDP roundtrip on a
    // freshly-associated WiFi link). Was 5s before, which often timed out.
    struct tm ti;
    if (getLocalTime(&ti, 15000)) {
      ntpSynced = true;
      nextNtpRetryMs = 0;
      Serial.println("NTP: synced");
    } else {
      Serial.println("NTP: sync failed (will retry in background)");
      // Try again in 30 seconds
      nextNtpRetryMs = millis() + 30000;
    }
  } else {
    ntpSynced = false;
  }
}

// Periodic loop hook: if we never got NTP, try again every 30s while
// the previous attempt is still pending. Also detect deferred success
// (configTime() can succeed asynchronously after our blocking timeout).
void serviceNtp() {
  if (!wifiConnected || !ntpEnabled || ntpSynced) return;

  // Async detection: if configTime fired and time has now become valid,
  // mark synced without re-issuing the request.
  if (lastNtpAttemptMs != 0) {
    time_t now = time(nullptr);
    if (now > 1700000000) {     // any sane unix time after 2023
      ntpSynced = true;
      nextNtpRetryMs = 0;
      Serial.println("NTP: synced (deferred)");
      return;
    }
  }

  // Retry on schedule
  if (nextNtpRetryMs != 0 && (long)(millis() - nextNtpRetryMs) >= 0) {
    syncNtp(false);
    nextNtpRetryMs = millis() + 30000;
  }
}

void startMdns() {
  if (mdnsRunning) return;
  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    mdnsRunning = true;
    Serial.printf("mDNS: http://%s.local/\n", MDNS_HOSTNAME);
  } else {
    Serial.println("mDNS: failed");
  }
}

void tryWifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME);

  // Use WiFiManager to attempt connection with stored credentials.
  // setEnableConfigPortal(false) means: don't auto-launch the portal
  // on failure - we open it explicitly when the user taps the WiFi
  // tile. autoConnect returns true if connected, false otherwise.
  wm.setEnableConfigPortal(false);
  wm.setConnectTimeout(WIFI_CONNECT_MS / 1000);
  Serial.println("WiFi: attempting connect with stored credentials...");

  bool ok = wm.autoConnect();
  // Re-enable portal launch for later explicit calls
  wm.setEnableConfigPortal(true);

  if (ok && WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.printf("WiFi: connected to '%s', IP %s\n",
                  WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str());
    if (ntpEnabled) syncNtp(true);
    startMdns();
    startWebServer();
  } else {
    Serial.println("WiFi: not connected (no stored credentials or connect failed)");
  }
}

void startWifiPortal() {
  Serial.println("WiFi: starting portal");
  stopWebServer();
  if (mdnsRunning) { MDNS.end(); mdnsRunning = false; }
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);
  wm.setBreakAfterConfig(true);
  wm.startConfigPortal(WIFI_AP_NAME);
  wifiPortalActive = true;
  currentScreen = SCREEN_WIFI_SETUP;
  drawWifiSetupScreen();
}

void stopWifiPortal() {
  if (wifiPortalActive) {
    wm.stopConfigPortal();
    wifiPortalActive = false;
  }
}

void serviceWifiPortal() {
  if (!wifiPortalActive) return;
  wm.process();
  if (millis() - lastWifiUiUpdate > 2000) {
    lastWifiUiUpdate = millis();
    if (!wm.getConfigPortalActive()) {
      wifiPortalActive = false;
      WiFi.mode(WIFI_STA);
      delay(200);
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) delay(200);
      if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.print("WiFi: IP "); Serial.println(WiFi.localIP());
        if (ntpEnabled) syncNtp(true);
        startMdns();
        startWebServer();
        drawWifiSetupStatus("Connected!", TFT_GREEN);
        delay(1500);
      } else {
        wifiConnected = false;
        drawWifiSetupStatus("Not connected", TFT_RED);
        delay(1500);
      }
      currentScreen = SCREEN_SETTINGS;
      drawSettingsScreen();
    }
  }
}

// ============================================================
//                     WEB SERVER
// ============================================================

void startWebServer() {
  if (webServerRunning) return;
  server.on("/",             handleRoot);
  server.on("/data.json",    handleData);
  server.on("/history.csv",  handleHistoryCsv);
  server.on("/history.json", handleHistoryJson);
  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
  ElegantOTA.begin(&server);    // adds /update endpoint
  server.begin();
  webServerRunning = true;
  Serial.println("Web: running on port 80 (OTA at /update)");
}

void stopWebServer() {
  if (!webServerRunning) return;
  server.stop();
  webServerRunning = false;
}

void handleRoot() {
  String html;
  html.reserve(12000);
  html += F(
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>CO2 Monitor</title>"
    "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>"
    "<style>"
    "body{font-family:system-ui,sans-serif;background:#111;color:#eee;"
      "margin:0;padding:20px;max-width:760px}"
    "h1{margin:0 0 16px;border-bottom:1px solid #444;padding-bottom:8px;"
      "display:flex;justify-content:space-between;align-items:baseline}"
    "h1 small{font-size:.45em;color:#888;font-weight:400}"
    "h2{margin:24px 0 8px;font-size:1.1em;color:#bbb}"
    ".co2{font-size:5em;font-weight:700;text-align:center;margin:16px 0;"
      "line-height:1}"
    ".co2 small{font-size:.3em;color:#888;font-weight:400}"
    ".status{text-align:center;font-size:1.4em;margin-bottom:16px}"
    ".good{color:#3c3}.moderate{color:#6cf}.poor{color:#fa3}.verypoor{color:#f55}"
    ".grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
    ".tile{background:#222;border-radius:6px;padding:10px 14px}"
    ".tile .label{color:#888;font-size:.85em}"
    ".tile .value{font-size:1.4em;font-weight:600}"
    ".alert{background:#332200;border-left:3px solid #fa3;padding:8px 12px;"
      "margin:12px 0}"
    ".chart-wrap{background:#1a1a1a;border-radius:6px;padding:8px;margin:12px 0}"
    ".actions{margin:16px 0}"
    ".actions a{display:inline-block;background:#2a4060;color:#cef;"
      "padding:8px 16px;border-radius:4px;text-decoration:none;"
      "margin:4px 4px 4px 0}"
    ".actions a:hover{background:#3a557a}"
    "</style></head><body>");

  // Header with hostname
  html += F("<h1>CO2 Monitor <small>");
  html += MDNS_HOSTNAME; html += F(".local</small></h1>");

  if (dataValid) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", currentCO2);
    html += "<div class='co2 "; html += co2StatusKey(currentCO2); html += "'>";
    html += buf; html += F("<small> ppm</small></div>");
    html += "<div class='status "; html += co2StatusKey(currentCO2); html += "'>";
    html += co2StatusText(currentCO2); html += F("</div>");

    // Grid of details
    html += F("<div class='grid'>");
    snprintf(buf, sizeof(buf), "%.1f %s", displayTemp(), tempUnit());
    html += F("<div class='tile'><div class='label'>Temperature</div>"
              "<div class='value'>");
    html += buf; html += F("</div></div>");

    snprintf(buf, sizeof(buf), "%.0f %%", currentRH);
    html += F("<div class='tile'><div class='label'>Humidity</div>"
              "<div class='value'>");
    html += buf; html += F("</div></div>");

    uint16_t mn, mx, avg;
    hourStats(mn, mx, avg);
    if (avg > 0) {
      snprintf(buf, sizeof(buf), "%u ppm", avg);
      html += F("<div class='tile'><div class='label'>1-hr avg</div>"
                "<div class='value'>");
      html += buf; html += F("</div></div>");
      snprintf(buf, sizeof(buf), "%u / %u", mn, mx);
      html += F("<div class='tile'><div class='label'>1-hr min / max</div>"
                "<div class='value'>");
      html += buf; html += F("</div></div>");
    }
    html += F("</div>");
  } else {
    html += F("<div class='co2'>--<small> ppm</small></div>");
    html += F("<div class='status'>Sensor warming up...</div>");
  }

  // Calibration alerts
  uint32_t days = daysSinceLastFrc();
  if (days == UINT32_MAX) {
    html += F("<div class='alert'>Sensor has never been calibrated. "
              "Take the device outdoors and run Force Recalibrate.</div>");
  } else if (days >= FRC_REMINDER_DAYS) {
    html += F("<div class='alert'>Calibration is ");
    html += String(days);
    html += F(" days old. Consider running Force Recalibrate outdoors.</div>");
  }

  // Chart container
  html += F("<h2>Recent CO2</h2>"
            "<div class='chart-wrap'>"
            "<canvas id='c' height='240'></canvas></div>");

  // Actions
  html += F("<div class='actions'>"
            "<a href='/history.csv' download>Download Full CSV</a> "
            "<a href='/data.json'>JSON</a> "
            "<a href='/update'>Firmware Update</a>"
            "</div>");

  // Sample count info
  html += F("<p style='color:#888;font-size:.9em'>");
  html += String(logHdr.count); html += F(" samples logged out of ");
  html += String(logHdr.capacity); html += F(" capacity. ");
  html += String(historyCount); html += F(" in live graph.</p>");

  // Chart script - fetch /history.json (last 288 = 24h) and draw
  html += F("<script>"
            "fetch('/history.json?n=288').then(r=>r.json()).then(d=>{"
            "  const ctx=document.getElementById('c');"
            "  new Chart(ctx,{type:'line',data:{"
            "    labels:d.times,"
            "    datasets:[{label:'CO2 ppm',data:d.co2,"
            "      borderColor:'#6cf',borderWidth:1.5,"
            "      pointRadius:0,fill:false,tension:0.2}]"
            "  },options:{"
            "    responsive:true,maintainAspectRatio:false,"
            "    scales:{"
            "      y:{suggestedMin:400,suggestedMax:1500,"
            "         grid:{color:'#333'},ticks:{color:'#aaa'}},"
            "      x:{grid:{color:'#333'},ticks:{color:'#aaa',maxTicksLimit:8}}"
            "    },"
            "    plugins:{legend:{labels:{color:'#aaa'}}}"
            "  }});"
            "});"
            "setTimeout(()=>location.reload(),60000);"
            "</script>");

  html += F("</body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleData() {
  String j = "{";
  j += "\"co2_ppm\":";    j += String(currentCO2); j += ",";
  j += "\"temp_c\":";     j += String(currentTemp, 2); j += ",";
  j += "\"temp_f\":";     j += String(currentTemp * 9.0f / 5.0f + 32.0f, 2); j += ",";
  j += "\"rh_pct\":";     j += String(currentRH, 1); j += ",";
  j += "\"status\":\"";   j += co2StatusKey(currentCO2); j += "\",";
  j += "\"data_valid\":"; j += (dataValid ? "true" : "false"); j += ",";
  j += "\"altitude_m\":"; j += String(sensorAltitudeM); j += ",";
  j += "\"temp_offset_c\":"; j += String(tempOffsetC, 2); j += ",";
  uint32_t d = daysSinceLastFrc();
  j += "\"days_since_cal\":";
  j += (d == UINT32_MAX ? "null" : String(d));
  j += ",";
  j += "\"log_count\":";  j += String(logHdr.count); j += ",";
  j += "\"log_capacity\":"; j += String(logHdr.capacity); j += ",";
  j += "\"unix_time\":";  j += String((uint32_t)(ntpSynced ? time(nullptr) : 0));
  j += "}";
  server.send(200, "application/json", j);
}

// Stream the full log as CSV. Avoids buffering the whole thing in RAM.
void handleHistoryCsv() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=co2_history.csv");
  server.send(200, "text/csv", "");

  String header = "timestamp_utc,co2_ppm,temp_c,rh_pct\r\n";
  server.sendContent(header);

  if (!fsReady || logHdr.count == 0) return;

  File f = LittleFS.open(LOG_PATH, "r");
  if (!f) return;

  uint32_t start = (logHdr.count < logHdr.capacity) ? 0 : logHdr.head;

  String batch;
  batch.reserve(2048);
  for (uint32_t i = 0; i < logHdr.count; i++) {
    uint32_t pos = (start + i) % logHdr.capacity;
    f.seek(sizeof(LogHeader) + pos * sizeof(LogRecord));
    LogRecord rec;
    if (f.read((uint8_t*)&rec, sizeof(rec)) != sizeof(rec)) break;

    char line[80];
    if (rec.ts_utc != 0) {
      time_t t = (time_t)rec.ts_utc;
      struct tm tmv;
      gmtime_r(&t, &tmv);
      char tbuf[24];
      strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", &tmv);
      snprintf(line, sizeof(line), "%s,%u,%.2f,%.1f\r\n",
               tbuf, rec.co2_ppm,
               rec.temp_c100 / 100.0f, rec.rh_x10 / 10.0f);
    } else {
      snprintf(line, sizeof(line), ",%u,%.2f,%.1f\r\n",
               rec.co2_ppm,
               rec.temp_c100 / 100.0f, rec.rh_x10 / 10.0f);
    }
    batch += line;

    if (batch.length() > 1500) {
      server.sendContent(batch);
      batch = "";
    }
  }
  if (batch.length()) server.sendContent(batch);
  f.close();
  server.sendContent("");
}

// JSON for the chart on the main page. Returns the most recent N samples.
void handleHistoryJson() {
  uint32_t n = 288;  // default ~24h at 5 min/sample
  if (server.hasArg("n")) {
    int v = server.arg("n").toInt();
    if (v > 0 && v <= 5000) n = v;
  }
  if (n > logHdr.count) n = logHdr.count;

  String times = "[";
  String co2s  = "[";

  if (fsReady && n > 0) {
    File f = LittleFS.open(LOG_PATH, "r");
    if (f) {
      uint32_t total = logHdr.count;
      uint32_t start;
      if (total < logHdr.capacity) start = total - n;
      else start = (logHdr.head + (logHdr.capacity - n)) % logHdr.capacity;

      for (uint32_t i = 0; i < n; i++) {
        uint32_t pos = (start + i) % logHdr.capacity;
        f.seek(sizeof(LogHeader) + pos * sizeof(LogRecord));
        LogRecord rec;
        if (f.read((uint8_t*)&rec, sizeof(rec)) != sizeof(rec)) break;
        if (i > 0) { times += ","; co2s += ","; }
        char tbuf[24];
        if (rec.ts_utc != 0) {
          time_t t = (time_t)rec.ts_utc + tzOffsetMin * 60;
          struct tm tmv;
          gmtime_r(&t, &tmv);
          strftime(tbuf, sizeof(tbuf), "\"%H:%M\"", &tmv);
        } else {
          snprintf(tbuf, sizeof(tbuf), "\"-%lu\"", (unsigned long)((n - i) * 5));
        }
        times += tbuf;
        co2s  += String(rec.co2_ppm);
      }
      f.close();
    }
  }
  times += "]"; co2s += "]";

  String body = "{\"times\":";
  body += times;
  body += ",\"co2\":";
  body += co2s;
  body += "}";
  server.send(200, "application/json", body);
}

// ============================================================
//                  SERIAL COMMANDS
// ============================================================

void doFRCSerial(uint16_t target) {
  if (target < 350 || target > 2000) {
    Serial.println("FRC target must be 350-2000 ppm");
    return;
  }
  scd4x.stopPeriodicMeasurement(); delay(500);
  uint16_t correction = 0;
  int16_t err = scd4x.performForcedRecalibration(target, correction);
  if (err) logError("performForcedRecalibration", err);
  else if (correction == 0xFFFF) Serial.println("FRC failed - sensor not stable");
  else {
    int16_t d = (int16_t)(correction - 0x8000);
    Serial.printf("FRC correction: %d ppm\n", d);
    lastFrcUptimeSec = getTotalUptimeSec();
    savePref("frc_at", lastFrcUptimeSec);
  }
  scd4x.startPeriodicMeasurement();
  lastReadTime = millis();
}

void setAscFromSerial(bool enable) {
  ascEnabled = enable;
  savePref("asc", ascEnabled);
  scd4x.stopPeriodicMeasurement(); delay(20);
  scd4x.setAutomaticSelfCalibrationEnabled(ascEnabled ? 1 : 0);
  scd4x.startPeriodicMeasurement();
  lastReadTime = millis();
  Serial.printf("ASC %s\n", enable ? "enabled" : "disabled");
}

void wifiResetSerial() {
  wm.resetSettings();
  WiFi.disconnect(true, true);
  wifiConnected = false; ntpSynced = false;
  if (mdnsRunning) { MDNS.end(); mdnsRunning = false; }
  stopWebServer();
  Serial.println("WiFi: credentials cleared.");
}

void wifiStatusSerial() {
  Serial.printf("WiFi SSID: %s\n", WiFi.SSID().c_str());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected, IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("NTP synced: %s\n", ntpSynced ? "yes" : "no");
    Serial.printf("mDNS: http://%s.local/\n", MDNS_HOSTNAME);
  } else {
    Serial.println("Not connected");
  }
}

void printInfo() {
  Serial.printf("ASC:         %s\n", ascEnabled ? "ON" : "OFF");
  Serial.printf("Altitude:    %u m\n", sensorAltitudeM);
  Serial.printf("Temp off:    %.1f C  (%.1f F)\n",
                tempOffsetC, tempOffsetC * 9.0f / 5.0f);
  Serial.printf("Temp unit:   %s\n", tempInFahrenheit ? "F" : "C");
  Serial.printf("Brightness:  %u%%\n", (unsigned)brightnessPct);
  Serial.printf("Timezone:    UTC%+d:%02d\n", tzOffsetMin / 60, abs(tzOffsetMin) % 60);
  Serial.printf("Clock:       %s\n", use24hClock ? "24h" : "12h");
  Serial.printf("NTP:         %s (%s)\n", ntpEnabled ? "on" : "off", ntpServer.c_str());
  Serial.printf("Uptime:      %lu s total\n", (unsigned long)getTotalUptimeSec());
  uint32_t d = daysSinceLastFrc();
  if (d == UINT32_MAX) Serial.println("Last cal:    never");
  else                 Serial.printf("Last cal:    %lu days ago\n", (unsigned long)d);
  Serial.printf("Log:         %lu / %lu records\n",
                (unsigned long)logHdr.count, (unsigned long)logHdr.capacity);
  Serial.printf("Live graph:  %d samples\n", historyCount);
  wifiStatusSerial();
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  String lower = cmd; lower.toLowerCase();

  if      (lower.startsWith("frc "))   doFRCSerial((uint16_t) lower.substring(4).toInt());
  else if (lower == "asc on")          setAscFromSerial(true);
  else if (lower == "asc off")         setAscFromSerial(false);
  else if (lower == "wifi-setup")      { if (!wifiPortalActive) startWifiPortal(); }
  else if (lower == "wifi-reset")      wifiResetSerial();
  else if (lower == "wifi-status")     wifiStatusSerial();
  else if (lower == "info")            printInfo();
  else if (lower == "reset")           resetPrefs();
  else if (lower == "erase-log")       eraseLog();
  else Serial.println("Commands: 'frc <ppm>', 'asc on/off', "
                      "'wifi-setup', 'wifi-reset', 'wifi-status', "
                      "'info', 'reset', 'erase-log'");
}

// ============================================================
//                       SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n--- CO2 Monitor booting ---");

  disableOnboardLeds();

  loadPrefs();
  initLog();

  tft.init();
  tft.setRotation(0);
  tft.invertDisplay(INVERT_DISPLAY_COLORS);

  // IMPORTANT: must come AFTER tft.init(). TFT_eSPI's init() calls
  // pinMode/digitalWrite on the TFT_BL pin (configured as 21 in our
  // User_Setup.h), which would otherwise clobber the LEDC attach
  // and leave the pin stuck at full brightness regardless of our
  // ledcWrite() calls.
  setupBacklight();
  applyBrightness();   // honor saved level

  touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchSPI);

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("CO2 Monitor", SCREEN_W / 2, SCREEN_H / 2 - 20, 4);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Initializing...", SCREEN_W / 2, SCREEN_H / 2 + 20, 2);

  initSensor();
  delay(5500);

  wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);
  wm.setConfigPortalBlocking(false);
  wm.setBreakAfterConfig(true);
  wm.setShowInfoErase(false);

  tryWifiConnect();

  drawMainScreen();
  Serial.println("Ready. 'info' for status.");
}

void loop() {
  unsigned long now = millis();

  handleSerialCommands();
  handleTouch();

  if (wifiPortalActive) {
    serviceWifiPortal();
  } else if (wifiConnected && webServerRunning) {
    server.handleClient();
    ElegantOTA.loop();
  }

  saveUptimePeriodic();
  serviceNtp();

  // Tick the clock once a second on the main screen.
  // Also redraw the date row when the date changes (or just became
  // valid for the first time after NTP sync).
  if (currentScreen == SCREEN_MAIN && now - lastClockDraw > CLOCK_REDRAW_MS) {
    lastClockDraw = now;
    drawClockRow(false);
    static char lastDate[40] = "";
    char dnow[40];
    formatDate(dnow, sizeof(dnow));
    if (strcmp(dnow, lastDate) != 0) {
      strncpy(lastDate, dnow, sizeof(lastDate));
      drawDateRow();
    }
  }

  bool isMainScreen = (currentScreen == SCREEN_MAIN);

  if (now - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = now;
    if (readSensor() && isMainScreen) drawReadings();
  }

  if (dataValid &&
      (lastSampleTime == 0 || now - lastSampleTime >= SAMPLE_INTERVAL_MS)) {
    lastSampleTime = now;
    addGraphSample(currentCO2);
    appendLog(currentCO2, currentTemp, currentRH);
    if (isMainScreen) {
      drawGraph();
      drawReadings();   // refresh trend arrow
    }
  }
}
