/*
 * CO2 Air Quality Monitor - PORTRAIT layout (240 x 320)
 *   Board:  ESP32-2432S028R ("CYD" - Cheap Yellow Display)
 *   Sensor: Sensirion SCD41 (I2C, address 0x62)
 *
 * UI (touch):
 *   - Top-left "CAL" button   -> Settings screen
 *     (turns amber + reads "CAL!" when calibration is overdue)
 *   - Top-right sun/moon icon -> dark / light theme
 *   - Tap on temperature      -> toggle F <-> C
 *
 * Settings screen:
 *   - Sensor altitude (+- 10 m steps)
 *   - Temperature offset (in your displayed unit)
 *   - Force Recalibrate (with confirmation, must be done outdoors)
 *   - WiFi tile: shows IP when connected, or "Tap to set up" otherwise.
 *     Tap to launch the WiFiManager captive portal.
 *
 * WiFi setup:
 *   No serial typing needed. Tap the WiFi tile in Settings; the device
 *   becomes an access point named "CO2-Monitor-Setup". Connect a phone
 *   or computer to that network. A captive portal opens automatically
 *   (or visit http://192.168.4.1). Pick your network, enter the
 *   password. The device saves the credentials and reconnects.
 *
 * Web endpoints (when WiFi is connected):
 *   /              auto-refreshing HTML status page
 *   /data.json     current readings as JSON
 *   /history.csv   full sample history
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
 *   - WiFiManager              by tzapu
 *
 * TFT_eSPI User_Setup.h block (in libraries/TFT_eSPI/User_Setup.h):
 *   #define ILI9341_2_DRIVER     // try ILI9341_DRIVER if needed
 *   #define TFT_MISO 12
 *   #define TFT_MOSI 13
 *   #define TFT_SCLK 14
 *   #define TFT_CS   15
 *   #define TFT_DC    2
 *   #define TFT_RST  -1
 *   #define TFT_BL   21
 *   #define TFT_BACKLIGHT_ON HIGH
 *   #define LOAD_GLCD
 *   #define LOAD_FONT2
 *   #define LOAD_FONT4
 *   #define LOAD_FONT6
 *   #define LOAD_FONT7
 *   #define SMOOTH_FONT
 *   #define SPI_FREQUENCY 55000000
 *
 * Serial commands (115200 baud):
 *   wifi-setup     start the WiFi config portal
 *   wifi-reset     forget WiFi credentials
 *   wifi-status    print connection state and IP
 *   frc <ppm>      manual Forced Recalibration
 *   asc on / asc off  toggle Automatic Self-Calibration
 *                     (off by default; not exposed in the UI)
 *   info           print all current settings
 *   reset          clear all stored preferences
 */

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <time.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <SensirionI2cScd4x.h>

// ----------------- Pins -----------------
#define I2C_SDA_PIN     27
#define I2C_SCL_PIN     22
#define BACKLIGHT_PIN   21

#define XPT2046_IRQ     36
#define XPT2046_MOSI    32
#define XPT2046_MISO    39
#define XPT2046_CLK     25
#define XPT2046_CS      33

// On-board RGB LED (active LOW: HIGH = off)
#define LED_R           4
#define LED_G           16
#define LED_B           17

// ----------------- Behaviour -----------------
#define READ_INTERVAL_MS    5000UL
#define SAMPLE_INTERVAL_MS  (5UL * 60UL * 1000UL)
#define MAX_SAMPLES         60
#define TAP_DEBOUNCE_MS     250
#define UPTIME_SAVE_MS      (60UL * 60UL * 1000UL)
#define WIFI_CONNECT_MS     10000UL
#define WIFI_PORTAL_TIMEOUT 180         // seconds
#define WIFI_AP_NAME        "CO2-Monitor-Setup"
#define FRC_REMINDER_DAYS   90
#define FRC_TARGET_PPM      420

// SCD41 first-boot defaults
#define DEFAULT_ASC          false
#define DEFAULT_ALTITUDE_M   340      // Tempe AZ
#define DEFAULT_TEMP_OFFSET  4.0f
#define DEFAULT_DARK_MODE    true
#define DEFAULT_TEMP_F       true

// Setting ranges (Celsius internally for offset)
#define ALT_MIN              0
#define ALT_MAX              3000
#define ALT_STEP             10
#define TEMP_OFF_MIN         -5.0f
#define TEMP_OFF_MAX         15.0f
#define TEMP_OFF_STEP_C      0.5f
#define TEMP_OFF_STEP_F_IN_C (5.0f / 9.0f)

// CO2 thresholds (good -> bad: green / blue / orange / red)
#define CO2_GOOD_MAX        800
#define CO2_MODERATE_MAX    1200
#define CO2_POOR_MAX        2000
#define CO2_CHART_MIN       400
#define CO2_CHART_MAX       2000

// Flip if green looks magenta or red looks cyan on your CYD revision.
#define INVERT_DISPLAY_COLORS true

// ----------------- Touch calibration -----------------
// Calibrated from corner taps. To re-tune: tap each corner of the
// screen and watch the TAP raw=... lines on serial. Set MIN to the
// raw value at the low edge of each axis and MAX to the raw value
// at the high edge.
//   top-left  -> (0, 0)
//   top-right -> (240, 0)
//   bot-left  -> (0, 320)
//   bot-right -> (240, 320)
#define TOUCH_PRESSURE_MIN   50
#define TS_X_MIN             500    // raw.x at LEFT edge
#define TS_X_MAX             3700   // raw.x at RIGHT edge
#define TS_Y_MIN             350    // raw.y at TOP edge
#define TS_Y_MAX             3700   // raw.y at BOTTOM edge
#define TS_SWAP_XY           0

// ----------------- Layout (portrait 240 x 320) -----------------
#define SCREEN_W        240
#define SCREEN_H        320

#define TITLE_H         28
#define BTN_W           44

#define CO2_LABEL_Y     34
#define CO2_VALUE_Y     50
#define CO2_UNITS_Y     104
#define STATUS_Y        130

#define TR_TOP          164
#define TR_VALUE_Y      184

#define GRAPH_X         30
#define GRAPH_CAPTION_Y 222
#define GRAPH_Y         236
#define GRAPH_W         205
#define GRAPH_H         74

// Settings screen
#define SET_HEADER_H    32
#define SET_ROW_H       60
#define SET_ROW1_Y      40
#define SET_ROW2_Y      104
#define SET_LASTCAL_Y   172
#define SET_FRC_Y       196
#define SET_FRC_H       56
#define SET_WIFI_Y      264
#define SET_WIFI_H      50

// ----------------- Theme -----------------
struct Theme {
  uint16_t bg, text, textDim;
  uint16_t titleBg, titleText;
  uint16_t graphAxis;
  uint16_t btnBg, btnText;
  uint16_t dialogBg, accent;
};

const Theme darkTheme = {
  TFT_BLACK, TFT_WHITE, TFT_LIGHTGREY,
  TFT_NAVY,  TFT_WHITE,
  TFT_WHITE,
  0x10A2,    TFT_WHITE,
  0x18C3,    TFT_CYAN,
};

const Theme lightTheme = {
  TFT_WHITE, TFT_BLACK, 0x4208,
  TFT_NAVY,  TFT_WHITE,
  TFT_BLACK,
  0x9CD3,    TFT_WHITE,
  0xE71C,    TFT_BLUE,
};

// ----------------- Hit-test rectangles -----------------
struct Rect {
  int16_t x, y, w, h;
  bool contains(int16_t px, int16_t py) const {
    return px >= x && px < x + w && py >= y && py < y + h;
  }
};

// Main screen
const Rect btnCal       = {0,                     0, BTN_W + 12, TITLE_H + 6};
const Rect btnTheme     = {SCREEN_W - BTN_W - 12, 0, BTN_W + 12, TITLE_H + 6};
const Rect btnTempUnit  = {0, TR_TOP - 6, SCREEN_W / 2, 60};

// Settings screen
const Rect btnSetBack    = {0, 0, 70, SET_HEADER_H + 6};
const Rect btnAltMinus   = {SCREEN_W - 116, SET_ROW1_Y + 14, 36, 32};
const Rect btnAltPlus    = {SCREEN_W - 44,  SET_ROW1_Y + 14, 36, 32};
const Rect btnTempMinus  = {SCREEN_W - 116, SET_ROW2_Y + 14, 36, 32};
const Rect btnTempPlus   = {SCREEN_W - 44,  SET_ROW2_Y + 14, 36, 32};
const Rect btnForceRecal = {12, SET_FRC_Y, SCREEN_W - 24, SET_FRC_H};
const Rect btnWifiTile   = {12, SET_WIFI_Y, SCREEN_W - 24, SET_WIFI_H};

// Dialog
const Rect btnDlgCancel  = {12,             SCREEN_H - 64, 102, 52};
const Rect btnDlgConfirm = {SCREEN_W - 114, SCREEN_H - 64, 102, 52};

// WiFi setup screen
const Rect btnWifiBack   = {0, 0, 70, SET_HEADER_H + 6};

// ----------------- Screens -----------------
enum Screen {
  SCREEN_MAIN,
  SCREEN_SETTINGS,
  SCREEN_CONFIRM_CAL,
  SCREEN_CALIBRATING,
  SCREEN_CAL_RESULT,
  SCREEN_WIFI_SETUP,
};

// ----------------- Globals -----------------
TFT_eSPI            tft = TFT_eSPI();
// Touch must be on a SEPARATE SPI bus from the display.
// TFT_eSPI uses VSPI by default on ESP32 (unless USE_HSPI_PORT is set
// in User_Setup.h). So we put touch on HSPI to avoid conflict.
SPIClass            touchSPI(HSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
SensirionI2cScd4x   scd4x;
Preferences         prefs;
WebServer           server(80);
WiFiManager         wm;

// User-tunable settings
bool     ascEnabled       = DEFAULT_ASC;
uint16_t sensorAltitudeM  = DEFAULT_ALTITUDE_M;
float    tempOffsetC      = DEFAULT_TEMP_OFFSET;
bool     darkMode         = DEFAULT_DARK_MODE;
bool     tempInFahrenheit = DEFAULT_TEMP_F;

// Time tracking (seconds)
uint32_t baseUptimeSec    = 0;
uint32_t lastFrcUptimeSec = 0;
unsigned long lastUptimeSaveMs = 0;

// WiFi state
bool     wifiConnected      = false;
bool     ntpSynced          = false;
bool     webServerRunning   = false;
bool     wifiPortalActive   = false;
unsigned long lastWifiUiUpdate = 0;

Screen   currentScreen    = SCREEN_MAIN;
bool     calSuccess       = false;
char     calResultMsg[80] = "";

// CO2 history
uint16_t co2History[MAX_SAMPLES];
int      historyCount = 0;
int      historyHead  = 0;

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

const Theme& T() { return darkMode ? darkTheme : lightTheme; }

// ----------------- Forward decls -----------------
void drawDialogButton(const Rect& r, const char* label,
                      uint16_t bg, uint16_t fg);
void drawSmallButton(const Rect& r, const char* label,
                     uint16_t bg, uint16_t fg, uint8_t font);
void drawWifiTile(const Rect& r);

void drawMainScreen();
void drawSettingsScreen();
void drawConfirmCalScreen();
void drawCalibratingScreen();
void drawCalResultScreen();
void drawWifiSetupScreen();
void drawTitleBar();
void drawReadings();
void drawGraph();
void runForcedCalibration();
bool applySensorSettings();
void tryWifiConnect();
void startWifiPortal();
void stopWifiPortal();
void startWebServer();
void stopWebServer();
void handleRoot();
void handleData();
void handleCsv();

// ----------------- Helpers -----------------
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

int co2ToY(uint16_t co2) {
  if (co2 < CO2_CHART_MIN) co2 = CO2_CHART_MIN;
  if (co2 > CO2_CHART_MAX) co2 = CO2_CHART_MAX;
  long range = CO2_CHART_MAX - CO2_CHART_MIN;
  return GRAPH_Y + GRAPH_H - (int)(((long)(co2 - CO2_CHART_MIN) * GRAPH_H) / range);
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

// ----------------- LEDs (turn off on-board RGB) -----------------
void disableOnboardLeds() {
  pinMode(LED_R, OUTPUT); digitalWrite(LED_R, HIGH);
  pinMode(LED_G, OUTPUT); digitalWrite(LED_G, HIGH);
  pinMode(LED_B, OUTPUT); digitalWrite(LED_B, HIGH);
}

// ----------------- Preferences -----------------
void loadPrefs() {
  prefs.begin("co2mon", false);
  ascEnabled       = prefs.getBool ("asc",     DEFAULT_ASC);
  sensorAltitudeM  = prefs.getUShort("alt",    DEFAULT_ALTITUDE_M);
  tempOffsetC      = prefs.getFloat ("tempoff", DEFAULT_TEMP_OFFSET);
  darkMode         = prefs.getBool ("dark",    DEFAULT_DARK_MODE);
  tempInFahrenheit = prefs.getBool ("tempf",   DEFAULT_TEMP_F);
  baseUptimeSec    = prefs.getUInt ("uptime",  0);
  lastFrcUptimeSec = prefs.getUInt ("frc_at",  0);

  size_t bytesAvail = prefs.getBytesLength("hist");
  if (bytesAvail == sizeof(co2History)) {
    prefs.getBytes("hist", co2History, sizeof(co2History));
    historyCount = prefs.getUShort("histcnt", 0);
    historyHead  = prefs.getUShort("histhd",  0);
    if (historyCount > MAX_SAMPLES) historyCount = MAX_SAMPLES;
    if (historyHead  >= MAX_SAMPLES) historyHead  = 0;
  }
  prefs.end();

  Serial.printf("Loaded: alt=%u toff=%.1f dark=%d tempF=%d hist=%d uptime=%lu frc_at=%lu\n",
                sensorAltitudeM, tempOffsetC, darkMode, tempInFahrenheit,
                historyCount, (unsigned long)baseUptimeSec,
                (unsigned long)lastFrcUptimeSec);
}

void savePref(const char* key, bool v)     { prefs.begin("co2mon", false); prefs.putBool   (key, v); prefs.end(); }
void savePref(const char* key, uint16_t v) { prefs.begin("co2mon", false); prefs.putUShort (key, v); prefs.end(); }
void savePref(const char* key, uint32_t v) { prefs.begin("co2mon", false); prefs.putUInt   (key, v); prefs.end(); }
void savePref(const char* key, float v)    { prefs.begin("co2mon", false); prefs.putFloat  (key, v); prefs.end(); }

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

// ----------------- Touch -----------------
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

// ----------------- Icons -----------------
void drawSunIcon(int cx, int cy, uint16_t color) {
  tft.fillCircle(cx, cy, 4, color);
  tft.drawLine(cx - 8, cy,     cx - 6, cy,     color);
  tft.drawLine(cx + 6, cy,     cx + 8, cy,     color);
  tft.drawLine(cx,     cy - 8, cx,     cy - 6, color);
  tft.drawLine(cx,     cy + 6, cx,     cy + 8, color);
  tft.drawLine(cx - 6, cy - 6, cx - 5, cy - 5, color);
  tft.drawLine(cx + 5, cy + 5, cx + 6, cy + 6, color);
  tft.drawLine(cx - 6, cy + 6, cx - 5, cy + 5, color);
  tft.drawLine(cx + 5, cy - 5, cx + 6, cy - 6, color);
}

void drawMoonIcon(int cx, int cy, uint16_t color, uint16_t bgColor) {
  tft.fillCircle(cx, cy, 7, color);
  tft.fillCircle(cx + 3, cy - 2, 6, bgColor);
}

// ----------------- Title bar -----------------
void drawTitleBar() {
  tft.fillRect(0, 0, SCREEN_W, TITLE_H, T().titleBg);

  int16_t bx = 4, by = 4;
  int16_t bw = BTN_W, bh = TITLE_H - 8;

  bool overdue = calibrationOverdue();
  uint16_t calBg = overdue ? TFT_ORANGE : T().btnBg;
  uint16_t calFg = overdue ? TFT_BLACK  : T().btnText;
  tft.fillRoundRect(bx, by, bw, bh, 4, calBg);
  tft.drawRoundRect(bx, by, bw, bh, 4, calFg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(calFg, calBg);
  tft.drawString(overdue ? "CAL!" : "CAL", bx + bw / 2, by + bh / 2 + 1, 2);

  tft.setTextColor(T().titleText, T().titleBg);
  tft.drawString("CO2 Monitor", SCREEN_W / 2, TITLE_H / 2, 2);

  int16_t tx = SCREEN_W - bw - 4;
  tft.fillRoundRect(tx, by, bw, bh, 4, T().btnBg);
  tft.drawRoundRect(tx, by, bw, bh, 4, T().btnText);
  if (darkMode) drawSunIcon(tx + bw / 2, by + bh / 2 + 1, TFT_YELLOW);
  else          drawMoonIcon(tx + bw / 2, by + bh / 2 + 1, TFT_DARKGREY, T().btnBg);
}

// ----------------- Graph -----------------
void drawThresholdLines() {
  int y800  = co2ToY(800);
  int y1200 = co2ToY(1200);
  uint16_t cGood = darkMode ? TFT_DARKGREEN : 0x9F93;
  uint16_t cWarn = darkMode ? 0x4225        : 0x8DBE;
  for (int x = GRAPH_X + 2; x < GRAPH_X + GRAPH_W; x += 4) {
    tft.drawPixel(x, y800,  cGood);
    tft.drawPixel(x, y1200, cWarn);
  }
}

void drawGraphFrame() {
  tft.drawFastVLine(GRAPH_X,           GRAPH_Y, GRAPH_H, T().graphAxis);
  tft.drawFastHLine(GRAPH_X, GRAPH_Y + GRAPH_H, GRAPH_W, T().graphAxis);

  tft.setTextColor(T().textDim, T().bg);
  tft.setTextDatum(MR_DATUM);
  tft.drawString("2000", GRAPH_X - 2, GRAPH_Y,            1);
  tft.drawString("1200", GRAPH_X - 2, co2ToY(1200),       1);
  tft.drawString("800",  GRAPH_X - 2, co2ToY(800),        1);
  tft.drawString("400",  GRAPH_X - 2, GRAPH_Y + GRAPH_H,  1);

  tft.setTextDatum(TR_DATUM);
  tft.drawString("CO2 ppm  -  5 min/sample", SCREEN_W - 4, GRAPH_CAPTION_Y, 1);
}

void drawGraph() {
  tft.fillRect(GRAPH_X + 1, GRAPH_Y, GRAPH_W - 1, GRAPH_H, T().bg);
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

// ----------------- Readings -----------------
void drawReadings() {
  char buf[16];

  if (!dataValid) {
    tft.fillRect(0, CO2_VALUE_Y - 2, SCREEN_W,
                 CO2_UNITS_Y - CO2_VALUE_Y + 18, T().bg);
    tft.setTextColor(T().textDim, T().bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("warming up...", SCREEN_W / 2, CO2_VALUE_Y + 24, 4);
    return;
  }

  tft.fillRect(0, CO2_VALUE_Y, SCREEN_W, 56, T().bg);
  tft.setTextColor(co2Color(currentCO2), T().bg);
  tft.setTextDatum(TC_DATUM);
  snprintf(buf, sizeof(buf), "%4u", currentCO2);
  tft.drawString(buf, SCREEN_W / 2, CO2_VALUE_Y, 7);

  tft.fillRect(0, CO2_UNITS_Y, SCREEN_W, 18, T().bg);
  tft.setTextColor(T().textDim, T().bg);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("ppm", SCREEN_W / 2, CO2_UNITS_Y, 2);

  tft.fillRect(0, STATUS_Y - 4, SCREEN_W, 30, T().bg);
  tft.setTextColor(co2Color(currentCO2), T().bg);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(co2StatusText(currentCO2), SCREEN_W / 2, STATUS_Y, 4);

  tft.fillRect(0, TR_VALUE_Y, SCREEN_W / 2, 30, T().bg);
  tft.setTextColor(T().text, T().bg);
  tft.setTextDatum(TC_DATUM);
  snprintf(buf, sizeof(buf), "%.1f", displayTemp());
  tft.drawString(buf, SCREEN_W / 4 - 6, TR_VALUE_Y, 4);
  tft.setTextColor(T().textDim, T().bg);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(tempUnit(), SCREEN_W / 4 + 30, TR_VALUE_Y + 4, 2);

  tft.fillRect(SCREEN_W / 2, TR_VALUE_Y, SCREEN_W / 2, 30, T().bg);
  tft.setTextColor(T().text, T().bg);
  tft.setTextDatum(TC_DATUM);
  snprintf(buf, sizeof(buf), "%.0f", currentRH);
  tft.drawString(buf, SCREEN_W * 3 / 4 - 6, TR_VALUE_Y, 4);
  tft.setTextColor(T().textDim, T().bg);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("%", SCREEN_W * 3 / 4 + 18, TR_VALUE_Y + 4, 2);
}

// ----------------- Main screen -----------------
void drawMainScreen() {
  tft.fillScreen(T().bg);
  drawTitleBar();

  tft.setTextColor(T().textDim, T().bg);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("CO2",  SCREEN_W / 2, CO2_LABEL_Y, 2);

  tft.setTextDatum(TC_DATUM);
  tft.drawString("TEMP", SCREEN_W / 4,     TR_TOP, 2);
  tft.drawString("RH",   SCREEN_W * 3 / 4, TR_TOP, 2);

  drawGraphFrame();
  drawThresholdLines();
  drawReadings();
  drawGraph();
}

// ----------------- Buttons -----------------
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

// WiFi tile shows status and is always tappable to start setup
void drawWifiTile(const Rect& r) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 8, T().btnBg);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 8, T().btnText);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(T().btnText, T().btnBg);
  tft.drawString("WiFi", r.x + 12, r.y + 8, 2);

  tft.setTextDatum(TR_DATUM);
  if (wifiConnected) {
    String ip = WiFi.localIP().toString();
    tft.setTextColor(TFT_GREEN, T().btnBg);
    tft.drawString(ip.c_str(), r.x + r.w - 12, r.y + 8, 2);
    tft.setTextDatum(BR_DATUM);
    tft.setTextColor(T().textDim, T().btnBg);
    tft.drawString("Tap to reconfigure", r.x + r.w - 12, r.y + r.h - 8, 1);
  } else {
    tft.setTextColor(TFT_ORANGE, T().btnBg);
    tft.drawString("Not connected", r.x + r.w - 12, r.y + 8, 2);
    tft.setTextDatum(BR_DATUM);
    tft.setTextColor(T().textDim, T().btnBg);
    tft.drawString("Tap to set up", r.x + r.w - 12, r.y + r.h - 8, 1);
  }
}

// ----------------- Settings screen -----------------
void drawAdjusterRow(int y, const char* label, const char* valueText,
                     const Rect& minusRect, const Rect& plusRect) {
  tft.setTextColor(T().text, T().bg);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(label, 16, y + SET_ROW_H / 2, 4);

  drawSmallButton(minusRect, "-", T().btnBg, T().btnText, 4);
  drawSmallButton(plusRect,  "+", T().btnBg, T().btnText, 4);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(T().text, T().bg);
  int cx = (minusRect.x + minusRect.w + plusRect.x) / 2;
  tft.drawString(valueText, cx, y + SET_ROW_H / 2, 2);

  tft.drawFastHLine(8, y + SET_ROW_H, SCREEN_W - 16, T().textDim);
}

void drawSettingsScreen() {
  tft.fillScreen(T().bg);

  tft.fillRect(0, 0, SCREEN_W, SET_HEADER_H, T().titleBg);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(T().titleText, T().titleBg);
  tft.drawString("< Back", 8, SET_HEADER_H / 2, 2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Settings", SCREEN_W / 2, SET_HEADER_H / 2, 4);

  char buf[24];

  snprintf(buf, sizeof(buf), "%u m", sensorAltitudeM);
  drawAdjusterRow(SET_ROW1_Y, "Altitude", buf, btnAltMinus, btnAltPlus);

  snprintf(buf, sizeof(buf), "%+.1f %s", tempOffsetDisplay(), tempUnit());
  drawAdjusterRow(SET_ROW2_Y, "Temp Off.", buf, btnTempMinus, btnTempPlus);

  // Last calibration line
  tft.setTextDatum(MC_DATUM);
  uint32_t days = daysSinceLastFrc();
  if (days == UINT32_MAX) {
    tft.setTextColor(T().textDim, T().bg);
    tft.drawString("Never calibrated", SCREEN_W / 2, SET_LASTCAL_Y, 2);
  } else {
    bool overdue = days >= FRC_REMINDER_DAYS;
    tft.setTextColor(overdue ? TFT_ORANGE : T().text, T().bg);
    snprintf(buf, sizeof(buf), "Last cal: %lu day%s ago",
             (unsigned long)days, days == 1 ? "" : "s");
    tft.drawString(buf, SCREEN_W / 2, SET_LASTCAL_Y, 2);
  }

  drawDialogButton(btnForceRecal, "Force Recalibrate",
                   TFT_DARKGREEN, TFT_WHITE);

  drawWifiTile(btnWifiTile);
}

// ----------------- Word-wrapped text -----------------
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

// ----------------- Calibration screens -----------------
void drawConfirmCalScreen() {
  tft.fillScreen(T().dialogBg);
  tft.fillRect(0, 0, SCREEN_W, 36, T().titleBg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(T().titleText, T().titleBg);
  tft.drawString("Force Calibration", SCREEN_W / 2, 18, 4);

  drawWrappedText(
    "Take the device OUTSIDE in fresh air, away from people and "
    "exhaust. Wait 3+ minutes for the reading to stabilise. Then "
    "tap CALIBRATE to set the current reading to 420 ppm "
    "(typical outdoor CO2).",
    12, 50, SCREEN_W - 24, 2, T().text, T().dialogBg, 18);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(T().accent, T().dialogBg);
  tft.drawString("Cancel if you are still indoors.",
                 SCREEN_W / 2, 220, 2);

  drawDialogButton(btnDlgCancel,  "Cancel", T().btnBg,    T().btnText);
  drawDialogButton(btnDlgConfirm, "Calib.", TFT_DARKGREEN, TFT_WHITE);
}

void drawCalibratingScreen() {
  tft.fillScreen(T().dialogBg);
  tft.fillRect(0, 0, SCREEN_W, 36, T().titleBg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(T().titleText, T().titleBg);
  tft.drawString("Calibrating...", SCREEN_W / 2, 18, 4);
  tft.setTextColor(T().text, T().dialogBg);
  tft.drawString("Holding sensor steady", SCREEN_W / 2, 140, 2);
  tft.drawString("and writing reference", SCREEN_W / 2, 162, 2);
  tft.drawString("to the SCD41.",         SCREEN_W / 2, 184, 2);
}

void drawCalResultScreen() {
  tft.fillScreen(T().dialogBg);
  tft.fillRect(0, 0, SCREEN_W, 36, T().titleBg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(T().titleText, T().titleBg);
  tft.drawString(calSuccess ? "Calibration Done" : "Calibration Failed",
                 SCREEN_W / 2, 18, 4);

  uint16_t resultColor = calSuccess ? TFT_GREEN : TFT_RED;
  tft.setTextColor(resultColor, T().dialogBg);
  tft.drawString(calSuccess ? "Success" : "Failed",
                 SCREEN_W / 2, 100, 4);

  drawWrappedText(calResultMsg, 16, 150, SCREEN_W - 32, 2,
                  T().text, T().dialogBg, 20);

  tft.setTextColor(T().textDim, T().dialogBg);
  tft.drawString("Tap anywhere to return", SCREEN_W / 2, SCREEN_H - 30, 2);
}

// ----------------- WiFi setup screen -----------------
void drawWifiSetupHeader() {
  tft.fillRect(0, 0, SCREEN_W, SET_HEADER_H, T().titleBg);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(T().titleText, T().titleBg);
  tft.drawString("< Back", 8, SET_HEADER_H / 2, 2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("WiFi Setup", SCREEN_W / 2, SET_HEADER_H / 2, 4);
}

void drawWifiSetupStatus(const char* state, uint16_t color) {
  // Re-paint status area without redrawing whole screen
  tft.fillRect(0, 250, SCREEN_W, 40, T().bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(T().textDim, T().bg);
  tft.drawString("Status:", SCREEN_W / 2, 254, 2);
  tft.setTextColor(color, T().bg);
  tft.drawString(state, SCREEN_W / 2, 274, 4);
}

void drawWifiSetupScreen() {
  tft.fillScreen(T().bg);
  drawWifiSetupHeader();

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(T().text, T().bg);
  tft.drawString("On your phone or PC:", 12, 48, 2);

  tft.setTextColor(T().textDim, T().bg);
  tft.drawString("1. Connect to WiFi:", 16, 76, 2);
  tft.setTextColor(T().accent, T().bg);
  tft.drawString(WIFI_AP_NAME, 16, 96, 4);

  tft.setTextColor(T().textDim, T().bg);
  tft.drawString("2. A page should open.", 16, 134, 2);
  tft.drawString("   Or visit:", 16, 154, 2);
  tft.setTextColor(T().accent, T().bg);
  tft.drawString("192.168.4.1", 16, 174, 4);

  tft.setTextColor(T().textDim, T().bg);
  tft.drawString("3. Pick your network", 16, 212, 2);
  tft.drawString("   and enter password.", 16, 230, 2);

  drawWifiSetupStatus("Active", TFT_GREEN);
}

// ----------------- Force calibration -----------------
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
    Serial.print("FRC error: "); Serial.println(err);
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

// ----------------- Settings change handlers -----------------
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

// ----------------- Touch dispatch -----------------
void handleTouch() {
  int16_t x, y;
  if (!readTap(x, y)) return;

  if (currentScreen == SCREEN_MAIN) {
    if (btnCal.contains(x, y)) {
      currentScreen = SCREEN_SETTINGS;
      drawSettingsScreen();
    } else if (btnTheme.contains(x, y)) {
      darkMode = !darkMode;
      savePref("dark", darkMode);
      drawMainScreen();
    } else if (btnTempUnit.contains(x, y) && dataValid) {
      tempInFahrenheit = !tempInFahrenheit;
      savePref("tempf", tempInFahrenheit);
      drawReadings();
    }
  } else if (currentScreen == SCREEN_SETTINGS) {
    if (btnSetBack.contains(x, y)) {
      currentScreen = SCREEN_MAIN;
      drawMainScreen();
    } else if (btnAltMinus.contains(x, y))   { changeAltitude(-ALT_STEP); drawSettingsScreen(); }
    else if   (btnAltPlus.contains(x, y))    { changeAltitude(+ALT_STEP); drawSettingsScreen(); }
    else if   (btnTempMinus.contains(x, y))  { changeTempOffset(-1);      drawSettingsScreen(); }
    else if   (btnTempPlus.contains(x, y))   { changeTempOffset(+1);      drawSettingsScreen(); }
    else if   (btnForceRecal.contains(x, y)) {
      currentScreen = SCREEN_CONFIRM_CAL;
      drawConfirmCalScreen();
    } else if (btnWifiTile.contains(x, y)) {
      startWifiPortal();
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

// ----------------- Sensor -----------------
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

// ----------------- WiFi -----------------
// Try to connect using credentials saved by ESP32/WiFiManager.
// Non-blocking after the timeout window. No portal is started here.
void tryWifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("co2monitor");
  WiFi.begin();   // uses last-saved SSID/password from system NVS

  if (WiFi.SSID().length() == 0) {
    Serial.println("WiFi: no saved credentials, skipping connect");
    return;
  }

  Serial.printf("WiFi: connecting to '%s'", WiFi.SSID().c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.print("WiFi: connected, IP "); Serial.println(WiFi.localIP());

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5000)) {
      ntpSynced = true;
      Serial.println("NTP: synced");
    }
    startWebServer();
  } else {
    Serial.println("WiFi: connect failed");
  }
}

void startWifiPortal() {
  Serial.println("WiFi: starting config portal");
  stopWebServer();   // free port 80 for the portal

  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);
  wm.setBreakAfterConfig(true);   // exit after creds are saved

  wm.startConfigPortal(WIFI_AP_NAME);
  wifiPortalActive = true;

  currentScreen = SCREEN_WIFI_SETUP;
  drawWifiSetupScreen();
}

void stopWifiPortal() {
  if (wifiPortalActive) {
    wm.stopConfigPortal();
    wifiPortalActive = false;
    Serial.println("WiFi: portal cancelled");
  }
}

void serviceWifiPortal() {
  if (!wifiPortalActive) return;
  wm.process();

  // Periodically update the on-screen status while portal is active
  if (millis() - lastWifiUiUpdate > 2000) {
    lastWifiUiUpdate = millis();

    if (!wm.getConfigPortalActive()) {
      // Portal closed (success, timeout, or stop)
      wifiPortalActive = false;
      Serial.println("WiFi: portal closed");

      // Give the chip a moment to settle into STA mode
      WiFi.mode(WIFI_STA);
      delay(200);

      // Did we end up connected?
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
        delay(200);
      }

      if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.print("WiFi: connected, IP "); Serial.println(WiFi.localIP());
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        struct tm ti;
        ntpSynced = getLocalTime(&ti, 3000);
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

// ----------------- Web server -----------------
void startWebServer() {
  if (webServerRunning) return;
  server.on("/",            handleRoot);
  server.on("/data.json",   handleData);
  server.on("/history.csv", handleCsv);
  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
  server.begin();
  webServerRunning = true;
  Serial.println("Web server started on port 80");
}

void stopWebServer() {
  if (!webServerRunning) return;
  server.stop();
  webServerRunning = false;
  Serial.println("Web server stopped");
}

void handleRoot() {
  String html;
  html.reserve(8192);
  html += F(
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='30'>"
    "<title>CO2 Monitor</title>"
    "<style>"
    "body{font-family:system-ui,sans-serif;background:#111;color:#eee;"
      "margin:0;padding:20px;max-width:640px}"
    "h1{margin:0 0 16px;border-bottom:1px solid #444;padding-bottom:8px}"
    "h2{margin-top:24px}"
    ".co2{font-size:5em;font-weight:700;text-align:center;margin:16px 0;"
      "line-height:1}"
    ".co2 small{font-size:.3em;color:#888;font-weight:400}"
    ".status{text-align:center;font-size:1.5em;margin-bottom:24px}"
    ".good{color:#3c3}.moderate{color:#6cf}.poor{color:#fa3}.verypoor{color:#f55}"
    ".row{display:flex;justify-content:space-between;padding:8px 12px;"
      "background:#222;border-radius:4px;margin:6px 0}"
    ".alert{background:#332200;border-left:3px solid #fa3;padding:8px 12px;"
      "margin:12px 0}"
    "table{width:100%;border-collapse:collapse;margin-top:8px}"
    "th,td{text-align:left;padding:6px 12px;border-bottom:1px solid #333}"
    ".actions a{display:inline-block;background:#2a4060;color:#cef;"
      "padding:8px 16px;border-radius:4px;text-decoration:none;"
      "margin:4px 4px 4px 0}"
    ".actions a:hover{background:#3a557a}"
    "</style></head><body>");

  html += F("<h1>CO2 Air Monitor</h1>");

  if (dataValid) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", currentCO2);
    html += "<div class='co2 "; html += co2StatusKey(currentCO2); html += "'>";
    html += buf;
    html += F("<small> ppm</small></div>");

    html += "<div class='status "; html += co2StatusKey(currentCO2); html += "'>";
    html += co2StatusText(currentCO2);
    html += F("</div>");

    snprintf(buf, sizeof(buf), "%.1f %s", displayTemp(), tempUnit());
    html += F("<div class='row'><span>Temperature</span><span>");
    html += buf; html += F("</span></div>");

    snprintf(buf, sizeof(buf), "%.0f %%", currentRH);
    html += F("<div class='row'><span>Humidity</span><span>");
    html += buf; html += F("</span></div>");
  } else {
    html += F("<div class='co2'>--<small> ppm</small></div>");
    html += F("<div class='status'>Sensor warming up...</div>");
  }

  uint32_t days = daysSinceLastFrc();
  if (days == UINT32_MAX) {
    html += F("<div class='alert'>Sensor has never been calibrated. "
              "Take the device outdoors and use Force Recalibrate.</div>");
  } else if (days >= FRC_REMINDER_DAYS) {
    html += F("<div class='alert'>Calibration is ");
    html += String(days);
    html += F(" days old. Consider running Force Recalibrate outdoors.</div>");
  } else {
    html += F("<div class='row'><span>Last calibrated</span><span>");
    html += String(days); html += F(" days ago</span></div>");
  }

  html += F("<div class='row'><span>Altitude</span><span>");
  html += String(sensorAltitudeM); html += F(" m</span></div>");
  html += F("<div class='row'><span>Temp offset</span><span>");
  char buf2[16]; snprintf(buf2, sizeof(buf2), "%+.1f %s",
                          tempOffsetDisplay(), tempUnit());
  html += buf2; html += F("</span></div>");
  html += F("<div class='row'><span>Total uptime</span><span>");
  uint32_t up = getTotalUptimeSec();
  snprintf(buf2, sizeof(buf2), "%lu d %lu h",
           (unsigned long)(up / 86400UL), (unsigned long)((up / 3600UL) % 24));
  html += buf2; html += F("</span></div>");

  html += F("<div class='actions'>"
            "<a href='/history.csv' download>Download CSV</a> "
            "<a href='/data.json'>JSON</a></div>");

  if (historyCount > 0) {
    html += F("<h2>Recent samples</h2><table>"
              "<tr><th>Time ago</th><th>CO2 (ppm)</th></tr>");
    int show = historyCount < 20 ? historyCount : 20;
    for (int i = 0; i < show; i++) {
      int idx = (historyHead - 1 - i + MAX_SAMPLES) % MAX_SAMPLES;
      uint16_t v = co2History[idx];
      uint32_t minsAgo = (uint32_t)i * 5;
      char tbuf[24];
      if (minsAgo < 60) snprintf(tbuf, sizeof(tbuf), "%lu min", (unsigned long)minsAgo);
      else snprintf(tbuf, sizeof(tbuf), "%lu h %lu min",
                    (unsigned long)(minsAgo / 60), (unsigned long)(minsAgo % 60));
      html += F("<tr><td>"); html += tbuf;
      html += F("</td><td class='"); html += co2StatusKey(v); html += "'>";
      html += String(v); html += F("</td></tr>");
    }
    html += F("</table>");
  }

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
  j += "\"total_uptime_s\":"; j += String(getTotalUptimeSec()); j += ",";
  j += "\"days_since_cal\":";
  uint32_t d = daysSinceLastFrc();
  j += (d == UINT32_MAX ? "null" : String(d));
  j += ",";
  j += "\"sample_count\":"; j += String(historyCount);
  j += "}";
  server.send(200, "application/json", j);
}

void handleCsv() {
  String csv;
  csv.reserve(2048);

  if (ntpSynced) {
    csv += "# CO2 monitor history. Timestamps are UTC ISO8601. "
           "Samples taken every 5 minutes.\r\n";
    csv += "timestamp_utc,co2_ppm\r\n";
  } else {
    csv += "# CO2 monitor history. Times are seconds before now "
           "(NTP not synced). Samples every 5 minutes.\r\n";
    csv += "seconds_before_now,co2_ppm\r\n";
  }

  time_t now = time(nullptr);
  for (int i = 0; i < historyCount; i++) {
    int idx = (historyHead - 1 - i + MAX_SAMPLES) % MAX_SAMPLES;
    uint16_t v = co2History[idx];
    uint32_t secsAgo = (uint32_t)i * 5UL * 60UL;

    if (ntpSynced) {
      time_t t = now - (time_t)secsAgo;
      struct tm tmv;
      gmtime_r(&t, &tmv);
      char tbuf[24];
      strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", &tmv);
      csv += tbuf;
    } else {
      csv += String((unsigned long)secsAgo);
    }
    csv += ",";
    csv += String(v);
    csv += "\r\n";
  }

  server.sendHeader("Content-Disposition", "attachment; filename=co2_history.csv");
  server.send(200, "text/csv", csv);
}

// ----------------- Serial commands -----------------
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
  stopWebServer();
  Serial.println("WiFi: credentials cleared. Reboot or use 'wifi-setup'.");
}

void wifiStatusSerial() {
  Serial.printf("WiFi SSID: %s\n", WiFi.SSID().c_str());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected, IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("NTP synced: %s\n", ntpSynced ? "yes" : "no");
  } else {
    Serial.println("Not connected");
  }
}

void printInfo() {
  Serial.printf("ASC:        %s\n", ascEnabled ? "ON" : "OFF");
  Serial.printf("Altitude:   %u m\n", sensorAltitudeM);
  Serial.printf("Temp off:   %.1f C  (%.1f F)\n",
                tempOffsetC, tempOffsetC * 9.0f / 5.0f);
  Serial.printf("Dark mode:  %s\n", darkMode ? "ON" : "OFF");
  Serial.printf("Temp unit:  %s\n", tempInFahrenheit ? "F" : "C");
  Serial.printf("Uptime:     %lu s total\n", (unsigned long)getTotalUptimeSec());
  uint32_t d = daysSinceLastFrc();
  if (d == UINT32_MAX) Serial.println("Last cal:   never");
  else                 Serial.printf("Last cal:   %lu days ago\n", (unsigned long)d);
  Serial.printf("History:    %d samples\n", historyCount);
  wifiStatusSerial();
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  String lower = cmd; lower.toLowerCase();

  if      (lower.startsWith("frc "))      doFRCSerial((uint16_t) lower.substring(4).toInt());
  else if (lower == "asc on")             setAscFromSerial(true);
  else if (lower == "asc off")            setAscFromSerial(false);
  else if (lower == "wifi-setup")         { if (!wifiPortalActive) startWifiPortal(); }
  else if (lower == "wifi-reset")         wifiResetSerial();
  else if (lower == "wifi-status")        wifiStatusSerial();
  else if (lower == "info")               printInfo();
  else if (lower == "reset")              resetPrefs();
  else Serial.println("Commands: 'frc <ppm>', 'asc on/off', "
                      "'wifi-setup', 'wifi-reset', 'wifi-status', "
                      "'info', 'reset'");
}

// ----------------- Setup / Loop -----------------
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n--- CO2 Monitor booting ---");

  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);

  // Turn off the on-board RGB LED (it floats on red at boot)
  disableOnboardLeds();

  loadPrefs();

  tft.init();
  tft.setRotation(0);
  tft.invertDisplay(INVERT_DISPLAY_COLORS);

  touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchSPI);
  ts.setRotation(0);   // portrait; matches tft.setRotation(0) above

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("CO2 Monitor", SCREEN_W / 2, SCREEN_H / 2 - 20, 4);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Initializing...", SCREEN_W / 2, SCREEN_H / 2 + 20, 2);

  initSensor();
  delay(5500);

  // Configure WiFiManager defaults (used when portal is launched later)
  wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);
  wm.setConfigPortalBlocking(false);
  wm.setBreakAfterConfig(true);
  wm.setShowInfoErase(false);
  // Don't auto-launch the portal - we only do it on user request

  tryWifiConnect();   // attempt connection with stored creds, then carry on

  drawMainScreen();

  Serial.println("Ready. Type 'info' for status or 'wifi-setup' to configure WiFi.");
}

void loop() {
  unsigned long now = millis();

  handleSerialCommands();
  handleTouch();

  if (wifiPortalActive)         serviceWifiPortal();
  else if (wifiConnected && webServerRunning) server.handleClient();

  saveUptimePeriodic();

  bool isMainScreen = (currentScreen == SCREEN_MAIN);

  if (now - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = now;
    if (readSensor() && isMainScreen) drawReadings();
  }

  if (dataValid &&
      (lastSampleTime == 0 || now - lastSampleTime >= SAMPLE_INTERVAL_MS)) {
    lastSampleTime = now;
    addGraphSample(currentCO2);
    if (isMainScreen) drawGraph();
  }
}
