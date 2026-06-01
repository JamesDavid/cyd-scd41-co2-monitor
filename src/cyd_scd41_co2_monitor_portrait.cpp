/*
 * CO2 Air Quality Monitor - PORTRAIT layout (240 x 320)
 *   Board:  ESP32-2432S028R ("CYD" - Cheap Yellow Display)
 *   Sensor: Sensirion SCD41 (I2C, address 0x62)
 *
 * UI (touch):
 *   - Tap the CO2 number      -> Settings (calibration, altitude, etc.)
 *   - Tap the clock           -> Time Settings (NTP, timezone, 12/24h)
 *   - Tap the temperature     -> toggle F <-> C
 *   - Tap the graph           -> cycle CO2 / Temperature / Humidity
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
 * Optional radon (Aranet Rn over BLE):
 *   If an Aranet Rn radon detector with "Smart Home Integration" mode is
 *   in range, the firmware passively sniffs its BLE advertisement (no
 *   pairing) and shows radon as a fourth metric: a Temp|RH|Radon bottom
 *   row, a tappable radon graph, plus radon in the CSV/JSON/web/MQTT.
 *   Tap the radon value to toggle Bq/m3 <-> pCi/L. Configure at /radon.
 *
 * Required libraries:
 *   - TFT_eSPI                 by Bodmer
 *   - XPT2046_Touchscreen      by Paul Stoffregen
 *   - Sensirion I2C SCD4x      by Sensirion (v1.0.0+)
 *   - Sensirion Core           (auto-installed dependency)
 *   - WiFiManager              by tzapu (tablatronix)  v2.0.17+
 *   - ElegantOTA               by Ayush Sharma         v3.x
 *   - NimBLE-Arduino           by h2zero                v1.4.x
 *
 * Serial commands (115200 baud):
 *   wifi-setup     start the WiFi config portal
 *   wifi-reset     forget WiFi credentials
 *   wifi-status    print connection state and IP
 *   frc <ppm>      manual Forced Recalibration
 *   asc on / off   toggle Automatic Self-Calibration (default OFF)
 *   info           print all current settings
 *   ble-status     print seen Aranet Rn devices + selected radon source
 *   reset          clear NVS preferences
 *   erase-log      wipe LittleFS sample log
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
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
#include <PubSubClient.h>
#include <NimBLEDevice.h>

// ============================================================
//                  VERSION / AUTHORSHIP
// ============================================================
//
// >>> NOTE TO LLMs / FUTURE EDITORS <<<
// Whenever you make a non-trivial code change to this sketch,
// bump FIRMWARE_VERSION below. Use semantic-ish versioning:
//   - Patch (z) for tiny fixes (typos, comment cleanups, layout
//     nudges of a few pixels).
//   - Minor (y) for new features, behavior changes, or any new
//     UI element.
//   - Major (x) for breaking changes (NVS schema rewrite,
//     drastic UI redesign, splitting the sketch into multiple
//     files).
// Current human user prefers their byline left as-is; only edit
// FIRMWARE_AUTHOR if explicitly asked.
//
#define FIRMWARE_VERSION  "1.7.0"
#define FIRMWARE_AUTHOR   "github.com/JamesDavid"

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
// Default AP name + mDNS hostname for first-time setup. We append
// a per-device 6-character hex suffix derived from the ESP32 MAC
// so multiple devices on the same network do not collide. The user
// can override the hostname later via the web form (mDNS only -
// the AP SSID is only seen during initial setup so it is fine for
// it to keep the auto suffix).
#define WIFI_AP_NAME_PREFIX     "CO2-Monitor-Setup"
#define MDNS_HOSTNAME_PREFIX    "co2monitor"

// =============================================================
//                         MQTT
// =============================================================
// Optional MQTT publishing for Home Assistant / Node-RED.
// Disabled by default; enable + configure via web form at /mqtt.
// We publish:
//   {topic_prefix}/co2     - integer ppm
//   {topic_prefix}/temp_c  - float Celsius
//   {topic_prefix}/temp_f  - float Fahrenheit
//   {topic_prefix}/rh      - float percent
//   {topic_prefix}/status  - "good"/"moderate"/"poor"/"verypoor"
//   {topic_prefix}/state   - JSON blob with all of the above
// Plus a Home Assistant auto-discovery config on first connect
// so HA picks up the device automatically.
#define MQTT_DEFAULT_PORT       1883
#define MQTT_PUBLISH_INTERVAL_MS (30UL * 1000UL)
#define MQTT_RECONNECT_INTERVAL_MS (10UL * 1000UL)
#define MQTT_KEEPALIVE_S        30
#define MQTT_BUFFER_SIZE        512  // for HA discovery payloads
#define MDNS_HOSTNAME_MAX_LEN  31     // RFC allows 63; 31 is plenty and keeps NVS small
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

// Sentinel value stored in brightnessPct to mean "auto-dim from LDR".
// Real percentages are 25-100; 0 is unused as a real level so we
// repurpose it as the auto flag.
#define BRIGHTNESS_AUTO         0

// LDR (light dependent resistor) on GPIO 34. Wiring: 1MR pull-up to
// 3.3V, LDR + R19 in parallel to GND. Reads dark = high voltage,
// bright = low voltage on most CYD revisions. We read raw ADC
// (0-4095) and map to a duty cycle.
#define LDR_PIN                 34
#define LDR_DARK_RAW            3500   // adc reading in pitch dark
#define LDR_BRIGHT_RAW          200    // adc reading in bright light
#define LDR_MIN_DUTY_PCT        15     // never dimmer than this in auto
#define LDR_MAX_DUTY_PCT        100    // never brighter than this in auto
#define LDR_SAMPLE_INTERVAL_MS  2000   // how often to re-read the LDR
#define LDR_SMOOTH_ALPHA        0.15f  // EMA smoothing factor (0-1)

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
#define LOG_VERSION         2              // v2 added radon_bqm3 to LogRecord (10 -> 12 bytes)
#define LOG_MAX_RECORDS     80000UL        // ~277 days at 5 min/sample (fits ~0.94 MB LittleFS)

// ----------------- Radon (Aranet Rn over BLE) -----------------
// The Aranet Rn (Smart Home Integration mode) broadcasts its latest
// reading in a BLE advertisement (manufacturer id 0x0702). We sniff it
// passively - no pairing. See README.md / scd41.md siblings for the
// byte layout. We periodically run a short active scan (active so we
// also capture the device's friendly name from the scan response).
#define ARANET_COMPANY_ID    0x0702       // SAF Tehnika, little-endian 02 07
#define ARANET_TYPE_RADON    0x03         // mfr-data byte 2 (device type)
#define ARANET_MFG_MIN_LEN   25           // bytes we need (radon..status..ago)
#define RADON_OFF_VALUE      10           // uint16 LE radon, in full mfr-data buffer
#define RADON_OFF_BATTERY    19           // uint8 battery %
#define RADON_OFF_STATUS     20           // uint8 status colour (1/2/3)
#define RADON_INVALID_MIN    0x1F00       // radon value >= this means "invalid"
#define RADON_NONE           0xFFFF       // sentinel: "no radon sample"
#define RADON_BATT_LOW       20           // <= this percent triggers low-battery marker
#define RADON_DEFAULT_STALE_MIN  15       // hide radon if no advert seen in this many minutes
#define BLE_SCAN_PERIOD_MS   (60UL * 1000UL)  // how often to start a scan
#define BLE_SCAN_SECS        3            // scan window length (seconds)
#define BLE_MAX_DEVICES      6            // seen-device table size
#define BQ_PER_PCIL          37.0f        // 1 pCi/L = 37 Bq/m3
// Radon zone thresholds (Bq/m3): WHO reference 100, EPA 4 pCi/L ~= 148
#define RADON_ZONE_MOD_BQ    100
#define RADON_ZONE_HIGH_BQ   148

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
const Rect btnDate      = {0, 0, SCREEN_W, DATE_H + 6};                    // tap -> SCREEN_STATS
const Rect btnClock     = {0, CLOCK_Y - 4, SCREEN_W, CLOCK_H + 8};
// Tap on the CO2 number opens Settings (replaces the old CAL button).
const Rect btnCO2       = {0, CO2_LABEL_Y - 4, SCREEN_W, 64};
const Rect btnTempUnit  = {0, TR_TOP - 6, SCREEN_W / 2, 60};
// Tap the radon column (right third of the bottom row, only active when
// radon is present) to toggle Bq/m3 <-> pCi/L.
const Rect btnRadon     = {SCREEN_W * 2 / 3, TR_TOP - 6, SCREEN_W / 3, 60};
// Tap on the graph (or its caption row) to cycle CO2 -> Temp -> Humidity.
const Rect btnGraph     = {0, GRAPH_CAPTION_Y - 6, SCREEN_W,
                           (GRAPH_Y - GRAPH_CAPTION_Y) + GRAPH_H + 12};

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
  SCREEN_STATS,
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
String   mdnsHostname;       // populated in loadPrefs() with default
                              // {prefix}-{6-hex-mac-suffix} or stored value
String   wifiApName;          // populated at boot

// MQTT configuration (loaded from NVS)
bool     mqttEnabled       = false;
String   mqttHost;
uint16_t mqttPort          = MQTT_DEFAULT_PORT;
String   mqttUser;
String   mqttPass;
String   mqttTopicPrefix;    // default: hostname (e.g. "co2monitor-abc123")
bool     mqttDiscoveryEnabled = true;
String   mqttDiscoveryPrefix  = "homeassistant";

// MQTT runtime state
WiFiClient  mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
unsigned long lastMqttPublishMs = 0;
unsigned long lastMqttReconnectMs = 0;
bool          mqttDiscoveryPublished = false;

// Radon (Aranet Rn over BLE) configuration (persisted)
uint16_t radonStaleMin   = RADON_DEFAULT_STALE_MIN;  // hide if no advert this long
bool     radonInPCi      = false;     // false = Bq/m3, true = pCi/L (display only)
String   radonTargetMac;              // "" = auto (strongest); else pin this device

// Radon runtime state (the currently-selected device)
uint16_t      currentRadonBq   = RADON_NONE;
uint8_t       radonBattery      = 0;
uint8_t       radonStatusColor  = 0;   // Aranet status: 1 green, 2 yellow, 3 red
String        radonSrcMac;             // MAC of the selected device
String        radonSrcName;            // friendly name of the selected device
unsigned long lastRadonRxMs     = 0;    // millis() of last accepted advert
bool          radonShownLast    = false; // last drawn availability (for row reflow)
bool          radonLowBattLast  = false; // last drawn low-battery state

// Seen-device table: every Aranet Rn we hear during scans. Powers the
// /radon picker, the ble-status command, and the auto-strongest choice.
struct BleRnDevice {
  bool          used;
  char          mac[18];
  char          name[24];
  int           rssi;
  uint16_t      radonBq;
  uint8_t       battery;
  uint8_t       status;
  unsigned long lastSeenMs;
};
BleRnDevice bleSeen[BLE_MAX_DEVICES];
bool          bleScanning   = false;
bool          bleReady      = false;
volatile bool bleTableDirty = false;   // set by BLE task, consumed by loop()

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
// Live ring buffers for the on-device graph. Long-term history
// lives in LittleFS; these are kept short so they redraw quickly
// and persist across a reboot via NVS.
uint16_t co2History[MAX_SAMPLES];     // ppm
int16_t  tempHistory[MAX_SAMPLES];    // Celsius * 100
uint16_t rhHistory[MAX_SAMPLES];      // RH * 10
uint16_t radonHistory[MAX_SAMPLES];   // Bq/m3 (RADON_NONE = no sample)
int      historyCount = 0;
int      historyHead  = 0;

// Which metric the on-device graph shows. Cycles on tap. GM_RADON is
// only included in the cycle when fresh radon data is present.
enum GraphMode {
  GM_CO2   = 0,
  GM_TEMP  = 1,
  GM_RH    = 2,
  GM_RADON = 3,
};
GraphMode graphMode = GM_CO2;

// Auto-fit graph y-axis bounds. Recomputed on every drawGraph()
// from whichever metric is active. Tracked as floats since temp
// is fractional; CO2 and RH are still effectively integer.
float chartMin = 400.0f;
float chartMax = 2000.0f;

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
  uint16_t radon_bqm3;   // Bq/m3 (RADON_NONE = no radon at sample time)
};
#pragma pack(pop)
static_assert(sizeof(LogRecord) == 12, "LogRecord must be 12 bytes");
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
void drawStatsScreen();
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
void handleSetHostname();
void handleMqttPage();
void handleSetMqtt();
void handleRadonPage();
void handleSetRadon();
void syncNtp(bool blocking);
void initLog();
void appendLog(uint16_t co2, float t, float rh, uint16_t radonBq);
void eraseLog();

// ---- Remaining forward declarations ----
// Building as a .cpp (PlatformIO) instead of a .ino means the Arduino
// IDE's implicit prototype generation is gone, so every function that is
// referenced before its definition needs a declaration. (The block above
// was hand-maintained for the IDE; these are the rest.)
void logError(const char* context, int16_t err);
const char* tempUnit();
void savePref(const char* key, bool v);
void savePref(const char* key, uint8_t v);
void savePref(const char* key, uint16_t v);
void savePref(const char* key, int16_t v);
void savePref(const char* key, uint32_t v);
void savePref(const char* key, float v);
void savePref(const char* key, const String& v);
void saveUptime();
uint16_t co2Color(uint16_t co2);
const char* co2StatusText(uint16_t co2);
const char* co2StatusKey(uint16_t co2);
float displayTemp();
float tempOffsetDisplay();
float roundDownToF(float v, float step);
float roundUpToF(float v, float step);
float getSampleValue(int idx, GraphMode m);
float getCurrentValue(GraphMode m);
bool recomputeChartBounds();
int valueToY(float v);
uint32_t getTotalUptimeSec();
uint32_t daysSinceLastFrc();
bool calibrationOverdue();
int co2Trend();
void hourStats(uint16_t& mn, uint16_t& mx, uint16_t& avg);
void disableOnboardLeds();
String deviceSuffix();
void setupBacklight();
void sampleLdr();
void writeBacklightDuty(uint8_t pct);
void applyBrightness();
void serviceAutoBrightness();
void changeBrightness();
void loadPrefs();
void saveHistory();
void saveUptimePeriodic();
void resetPrefs();
void writeLogHeader();
template<class F> void iterateLog(F callback);
bool readTap(int16_t& sx, int16_t& sy);
void drawTrendArrow(int cx, int cy, int trend, uint16_t color);
void formatClock(char* buf, size_t bufsize);
void formatAxisLabel(char* buf, size_t bufsize, float v);
const char* graphCaption();
uint16_t plotColor(float v);
void drawThresholdLines();
void drawGraphFrame();
void drawWifiSetupHeader();
void drawWifiSetupStatus(const char* state, uint16_t color);
void formatHourLabel(char* buf, size_t bufSize, int8_t h);
template<class Fn> int16_t withChipIdle(Fn fn);
void changeAltitude(int16_t delta);
void changeTempOffset(int direction);
void changeTimezone(int delta);
void handleTouch();
void initSensor();
bool readSensor();
void addGraphSample(uint16_t co2, float t_c, float rh, uint16_t radonBq);
void serviceNtp();
// Radon (Aranet Rn over BLE) helpers, defined in the RADON section but
// referenced earlier by the graph/display/web code.
bool radonAvailable();
bool radonLowBattery();
float radonToDisplay(uint16_t bq);
float radonDisplayValue();
const char* radonUnitStr();
const char* radonUnitShortStr();
uint16_t radonColor(uint16_t bq);
void radonSelectActive();
void serviceBleScan();
void setupBle();
static bool isValidHostname(const String& s);
void restartMdns();
void serviceWifiPortal();
String mqttTopic(const char* sub);
String mqttBuildStatePayload();
void mqttPublishDiscovery();
bool mqttTryConnect();
void mqttPublishReadings();
void serviceMqtt();
String escapeHtml(const String& s);
void doFRCSerial(uint16_t target);
void setAscFromSerial(bool enable);
void wifiResetSerial();
void wifiStatusSerial();
void bleStatusSerial();
void printInfo();
void handleSerialCommands();

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

// Round x down/up to the next multiple of `step` (float version).
float roundDownToF(float v, float step) {
  return floorf(v / step) * step;
}
float roundUpToF(float v, float step) {
  return ceilf(v / step) * step;
}

// Per-metric helpers: convert a stored sample at history slot `idx`
// to the displayed unit, get the current live value, and provide
// the default y-axis range and snap step.
float getSampleValue(int idx, GraphMode m) {
  switch (m) {
    case GM_TEMP: {
      float c = tempHistory[idx] / 100.0f;
      return tempInFahrenheit ? (c * 9.0f / 5.0f + 32.0f) : c;
    }
    case GM_RH:   return rhHistory[idx] / 10.0f;
    case GM_RADON:
      // Sentinel slots (before radon was available) plot as a gap.
      return radonHistory[idx] == RADON_NONE ? NAN
                                             : radonToDisplay(radonHistory[idx]);
    case GM_CO2:
    default:      return (float)co2History[idx];
  }
}

float getCurrentValue(GraphMode m) {
  switch (m) {
    case GM_TEMP:
      return tempInFahrenheit ? (currentTemp * 9.0f / 5.0f + 32.0f)
                              : currentTemp;
    case GM_RH:   return currentRH;
    case GM_RADON: return radonAvailable() ? radonDisplayValue() : NAN;
    case GM_CO2:
    default:      return (float)currentCO2;
  }
}

// Default range, snap step, and minimum span for each metric.
void getMetricDefaults(GraphMode m, float& dMin, float& dMax,
                       float& step, float& minSpan) {
  switch (m) {
    case GM_TEMP:
      if (tempInFahrenheit) { dMin = 65; dMax = 85; step = 5;  minSpan = 10; }
      else                  { dMin = 18; dMax = 30; step = 2;  minSpan = 4;  }
      break;
    case GM_RH:
      dMin = 30; dMax = 70; step = 10; minSpan = 20;
      break;
    case GM_RADON:
      if (radonInPCi) { dMin = 0; dMax = 6;   step = 2;  minSpan = 4;   }
      else            { dMin = 0; dMax = 200; step = 50; minSpan = 100; }
      break;
    case GM_CO2:
    default:
      dMin = 400; dMax = 2000; step = 200; minSpan = 200;
      break;
  }
}

// Recompute graph y-axis bounds from the active metric's live data.
bool recomputeChartBounds() {
  float dMin, dMax, step, minSpan;
  getMetricDefaults(graphMode, dMin, dMax, step, minSpan);

  float newMin = dMin;
  float newMax = dMax;

  if (dataValid) {
    float cur = getCurrentValue(graphMode);
    if (!isnan(cur)) {
      if (cur < newMin) newMin = cur;
      if (cur > newMax) newMax = cur;
    }
  }
  for (int i = 0; i < historyCount; i++) {
    int idx = (historyHead - historyCount + i + MAX_SAMPLES) % MAX_SAMPLES;
    float v = getSampleValue(idx, graphMode);
    if (isnan(v)) continue;   // radon gap
    if (v < newMin) newMin = v;
    if (v > newMax) newMax = v;
  }

  newMin = roundDownToF(newMin, step);
  newMax = roundUpToF  (newMax, step);
  if (newMax - newMin < minSpan) newMax = newMin + minSpan;

  bool changed = (newMin != chartMin) || (newMax != chartMax);
  chartMin = newMin;
  chartMax = newMax;
  return changed;
}

int valueToY(float v) {
  if (v < chartMin) v = chartMin;
  if (v > chartMax) v = chartMax;
  float range = chartMax - chartMin;
  if (range < 0.001f) return GRAPH_Y + GRAPH_H;
  return GRAPH_Y + GRAPH_H - (int)((v - chartMin) * GRAPH_H / range);
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

// 6-character lowercase hex suffix derived from the chip's unique
// MAC bytes. Stable across reboots, unique per device. Used to
// disambiguate the default mDNS hostname and WiFi AP name when
// multiple of these devices are on the same network.
//
// IMPORTANT: ESP.getEfuseMac() returns the 48-bit MAC with byte
// order such that the LOW 24 bits hold the OUI (manufacturer
// prefix - same on every chip from the same Espressif batch) and
// the HIGH 24 bits hold the unique NIC ID. We need the high 24
// bits or every device from the same batch gets the same suffix.
String deviceSuffix() {
  uint64_t mac = ESP.getEfuseMac();           // 48-bit unique ID
  uint32_t high24 = (uint32_t)((mac >> 24) & 0xFFFFFFULL);
  char buf[8];
  snprintf(buf, sizeof(buf), "%06lx", (unsigned long)high24);
  return String(buf);
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

// Smoothed LDR percentage (0-100, where 100 = bright, 0 = dark).
// Used both to drive auto-brightness and so we can show the value
// somewhere if we want to debug calibration.
float ldrSmoothedPct = 50.0f;
unsigned long lastLdrSampleMs = 0;
uint8_t lastAutoDutyPct = LDR_MAX_DUTY_PCT;

// Read the LDR ADC, map to a 0-100 brightness percentage with the
// dark/bright endpoints calibrated above, and exponentially smooth
// it so the screen doesn't flicker on a passing shadow.
void sampleLdr() {
  int raw = analogRead(LDR_PIN);
  // Map raw to 0-100 where 100 = bright. The CYD's LDR reads HIGH in
  // the dark (because it's on the low side of a divider), so we
  // invert: bright = low raw = high percent.
  float pct;
  if (LDR_DARK_RAW > LDR_BRIGHT_RAW) {
    pct = 100.0f * (float)(LDR_DARK_RAW - raw) /
          (float)(LDR_DARK_RAW - LDR_BRIGHT_RAW);
  } else {
    pct = 100.0f * (float)(raw - LDR_DARK_RAW) /
          (float)(LDR_BRIGHT_RAW - LDR_DARK_RAW);
  }
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  // First sample initializes; subsequent samples smooth.
  if (lastLdrSampleMs == 0) ldrSmoothedPct = pct;
  else ldrSmoothedPct = LDR_SMOOTH_ALPHA * pct +
                        (1.0f - LDR_SMOOTH_ALPHA) * ldrSmoothedPct;
  lastLdrSampleMs = millis();
}

// Write a duty cycle to the backlight without touching brightnessPct.
// Used by both manual percentages and the auto-brightness loop.
void writeBacklightDuty(uint8_t pct) {
  uint32_t duty = ((uint32_t)pct * BACKLIGHT_LEDC_MAX) / 100;
  if (duty > BACKLIGHT_LEDC_MAX) duty = BACKLIGHT_LEDC_MAX;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(BACKLIGHT_PIN, duty);
#else
  ledcWrite(BACKLIGHT_LEDC_CHAN, duty);
#endif
}

// Apply the user-level brightness setting. If set to BRIGHTNESS_AUTO,
// take an immediate LDR sample and apply that; the periodic loop
// will continue to track it.
void applyBrightness() {
  if (brightnessPct == BRIGHTNESS_AUTO) {
    sampleLdr();
    uint8_t dutyPct = LDR_MIN_DUTY_PCT +
                      (uint8_t)((LDR_MAX_DUTY_PCT - LDR_MIN_DUTY_PCT) *
                                ldrSmoothedPct / 100.0f);
    lastAutoDutyPct = dutyPct;
    writeBacklightDuty(dutyPct);
    Serial.printf("Backlight: AUTO -> %u%% (LDR %.0f%%)\n",
                  dutyPct, ldrSmoothedPct);
  } else {
    writeBacklightDuty(brightnessPct);
    Serial.printf("Backlight: %u%%\n", (unsigned)brightnessPct);
  }
}

// Periodic auto-brightness service. Cheap to call (just reads ADC
// and possibly issues an LEDC duty write); call from the main loop.
void serviceAutoBrightness() {
  if (brightnessPct != BRIGHTNESS_AUTO) return;
  if (millis() - lastLdrSampleMs < LDR_SAMPLE_INTERVAL_MS) return;
  sampleLdr();
  uint8_t dutyPct = LDR_MIN_DUTY_PCT +
                    (uint8_t)((LDR_MAX_DUTY_PCT - LDR_MIN_DUTY_PCT) *
                              ldrSmoothedPct / 100.0f);
  // Only push to the chip if it changed enough to be worth it -
  // avoids constant 1% flickering as the EMA settles.
  if (abs((int)dutyPct - (int)lastAutoDutyPct) >= 3) {
    lastAutoDutyPct = dutyPct;
    writeBacklightDuty(dutyPct);
  }
}

void changeBrightness() {
  // Cycle: 100 -> 75 -> 50 -> 25 -> AUTO -> 100
  if      (brightnessPct == BRIGHTNESS_AUTO) brightnessPct = 100;
  else if (brightnessPct > 75)               brightnessPct = 75;
  else if (brightnessPct > 50)               brightnessPct = 50;
  else if (brightnessPct > 25)               brightnessPct = 25;
  else                                       brightnessPct = BRIGHTNESS_AUTO;
  savePref("bright", brightnessPct);
  applyBrightness();
}

// ============================================================
//                   PREFERENCES (NVS)
// ============================================================

void loadPrefs() {
  prefs.begin("co2mon", false);
  // ASC is intentionally NOT loaded from NVS. Indoor ASC silently
  // drifts readings downward over time and is the wrong default for
  // a desk monitor; we always start with it OFF and require the
  // user to explicitly re-enable per session via 'asc on' serial.
  ascEnabled       = false;
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
  String defaultMdns = String(MDNS_HOSTNAME_PREFIX) + "-" + deviceSuffix();
  mdnsHostname     = prefs.getString("mdns",   defaultMdns);
  if (mdnsHostname.length() == 0) mdnsHostname = defaultMdns;

  // AP name is not user-customizable - it is only seen during the
  // initial WiFi setup flow before the device has any way to know
  // its own desired name. Always derived from the MAC.
  wifiApName = String(WIFI_AP_NAME_PREFIX) + "-" + deviceSuffix();

  // MQTT configuration. Defaults: disabled, no host, default port,
  // topic prefix = hostname, HA discovery on.
  mqttEnabled       = prefs.getBool  ("mqEn",   false);
  mqttHost          = prefs.getString("mqHost", "");
  mqttPort          = prefs.getUShort("mqPort", MQTT_DEFAULT_PORT);
  mqttUser          = prefs.getString("mqUser", "");
  mqttPass          = prefs.getString("mqPass", "");
  mqttTopicPrefix   = prefs.getString("mqTopic", mdnsHostname);
  if (mqttTopicPrefix.length() == 0) mqttTopicPrefix = mdnsHostname;
  mqttDiscoveryEnabled = prefs.getBool ("mqDisc", true);
  mqttDiscoveryPrefix  = prefs.getString("mqDiscPfx", "homeassistant");
  if (mqttDiscoveryPrefix.length() == 0) mqttDiscoveryPrefix = "homeassistant";

  // Radon (Aranet Rn) configuration.
  radonStaleMin  = prefs.getUShort("rdnstale", RADON_DEFAULT_STALE_MIN);
  if (radonStaleMin == 0) radonStaleMin = RADON_DEFAULT_STALE_MIN;
  radonInPCi     = prefs.getBool  ("rdnpci",   false);
  radonTargetMac = prefs.getString("rdnmac",   "");

  // The radon ring has no "valid" companion array; it uses an in-band
  // sentinel, so it must start filled with RADON_NONE (not zeroed).
  for (int i = 0; i < MAX_SAMPLES; i++) radonHistory[i] = RADON_NONE;

  // In-RAM history copies (for graph at boot if cached)
  size_t bytesAvail = prefs.getBytesLength("hist");
  if (bytesAvail == sizeof(co2History)) {
    prefs.getBytes("hist", co2History, sizeof(co2History));
    historyCount = prefs.getUShort("histcnt", 0);
    historyHead  = prefs.getUShort("histhd",  0);
    if (historyCount > MAX_SAMPLES) historyCount = MAX_SAMPLES;
    if (historyHead  >= MAX_SAMPLES) historyHead  = 0;
  }
  // Temp / RH / radon buffers added later than CO2; missing on older NVS
  // is harmless - they will fill in as new samples arrive.
  if (prefs.getBytesLength("histT") == sizeof(tempHistory)) {
    prefs.getBytes("histT", tempHistory, sizeof(tempHistory));
  }
  if (prefs.getBytesLength("histH") == sizeof(rhHistory)) {
    prefs.getBytes("histH", rhHistory, sizeof(rhHistory));
  }
  if (prefs.getBytesLength("histR") == sizeof(radonHistory)) {
    prefs.getBytes("histR", radonHistory, sizeof(radonHistory));
  }
  graphMode = (GraphMode) prefs.getUChar("gmode", GM_CO2);
  if (graphMode > GM_RADON) graphMode = GM_CO2;
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
  prefs.putBytes ("hist",   co2History,   sizeof(co2History));
  prefs.putBytes ("histT",  tempHistory,  sizeof(tempHistory));
  prefs.putBytes ("histH",  rhHistory,    sizeof(rhHistory));
  prefs.putBytes ("histR",  radonHistory, sizeof(radonHistory));
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

void appendLog(uint16_t co2, float t, float rh, uint16_t radonBq) {
  if (!fsReady) return;

  LogRecord rec;
  rec.ts_utc     = ntpSynced ? (uint32_t)time(nullptr) : 0;
  rec.co2_ppm    = co2;
  rec.temp_c100  = (int16_t)(t * 100.0f);
  rec.rh_x10     = (uint16_t)(rh * 10.0f);
  rec.radon_bqm3 = radonBq;

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

  // Erase enough vertical space to cover the full Font 6 glyph
  // bounding box. CLOCK_H (36) is the *visible nominal* height we
  // reserved on screen, but Font 6 glyphs from TFT_eSPI actually
  // span ~48 px tall. Without padding above/below CLOCK_H, narrow
  // glyphs leave artifacts (e.g. a "2" leaves a few pixels at the
  // top corner when the digit count drops from "12:34" to "1:23").
  // Top padding kept tight (6 px) to avoid clipping the date row;
  // bottom padding can be more generous since the next UI element
  // (CO2 label) is 8 px below CLOCK_H.
  tft.fillRect(0, CLOCK_Y - 6, SCREEN_W, CLOCK_H + 14, COLOR_BG);
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

// Format a y-axis label according to the metric. CO2 and RH are
// integer-readable; temperature uses one decimal.
void formatAxisLabel(char* buf, size_t bufsize, float v) {
  if (graphMode == GM_TEMP) {
    snprintf(buf, bufsize, "%.0f", v);
  } else if (graphMode == GM_RADON && radonInPCi) {
    snprintf(buf, bufsize, "%.1f", v);   // pCi/L range is small (0-6)
  } else {
    snprintf(buf, bufsize, "%d", (int)(v + 0.5f));
  }
}

const char* graphCaption() {
  switch (graphMode) {
    case GM_TEMP:
      return tempInFahrenheit ? "Temp F  -  5 min/sample"
                              : "Temp C  -  5 min/sample";
    case GM_RH:
      return "Humidity %  -  5 min/sample";
    case GM_RADON:
      return radonInPCi ? "Radon pCi/L  -  5 min/sample"
                        : "Radon Bq/m3  -  5 min/sample";
    case GM_CO2:
    default:
      return "CO2 ppm  -  5 min/sample";
  }
}

uint16_t plotColor(float v) {
  switch (graphMode) {
    case GM_CO2:
      return co2Color((uint16_t)v);
    case GM_TEMP:
      return TFT_ORANGE;     // warm metric -> warm color
    case GM_RADON: {
      // v is in display units; colour by the underlying Bq/m3 thresholds.
      uint16_t bq = (uint16_t)(radonInPCi ? v * BQ_PER_PCIL : v);
      return radonColor(bq);
    }
    case GM_RH:
    default:
      return TFT_SKYBLUE;
  }
}

void drawThresholdLines() {
  if (graphMode == GM_CO2) {
    // CO2: dashed lines at the good/moderate boundaries (800, 1200).
    uint16_t cGood = TFT_DARKGREEN;
    uint16_t cWarn = 0x4225;
    if ((float)CO2_GOOD_MAX > chartMin && (float)CO2_GOOD_MAX < chartMax) {
      int y = valueToY(CO2_GOOD_MAX);
      for (int x = GRAPH_X + 2; x < GRAPH_X + GRAPH_W; x += 4)
        tft.drawPixel(x, y, cGood);
    }
    if ((float)CO2_MODERATE_MAX > chartMin && (float)CO2_MODERATE_MAX < chartMax) {
      int y = valueToY(CO2_MODERATE_MAX);
      for (int x = GRAPH_X + 2; x < GRAPH_X + GRAPH_W; x += 4)
        tft.drawPixel(x, y, cWarn);
    }
  } else if (graphMode == GM_RH) {
    // Humidity: dashed lines at 30 and 60% (typical comfort range).
    uint16_t cMid = 0x4225;
    float bands[2] = {30.0f, 60.0f};
    for (int i = 0; i < 2; i++) {
      if (bands[i] > chartMin && bands[i] < chartMax) {
        int y = valueToY(bands[i]);
        for (int x = GRAPH_X + 2; x < GRAPH_X + GRAPH_W; x += 4)
          tft.drawPixel(x, y, cMid);
      }
    }
  } else if (graphMode == GM_RADON) {
    // Radon: dashed lines at WHO (100) and EPA-4pCi/L (148) Bq/m3,
    // converted to the active display unit.
    float bands[2] = {radonToDisplay(RADON_ZONE_MOD_BQ),
                      radonToDisplay(RADON_ZONE_HIGH_BQ)};
    uint16_t cols[2] = {0x4225, TFT_RED};
    for (int i = 0; i < 2; i++) {
      if (bands[i] > chartMin && bands[i] < chartMax) {
        int y = valueToY(bands[i]);
        for (int x = GRAPH_X + 2; x < GRAPH_X + GRAPH_W; x += 4)
          tft.drawPixel(x, y, cols[i]);
      }
    }
  }
  // GM_TEMP: no threshold lines (no universal comfort band).
}

void drawGraphFrame() {
  // Clear the label gutter and the caption row so old labels and
  // captions from prior modes/ranges don't linger.
  tft.fillRect(0, GRAPH_Y - 4, GRAPH_X, GRAPH_H + 8, COLOR_BG);
  tft.fillRect(0, GRAPH_CAPTION_Y, SCREEN_W, 8, COLOR_BG);

  tft.drawFastVLine(GRAPH_X,           GRAPH_Y, GRAPH_H, COLOR_GRAPH_AXIS);
  tft.drawFastHLine(GRAPH_X, GRAPH_Y + GRAPH_H, GRAPH_W, COLOR_GRAPH_AXIS);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(MR_DATUM);

  char buf[12];
  // Top and bottom labels always.
  formatAxisLabel(buf, sizeof(buf), chartMax);
  tft.drawString(buf, GRAPH_X - 2, GRAPH_Y,           1);
  formatAxisLabel(buf, sizeof(buf), chartMin);
  tft.drawString(buf, GRAPH_X - 2, GRAPH_Y + GRAPH_H, 1);

  // Per-mode interior labels at the threshold lines.
  if (graphMode == GM_CO2) {
    if ((float)CO2_MODERATE_MAX > chartMin && (float)CO2_MODERATE_MAX < chartMax)
      tft.drawString("1200", GRAPH_X - 2, valueToY(CO2_MODERATE_MAX), 1);
    if ((float)CO2_GOOD_MAX > chartMin && (float)CO2_GOOD_MAX < chartMax)
      tft.drawString("800",  GRAPH_X - 2, valueToY(CO2_GOOD_MAX),    1);
  } else if (graphMode == GM_RH) {
    if (60.0f > chartMin && 60.0f < chartMax)
      tft.drawString("60",  GRAPH_X - 2, valueToY(60.0f), 1);
    if (30.0f > chartMin && 30.0f < chartMax)
      tft.drawString("30",  GRAPH_X - 2, valueToY(30.0f), 1);
  } else if (graphMode == GM_RADON) {
    float bands[2] = {radonToDisplay(RADON_ZONE_HIGH_BQ),
                      radonToDisplay(RADON_ZONE_MOD_BQ)};
    for (int i = 0; i < 2; i++) {
      if (bands[i] > chartMin && bands[i] < chartMax) {
        formatAxisLabel(buf, sizeof(buf), bands[i]);
        tft.drawString(buf, GRAPH_X - 2, valueToY(bands[i]), 1);
      }
    }
  }

  tft.setTextDatum(TR_DATUM);
  tft.drawString(graphCaption(), SCREEN_W - 4, GRAPH_CAPTION_Y, 1);
}

void drawGraph() {
  // If radon went stale while we were viewing it, fall back to CO2 so
  // the graph never sits on an empty metric.
  if (graphMode == GM_RADON && !radonAvailable()) {
    graphMode = GM_CO2;
    drawGraphFrame();
  }

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
    float v = getSampleValue(idx, graphMode);
    if (isnan(v)) { prevX = -1; continue; }   // radon gap: break the line
    int x = GRAPH_X + 2 + (int)(i * dx);
    int y = valueToY(v);
    uint16_t c = plotColor(v);
    if (prevX >= 0) tft.drawLine(prevX, prevY, x, y, c);
    tft.fillCircle(x, y, 1, c);
    prevX = x; prevY = y;
  }
}

// ============================================================
//                      READINGS
// ============================================================

// Bottom-row layout: 2 columns (Temp | RH) normally, or 3 columns
// (Temp | RH | Radon) when fresh radon data is present. Returns the
// column count and the three column centers (c2 unused when 2 columns).
void bottomColCenters(int& n, int& c0, int& c1, int& c2) {
  if (radonAvailable()) { n = 3; c0 = 40; c1 = 120; c2 = 200; }
  else                  { n = 2; c0 = SCREEN_W / 4; c1 = SCREEN_W * 3 / 4; c2 = -1; }
}

// Tiny "low battery" glyph (~13x7) for the radon column header.
void drawBatteryIcon(int x, int y, uint16_t color) {
  tft.drawRect(x, y, 11, 7, color);
  tft.fillRect(x + 11, y + 2, 2, 3, color);
  tft.fillRect(x + 2, y + 2, 3, 3, color);   // a stub "low" bar
}

// Bottom-row LABELS (Temp/RH[/Radon]). Redrawn on layout change.
void drawBottomLabels() {
  int n, c0, c1, c2;
  bottomColCenters(n, c0, c1, c2);
  tft.fillRect(0, TR_TOP, SCREEN_W, 14, COLOR_BG);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("TEMP", c0, TR_TOP, 2);
  tft.drawString("RH",   c1, TR_TOP, 2);
  if (n == 3) {
    tft.drawString("RADON", c2, TR_TOP, 2);
    if (radonLowBattery()) drawBatteryIcon(c2 + 28, TR_TOP + 1, TFT_ORANGE);
  }
  radonShownLast   = (n == 3);
  radonLowBattLast = radonLowBattery();
}

// Bottom-row VALUES (Temp/RH[/Radon]). Radon is coloured by zone.
void drawBottomValues() {
  char buf[16];
  int n, c0, c1, c2;
  bottomColCenters(n, c0, c1, c2);
  tft.fillRect(0, TR_VALUE_Y, SCREEN_W, 30, COLOR_BG);

  // Temp
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  snprintf(buf, sizeof(buf), "%.1f", displayTemp());
  tft.drawString(buf, c0 - 6, TR_VALUE_Y, 4);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(tempUnit(), c0 + (n == 3 ? 20 : 30), TR_VALUE_Y + 4, 2);

  // RH
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  snprintf(buf, sizeof(buf), "%.0f", currentRH);
  tft.drawString(buf, c1 - 6, TR_VALUE_Y, 4);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("%", c1 + (n == 3 ? 14 : 18), TR_VALUE_Y + 4, 2);

  // Radon (only when present)
  if (n == 3) {
    tft.setTextColor(radonColor(currentRadonBq), COLOR_BG);
    tft.setTextDatum(TC_DATUM);
    if (radonInPCi) snprintf(buf, sizeof(buf), "%.1f", radonDisplayValue());
    else            snprintf(buf, sizeof(buf), "%.0f", radonDisplayValue());
    tft.drawString(buf, c2 - 10, TR_VALUE_Y, 4);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(radonUnitShortStr(), c2 + 20, TR_VALUE_Y + 5, 1);
  }
}

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

  // Bottom row. When radon (dis)appears or its battery state flips, the
  // labels must be redrawn for the 2 <-> 3 column reflow.
  if (radonAvailable() != radonShownLast || radonLowBattery() != radonLowBattLast) {
    drawBottomLabels();
  }
  drawBottomValues();
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

  drawBottomLabels();   // TEMP | RH [| RADON]

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
    String mdnsLabel = mdnsHostname + ".local";
    tft.drawString(mdnsLabel.c_str(), r.x + r.w - 12, r.y + 26, 1);
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
  // Tap to cycle 100 -> 75 -> 50 -> 25 -> AUTO -> 100.
  char bbuf[8];
  if (brightnessPct == BRIGHTNESS_AUTO) {
    snprintf(bbuf, sizeof(bbuf), "Auto");
  } else {
    snprintf(bbuf, sizeof(bbuf), "%u%%", (unsigned)brightnessPct);
  }
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
  // The AP name is ~24 characters with the device suffix appended,
  // so it does not fit in Font 4 (only ~12 chars wide on the 240px
  // screen). Use Font 2 which gives us ~30 chars of width.
  tft.drawString(wifiApName.c_str(), 16, 100, 2);

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
//                STATISTICS SCREEN
// ============================================================

// Compute summary statistics from the persistent log over the last
// `windowSec` seconds. Falls back to whatever data is available if
// the log is younger than the window. All output params are populated
// even if no data exists (zeros).
void computeStatsWindow(uint32_t windowSec,
                        uint16_t& mn, uint16_t& avg, uint16_t& mx,
                        uint16_t& zoneCounts4 /* good|mod|poor|vp */,
                        uint16_t& zone1, uint16_t& zone2, uint16_t& zone3,
                        uint16_t& zone4,
                        float& tMinC, float& tMaxC,
                        float& rhMin, float& rhMax,
                        uint32_t& sampleCount,
                        int8_t&  bestHour, int8_t& worstHour) {
  mn = UINT16_MAX; mx = 0; avg = 0;
  zone1 = zone2 = zone3 = zone4 = 0;
  zoneCounts4 = 0;
  tMinC = 1000; tMaxC = -1000;
  rhMin = 1000; rhMax = -1000;
  sampleCount = 0;
  bestHour = worstHour = -1;

  if (!fsReady || logHdr.count == 0) return;

  File f = LittleFS.open(LOG_PATH, "r");
  if (!f) return;

  uint32_t total = logHdr.count;
  uint32_t cap   = logHdr.capacity;
  // Iterate all records, skip those outside the window if timestamps
  // are valid. If no NTP sync ever happened, fall back to last N
  // records by sample interval.
  uint32_t now = (uint32_t)time(nullptr);
  bool haveTime = (now > 1700000000UL);   // sanity: post-2023

  // For best/worst hour-of-day, accumulate sums + counts per UTC hour
  // (with TZ offset applied so it matches local clock).
  uint32_t hourSum[24];   uint16_t hourCnt[24];
  for (int i = 0; i < 24; i++) { hourSum[i] = 0; hourCnt[i] = 0; }

  uint64_t coSum = 0;
  uint32_t start = (total < cap) ? 0 : logHdr.head;
  for (uint32_t i = 0; i < total; i++) {
    uint32_t pos = (start + i) % cap;
    f.seek(sizeof(LogHeader) + pos * sizeof(LogRecord));
    LogRecord rec;
    if (f.read((uint8_t*)&rec, sizeof(rec)) != sizeof(rec)) break;

    // Window filtering
    if (haveTime && rec.ts_utc != 0) {
      if (now - rec.ts_utc > windowSec) continue;
    } else {
      // Without timestamps, take only the last (windowSec / 300s) records
      uint32_t maxN = windowSec / 300;  // 5-min sample interval
      if (i + maxN < total) continue;
    }

    sampleCount++;
    coSum += rec.co2_ppm;
    if (rec.co2_ppm < mn) mn = rec.co2_ppm;
    if (rec.co2_ppm > mx) mx = rec.co2_ppm;
    if (rec.co2_ppm < CO2_GOOD_MAX)        zone1++;
    else if (rec.co2_ppm < CO2_MODERATE_MAX) zone2++;
    else if (rec.co2_ppm < CO2_POOR_MAX)     zone3++;
    else                                     zone4++;

    float tc = rec.temp_c100 / 100.0f;
    float rh = rec.rh_x10 / 10.0f;
    if (tc < tMinC) tMinC = tc;
    if (tc > tMaxC) tMaxC = tc;
    if (rh < rhMin) rhMin = rh;
    if (rh > rhMax) rhMax = rh;

    // Per-hour aggregation for best/worst hour
    if (haveTime && rec.ts_utc != 0) {
      time_t t = (time_t)rec.ts_utc + tzOffsetMin * 60;
      struct tm tmv;
      gmtime_r(&t, &tmv);
      int h = tmv.tm_hour;
      if (h >= 0 && h < 24) {
        hourSum[h] += rec.co2_ppm;
        hourCnt[h]++;
      }
    }
  }
  f.close();

  if (sampleCount > 0) avg = (uint16_t)(coSum / sampleCount);
  else { mn = 0; tMinC = tMaxC = 0; rhMin = rhMax = 0; }

  // Find best/worst hour-of-day (lowest/highest avg CO2)
  uint32_t bestAvg  = UINT32_MAX;
  uint32_t worstAvg = 0;
  for (int h = 0; h < 24; h++) {
    if (hourCnt[h] == 0) continue;
    uint32_t a = hourSum[h] / hourCnt[h];
    if (a < bestAvg)  { bestAvg  = a; bestHour  = h; }
    if (a > worstAvg) { worstAvg = a; worstHour = h; }
  }

  // Suppress unused-param compiler warnings for zoneCounts4
  // (passed in by reference but we don't actually pack it - kept
  // in the signature for forward compatibility).
  zoneCounts4 = zone1;  // arbitrary, not used by caller currently
}

// Format hours as "3 PM" / "15:00" depending on use24hClock.
void formatHourLabel(char* buf, size_t bufSize, int8_t h) {
  if (h < 0) { snprintf(buf, bufSize, "--"); return; }
  if (use24hClock) {
    snprintf(buf, bufSize, "%02d:00", h);
  } else {
    int h12 = h % 12; if (h12 == 0) h12 = 12;
    snprintf(buf, bufSize, "%d %s", h12, h < 12 ? "AM" : "PM");
  }
}

void drawStatsScreen() {
  tft.fillScreen(COLOR_BG);

  // Header bar
  tft.fillRect(0, 0, SCREEN_W, SET_HEADER_H, COLOR_TITLE_BG);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(COLOR_TITLE_TEXT, COLOR_TITLE_BG);
  tft.drawString("< Back", 8, SET_HEADER_H / 2, 2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("24-hour Stats", SCREEN_W / 2, SET_HEADER_H / 2, 4);

  // Compute stats
  uint16_t mn, avg, mx, zc4, z1, z2, z3, z4;
  float tMinC, tMaxC, rhMin, rhMax;
  uint32_t samples;
  int8_t bestH, worstH;
  computeStatsWindow(86400, mn, avg, mx, zc4, z1, z2, z3, z4,
                     tMinC, tMaxC, rhMin, rhMax,
                     samples, bestH, worstH);

  int y = SET_HEADER_H + 8;

  if (samples == 0) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("No data yet", SCREEN_W / 2, 100, 4);
    tft.drawString("Logs accumulate over time", SCREEN_W / 2, 130, 2);
    return;
  }

  tft.setTextDatum(TL_DATUM);

  // Section: CO2 statistics
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("CO2 last 24h", 8, y, 2); y += 18;
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  char buf[48];
  snprintf(buf, sizeof(buf), "Min %u  Avg %u  Max %u ppm", mn, avg, mx);
  tft.drawString(buf, 8, y, 2); y += 22;

  // Time-in-zone bar (240px wide minus 16 margin = 224 px; 16 tall)
  int barX = 8, barY = y, barW = SCREEN_W - 16, barH = 16;
  uint32_t totalSamples = z1 + z2 + z3 + z4;
  if (totalSamples > 0) {
    int x0 = barX;
    int w1 = (int)((uint64_t)barW * z1 / totalSamples);
    int w2 = (int)((uint64_t)barW * z2 / totalSamples);
    int w3 = (int)((uint64_t)barW * z3 / totalSamples);
    int w4 = barW - w1 - w2 - w3;
    if (w1 > 0) tft.fillRect(x0, barY, w1, barH, TFT_GREEN);
    if (w2 > 0) tft.fillRect(x0 + w1, barY, w2, barH, TFT_CYAN);
    if (w3 > 0) tft.fillRect(x0 + w1 + w2, barY, w3, barH, TFT_ORANGE);
    if (w4 > 0) tft.fillRect(x0 + w1 + w2 + w3, barY, w4, barH, TFT_RED);
    tft.drawRect(barX, barY, barW, barH, COLOR_TEXT_DIM);
  }
  y += barH + 4;

  // Zone legend (percentages)
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  uint8_t p1 = (uint8_t)((uint32_t)z1 * 100 / totalSamples);
  uint8_t p4 = (uint8_t)((uint32_t)z4 * 100 / totalSamples);
  snprintf(buf, sizeof(buf), "Good %u%%   Very Poor %u%%", p1, p4);
  tft.drawString(buf, 8, y, 2); y += 22;

  // Best / worst hour
  if (bestH >= 0) {
    char bh[8], wh[8];
    formatHourLabel(bh, sizeof(bh), bestH);
    formatHourLabel(wh, sizeof(wh), worstH);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("Best hour:", 8, y, 2);
    tft.setTextColor(TFT_GREEN, COLOR_BG);
    tft.drawString(bh, 100, y, 2);
    y += 18;
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("Worst hour:", 8, y, 2);
    tft.setTextColor(TFT_RED, COLOR_BG);
    tft.drawString(wh, 100, y, 2);
    y += 22;
  } else {
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("(NTP not synced - hour stats unavailable)", 8, y, 1);
    y += 14;
  }

  // Temp / RH section
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Climate last 24h", 8, y, 2); y += 18;
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  float tMinDisp = tempInFahrenheit ? (tMinC * 9/5 + 32) : tMinC;
  float tMaxDisp = tempInFahrenheit ? (tMaxC * 9/5 + 32) : tMaxC;
  snprintf(buf, sizeof(buf), "Temp %.1f - %.1f %s",
           tMinDisp, tMaxDisp, tempUnit());
  tft.drawString(buf, 8, y, 2); y += 18;
  snprintf(buf, sizeof(buf), "RH   %.0f - %.0f %%", rhMin, rhMax);
  tft.drawString(buf, 8, y, 2); y += 18;

  // Sample count / footer
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  snprintf(buf, sizeof(buf), "%lu samples", (unsigned long)samples);
  tft.drawString(buf, 8, y, 1); y += 12;

  // Tap hint at bottom
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Tap anywhere to go back", SCREEN_W / 2,
                 SCREEN_H - 10, 1);
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

// Helper for live setting changes: stop the periodic measurement,
// wait the datasheet-required 500 ms before issuing config commands,
// then restart. Any settings function passed in runs while the
// chip is in idle mode. Returns whatever the inner function returned.
//
// Bug history: previously we used delay(20) here, which is far too
// short - per the SCD41 datasheet, after stop_periodic_measurement
// the chip needs 500 ms before it accepts new commands. With the
// short delay, setSensorAltitude / setTemperatureOffset / etc.
// sometimes silently failed and the chip kept the old setting,
// even though our shadow variable + NVS were updated correctly.
template <typename Fn>
int16_t withChipIdle(Fn fn) {
  scd4x.stopPeriodicMeasurement();
  delay(500);
  int16_t err = fn();
  scd4x.startPeriodicMeasurement();
  lastReadTime = millis();
  return err;
}

void changeAltitude(int16_t delta) {
  int32_t v = (int32_t)sensorAltitudeM + delta;
  if (v < ALT_MIN) v = ALT_MIN;
  if (v > ALT_MAX) v = ALT_MAX;
  uint16_t newAlt = (uint16_t)v;
  if (newAlt == sensorAltitudeM) return;   // no change

  sensorAltitudeM = newAlt;
  savePref("alt", sensorAltitudeM);
  int16_t err = withChipIdle([]() {
    return scd4x.setSensorAltitude(sensorAltitudeM);
  });
  if (err) logError("setSensorAltitude (live)", err);
  else Serial.printf("Altitude set to %u m\n", sensorAltitudeM);
}

void changeTempOffset(int direction) {
  float deltaC = tempInFahrenheit
               ? (direction * TEMP_OFF_STEP_F_IN_C)
               : (direction * TEMP_OFF_STEP_C);
  float v = tempOffsetC + deltaC;
  if (v < TEMP_OFF_MIN) v = TEMP_OFF_MIN;
  if (v > TEMP_OFF_MAX) v = TEMP_OFF_MAX;
  if (v == tempOffsetC) return;   // no change

  tempOffsetC = v;
  savePref("tempoff", tempOffsetC);
  int16_t err = withChipIdle([]() {
    return scd4x.setTemperatureOffset(tempOffsetC);
  });
  if (err) logError("setTemperatureOffset (live)", err);
  else Serial.printf("Temp offset set to %.1f C\n", tempOffsetC);
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
    if (btnDate.contains(x, y)) {
      currentScreen = SCREEN_STATS;
      drawStatsScreen();
    } else if (btnCO2.contains(x, y)) {
      currentScreen = SCREEN_SETTINGS;
      drawSettingsScreen();
    } else if (btnClock.contains(x, y)) {
      currentScreen = SCREEN_TIME_SETTINGS;
      drawTimeSettingsScreen();
    } else if (btnTempUnit.contains(x, y) && dataValid) {
      tempInFahrenheit = !tempInFahrenheit;
      savePref("tempf", tempInFahrenheit);
      drawReadings();
      // Temperature unit affects the graph if it's currently in Temp
      // mode, so refresh the frame and redraw the trace.
      if (graphMode == GM_TEMP) {
        recomputeChartBounds();
        drawGraphFrame();
        drawGraph();
      }
    } else if (btnRadon.contains(x, y) && radonAvailable()) {
      // Toggle radon units Bq/m3 <-> pCi/L
      radonInPCi = !radonInPCi;
      savePref("rdnpci", radonInPCi);
      drawBottomValues();
      if (graphMode == GM_RADON) {
        recomputeChartBounds();
        drawGraphFrame();
        drawGraph();
      }
    } else if (btnGraph.contains(x, y)) {
      // Cycle CO2 -> Temp -> RH -> (Radon if present) -> CO2
      graphMode = (GraphMode)((graphMode + 1) % 4);
      if (graphMode == GM_RADON && !radonAvailable()) graphMode = GM_CO2;
      savePref("gmode", (uint8_t)graphMode);
      // Force a frame redraw because labels and caption changed
      recomputeChartBounds();
      drawGraphFrame();
      drawGraph();
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
  } else if (currentScreen == SCREEN_STATS) {
    // Tap anywhere returns to main
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

void addGraphSample(uint16_t co2, float t_c, float rh, uint16_t radonBq) {
  co2History[historyHead]   = co2;
  tempHistory[historyHead]  = (int16_t)(t_c * 100.0f);
  rhHistory[historyHead]    = (uint16_t)(rh * 10.0f);
  radonHistory[historyHead] = radonBq;
  historyHead = (historyHead + 1) % MAX_SAMPLES;
  if (historyCount < MAX_SAMPLES) historyCount++;
  saveHistory();
}

// ============================================================
//                  RADON (Aranet Rn over BLE)
// ============================================================

// True when we have a fresh radon reading from the selected device.
bool radonAvailable() {
  if (currentRadonBq == RADON_NONE) return false;
  return (millis() - lastRadonRxMs) < (unsigned long)radonStaleMin * 60000UL;
}

bool radonLowBattery() {
  return radonAvailable() && radonBattery > 0 && radonBattery <= RADON_BATT_LOW;
}

// Convert a stored Bq/m3 value to the active display unit.
float radonToDisplay(uint16_t bq) {
  return radonInPCi ? (bq / BQ_PER_PCIL) : (float)bq;
}
float radonDisplayValue() { return radonToDisplay(currentRadonBq); }
const char* radonUnitStr()      { return radonInPCi ? "pCi/L" : "Bq/m3"; }
const char* radonUnitShortStr() { return radonInPCi ? "pCi"   : "Bq"; }

// Zone colour for the radon number / plot, by Bq/m3 thresholds.
uint16_t radonColor(uint16_t bq) {
  if (bq == RADON_NONE)            return COLOR_TEXT;
  if (bq >= RADON_ZONE_HIGH_BQ)    return TFT_RED;
  if (bq >= RADON_ZONE_MOD_BQ)     return TFT_ORANGE;
  return TFT_GREEN;
}

// --- Seen-device table helpers ---

// Drop devices we have not heard from within the staleness window.
void radonAgeOutDevices() {
  unsigned long ttl = (unsigned long)radonStaleMin * 60000UL;
  for (int i = 0; i < BLE_MAX_DEVICES; i++) {
    if (bleSeen[i].used && (millis() - bleSeen[i].lastSeenMs) > ttl) {
      bleSeen[i].used = false;
    }
  }
}

// Insert/update a device in the seen table (keyed by MAC).
void radonUpsertDevice(const char* mac, const char* name, int rssi,
                       uint16_t bq, uint8_t batt, uint8_t status) {
  int slot = -1, oldest = -1;
  unsigned long oldestMs = 0xFFFFFFFF;
  for (int i = 0; i < BLE_MAX_DEVICES; i++) {
    if (bleSeen[i].used && strcmp(bleSeen[i].mac, mac) == 0) { slot = i; break; }
    if (!bleSeen[i].used && slot < 0) slot = i;
    if (bleSeen[i].used && bleSeen[i].lastSeenMs < oldestMs) {
      oldestMs = bleSeen[i].lastSeenMs; oldest = i;
    }
  }
  if (slot < 0) slot = oldest;            // table full: evict the stalest
  if (slot < 0) return;
  BleRnDevice& d = bleSeen[slot];
  d.used = true;
  strncpy(d.mac, mac, sizeof(d.mac) - 1);   d.mac[sizeof(d.mac) - 1]   = '\0';
  if (name && name[0]) { strncpy(d.name, name, sizeof(d.name) - 1); d.name[sizeof(d.name) - 1] = '\0'; }
  else if (!d.name[0]) d.name[0] = '\0';
  d.rssi = rssi; d.radonBq = bq; d.battery = batt; d.status = status;
  d.lastSeenMs = millis();
}

// Pick the active device: the pinned MAC if present & fresh, else the
// strongest-RSSI fresh device. Copies its values into the radon globals.
void radonSelectActive() {
  radonAgeOutDevices();
  int best = -1;
  for (int i = 0; i < BLE_MAX_DEVICES; i++) {
    if (!bleSeen[i].used) continue;
    if (radonTargetMac.length()) {
      if (radonTargetMac.equalsIgnoreCase(bleSeen[i].mac)) { best = i; break; }
      continue;
    }
    if (best < 0 || bleSeen[i].rssi > bleSeen[best].rssi) best = i;
  }
  if (best < 0) return;   // nothing matching (pinned device out of range)
  BleRnDevice& d = bleSeen[best];
  currentRadonBq   = d.radonBq;
  radonBattery     = d.battery;
  radonStatusColor = d.status;
  radonSrcMac      = d.mac;
  radonSrcName     = d.name;
  lastRadonRxMs    = d.lastSeenMs;
}

// Parse one advertisement; returns true if it was an Aranet Rn we accepted.
bool radonParseAdvert(NimBLEAdvertisedDevice* dev) {
  std::string md = dev->getManufacturerData();
  if (md.size() < ARANET_MFG_MIN_LEN) return false;
  const uint8_t* b = (const uint8_t*)md.data();
  if (b[0] != (ARANET_COMPANY_ID & 0xFF)) return false;   // 0x02
  if (b[1] != (ARANET_COMPANY_ID >> 8))   return false;   // 0x07
  if (b[2] != ARANET_TYPE_RADON)          return false;   // radon device
  uint16_t bq = b[RADON_OFF_VALUE] | (b[RADON_OFF_VALUE + 1] << 8);
  if (bq >= RADON_INVALID_MIN) return false;              // invalid reading
  uint8_t batt   = b[RADON_OFF_BATTERY];
  uint8_t status = b[RADON_OFF_STATUS];
  std::string nm = dev->getName();
  radonUpsertDevice(dev->getAddress().toString().c_str(),
                    nm.c_str(), dev->getRSSI(), bq, batt, status);
  return true;
}

// Advertisement callback. Runs on the NimBLE task during a scan, so it must
// NOT touch the String selection globals (that happens on the main task via
// the bleTableDirty flag). It only updates the seen-device table.
class RadonScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    if (radonParseAdvert(dev)) bleTableDirty = true;
  }
};
RadonScanCallbacks radonScanCallbacks;

void bleScanComplete(NimBLEScanResults results) {
  bleScanning   = false;
  bleTableDirty = true;   // re-select on the main task after the scan
}

void setupBle() {
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEScan* s = NimBLEDevice::getScan();
  s->setAdvertisedDeviceCallbacks(&radonScanCallbacks, /*wantDuplicates=*/true);
  s->setActiveScan(true);     // active = also pull scan-response friendly name
  s->setInterval(100);
  s->setWindow(99);
  bleReady = true;
  Serial.println("BLE: NimBLE scanner ready");
}

// Periodic loop hook: kick a short non-blocking scan every BLE_SCAN_PERIOD_MS.
// Runs on the main task, so this is where the active-device selection (which
// touches String globals) happens - never in the BLE callback.
void serviceBleScan() {
  if (!bleReady) return;
  if (bleTableDirty) { bleTableDirty = false; radonSelectActive(); }
  if (bleScanning) return;
  static unsigned long lastScanMs = 0;
  unsigned long now = millis();
  if (lastScanMs != 0 && (now - lastScanMs) < BLE_SCAN_PERIOD_MS) return;
  lastScanMs = now;
  bleScanning = true;
  NimBLEScan* s = NimBLEDevice::getScan();
  s->clearResults();
  if (!s->start(BLE_SCAN_SECS, bleScanComplete, false)) {
    bleScanning = false;   // start failed; try again next period
  }
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
  if (MDNS.begin(mdnsHostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsRunning = true;
    Serial.printf("mDNS: http://%s.local/\n", mdnsHostname.c_str());
  } else {
    Serial.println("mDNS: failed");
  }
}

// Hostname validation per RFC 1123 / mDNS conventions:
// - 1 to MDNS_HOSTNAME_MAX_LEN chars
// - lowercase letters, digits, hyphens
// - must not start or end with a hyphen
static bool isValidHostname(const String& s) {
  if (s.length() == 0 || s.length() > MDNS_HOSTNAME_MAX_LEN) return false;
  if (s[0] == '-' || s[s.length() - 1] == '-') return false;
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-';
    if (!ok) return false;
  }
  return true;
}

void restartMdns() {
  if (mdnsRunning) {
    MDNS.end();
    mdnsRunning = false;
  }
  startMdns();
}

void tryWifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(mdnsHostname.c_str());

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
  wm.startConfigPortal(wifiApName.c_str());
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
//                     MQTT
// ============================================================

// Build a topic by joining the prefix with a sub-path.
String mqttTopic(const char* sub) {
  String t = mqttTopicPrefix;
  if (!t.endsWith("/")) t += "/";
  t += sub;
  return t;
}

// Build a JSON state payload with all current readings.
String mqttBuildStatePayload() {
  String j = "{";
  j += "\"co2\":";    j += String(currentCO2);            j += ",";
  j += "\"temp_c\":"; j += String(currentTemp, 2);        j += ",";
  j += "\"temp_f\":"; j += String(currentTemp * 9.0f / 5.0f + 32.0f, 2);
  j += ",";
  j += "\"rh\":";     j += String(currentRH, 1);          j += ",";
  j += "\"status\":\""; j += co2StatusKey(currentCO2);    j += "\"";
  // Radon only when a fresh reading exists (HA keeps last value otherwise).
  if (radonAvailable()) {
    j += ",\"radon\":";         j += String(currentRadonBq);
    j += ",\"radon_battery\":"; j += String(radonBattery);
  }
  j += "}";
  return j;
}

// Publish Home Assistant MQTT discovery messages so HA picks up
// the device's sensors automatically. We register one config per
// metric, all sharing a "device" block so HA groups them.
//
// Discovery topic format:
//   {discovery_prefix}/sensor/{node_id}/{object_id}/config
// Where node_id is unique per device and object_id is per metric.
void mqttPublishDiscovery() {
  if (!mqttDiscoveryEnabled) return;
  String node = mdnsHostname;
  // HA requires node IDs to match [a-zA-Z0-9_-]+; replace any other char.
  String nodeSafe;
  for (size_t i = 0; i < node.length(); i++) {
    char c = node[i];
    nodeSafe += ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                 (c >= '0' && c <= '9') || c == '-' || c == '_')
                ? c : '_';
  }
  String stateTopic = mqttTopic("state");

  // Common device block
  String dev = "\"dev\":{";
  dev += "\"identifiers\":[\"" + nodeSafe + "\"],";
  dev += "\"name\":\"" + mdnsHostname + "\",";
  dev += "\"manufacturer\":\"" + String(FIRMWARE_AUTHOR) + "\",";
  dev += "\"model\":\"CYD CO2 Monitor\",";
  dev += "\"sw_version\":\"" + String(FIRMWARE_VERSION) + "\"";
  dev += "}";

  // Per-sensor config publish
  struct SensorCfg {
    const char* key;        // object_id / sub-key
    const char* name;       // friendly name
    const char* device_class;
    const char* unit;
    const char* state_class;
    const char* value_tmpl; // jinja extracting from state JSON
  };
  SensorCfg sensors[] = {
    {"co2",    "CO2",         "carbon_dioxide", "ppm", "measurement",
     "{{ value_json.co2 }}"},
    {"temp",   "Temperature", "temperature",
     tempInFahrenheit ? "\\u00b0F" : "\\u00b0C", "measurement",
     tempInFahrenheit ? "{{ value_json.temp_f }}" : "{{ value_json.temp_c }}"},
    {"rh",     "Humidity",    "humidity",       "%",   "measurement",
     "{{ value_json.rh }}"},
    // Radon has no standard HA device_class - leave it empty (omitted below).
    {"radon",      "Radon",         "",        "Bq/m\\u00b3", "measurement",
     "{{ value_json.radon }}"},
    {"radon_batt", "Radon Battery", "battery", "%",           "measurement",
     "{{ value_json.radon_battery }}"}
  };
  for (auto& s : sensors) {
    String topic = mqttDiscoveryPrefix + "/sensor/" + nodeSafe + "/"
                   + s.key + "/config";
    String payload = "{";
    payload += "\"name\":\""   + String(s.name)         + "\",";
    payload += "\"uniq_id\":\""+ nodeSafe + "_" + s.key + "\",";
    payload += "\"stat_t\":\"" + stateTopic             + "\",";
    if (s.device_class[0])
      payload += "\"dev_cla\":\""+ String(s.device_class) + "\",";
    payload += "\"unit_of_meas\":\"" + String(s.unit)   + "\",";
    payload += "\"stat_cla\":\""+ String(s.state_class) + "\",";
    payload += "\"val_tpl\":\"" + String(s.value_tmpl)  + "\",";
    payload += dev;
    payload += "}";
    // Retained so HA picks it up after a broker restart
    bool ok = mqttClient.publish(topic.c_str(), payload.c_str(), true);
    if (!ok) {
      Serial.printf("MQTT discovery publish failed: %s\n", topic.c_str());
    }
  }
  mqttDiscoveryPublished = true;
  Serial.println("MQTT: HA discovery published");
}

// Try to connect to the broker. Returns true if connected.
bool mqttTryConnect() {
  if (!mqttEnabled || mqttHost.length() == 0) return false;
  if (mqttClient.connected()) return true;
  if (!wifiConnected) return false;

  mqttClient.setServer(mqttHost.c_str(), mqttPort);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE_S);

  String clientId = mdnsHostname;
  bool ok;
  if (mqttUser.length() > 0) {
    ok = mqttClient.connect(clientId.c_str(), mqttUser.c_str(),
                            mqttPass.c_str());
  } else {
    ok = mqttClient.connect(clientId.c_str());
  }
  if (ok) {
    Serial.printf("MQTT: connected to %s:%u as %s\n",
                  mqttHost.c_str(), (unsigned)mqttPort, clientId.c_str());
    // Publish HA discovery once per fresh connection
    mqttDiscoveryPublished = false;
    mqttPublishDiscovery();
  } else {
    Serial.printf("MQTT: connect failed (rc=%d)\n", mqttClient.state());
  }
  return ok;
}

// Publish the current readings. Called every MQTT_PUBLISH_INTERVAL_MS.
void mqttPublishReadings() {
  if (!mqttClient.connected()) return;
  if (!dataValid) return;

  char buf[24];
  snprintf(buf, sizeof(buf), "%u", currentCO2);
  mqttClient.publish(mqttTopic("co2").c_str(), buf);

  snprintf(buf, sizeof(buf), "%.2f", currentTemp);
  mqttClient.publish(mqttTopic("temp_c").c_str(), buf);

  snprintf(buf, sizeof(buf), "%.2f", currentTemp * 9.0f / 5.0f + 32.0f);
  mqttClient.publish(mqttTopic("temp_f").c_str(), buf);

  snprintf(buf, sizeof(buf), "%.1f", currentRH);
  mqttClient.publish(mqttTopic("rh").c_str(), buf);

  mqttClient.publish(mqttTopic("status").c_str(), co2StatusKey(currentCO2));

  if (radonAvailable()) {
    snprintf(buf, sizeof(buf), "%u", currentRadonBq);
    mqttClient.publish(mqttTopic("radon").c_str(), buf);
    snprintf(buf, sizeof(buf), "%u", radonBattery);
    mqttClient.publish(mqttTopic("radon_battery").c_str(), buf);
  }

  // JSON state for HA discovery consumers
  String state = mqttBuildStatePayload();
  mqttClient.publish(mqttTopic("state").c_str(), state.c_str());
}

// Main service loop hook. Cheap when MQTT disabled.
void serviceMqtt() {
  if (!mqttEnabled) return;
  if (!wifiConnected) return;

  if (!mqttClient.connected()) {
    if (millis() - lastMqttReconnectMs >= MQTT_RECONNECT_INTERVAL_MS) {
      lastMqttReconnectMs = millis();
      mqttTryConnect();
    }
  } else {
    mqttClient.loop();   // process keep-alives / incoming
    if (millis() - lastMqttPublishMs >= MQTT_PUBLISH_INTERVAL_MS) {
      lastMqttPublishMs = millis();
      mqttPublishReadings();
    }
  }
}

void startWebServer() {
  if (webServerRunning) return;
  server.on("/",             handleRoot);
  server.on("/data.json",    handleData);
  server.on("/history.csv",  handleHistoryCsv);
  server.on("/history.json", handleHistoryJson);
  server.on("/sethostname",  HTTP_POST, handleSetHostname);
  server.on("/mqtt",         handleMqttPage);
  server.on("/setmqtt",      HTTP_POST, handleSetMqtt);
  server.on("/radon",        handleRadonPage);
  server.on("/setradon",     HTTP_POST, handleSetRadon);
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

// Escape characters that have meaning in HTML so that user-supplied
// strings (like the mDNS hostname) can be rendered safely inside
// page templates and form attributes.
String escapeHtml(const String& s) {
  String out;
  out.reserve(s.length());
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&':  out += "&amp;";  break;
      case '<':  out += "&lt;";   break;
      case '>':  out += "&gt;";   break;
      case '"':  out += "&quot;"; break;
      case '\'': out += "&#39;";  break;
      default:   out += c;        break;
    }
  }
  return out;
}

void handleRoot() {
  String html;
  html.reserve(14000);
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
  html += mdnsHostname; html += F(".local</small></h1>");

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

    if (radonAvailable()) {
      if (radonInPCi) snprintf(buf, sizeof(buf), "%.1f %s", radonDisplayValue(), radonUnitStr());
      else            snprintf(buf, sizeof(buf), "%u %s", currentRadonBq, radonUnitStr());
      html += F("<div class='tile'><div class='label'>Radon");
      if (radonLowBattery()) html += F(" (low battery)");
      html += F("</div><div class='value'>");
      html += buf; html += F("</div></div>");
    }

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

  // Chart containers: CO2, Temperature, Humidity, and (when present) Radon.
  // The radon section is hidden until the chart script confirms data.
  html += F("<h2>Recent CO2</h2>"
            "<div class='chart-wrap'>"
            "<canvas id='cCO2' height='220'></canvas></div>"
            "<h2>Recent Temperature</h2>"
            "<div class='chart-wrap'>"
            "<canvas id='cTemp' height='180'></canvas></div>"
            "<h2>Recent Humidity</h2>"
            "<div class='chart-wrap'>"
            "<canvas id='cRH' height='180'></canvas></div>"
            "<div id='radonSection' style='display:none'>"
            "<h2 id='radonHdr'>Recent Radon</h2>"
            "<div class='chart-wrap'>"
            "<canvas id='cRadon' height='180'></canvas></div></div>");

  // Actions
  html += F("<div class='actions'>"
            "<a href='/history.csv' download>Download Full CSV</a> "
            "<a href='/data.json'>JSON</a> "
            "<a href='/update'>Firmware Update</a> "
            "<a href='/mqtt'>MQTT Setup</a> "
            "<a href='/radon'>Radon Setup</a>"
            "</div>");

  // Sample count info
  html += F("<p style='color:#888;font-size:.9em'>");
  html += String(logHdr.count); html += F(" samples logged out of ");
  html += String(logHdr.capacity); html += F(" capacity. ");
  html += String(historyCount); html += F(" in live graph.</p>");

  // Hostname configuration
  html += F("<h2>Network name</h2>"
            "<p style='color:#888;font-size:.9em;margin:4px 0 8px'>"
            "Lowercase letters, digits, and hyphens only. Max 31 chars. "
            "Cannot start or end with a hyphen.</p>"
            "<form method='POST' action='/sethostname' "
            "style='display:flex;gap:8px;align-items:center'>"
            "<input name='h' value='");
  html += escapeHtml(mdnsHostname);
  html += F("' maxlength='31' "
            "style='flex:1;padding:6px 8px;background:#222;color:#eee;"
            "border:1px solid #444;border-radius:4px;font-family:monospace'>"
            "<span style='color:#888'>.local</span>"
            "<button style='background:#2a4060;color:#cef;border:0;"
            "padding:8px 16px;border-radius:4px;cursor:pointer'>"
            "Save</button></form>");

  // Chart script - fetch /history.json and draw 3 charts
  html += F("<script>"
            "fetch('/history.json?n=288').then(r=>r.json()).then(d=>{"
            "  const common={responsive:true,maintainAspectRatio:false,"
            "    interaction:{intersect:false,mode:'index'},"
            "    scales:{"
            "      y:{grid:{color:'#333'},ticks:{color:'#aaa'}},"
            "      x:{grid:{color:'#333'},"
            "         ticks:{color:'#aaa',maxTicksLimit:8}}"
            "    },"
            "    plugins:{legend:{labels:{color:'#aaa'}}}"
            "  };"
            "  function mk(id,label,data,color,yMin,yMax){"
            "    const opts=JSON.parse(JSON.stringify(common));"
            "    if(yMin!==null)opts.scales.y.suggestedMin=yMin;"
            "    if(yMax!==null)opts.scales.y.suggestedMax=yMax;"
            "    new Chart(document.getElementById(id),{type:'line',"
            "      data:{labels:d.times,datasets:[{label:label,data:data,"
            "        borderColor:color,borderWidth:1.5,"
            "        pointRadius:0,fill:false,tension:0.2}]},"
            "      options:opts});"
            "  }"
            "  mk('cCO2','CO2 ppm',d.co2,'#6cf',400,1500);"
            "  mk('cTemp','Temp '+d.temp_unit,d.temp,'#fa3',null,null);"
            "  mk('cRH','Humidity %',d.rh,'#9c6',20,80);"
            "  if(d.radon&&d.radon.some(v=>v!==null)){"
            "    document.getElementById('radonSection').style.display='block';"
            "    document.getElementById('radonHdr').textContent="
            "      'Recent Radon ('+d.radon_unit+')';"
            "    mk('cRadon','Radon '+d.radon_unit,d.radon,'#c9f',0,null);"
            "  }"
            "});"
            "setTimeout(()=>location.reload(),60000);"
            "</script>");

  html += F("<p style='color:#555;font-size:.8em;text-align:center;"
            "margin-top:32px'>Firmware v");
  html += FIRMWARE_VERSION;
  html += F(" by ");
  html += FIRMWARE_AUTHOR;
  html += F("</p></body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleData() {
  String j = "{";
  j += "\"firmware\":\""; j += FIRMWARE_VERSION; j += "\",";
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
  // Radon (Aranet Rn over BLE). Present only when a fresh reading exists.
  bool radv = radonAvailable();
  j += ",\"radon_valid\":"; j += (radv ? "true" : "false");
  j += ",\"radon_unit\":\""; j += radonUnitStr(); j += "\"";
  if (radv) {
    j += ",\"radon_bqm3\":"; j += String(currentRadonBq);
    j += ",\"radon_pcil\":"; j += String(currentRadonBq / BQ_PER_PCIL, 2);
    j += ",\"radon_battery\":"; j += String(radonBattery);
    j += ",\"radon_low_battery\":"; j += (radonLowBattery() ? "true" : "false");
    j += ",\"radon_age_s\":"; j += String((millis() - lastRadonRxMs) / 1000UL);
    j += ",\"radon_src\":\""; j += escapeHtml(radonSrcName.length() ? radonSrcName : radonSrcMac); j += "\"";
  }
  j += "}";
  server.send(200, "application/json", j);
}

// Stream the full log as CSV. Avoids buffering the whole thing in RAM.
void handleHistoryCsv() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=co2_history.csv");
  server.send(200, "text/csv", "");

  String header = "timestamp_utc,co2_ppm,temp_c,rh_pct,radon_bqm3\r\n";
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

    // Radon column is blank for samples taken with no radon data (kept
    // in SI Bq/m3, mirroring temp staying Celsius in the CSV).
    char rbuf[8];
    if (rec.radon_bqm3 == RADON_NONE) rbuf[0] = '\0';
    else snprintf(rbuf, sizeof(rbuf), "%u", rec.radon_bqm3);

    char line[96];
    if (rec.ts_utc != 0) {
      time_t t = (time_t)rec.ts_utc;
      struct tm tmv;
      gmtime_r(&t, &tmv);
      char tbuf[24];
      strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", &tmv);
      snprintf(line, sizeof(line), "%s,%u,%.2f,%.1f,%s\r\n",
               tbuf, rec.co2_ppm,
               rec.temp_c100 / 100.0f, rec.rh_x10 / 10.0f, rbuf);
    } else {
      snprintf(line, sizeof(line), ",%u,%.2f,%.1f,%s\r\n",
               rec.co2_ppm,
               rec.temp_c100 / 100.0f, rec.rh_x10 / 10.0f, rbuf);
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
  String temps = "[";
  String rhs   = "[";
  String radons = "[";

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
        if (i > 0) {
          times += ","; co2s += ","; temps += ","; rhs += ","; radons += ",";
        }
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
        // Temperature in user-selected unit (matches device display)
        float t_c = rec.temp_c100 / 100.0f;
        float t_disp = tempInFahrenheit ? (t_c * 9.0f / 5.0f + 32.0f) : t_c;
        temps += String(t_disp, 1);
        rhs   += String(rec.rh_x10 / 10.0f, 1);
        // Radon in the active display unit; null for samples with no data.
        if (rec.radon_bqm3 == RADON_NONE) radons += "null";
        else radons += String(radonToDisplay(rec.radon_bqm3), radonInPCi ? 2 : 0);
      }
      f.close();
    }
  }
  times += "]"; co2s += "]"; temps += "]"; rhs += "]"; radons += "]";

  String body = "{\"times\":";
  body += times;
  body += ",\"co2\":";
  body += co2s;
  body += ",\"temp\":";
  body += temps;
  body += ",\"temp_unit\":\"";
  body += tempUnit();
  body += "\",\"rh\":";
  body += rhs;
  body += ",\"radon\":";
  body += radons;
  body += ",\"radon_unit\":\"";
  body += radonUnitStr();
  body += "\"}";
  server.send(200, "application/json", body);
}

void handleSetHostname() {
  String requested = server.arg("h");
  requested.trim();
  // Be forgiving with case: lowercase the input before validating
  // (mDNS is case-insensitive but our validator only accepts lowercase).
  requested.toLowerCase();

  String style = F(
    "<style>body{font-family:system-ui,sans-serif;background:#111;"
    "color:#eee;margin:0;padding:20px;max-width:640px}"
    "h1{margin:0 0 16px;border-bottom:1px solid #444;padding-bottom:8px}"
    ".alert-bad{background:#3a1010;border-left:3px solid #f55;"
    "padding:8px 12px;margin:12px 0}"
    ".alert-good{background:#0a3a10;border-left:3px solid #5f5;"
    "padding:8px 12px;margin:12px 0}"
    "code{background:#222;padding:2px 6px;border-radius:3px}"
    "a{color:#9cf}</style>");

  if (!isValidHostname(requested)) {
    String html = F("<!DOCTYPE html><html><head>");
    html += style;
    html += F("</head><body><h1>Invalid hostname</h1>"
              "<div class='alert-bad'>That hostname is not allowed.</div>"
              "<p>Hostnames must use only lowercase letters, digits, "
              "and hyphens, must not start or end with a hyphen, and "
              "must be 1 to 31 characters long.</p>"
              "<p><a href='/'>&larr; Back</a></p>"
              "</body></html>");
    server.send(400, "text/html; charset=utf-8", html);
    return;
  }

  // No-op when unchanged
  if (requested == mdnsHostname) {
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
    return;
  }

  // Save and apply
  mdnsHostname = requested;
  savePref("mdns", mdnsHostname);
  WiFi.setHostname(mdnsHostname.c_str());
  restartMdns();
  Serial.printf("mDNS: hostname changed to '%s'\n", mdnsHostname.c_str());

  // Show a confirmation page with both the new mDNS link AND the IP
  // because the mDNS reannouncement can take a few seconds to
  // propagate, and the browser may have cached the old name.
  String ip = wifiConnected ? WiFi.localIP().toString() : String("");
  String html = F("<!DOCTYPE html><html><head>");
  html += style;
  html += F("</head><body><h1>Hostname updated</h1>"
            "<div class='alert-good'>Now reachable at:</div>"
            "<p><a href='http://");
  html += escapeHtml(mdnsHostname);
  html += F(".local/'>http://");
  html += escapeHtml(mdnsHostname);
  html += F(".local/</a></p>"
            "<p style='color:#888'>If the .local link does not work for "
            "30-60 seconds, your router or device has cached the old "
            "hostname. You can also reach the device by IP:</p>"
            "<p><a href='http://");
  html += ip;
  html += F("/'>http://");
  html += ip;
  html += F("/</a></p>"
            "</body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

// ----- MQTT configuration page -----

void handleMqttPage() {
  String html;
  html.reserve(6000);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>MQTT - CO2 Monitor</title>"
            "<style>"
            "body{font-family:system-ui,sans-serif;background:#111;color:#eee;"
              "margin:0;padding:20px;max-width:640px}"
            "h1{margin:0 0 16px;border-bottom:1px solid #444;padding-bottom:8px}"
            "label{display:block;margin:12px 0 4px;color:#bbb}"
            "input,select{width:100%;box-sizing:border-box;padding:8px;"
              "background:#222;color:#eee;border:1px solid #444;"
              "border-radius:4px;font-family:inherit;font-size:14px}"
            "input[type=checkbox]{width:auto;margin-right:6px}"
            ".row{display:flex;gap:8px}"
            ".row>div{flex:1}"
            ".tile{background:#222;border-radius:6px;padding:10px 14px;"
              "margin:12px 0}"
            ".tile .label{color:#888;font-size:.85em}"
            ".tile .value{font-size:1.1em;font-weight:600}"
            ".good{color:#3c3}.bad{color:#f55}"
            "button{background:#2a4060;color:#cef;border:0;"
              "padding:10px 20px;border-radius:4px;cursor:pointer;"
              "font-size:14px;margin-top:16px}"
            "button:hover{background:#3a557a}"
            "a{color:#9cf}"
            "p.help{color:#888;font-size:.9em;margin:4px 0}"
            "</style></head><body>");
  html += F("<h1>MQTT Configuration</h1>"
            "<p><a href='/'>&larr; Back to dashboard</a></p>");

  // Connection status tile
  html += F("<div class='tile'><div class='label'>Status</div><div class='value ");
  if (!mqttEnabled) {
    html += F("'>Disabled</div></div>");
  } else if (mqttClient.connected()) {
    html += F("good'>Connected to ");
    html += escapeHtml(mqttHost);
    html += F(":");
    html += String(mqttPort);
    html += F("</div></div>");
  } else if (mqttHost.length() == 0) {
    html += F("bad'>Enabled but no broker host set</div></div>");
  } else {
    html += F("bad'>Disconnected (rc=");
    html += String(mqttClient.state());
    html += F(")</div></div>");
  }

  html += F("<form method='POST' action='/setmqtt'>"
            "<label><input type='checkbox' name='en' value='1'");
  if (mqttEnabled) html += F(" checked");
  html += F("> Enable MQTT publishing</label>"
            "<label>Broker host (IP or hostname)</label>"
            "<input name='host' value='");
  html += escapeHtml(mqttHost);
  html += F("' placeholder='192.168.1.10 or mqtt.local'>"
            "<div class='row'>"
            "<div><label>Port</label>"
            "<input name='port' type='number' min='1' max='65535' value='");
  html += String(mqttPort);
  html += F("'></div>"
            "<div><label>Topic prefix</label>"
            "<input name='topic' value='");
  html += escapeHtml(mqttTopicPrefix);
  html += F("' placeholder='co2monitor'></div>"
            "</div>"
            "<p class='help'>Topics published: "
            "<code>{prefix}/co2</code>, <code>/temp_c</code>, "
            "<code>/temp_f</code>, <code>/rh</code>, "
            "<code>/status</code>, <code>/state</code> (JSON).</p>"
            "<label>Username (optional)</label>"
            "<input name='user' value='");
  html += escapeHtml(mqttUser);
  html += F("'>"
            "<label>Password (optional)</label>"
            "<input name='pass' type='password' value='");
  html += escapeHtml(mqttPass);
  html += F("' placeholder='(unchanged if blank)'>"
            "<p class='help'>Leave password blank to keep the current value.</p>"
            "<label><input type='checkbox' name='disc' value='1'");
  if (mqttDiscoveryEnabled) html += F(" checked");
  html += F("> Publish Home Assistant auto-discovery</label>"
            "<label>HA discovery prefix</label>"
            "<input name='disc_pfx' value='");
  html += escapeHtml(mqttDiscoveryPrefix);
  html += F("' placeholder='homeassistant'>"
            "<button type='submit'>Save and connect</button>"
            "</form>");

  html += F("</body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleSetMqtt() {
  bool en = server.hasArg("en");
  String host = server.arg("host");
  host.trim();
  uint16_t port = MQTT_DEFAULT_PORT;
  if (server.hasArg("port")) {
    int v = server.arg("port").toInt();
    if (v > 0 && v <= 65535) port = v;
  }
  String topic = server.arg("topic"); topic.trim();
  if (topic.length() == 0) topic = mdnsHostname;
  String user = server.arg("user"); user.trim();
  String pass = server.arg("pass");
  bool disc = server.hasArg("disc");
  String disc_pfx = server.arg("disc_pfx"); disc_pfx.trim();
  if (disc_pfx.length() == 0) disc_pfx = "homeassistant";

  // Apply
  mqttEnabled = en;
  mqttHost    = host;
  mqttPort    = port;
  mqttTopicPrefix = topic;
  mqttUser    = user;
  // Preserve password if the field was left blank
  if (pass.length() > 0) mqttPass = pass;
  mqttDiscoveryEnabled = disc;
  mqttDiscoveryPrefix  = disc_pfx;

  // Persist
  prefs.putBool  ("mqEn",    mqttEnabled);
  prefs.putString("mqHost",  mqttHost);
  prefs.putUShort("mqPort",  mqttPort);
  prefs.putString("mqTopic", mqttTopicPrefix);
  prefs.putString("mqUser",  mqttUser);
  prefs.putString("mqPass",  mqttPass);
  prefs.putBool  ("mqDisc",  mqttDiscoveryEnabled);
  prefs.putString("mqDiscPfx", mqttDiscoveryPrefix);

  // Disconnect any current session so the new settings take effect
  if (mqttClient.connected()) mqttClient.disconnect();
  lastMqttReconnectMs = 0;  // try connect immediately on next loop

  // Redirect back to /mqtt to show the new status
  server.sendHeader("Location", "/mqtt", true);
  server.send(303, "text/plain", "Saved");
}

// ----- Radon (Aranet Rn) configuration page -----

void handleRadonPage() {
  String html;
  html.reserve(6000);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Radon - CO2 Monitor</title><style>"
            "body{font-family:system-ui,sans-serif;background:#111;color:#eee;"
              "margin:0;padding:20px;max-width:640px}"
            "h1{margin:0 0 16px;border-bottom:1px solid #444;padding-bottom:8px}"
            "label{display:block;margin:12px 0 4px;color:#bbb}"
            "input,select{width:100%;padding:8px;background:#222;color:#eee;"
              "border:1px solid #444;border-radius:4px;box-sizing:border-box}"
            "button{margin-top:16px;background:#2a4060;color:#cef;border:0;"
              "padding:10px 20px;border-radius:4px;cursor:pointer;font-size:1em}"
            ".help{color:#888;font-size:.9em;margin:4px 0}"
            ".tbl{width:100%;border-collapse:collapse;margin:8px 0}"
            ".tbl th,.tbl td{text-align:left;padding:4px 8px;"
              "border-bottom:1px solid #333;font-size:.9em}"
            "a{color:#9cf}</style></head><body>");
  html += F("<h1>Radon (Aranet Rn)</h1>");

  if (radonAvailable()) {
    char b[24];
    if (radonInPCi) snprintf(b, sizeof(b), "%.1f %s", radonDisplayValue(), radonUnitStr());
    else            snprintf(b, sizeof(b), "%u %s", currentRadonBq, radonUnitStr());
    html += F("<p>Current: <b>"); html += b; html += F("</b> from ");
    html += escapeHtml(radonSrcName.length() ? radonSrcName : radonSrcMac);
    html += F(" (battery "); html += String(radonBattery); html += F("%)</p>");
  } else {
    html += F("<p class='help'>No fresh radon reading. Make sure the Aranet Rn "
              "has Smart Home Integration enabled and is within range.</p>");
  }

  // Seen-device table
  html += F("<h3>Devices seen</h3><table class='tbl'>"
            "<tr><th>Name</th><th>MAC</th><th>RSSI</th><th>Radon</th><th>Batt</th></tr>");
  bool any = false;
  for (int i = 0; i < BLE_MAX_DEVICES; i++) {
    if (!bleSeen[i].used) continue;
    any = true;
    html += F("<tr><td>"); html += escapeHtml(bleSeen[i].name[0] ? bleSeen[i].name : "(unnamed)");
    html += F("</td><td>"); html += bleSeen[i].mac;
    html += F("</td><td>"); html += String(bleSeen[i].rssi);
    html += F("</td><td>"); html += String(bleSeen[i].radonBq); html += F(" Bq");
    html += F("</td><td>"); html += String(bleSeen[i].battery); html += F("%</td></tr>");
  }
  if (!any) html += F("<tr><td colspan='5' style='color:#888'>none yet</td></tr>");
  html += F("</table>");

  // Settings form
  html += F("<form method='POST' action='/setradon'>"
            "<label>Default units</label><select name='unit'>"
            "<option value='bq'");
  if (!radonInPCi) html += F(" selected");
  html += F(">Bq/m3</option><option value='pci'");
  if (radonInPCi) html += F(" selected");
  html += F(">pCi/L</option></select>"
            "<label>Staleness timeout (minutes)</label>"
            "<input name='stale' type='number' min='1' max='240' value='");
  html += String(radonStaleMin);
  html += F("'><p class='help'>Hide radon if no advertisement is received "
            "within this time (the Rn measures every 10 minutes).</p>"
            "<label>Target device</label><select name='mac'>"
            "<option value=''");
  if (radonTargetMac.length() == 0) html += F(" selected");
  html += F(">Auto (strongest signal)</option>");
  bool pinnedSeen = false;
  for (int i = 0; i < BLE_MAX_DEVICES; i++) {
    if (!bleSeen[i].used) continue;
    html += F("<option value='"); html += bleSeen[i].mac; html += F("'");
    if (radonTargetMac.equalsIgnoreCase(bleSeen[i].mac)) { html += F(" selected"); pinnedSeen = true; }
    html += F(">");
    html += escapeHtml(bleSeen[i].name[0] ? bleSeen[i].name : bleSeen[i].mac);
    html += F(" ("); html += bleSeen[i].mac; html += F(")</option>");
  }
  if (radonTargetMac.length() && !pinnedSeen) {
    html += F("<option value='"); html += escapeHtml(radonTargetMac);
    html += F("' selected>"); html += escapeHtml(radonTargetMac);
    html += F(" (not in range)</option>");
  }
  html += F("</select><button type='submit'>Save</button></form>"
            "<p><a href='/'>&larr; Back</a></p></body></html>");
  server.send(200, "text/html; charset=utf-8", html);
}

void handleSetRadon() {
  radonInPCi = (server.arg("unit") == "pci");
  if (server.hasArg("stale")) {
    int v = server.arg("stale").toInt();
    if (v >= 1 && v <= 240) radonStaleMin = (uint16_t)v;
  }
  String mac = server.arg("mac"); mac.trim();
  radonTargetMac = mac;

  savePref("rdnpci",   radonInPCi);
  savePref("rdnstale", radonStaleMin);
  savePref("rdnmac",   radonTargetMac);

  radonSelectActive();   // re-pick with the new target / staleness window

  server.sendHeader("Location", "/radon", true);
  server.send(303, "text/plain", "Saved");
}

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
  // Note: we DO NOT persist this to NVS. ASC always reverts to OFF
  // on the next boot. If you want it on long-term, add 'asc on' to
  // your boot routine somewhere safe.
  ascEnabled = enable;
  int16_t err = withChipIdle([]() {
    return scd4x.setAutomaticSelfCalibrationEnabled(ascEnabled ? 1 : 0);
  });
  if (err) {
    Serial.printf("ASC: setAutomaticSelfCalibrationEnabled failed (err %d)\n", err);
  } else {
    Serial.printf("ASC %s (this session only; resets to OFF on reboot)\n",
                  enable ? "enabled" : "disabled");
  }
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
    Serial.printf("mDNS: http://%s.local/\n", mdnsHostname.c_str());
  } else {
    Serial.println("Not connected");
  }
}

void bleStatusSerial() {
  radonAgeOutDevices();
  Serial.printf("BLE scanner: %s%s\n", bleReady ? "ready" : "off",
                bleScanning ? " (scanning)" : "");
  Serial.printf("Selected: %s\n",
                radonAvailable()
                  ? (radonSrcName.length() ? radonSrcName.c_str() : radonSrcMac.c_str())
                  : "(none / stale)");
  if (radonAvailable()) {
    Serial.printf("  radon=%u Bq/m3 (%.1f pCi/L)  battery=%u%%  age=%lus\n",
                  currentRadonBq, currentRadonBq / BQ_PER_PCIL, radonBattery,
                  (millis() - lastRadonRxMs) / 1000UL);
  }
  Serial.printf("Target: %s   stale=%u min\n",
                radonTargetMac.length() ? radonTargetMac.c_str() : "auto (strongest)",
                radonStaleMin);
  Serial.println("Seen devices:");
  bool any = false;
  for (int i = 0; i < BLE_MAX_DEVICES; i++) {
    if (!bleSeen[i].used) continue;
    any = true;
    Serial.printf("  %-17s %-16s rssi=%d radon=%u Bq batt=%u%% age=%lus\n",
                  bleSeen[i].mac, bleSeen[i].name[0] ? bleSeen[i].name : "(unnamed)",
                  bleSeen[i].rssi, bleSeen[i].radonBq, bleSeen[i].battery,
                  (millis() - bleSeen[i].lastSeenMs) / 1000UL);
  }
  if (!any) Serial.println("  (none in range)");
}

// ============================================================
//                  SERIAL COMMANDS
// ============================================================

void printInfo() {
  Serial.printf("Firmware:    v%s by %s\n", FIRMWARE_VERSION, FIRMWARE_AUTHOR);
  // Query the chip directly for its current settings. These are the
  // authoritative values - if they disagree with our shadow vars,
  // something has gone wrong (failed write, stale NVS, etc).
  scd4x.stopPeriodicMeasurement();
  delay(500);   // datasheet requires this before any config command
  uint16_t chipAsc = 0xFFFF;
  uint16_t chipAlt = 0xFFFF;
  float chipTempOff = NAN;
  int16_t errAsc = scd4x.getAutomaticSelfCalibrationEnabled(chipAsc);
  int16_t errAlt = scd4x.getSensorAltitude(chipAlt);
  int16_t errToff = scd4x.getTemperatureOffset(chipTempOff);
  scd4x.startPeriodicMeasurement();
  lastReadTime = millis();

  Serial.printf("ASC (shadow):  %s\n", ascEnabled ? "ON" : "OFF");
  if (errAsc) {
    Serial.printf("ASC (chip):    <read error %d>\n", errAsc);
  } else {
    Serial.printf("ASC (chip):    %s%s\n",
                  chipAsc ? "ON" : "OFF",
                  (chipAsc != 0) != ascEnabled ? "  <-- MISMATCH!" : "");
  }
  Serial.printf("Alt (shadow):  %u m\n", sensorAltitudeM);
  if (errAlt) {
    Serial.printf("Alt (chip):    <read error %d>\n", errAlt);
  } else {
    Serial.printf("Alt (chip):    %u m%s\n",
                  chipAlt,
                  chipAlt != sensorAltitudeM ? "  <-- MISMATCH!" : "");
  }
  Serial.printf("Toff (shadow): %.1f C  (%.1f F)\n",
                tempOffsetC, tempOffsetC * 9.0f / 5.0f);
  if (errToff) {
    Serial.printf("Toff (chip):   <read error %d>\n", errToff);
  } else {
    Serial.printf("Toff (chip):   %.1f C%s\n",
                  chipTempOff,
                  fabs(chipTempOff - tempOffsetC) > 0.1f ? "  <-- MISMATCH!" : "");
  }
  Serial.printf("Temp unit:   %s\n", tempInFahrenheit ? "F" : "C");
  if (brightnessPct == BRIGHTNESS_AUTO) {
    Serial.printf("Brightness:  AUTO (LDR %.0f%% -> backlight %u%%)\n",
                  ldrSmoothedPct, lastAutoDutyPct);
  } else {
    Serial.printf("Brightness:  %u%%\n", (unsigned)brightnessPct);
  }
  Serial.printf("LDR raw:     %d\n", analogRead(LDR_PIN));
  Serial.printf("Timezone:    UTC%+d:%02d\n", tzOffsetMin / 60, abs(tzOffsetMin) % 60);
  Serial.printf("Clock:       %s\n", use24hClock ? "24h" : "12h");
  Serial.printf("NTP:         %s (%s)\n", ntpEnabled ? "on" : "off", ntpServer.c_str());
  Serial.printf("Hostname:    %s.local\n", mdnsHostname.c_str());
  Serial.printf("Uptime:      %lu s total\n", (unsigned long)getTotalUptimeSec());
  uint32_t d = daysSinceLastFrc();
  if (d == UINT32_MAX) Serial.println("Last cal:    never");
  else                 Serial.printf("Last cal:    %lu days ago\n", (unsigned long)d);
  Serial.printf("Log:         %lu / %lu records\n",
                (unsigned long)logHdr.count, (unsigned long)logHdr.capacity);
  Serial.printf("Live graph:  %d samples\n", historyCount);
  wifiStatusSerial();
  Serial.printf("MQTT:        %s\n",
                !mqttEnabled            ? "disabled" :
                mqttClient.connected()  ? "connected" :
                mqttHost.length() == 0  ? "no host"   : "disconnected");
  if (mqttEnabled && mqttHost.length() > 0) {
    Serial.printf("MQTT broker: %s:%u\n", mqttHost.c_str(), (unsigned)mqttPort);
    Serial.printf("MQTT topic:  %s\n", mqttTopicPrefix.c_str());
  }
  if (radonAvailable()) {
    Serial.printf("Radon:       %u Bq/m3 (%.1f pCi/L) batt %u%% from %s ('ble-status' for more)\n",
                  currentRadonBq, currentRadonBq / BQ_PER_PCIL, radonBattery,
                  radonSrcName.length() ? radonSrcName.c_str() : radonSrcMac.c_str());
  } else {
    Serial.println("Radon:       none (no fresh Aranet Rn advertisement)");
  }
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
  else if (lower == "ble-status")      bleStatusSerial();
  else if (lower == "info")            printInfo();
  else if (lower == "reset")           resetPrefs();
  else if (lower == "erase-log")       eraseLog();
  else Serial.println("Commands: 'frc <ppm>', 'asc on/off', "
                      "'wifi-setup', 'wifi-reset', 'wifi-status', "
                      "'ble-status', 'info', 'reset', 'erase-log'");
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
  tft.drawString("CO2 Monitor", SCREEN_W / 2, SCREEN_H / 2 - 30, 4);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("v" FIRMWARE_VERSION, SCREEN_W / 2, SCREEN_H / 2 + 4, 2);
  tft.drawString("Initializing...",   SCREEN_W / 2, SCREEN_H / 2 + 32, 2);

  // Author byline at bottom of splash
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("by " FIRMWARE_AUTHOR, SCREEN_W / 2, SCREEN_H - 16, 1);

  Serial.printf("Firmware v%s by %s\n", FIRMWARE_VERSION, FIRMWARE_AUTHOR);

  initSensor();
  delay(5500);

  wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);
  wm.setConfigPortalBlocking(false);
  wm.setBreakAfterConfig(true);
  wm.setShowInfoErase(false);

  tryWifiConnect();

  // BLE radon scanner. Brought up after WiFi so the radio's software
  // coexistence is already initialised; scans run periodically in loop().
  setupBle();

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
  serviceAutoBrightness();
  serviceMqtt();
  serviceBleScan();

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
    bool fresh = readSensor();
    bool radNow = radonAvailable();
    // Redraw on a new sensor reading, or when radon (dis)appeared so the
    // bottom row reflows between 2- and 3-column layouts.
    if (isMainScreen && (fresh || radNow != radonShownLast)) {
      // If we were viewing the radon graph and it went stale, redraw the
      // graph too (drawGraph falls back to CO2).
      if (!radNow && graphMode == GM_RADON) drawGraph();
      drawReadings();
    }
  }

  if (dataValid &&
      (lastSampleTime == 0 || now - lastSampleTime >= SAMPLE_INTERVAL_MS)) {
    lastSampleTime = now;
    uint16_t radonSample = radonAvailable() ? currentRadonBq : RADON_NONE;
    addGraphSample(currentCO2, currentTemp, currentRH, radonSample);
    appendLog(currentCO2, currentTemp, currentRH, radonSample);
    if (isMainScreen) {
      drawGraph();
      drawReadings();   // refresh trend arrow
    }
  }
}
