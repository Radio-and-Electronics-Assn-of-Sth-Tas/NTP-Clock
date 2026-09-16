/*
   Title:            NTP Dual Clock (v4.x)
   Author:           (original) Bruce E. Hall, w8bh.net (Original Version)
                     (current) RAHinsley (VK2ARH), MJCulross (KD5RXT)
   Date:             (original) 13 Feb 2021
                     (current) 25 Jul 2026
   Hardware:         ESP32-S2-mini or ESP32-S3-mini, with ILI9341 TFT display
   Software:         (see the README.TXT file for information on setup & required library versions)
   Legal:            Copyright (c) 2021 Bruce E. Hall.
                     Open Source under the terms of the MIT License.

   Description:      Dual UTC/Local NTP Clock with TFT display. Time is refreshed via
                     NTP. Status indicator for time freshness & WiFi strength.

                     Also, you can set the 'TITLE' symbol to your callsign or any
                     other 8 character (max) string if you wish to have it display
                     in the Clock Banner.

                     You can also define a list of timezones in the UserSettings.h
                     header file to be displayed in sequence in the local timezone area.

   Revision History: (see the README.txt file for detailed revision history)
*/

#define VERSION_TIMESTAMP "REAST 1.4"

//#define GPS_TRY_REVERSED_RXTX_FIRST                                            // uncomment/activate this to try the reversed GPS RX/TX pin definition first
//#define DISABLE_BUTTON_DEF_TIMEOUT                                             // uncomment to disable the automatic timeout on the initial button definition screen

#include <FS.h>                                                                // Needed for WiFi
#include <TFT_eSPI.h>                                                          // TFT display library
#include <ezTime.h>                                                            // Time & timezone management library
#include <ArduinoJson.h>                                                       // Used to parse the internet weather data
#include "tft_setup.h"                                                         // Customized settings for TFT display (varies by target processor)
#include "UserSettings.h"                                                      // User customizable defaults & settings
#include <Button2.h>                                                           // Changed from "Button.h" 12/12/25
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ESPAsyncWebServer.h>


#define BACKLIGHT_PIN        5                                                 // Display backlight control
#define I2C_SCL             35                                                 // BMx280 I2C Clock line
#define I2C_SDA             33                                                 // BMx280 I2C Data line
#define CONFIG_PIN          16                                                 // Make CONFIG changes on-the-fly
#define BRIGHT_PIN           9                                                 // Brightness
#define LIGHT_SENSOR_PIN     3                                                 // Light sensor analog pin
#define LED_PIN             15                                                 // Built-in LED is also on D4
#define INPUT_5VDC_PIN      10                                                 // 5VDC voltage measurement
#define INPUT_12VDC_PIN      8                                                 // 12VDC voltage measurement
#define GPS_DATA_OUT_PIN    40                                                 // Data output from GPS to processor
#define GPS_DATA_IN_PIN     38                                                 // Data input from processor to GPS


#ifdef ESP32S3_MINI

#undef BACKLIGHT_PIN
#undef I2C_SCL
#undef I2C_SDA
#undef CONFIG_PIN
#undef BRIGHT_PIN
#undef LIGHT_SENSOR_PIN
#undef LED_PIN
#undef INPUT_5VDC_PIN
#undef INPUT_12VDC_PIN
#undef GPS_DATA_OUT_PIN
#undef GPS_DATA_IN_PIN

#define BACKLIGHT_PIN        4                                                 // Display backlight control
#define I2C_SCL             36                                                 // BMx280 I2C Clock line
#define I2C_SDA             35                                                 // BMx280 I2C Data line
#define CONFIG_PIN          16                                                 // Make CONFIG changes on-the-fly
#define BRIGHT_PIN          13                                                 // Brightness
#define LIGHT_SENSOR_PIN     2                                                 // Light sensor analog pin
#define LED_PIN             47                                                 // Built-in LED
#define INPUT_5VDC_PIN       8                                                 // 5VDC voltage measurement
#define INPUT_12VDC_PIN      7                                                 // 12VDC voltage measurement
#define GPS_DATA_OUT_PIN    33                                                 // Data output from GPS to processor
#define GPS_DATA_IN_PIN     37                                                 // Data input from processor to GPS

#endif

#ifdef ESP32S3_SUPERMINI

#undef BACKLIGHT_PIN
#undef I2C_SCL
#undef I2C_SDA
#undef CONFIG_PIN
#undef BRIGHT_PIN
#undef LIGHT_SENSOR_PIN
#undef LED_PIN
#undef INPUT_5VDC_PIN
#undef INPUT_12VDC_PIN
#undef GPS_DATA_OUT_PIN
#undef GPS_DATA_IN_PIN

#define BACKLIGHT_PIN       13                                                 // Display backlight control
#define I2C_SCL              4                                                 // BMx280 I2C Clock line
#define I2C_SDA              5                                                 // BMx280 I2C Data line
#define CONFIG_PIN           3                                                 // Make CONFIG changes on-the-fly
#define BRIGHT_PIN           8                                                 // Brightness
#define LIGHT_SENSOR_PIN    12                                                 // Light sensor analog pin
#define LED_PIN             48                                                 // Built-in LED
#define INPUT_5VDC_PIN      11                                                 // 5VDC voltage measurement
#define INPUT_12VDC_PIN     -1                                                 // 12VDC voltage measurement
#define GPS_DATA_OUT_PIN     1                                                 // Data output from GPS to processor
#define GPS_DATA_IN_PIN      2                                                 // Data input from processor to GPS

#endif

#ifdef ESP32S3_MJC_TESTBED

#undef BACKLIGHT_PIN
#undef I2C_SCL
#undef I2C_SDA
#undef CONFIG_PIN
#undef BRIGHT_PIN
#undef LIGHT_SENSOR_PIN
#undef LED_PIN
#undef INPUT_5VDC_PIN
#undef INPUT_12VDC_PIN
#undef GPS_DATA_OUT_PIN
#undef GPS_DATA_IN_PIN

#define BACKLIGHT_PIN       38                                                 // Display backlight control
#define I2C_SCL             17                                                 // BMx280 I2C Clock line
#define I2C_SDA             18                                                 // BMx280 I2C Data line
#define CONFIG_PIN           1                                                 // Make CONFIG changes on-the-fly
#define BRIGHT_PIN           2                                                 // Brightness
#define LIGHT_SENSOR_PIN    14                                                 // Light sensor analog pin
#define LED_PIN             15                                                 // Built-in LED
#define INPUT_5VDC_PIN      10                                                 // 5VDC voltage measurement
#define INPUT_12VDC_PIN      8                                                 // 12VDC voltage measurement
#define GPS_DATA_OUT_PIN     4                                                 // Data output from GPS to processor
#define GPS_DATA_IN_PIN     16                                                 // Data input from processor to GPS

#endif

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGNSS;

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
TinyGPSCustom gpsFix(gps, "GPGSA", 2);                                         // $GPGSA sentence, 2nd element = GPS fix state
TinyGPSCustom gpsMode(gps, "GPGSA", 1);                                        // $GPGSA sentence, 1st element = GPS fix mode

EspSoftwareSerial::UART gpsSS;

#define BYTE_BUFFER_SIZE 1024
#define ISR_BIT_BUFFER_SIZE 768

#ifdef GPS_TRY_REVERSED_RXTX_FIRST

boolean gpsSSreversed = true;

#else

boolean gpsSSreversed = false;

#endif

uint8_t gps_tx_pin;
uint8_t gps_rx_pin;

boolean previousGPSActiveMode = false;

boolean gpsActiveMode = false;

#define GPS_NO_FIX -1
#define GPS_2D_FIX 2
#define GPS_3D_FIX 3

int8_t gpsFixState = GPS_NO_FIX;                                               // default: not present
int8_t gpsSatelliteCount = 0;                                                  // default: no satellites

#include <Preferences.h>

Preferences prefs;                                                             // Preferences saved here

boolean debugStackUsage        = false;                                        // enable/disable serial console DEBUG of stack usage
boolean debugEnvSensor         = false;                                        // enable/disable serial console DEBUG of environmental sensor data
boolean debugLightSensor       = false;                                        // enable/disable serial console DEBUG of LDR light sensor data
boolean debugSupplyVoltage     = false;                                        // enable/disable serial console DEBUG of supply voltage data
boolean debugSolar             = false;                                        // enable/disable serial console DEBUG of solar data
boolean debugWeather           = false;                                        // enable/disable serial console DEBUG of weather data
boolean debugWiFiConnect       = false;                                        // enable/disable serial console DEBUG of attempts to make WiFi connections
boolean debugGPSinitialization = false;                                        // enable/disable serial consile DEBUG of GPS initialization
boolean debugGPSposition       = false;                                        // enable/disable serial console DEBUG of GPS position data
boolean debugGPStime           = false;                                        // enable/disable serial console DEBUG of GPS time data
boolean debugGPSaltitude       = false;                                        // enable/disable serial console DEBUG og GPS altitude
boolean debugGPSfix            = false;                                        // enable/disable serial console DEBUG of GPS fix state
boolean debugIsEnabled         = false;                                        // enable/disable all of the other DEBUG capabilities

boolean blinkTimeUpdate = true;                                                // enable/disable blinking the onboard RGB/LED to show time updates
boolean activateLED = false;                                                   // signifies time to blink the LED (indicating a time update) when true
boolean previousLED = false;                                                   // LED history (to keep from unnecessarily writing to the LED)

#define CONNECT_TIME      3000                                                 // Time of inactivity to start connecting WiFi
#define RECONNECT_TIME  120000                                                 // Time of being disconnected, then time to start reconnecting WiFi

#define WIFI_NOT_CONNECTED   -1
#define WIFI_DISABLED         0
#define WIFI_CONNECTED        1
#define WIFI_NET_CONNECTED    2
#define NUM_WIFI_SLOTS        6

//
// Access Point (AP) mode settings
//
char apSSID[32] = { 0x00 };
const char *apPWD = 0;                                                         // No PSK on AP
uint8_t apChannel = DEFAULT_AP_CHANNEL;                                        // WiFi channel number (1..13)
const boolean apHideMe = false;                                                // Hide SSID broadcast when true
const uint8_t apClientsMax = 1;                                                // Maximum simultaneous connected clients

uint16_t ajaxInterval = 2500;

boolean itIsTimeToWiFi = false;                                                // Set true to (re)connect to WiFi
uint32_t connectTime = millis();
uint32_t reconnectTime = millis();

#define GPS_RETRY_INIT_INTERVAL_IN_MILLIS 120000

#define INITIAL_USE_GPS_TO_SET_INTERNAL_CLOCK_INTERVAL_IN_SECONDS 60           // update internal clock from GPS 60 seconds after GPS acquiring 3D lock, just to be sure
#define DEFAULT_USE_GPS_TO_SET_INTERNAL_CLOCK_INTERVAL_IN_SECONDS 600          // update internal clock fron GPS every 10 minutes
uint16_t useGPSToSetInternalClockIntervalInSeconds = DEFAULT_USE_GPS_TO_SET_INTERNAL_CLOCK_INTERVAL_IN_SECONDS;
uint16_t gpsDelayCount = DEFAULT_USE_GPS_TO_SET_INTERNAL_CLOCK_INTERVAL_IN_SECONDS;  // when to update the clock from GPS

// Settings
String loginUsername = String();
String loginPassword = String();

AsyncWebServer server(80);                                                     // AsyncWebServer object on port 80

#define CLOCK_TIME_INTERVAL_IN_MILLIS 1000                                     // update clock each second . . . 
uint32_t clockTime = millis();

#define SOLAR_FAIL_RETRY_INTERVAL_IN_SECONDS 21                                // How long to wait before retrying when a solar fetch fails - odd number so it drifts away from being right on every minute
#define SOLAR_INTERVAL_IN_SECONDS 601                                          // update solar data interval - odd number so it drifts away from being right on every minute
int32_t solarDelayCount = 0;

#define SOLAR_URL "http://www.hamqsl.com/solarxml.php"                         // hamqsl provides the solar data

#include <Wire.h>                                                              // For the SPI Bus
#include <Adafruit_Sensor.h>                                                   // Libraries for the BME280
#include <Adafruit_BME280.h>                                                   // Sensor module

Adafruit_BME280 bme;                                                           // Create sensor object
boolean BME280_available = true;                                               // Assume installed/working

#include <Adafruit_BMP280.h>                                                   // Sensor module

Adafruit_BMP280 bmp;                                                           // Create sensor object
boolean BMP280_available = true;                                               // Assume installed/working

time_t       weatherUnixTimestamp;
char         weatherLocation[24]    = {0x00};
int          weatherTemp            = 0;
int          weatherFeels           = 0;
int          weatherHumidity        = 0;
char         weatherLat[16]         = {0x00};
char         weatherLon[16]         = {0x00};
char         weatherConditions[32]  = {0x00};
char         weatherDetails[32]     = {0x00};
int          weatherWind            = 0;
int          weatherDirection       = 0;
int          weatherGust            = 0;
time_t       weatherUnixSunrise;
time_t       weatherUnixSunset;

String       weatherAppID           = DEFAULT_WEATHER_APP_ID;                  // MJC: b4149a7c3430d9b73efa0671f20b163d
String appID                = "&appid=" + weatherAppID;

int32_t weatherDelayCount = 0;

int8_t  weatherLatLonDecimalPlaces = DEFAULT_LAT_LON_DECIMAL_PLACES;

boolean updateDisplayFlag = false;

boolean needAppIDFlag = false;

//const String latDefault     = "-29.3";
//const String lonDefault     = "152.72";
const String latDefault     = "-42.862252";
const String lonDefault     = "147.319885";

String gpsManualLat = latDefault;
String gpsManualLon = lonDefault;

/*
    Note, it is critical that the 'User_Setups' in the 'TFT_eSPI' library are
    configured properly for the GPIO pin assignments you are using. Those
    assignments are different for the CalQRP ESP8266 PCB and my ESP32 PCB.
    The details for how to accomplish this are included in the 'TFT_eSPI'
    documentation (and/or in the header files themselves).

    These pins are defined in the TFT_eSPI User_Setup header file and are the
    SPI bus pins for the display; don't try to use them for other things. The
    assignments here are for the MWRS PCB with the ESP8366 (WEMOS) D1 Mini module:

        MISO - D6 (GPIO 12)
        MOSI - D7 (GPIO 13)
        SCK  - D5 (GPIO 14)
        CS   - D8 (GPIO 15)
        D/C  - D3 (GPIO  0)
*/

int8_t  tzCountdownIntervalSetting = DEFAULT_TZ_INTERVAL;
uint8_t tzCountdownInterval = tzCountdownIntervalSetting;

String bannerTitle = DEFAULT_TITLE;

#define EXTRA_LONG_BUTTON_PRESS_TIME_IN_MS 5000

Button2 configButton(CONFIG_PIN);                                              // Button used for changing configuration selection on-the-fly - SW1 (right button) on the PCB
Button2 brightButton(BRIGHT_PIN);                                              // Button used for changing the brightness - SW2 (left button) on the PCB

uint8_t brightness = DEFAULT_BRIGHTNESS;
boolean nightMode = DEFAULT_NIGHTMODE;
boolean useMetric = DEFAULT_USEMETRIC;

SHOW_AMPM_TYPE showAMPMMode = DEFAULT_SHOW_AMPM_MODE;

typedef enum
{
   SHOW_MODE_NONE = 0,
   SHOW_MODE_SOLAR_ONLY,
   SHOW_MODE_ENV_ONLY,
   SHOW_MODE_SOLAR_PLUS_ENV,
   SHOW_MODE_WEATHER_ONLY,
   SHOW_MODE_SOLAR_PLUS_WEATHER,
   SHOW_MODE_ENV_PLUS_WEATHER,
   SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER,
   SHOW_MODE_MLS_ONLY,
   SHOW_MODE_SOLAR_PLUS_MLS,
   SHOW_MODE_ENV_PLUS_MLS,
   SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS,
   SHOW_MODE_WEATHER_PLUS_MLS,
   SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS,
   SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS,
   SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS,
   SHOW_MODE_GPS_ONLY,
   SHOW_MODE_SOLAR_PLUS_GPS,
   SHOW_MODE_ENV_PLUS_GPS,
   SHOW_MODE_SOLAR_PLUS_ENV_PLUS_GPS,
   SHOW_MODE_WEATHER_PLUS_GPS,
   SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_GPS,
   SHOW_MODE_ENV_PLUS_WEATHER_PLUS_GPS,
   SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_GPS,
   SHOW_MODE_MLS_PLUS_GPS,
   SHOW_MODE_SOLAR_PLUS_MLS_PLUS_GPS,
   SHOW_MODE_ENV_PLUS_MLS_PLUS_GPS,
   SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS_PLUS_GPS,
   SHOW_MODE_WEATHER_PLUS_MLS_PLUS_GPS,
   SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS_PLUS_GPS,
   SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS,
   SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS,
   SHOW_MODE_END_MARKER
} SHOW_MODE_TYPE;

SHOW_MODE_TYPE showMode = SHOW_MODE_NONE;

#define SHOW_MODE_SOLAR_BIT_MASK   0x01
#define SHOW_MODE_ENV_BIT_MASK     0x02
#define SHOW_MODE_WEATHER_BIT_MASK 0x04
#define SHOW_MODE_MLS_BIT_MASK     0x08
#define SHOW_MODE_GPS_BIT_MASK     0x10

boolean showSolarSelection = false;
boolean showEnvSelection = false;
boolean showWeatherSelection = false;
boolean showMLSSelection = false;
boolean showGPSSelection = false;

int8_t showModeDelayCount = 0;
boolean showModeFlag = true;                                                   // when this flag gets set, the name of the currently selected mode will be temporarily displayed

typedef enum
{
   SHOW_TZ_EST_EDT = 0,
   SHOW_TZ_CST_CDT,
   SHOW_TZ_MST_MDT,
   SHOW_TZ_MST_NO_MDT,
   SHOW_TZ_PST_PDT,
   SHOW_TZ_AWST,
   SHOW_TZ_ACST_ACDT,
   SHOW_TZ_AEST_AEDT,
   SHOW_TZ_GMT_BST,
   SHOW_TZ_END_MARKER,
} SHOW_TZ_TYPE;

SHOW_TZ_TYPE tzIndex = SHOW_TZ_AEST_AEDT;                                      // Index to which local timezone to display

boolean showESTEDT   = DEFAULT_SHOW_EST_EDT_TZ;
boolean showCSTCDT   = DEFAULT_SHOW_CST_CDT_TZ;
boolean showMSTMDT   = DEFAULT_SHOW_MST_MDT_TZ;
boolean showMSTNOMDT = DEFAULT_SHOW_MST_NO_MDT_TZ;
boolean showPSTPDT   = DEFAULT_SHOW_PST_PDT_TZ;
boolean showAWST     = DEFAULT_SHOW_AWST_TZ;
boolean showACSTACDT = DEFAULT_SHOW_ACST_ACDT_TZ;
boolean showAESTAEDT = DEFAULT_SHOW_AEST_AEDT_TZ;
boolean showGMTBST   = DEFAULT_SHOW_GMT_BST_TZ;

boolean hidePasswords = true;
boolean colorDefaults = false;
boolean colorUpdateNeeded = false;

typedef enum
{
   SHOW_DATA_SFI = 0,
   SHOW_DATA_GMF,
   SHOW_DATA_S2N,
   SHOW_DATA_AUR,
   SHOW_DATA_SSN,
   SHOW_DATA_ENV,
   SHOW_DATA_LOC,
   SHOW_DATA_COND,
   SHOW_DATA_TEMP,
   SHOW_DATA_RH,
   SHOW_DATA_WIND,
   SHOW_DATA_SUNRISE,
   SHOW_DATA_SUNSET,
   SHOW_DATA_MLS,
   SHOW_DATA_LAT,
   SHOW_DATA_LON,
   SHOW_DATA_ALTITUDE,
   SHOW_DATA_END_MARKER
} SHOW_DATA_TYPE;

#define SHOW_FIRST_SOLAR_DATA       SHOW_DATA_SFI
#define SHOW_LAST_SOLAR_DATA        SHOW_DATA_SSN

#define SHOW_FIRST_WEATHER_DATA     SHOW_DATA_LOC
#define SHOW_LAST_WEATHER_DATA      SHOW_DATA_SUNSET

#define SHOW_FIRST_GPS_DATA         SHOW_DATA_LAT
#define SHOW_LAST_GPS_DATA          SHOW_DATA_ALTITUDE

SHOW_DATA_TYPE dataIndex = SHOW_FIRST_SOLAR_DATA;

boolean showDataHeaderBetweenDataGroups = false;                               // whether to show the selected data types header between data groups

#define CYCLE_TIME_IN_SECONDS 2                                                // Number of seconds to show each data item

#define WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS 21                              // How long to wait before retrying when a weather fetch fails - odd number so it drifts away from being right on the minute
uint16_t weatherFetchIntervalInSeconds = DEFAULT_WEATHER_FETCH_INTERVAL_IN_SECONDS; // Number of seconds between weather pulls

boolean localFormat24hr = DEFAULT_LOCAL_FORMAT_24HR;
boolean utcFormat24hr   = DEFAULT_UTC_FORMAT_24HR;
boolean hourLeadingZero = DEFAULT_HOUR_LEADING_ZERO;
boolean dateLeadingZero = DEFAULT_DATE_LEADING_ZERO;
boolean dateAboveMonth  = DEFAULT_DATE_ABOVE_MONTH;

#define NUMBER_OF_VOLTAGE_AVERAGES 16

boolean fiveVoltDisplay   = DEFAULT_5VDC_DISPLAY;
String fiveVoltCorrection = "1.0";

boolean twelveVoltDisplay = DEFAULT_12VDC_DISPLAY;
String twelveVoltCorrection = "1.0";

uint8_t printedTime = DEFAULT_PRINTED_TIME;

#define NUMBER_OF_INCHES_PER_METER 39.37

String qthAltitudeInFeet = DEFAULT_QTH_ALTITUDE_IN_FEET;
String temperatureOffsetInCelsius = DEFAULT_TEMPERATURE_OFFSET_IN_CELSIUS;
uint32_t humidityOffset = DEFAULT_HUMIDITY_OFFSET;

#define EZTIME_DEBUGLEVEL INFO                                                 // NONE, ERROR, INFO, or DEBUG
#define EZTIME_TIME_FORMAT COOKIE                                              // COOKIE, ISO8601, RFC822, RFC850, RFC3339, RSS

uint8_t screenBGColorIdx = DEFAULT_SCREEN_BG_COLOR_IDX;
uint8_t timeColorIdx     = DEFAULT_TIME_COLOR_IDX;
uint8_t dateColorIdx     = DEFAULT_DATE_COLOR_IDX;
uint8_t labelFGColorIdx  = DEFAULT_LABEL_FGCOLOR_IDX;
uint8_t labelBGColorIdx  = DEFAULT_LABEL_BGCOLOR_IDX;
uint8_t edgeColorIdx     = DEFAULT_EDGE_COLOR_IDX;
uint8_t normalColorIdx   = DEFAULT_NORMAL_COLOR_IDX;
uint8_t mediumColorIdx   = DEFAULT_MEDIUM_COLOR_IDX;
uint8_t highColorIdx     = DEFAULT_HIGH_COLOR_IDX;
uint8_t lostColorIdx     = DEFAULT_LOST_COLOR_IDX;
uint8_t warningColorIdx  = DEFAULT_WARNING_COLOR_IDX;

typedef struct __attribute__ ((packed))
{
   const char *name;
   uint16_t color;
} ColorChoice;

ColorChoice Colors[] =
{
   { "Black", TFT_BLACK },
   { "Red", TFT_RED },
   { "Orange", TFT_ORANGE },
   { "Yellow", TFT_YELLOW },
   { "Green", TFT_GREEN },
   { "Blue", TFT_BLUE },
   { "Cyan", TFT_CYAN },
   { "Purple", TFT_PURPLE },
   { "Gold", TFT_GOLD },
   { "Navy", TFT_NAVY },
   { "White", TFT_WHITE },
   { "Olive", TFT_OLIVE },
   { "Grey", TFT_DARKGREY },
   { "Maroon", TFT_MAROON },
   { "Magenta", TFT_MAGENTA },
   { "Pink", TFT_PINK },
   { "Brown", TFT_BROWN },
   { "Silver", TFT_SILVER },
   { "SkyBlue", TFT_SKYBLUE },
};

/*
    Initial definition of TFT Colours - if you wish to change
    colours do so in 'UserSettings.h'.
*/
uint16_t screenBGColor_day = Colors[DEFAULT_SCREEN_BG_COLOR_IDX].color;
uint16_t timeColor_day     = Colors[DEFAULT_TIME_COLOR_IDX].color;
uint16_t dateColor_day     = Colors[DEFAULT_DATE_COLOR_IDX].color;
uint16_t labelFGColor_day  = Colors[DEFAULT_LABEL_FGCOLOR_IDX].color;
uint16_t labelBGColor_day  = Colors[DEFAULT_LABEL_BGCOLOR_IDX].color;
uint16_t edgeColor_day     = Colors[DEFAULT_EDGE_COLOR_IDX].color;
uint16_t normalColor_day   = Colors[DEFAULT_NORMAL_COLOR_IDX].color;
uint16_t mediumColor_day   = Colors[DEFAULT_MEDIUM_COLOR_IDX].color;
uint16_t highColor_day     = Colors[DEFAULT_HIGH_COLOR_IDX].color;
uint16_t lostColor_day     = Colors[DEFAULT_LOST_COLOR_IDX].color;
uint16_t warningColor_day  = Colors[DEFAULT_WARNING_COLOR_IDX].color;

uint16_t screenBGColor = screenBGColor_day;                                    // Color of the screen background (e.g. area behind the time display)
uint16_t timeColor     = timeColor_day;                                        // Color of 7-segment time display
uint16_t dateColor     = dateColor_day;                                        // Color of displayed month & day
uint16_t labelFGColor  = labelFGColor_day;                                     // Color of label text
uint16_t labelBGColor  = labelBGColor_day;                                     // Color of label background
uint16_t edgeColor     = edgeColor_day;                                        // Color of the Edge around Local and UTC Time
uint16_t normalColor   = normalColor_day;                                      // Color of normal readings
uint16_t mediumColor   = mediumColor_day;                                      // Color of medium level readings
uint16_t highColor     = highColor_day;                                        // Color of Maximum or High readings
uint16_t lostColor     = lostColor_day;                                        // Color of Wifi lost
uint16_t warningColor  = warningColor_day;                                     // Color of warnings

boolean wifiRetryAuto = DEFAULT_WIFI_RETRY_MODE;

TFT_eSPI tft = TFT_eSPI();                                                     // Create the display object

Timezone local;                                                                // Local timezone variable

String xmlData = String();                                                     // Holds the XML data from 'hamqsl.com'




// forward declarations
void buttonLoopTask(void *parameter);
void clearDataArea(void);
void forceColorDefaults(void);
void forceDefaults(boolean requireConfirm);
boolean gpsFactoryReset(void);
void getGPSAltitudeUpdate(void);
void getGPSFixState(void);
void getGPSPositionUpdate(void);
void getGPSTimeUpdate(boolean secondSync = false);
void getSolarData(void);
void getWeatherData(void);
uint8_t getWiFiStatus(void);
String getXmlData(String xml, String tag);
void loop(void);
void netTickTime(void);
void netInit(void);
void newDualScreen(void);
void printTime(void);
void printValues(void);
void readSettings(void);
boolean selectForceDefaults(void);
int8_t selectGPSResetType(void);
boolean selectReboot(void);
void setup(void);
void show12VDC(uint16_t x, uint16_t y);
void show5VDC(uint16_t x, uint16_t y);
void showAltitude(void);
void showAppIDNagScreen(void);
void showAUR(void);
void showButtonDefinitions(void);
void showCond(void);
void showDataItems(void);
void showEnv(void);
void showGMF(void);
void showGPSStatus(uint16_t x, uint16_t y);
void showMLS(void);
void showLat(void);
void showLoc(void);
void showLon(void);
void showNextData(void);
void showRH(void);
void showS2N(void);
void showSFI(void);
void showSplash(void);
void showSSN(void);
void showSunrise(void);
void showSunset(void);
void showTemp(void);
void showTime(time_t t, boolean hr24, uint16_t x, uint16_t y);
void showTimeDate(boolean useLocalTime, time_t t, boolean hr24, uint16_t x, uint16_t y);
void showTimeZone(boolean useLocalTime, uint16_t x, uint16_t y);
void showWiFiInfo(void);
void showWiFiStatus(void);
void showWind(void);
void startButtonLoopTask(void);
void startGPS(boolean tryBothPinConfigs = true);
void startupScreen(void);
void stopAndWaitForever(void);
void switchDisplayMode(void);
void updateBrightness(void);
void updateDisplay(void);
const String webAMPMModeSelector(void);
const String webCheckbox(boolean isSelected, const String & idname, const String & text);
const String webColorSelector(uint16_t index);
const String webColorsPage(void);
const String webConfigPage(void);
const String webDebugPage(void);
const String webGPSModeSelector(void);
void webInit(void);
const String webInputField(const String & name, const String & value, boolean pass = false);
const String webNetworkPage(void);
const String webPage(const String & body);
const String webPrintedTimeSelector(void);
const String webScanPage(void);
void webSetColors(AsyncWebServerRequest * request);
void webSetConfig(AsyncWebServerRequest *request);
void webSetDebug(AsyncWebServerRequest * request);
void webSetNetwork(AsyncWebServerRequest * request);
const String webShowNetworkScan(void);
const String webStatusPage(void);
const String webStyleSheet(void);
void wifiInitAP(void);
boolean wifiConnect(void);



void buttonLoopTask(void *parameter)
{
   while (true)
   {
      configButton.loop();
      brightButton.loop();

      if (updateDisplayFlag)
      {
         updateDisplay();
      }

      if (activateLED)
      {
         activateLED = false;
         previousLED = true;

#ifdef RGB_BUILTIN

         rgbLedWrite(RGB_BUILTIN, 8, 0, 0);                                    // blink RED

#else

#ifdef LED_BUILTIN

         digitalWrite(LED_BUILTIN, HIGH);                                      // blink LED

#endif

#endif

      } else {
         if (previousLED)
         {
            previousLED = false;

#ifdef RGB_BUILTIN

            rgbLedWrite(RGB_BUILTIN, 0, 0, 0);                                 // blink OFF

#else

#ifdef LED_BUILTIN

            digitalWrite(LED_BUILTIN, LOW);                                    // blink OFF

#endif

#endif

         }
      }

      updateBrightness();

      vTaskDelay(40 / portTICK_PERIOD_MS);
   }
}  // buttonLoopTask


void clearDataArea(void)
{
   tft.fillRoundRect(80, 116, 240, 33, 10, labelBGColor);                      // Original Code
   tft.drawRoundRect(1, 116, 318, 34, 10, edgeColor);                          // Title bar for UTC
   tft.drawRoundRect(1, 116, 318, 110, 10, edgeColor);                         // Draw edge around UTC
}  // clearDataArea()


void forceColorDefaults(void)
{
   screenBGColorIdx = DEFAULT_SCREEN_BG_COLOR_IDX;
   timeColorIdx = DEFAULT_TIME_COLOR_IDX;
   dateColorIdx = DEFAULT_DATE_COLOR_IDX;
   labelFGColorIdx = DEFAULT_LABEL_FGCOLOR_IDX;
   labelBGColorIdx = DEFAULT_LABEL_BGCOLOR_IDX;
   edgeColorIdx = DEFAULT_EDGE_COLOR_IDX;
   normalColorIdx = DEFAULT_NORMAL_COLOR_IDX;
   mediumColorIdx = DEFAULT_MEDIUM_COLOR_IDX;
   highColorIdx = DEFAULT_HIGH_COLOR_IDX;
   lostColorIdx = DEFAULT_LOST_COLOR_IDX;
   warningColorIdx = DEFAULT_WARNING_COLOR_IDX;
}  // forceColorDefaults()


void forceDefaults(boolean requireConfirm)
{
   String loginUsername = String();
   String loginPassword = String();
   String apName = String(DEFAULT_AP_NAME);
   int apChannel = DEFAULT_AP_CHANNEL;

   if ((!requireConfirm) || (selectForceDefaults()))
   {
      // Start modifying network preferences
      prefs.begin("network", false);

      // wipe everything
      prefs.clear();

      // Done with clearing the network preferences
      prefs.end();


      // Start modifying settings preferences
      prefs.begin("settings", false);

      // wipe everything
      prefs.clear();

      // Done with clearing the settings preferences
      prefs.end();


      // Start modifying colors preferences
      prefs.begin("colors", false);

      // wipe everything
      prefs.clear();

      // Done with clearing the colors preferences
      prefs.end();


      // Start modifying config preferences
      prefs.begin("config", false);

      // wipe everything
      prefs.clear();

      // Done with the config preferences
      prefs.end();


      // Start modifying network preferences
      prefs.begin("network", false);

      prefs.putString("wifissid1", "");
      prefs.putString("wifipass1", "");

      prefs.putString("wifissid2", "");
      prefs.putString("wifipass2", "");

      prefs.putString("wifissid3", "");
      prefs.putString("wifipass3", "");

      prefs.putString("wifissid4", "");
      prefs.putString("wifipass4", "");

      prefs.putString("wifissid5", "");
      prefs.putString("wifipass5", "");

      prefs.putString("wifissid6", "");
      prefs.putString("wifipass6", "");

      prefs.putString("loginusername", "");
      prefs.putString("loginpassword", "");

      prefs.putString("apName", apName);
      prefs.putInt("apChannel", apChannel);

      wifiRetryAuto = DEFAULT_WIFI_RETRY_MODE;
      prefs.putBool("wifiRetryAuto", wifiRetryAuto);

      // Done with the network preferences
      prefs.end();

      // Start modifying settings preferences
      prefs.begin("settings", false);

      brightness = DEFAULT_BRIGHTNESS;
      prefs.putInt("brightness", brightness);

      nightMode = DEFAULT_NIGHTMODE;
      prefs.putBool("nightMode", nightMode);

      useMetric = DEFAULT_USEMETRIC;
      prefs.putBool("useMetric", useMetric);

      showDataHeaderBetweenDataGroups = false;
      prefs.putBool("showDataHeader", false);

      // Done with the settings preferences
      prefs.end();

      forceColorDefaults();

      colorUpdateNeeded = true;

      // Start modifying config preferences
      prefs.begin("config", false);

      showMode = SHOW_MODE_NONE;
      showSolarSelection = false;
      showEnvSelection = false;
      showWeatherSelection = false;
      showMLSSelection = false;
      showGPSSelection = false;
      prefs.putInt("showMode", int(showMode));

      gpsManualLat = latDefault;
      prefs.putString("gpsManualLat", latDefault);

      gpsManualLon = lonDefault;
      prefs.putString("gpsManualLon", lonDefault);

      weatherLatLonDecimalPlaces = DEFAULT_LAT_LON_DECIMAL_PLACES;
      prefs.putInt("decimals", weatherLatLonDecimalPlaces);

      tzCountdownIntervalSetting = DEFAULT_TZ_INTERVAL;
      tzCountdownInterval = 1;                                                 // force a change in the tz interval in the next second & apply the new interval
      prefs.putInt("tzInterval", tzCountdownIntervalSetting);

      showESTEDT = DEFAULT_SHOW_EST_EDT_TZ;
      prefs.putBool("showest", showESTEDT);

      showCSTCDT = DEFAULT_SHOW_CST_CDT_TZ;
      prefs.putBool("showcst", showCSTCDT);

      showMSTMDT = DEFAULT_SHOW_MST_MDT_TZ;
      prefs.putBool("showmst", showMSTMDT);

      showMSTNOMDT = DEFAULT_SHOW_MST_NO_MDT_TZ;
      prefs.putBool("showmstonly", showMSTNOMDT);

      showPSTPDT = DEFAULT_SHOW_PST_PDT_TZ;
      prefs.putBool("showpst", showPSTPDT);

      showAWST = DEFAULT_SHOW_AWST_TZ;
      prefs.putBool("showawst", showAWST);

      showACSTACDT = DEFAULT_SHOW_ACST_ACDT_TZ;
      prefs.putBool("showacst", showACSTACDT);

      showAESTAEDT = DEFAULT_SHOW_AEST_AEDT_TZ;
      prefs.putBool("showaest", showAESTAEDT);

      showGMTBST = DEFAULT_SHOW_GMT_BST_TZ;
      prefs.putBool("showgmt", showGMTBST);

      showAMPMMode = DEFAULT_SHOW_AMPM_MODE;
      prefs.putInt("showAMPM", (int)showAMPMMode);

      localFormat24hr = DEFAULT_LOCAL_FORMAT_24HR;
      prefs.putBool("local24hr", localFormat24hr);

      utcFormat24hr = DEFAULT_UTC_FORMAT_24HR;
      prefs.putBool("utc24hr", utcFormat24hr);

      hourLeadingZero = DEFAULT_HOUR_LEADING_ZERO;
      prefs.putBool("hrlead0", hourLeadingZero);

      dateLeadingZero = DEFAULT_DATE_LEADING_ZERO;
      prefs.putBool("dtlead0", dateLeadingZero);

      dateAboveMonth = DEFAULT_DATE_ABOVE_MONTH;
      prefs.putBool("dtovermo", dateAboveMonth);

      hidePasswords = true;

      fiveVoltDisplay = DEFAULT_5VDC_DISPLAY;
      prefs.putBool("show5vdc", fiveVoltDisplay);

      twelveVoltDisplay = DEFAULT_12VDC_DISPLAY;
      prefs.putBool("show12vdc", twelveVoltDisplay);

      useGPSToSetInternalClockIntervalInSeconds = DEFAULT_USE_GPS_TO_SET_INTERNAL_CLOCK_INTERVAL_IN_SECONDS;
      gpsDelayCount = 1;                                                       // force a change in the GPS countdown in the next second & apply the new interval
      prefs.putInt("gpsInterval", useGPSToSetInternalClockIntervalInSeconds);

      weatherFetchIntervalInSeconds = DEFAULT_WEATHER_FETCH_INTERVAL_IN_SECONDS;
      weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;              // force a change in the weather fetch countdown & apply the new interval
      prefs.putInt("wfInterval", weatherFetchIntervalInSeconds);

      printedTime = DEFAULT_PRINTED_TIME;
      prefs.putInt("printtime", (int)printedTime);

      // Done with the settings preferences
      prefs.end();
   }

   for (int i = 0; i < 200; i++)
   {
      delay(10);
   }

   switchDisplayMode();
   newDualScreen();
}  // forceDefaults()


boolean gpsFactoryReset(void)
{
   startupScreen();

   gpsFixState = GPS_NO_FIX;
   gpsSatelliteCount = 0;

   tft.setTextDatum(TC_DATUM);

   tft.drawString("GPS Factory Reset", 160, 50, 2);

   tft.setFreeFont(&FreeSansBold9pt7b);

   if (debugGPSinitialization)
   {
      Serial.print("GNSS: attempting factory reset at 38400 baud...");
   }

   tft.fillRect(5, 80, 310, 20, screenBGColor);

   tft.drawString("GNSS: attempting factory reset at 38400 baud", 160, 80, 2);

   if (gpsSSreversed)
   {
      gps_tx_pin = GPS_DATA_IN_PIN;
      gps_rx_pin = GPS_DATA_OUT_PIN;
   } else {
      gps_tx_pin = GPS_DATA_OUT_PIN;
      gps_rx_pin = GPS_DATA_IN_PIN;
   }

   gpsSS.end();
   gpsSS.begin(38400, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

   if (myGNSS.begin(gpsSS))
   {
      myGNSS.factoryReset();

      if (debugGPSinitialization)
      {
         Serial.println("SUCCESS !!");
      }

      tft.fillRect(5, 108, 310, 20, screenBGColor);

      tft.drawString("SUCCESS !!", 160, 110, 2);
      delay(2000);

      return (true);
   } else {
      if (debugGPSinitialization)
      {
         Serial.println("NOT FOUND !!");
      }

      tft.fillRect(5, 108, 310, 20, screenBGColor);

      tft.drawString("NOT FOUND !!", 160, 110, 2);
      delay(250);

      if (debugGPSinitialization)
      {
         Serial.print("GNSS: attempting factory reset at 19200 baud...");
      }

      tft.fillRect(5, 80, 310, 20, screenBGColor);
      tft.fillRect(5, 108, 310, 20, screenBGColor);

      tft.drawString("GNSS: attempting factory reset at 19200 baud", 160, 80, 2);

      gpsSS.end();
      gpsSS.begin(19200, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

      if (myGNSS.begin(gpsSS))
      {
         myGNSS.factoryReset();

         if (debugGPSinitialization)
         {
            Serial.println("SUCCESS !!");
         }

         tft.fillRect(5, 108, 310, 20, screenBGColor);

         tft.drawString("SUCCESS !!", 160, 110, 2);
         delay(2000);

         return (true);
      } else {
         if (debugGPSinitialization)
         {
            Serial.println("NOT FOUND !!");
         }

         tft.fillRect(5, 108, 310, 20, screenBGColor);

         tft.drawString("NOT FOUND !!", 160, 110, 2);
         delay(250);

         if (debugGPSinitialization)
         {
            Serial.print("GNSS: attempting factory reset at  9600 baud...");
         }

         tft.fillRect(5, 80, 310, 20, screenBGColor);
         tft.fillRect(5, 108, 310, 20, screenBGColor);

         tft.drawString("GNSS: attempting factory reset at  9600 baud", 160, 80, 2);

         gpsSS.end();
         gpsSS.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

         if (myGNSS.begin(gpsSS))
         {
            myGNSS.factoryReset();

            if (debugGPSinitialization)
            {
               Serial.println("SUCCESS !!");
            }

            tft.fillRect(5, 108, 310, 20, screenBGColor);

            tft.drawString("SUCCESS !!", 160, 110, 2);
            delay(2000);

            return (true);
         } else {
            if (debugGPSinitialization)
            {
               Serial.println("NOT FOUND !!");
            }

            tft.fillRect(5, 108, 310, 20, screenBGColor);

            tft.drawString("NOT FOUND !!", 160, 110, 2);
            delay(250);

            if (debugGPSinitialization)
            {
               Serial.print("GNSS: attempting factory reset at  4800 baud...");
            }

            tft.fillRect(5, 80, 310, 20, screenBGColor);
            tft.fillRect(5, 108, 310, 20, screenBGColor);

            tft.drawString("GNSS: attempting factory reset at  4800 baud", 160, 80, 2);

            gpsSS.end();
            gpsSS.begin(4800, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

            if (myGNSS.begin(gpsSS))
            {
               myGNSS.factoryReset();

               if (debugGPSinitialization)
               {
                  Serial.println("SUCCESS !!");
               }

               tft.fillRect(5, 108, 310, 20, screenBGColor);

               tft.drawString("SUCCESS !!", 160, 110, 2);
               delay(2000);

               return (true);
            } else {
               if (debugGPSinitialization)
               {
                  Serial.println("NOT FOUND !!");
               }

               tft.fillRect(5, 108, 310, 20, screenBGColor);

               tft.drawString("NOT FOUND !!", 160, 110, 2);
               delay(250);

               if (debugGPSinitialization)
               {
                  Serial.print("GNSS: attempting factory reset at  2400 baud...");
               }

               tft.fillRect(5, 80, 310, 20, screenBGColor);
               tft.fillRect(5, 108, 310, 20, screenBGColor);

               tft.drawString("GNSS: attempting factory reset at  2400 baud", 160, 80, 2);

               gpsSS.end();
               gpsSS.begin(2400, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

               if (myGNSS.begin(gpsSS))
               {
                  myGNSS.factoryReset();

                  if (debugGPSinitialization)
                  {
                     Serial.println("SUCCESS !!");
                  }

                  tft.fillRect(5, 108, 310, 20, screenBGColor);

                  tft.drawString("SUCCESS !!", 160, 110, 2);
                  delay(2000);

                  return (true);
               } else {
                  if (debugGPSinitialization)
                  {
                     Serial.println("NOT FOUND !!");
                  }

                  tft.fillRect(5, 108, 310, 20, screenBGColor);

                  tft.drawString("NOT FOUND !!", 160, 110, 2);
                  delay(250);

                  if (debugGPSinitialization)
                  {
                     Serial.print("GNSS: attempting factory reset at  1200 baud...");
                  }

                  tft.fillRect(5, 80, 310, 20, screenBGColor);
                  tft.fillRect(5, 108, 310, 20, screenBGColor);

                  tft.drawString("GNSS: attempting factory reset at  1200 baud", 160, 80, 2);

                  gpsSS.end();
                  gpsSS.begin(1200, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

                  if (myGNSS.begin(gpsSS))
                  {
                     myGNSS.factoryReset();

                     if (debugGPSinitialization)
                     {
                        Serial.println("SUCCESS !!");
                     }

                     tft.fillRect(5, 108, 310, 20, screenBGColor);

                     tft.drawString("SUCCESS !!", 160, 110, 2);
                     delay(2000);

                     return (true);
                  } else {
                     if (debugGPSinitialization)
                     {
                        Serial.println("NOT FOUND !!");
                     }

                     tft.fillRect(5, 108, 310, 20, screenBGColor);

                     tft.drawString("NOT FOUND !!", 160, 110, 2);
                     delay(250);
                  }
               }
            }
         }
      }
   }

   return (false);
}  // gpsFactoryReset()


void getGPSAltitudeUpdate(void)
{
   if (gpsActiveMode)
   {
      // process all data from the GPS before attempting to check fix state
      while (gpsSS.available() > 0)
      {
         gps.encode(gpsSS.read());
      }

      if (gps.altitude.isValid())
      {
         char altitudeInFeetStr[16] = { 0x00 };

         // altitude is returned as meters * 100
         float altitudeInFeetFloat = gps.altitude.value() * NUMBER_OF_INCHES_PER_METER / 1200.0;

         sprintf(altitudeInFeetStr, "%.2f", (float)((round(altitudeInFeetFloat * 100.0)) / 100.0));
         qthAltitudeInFeet = (String)altitudeInFeetStr;

         if (debugGPSaltitude)
         {
            Serial.print("GPS: ");
            Serial.print(qthAltitudeInFeet);
            Serial.println(" (altitude in feet)");
         }
      }
   } else {
      if (debugGPSaltitude)
      {
         Serial.println("getGPSAltitudeUpdate() called while GPS is inactive");
      }
   }
}  // getGPSAltitudeUpdate()


void getGPSFixState(void)
{
   if (gpsActiveMode)
   {
      // process all data from the GPS before attempting to check fix state
      while (gpsSS.available() > 0)
      {
         gps.encode(gpsSS.read());
      }

      if (gpsFix.isValid())
      {
         if (debugGPSfix)
         {
            Serial.println("gpsFix.valid() is TRUE");
            Serial.print("gpsFix.value() = ");
            Serial.println(*(gpsFix.value()));
         }

         switch (*(gpsFix.value()))
         {
            case '2':
               {
                  gpsFixState = GPS_2D_FIX;
               }
               break;

            case '3':
               {
                  gpsFixState = GPS_3D_FIX;
               }
               break;

            default:
               {
                  gpsFixState = GPS_NO_FIX;
               }
               break;
         }
      } else {
         if (debugGPSfix)
         {
            Serial.println("gpsFix.valid() is FALSE");
         }
      }

      if (debugGPSfix)
      {
         if (gpsMode.isValid())
         {
            Serial.println("gpsMode.valid() is TRUE");
            Serial.print("gpsMode.value() = ");
            Serial.println(*(gpsMode.value()));
         } else {
            Serial.println("gpsMode.valid() is FALSE");
         }
      }

      if (gps.satellites.isValid())
      {
         gpsSatelliteCount = gps.satellites.value();

         if (debugGPSfix)
         {
            Serial.println("gps.satellites.isValid() is TRUE");
            Serial.print("gps.satellites.value() = ");
            Serial.println(gpsSatelliteCount);
         }

         if (gpsFixState == GPS_NO_FIX)
         {
            if (gpsSatelliteCount == 3)
            {
               gpsFixState = GPS_2D_FIX;
            } else {
               if (gpsSatelliteCount >= 4)
               {
                  gpsFixState = GPS_3D_FIX;
               } else {
                  gpsFixState = GPS_NO_FIX;
               }
            }
         }
      } else {
         if (debugGPSfix)
         {
            Serial.println("gps.satellites.isValid() is FALSE");
         }
      }

      if (debugGPSfix)
      {
         Serial.print("GPS fix state: ");
         Serial.println(gpsFixState);
      }
   } else {
      if (debugGPSfix)
      {
         Serial.println("getGPSFixState() called while GPS is inactive");
      }
   }
}  // getGPSFixState()


void getGPSPositionUpdate(void)
{
   char str[16];

   if (gps.location.isValid())
   {
      sprintf(str, "%.6f", gps.location.lat());
      gpsManualLat = (String)(str);
      sprintf(str, "%.6f", gps.location.lng());
      gpsManualLon = (String)(str);

      if (debugGPSposition)
      {
         Serial.print("  Location: ");
         if (gps.location.isValid())
         {
            Serial.print(gpsManualLat);
            Serial.print(",");
            Serial.print(gpsManualLon);
         } else {
            Serial.print("INVALID");
         }

         Serial.println("");
      }
   }
}  // getGPSPositionUpdate()


void getGPSTimeUpdate(boolean secondSync)
{
   uint16_t d_year;
   uint8_t t_hour, t_minute, t_second, t_centisecond, d_day, d_month, previous_second;
   boolean notDone = true;

   if (gpsActiveMode)
   {
      if ((gps.date.isValid()) && (gps.time.isValid()))
      {
         // process all data from the GPS before attempting to read time
         while (gpsSS.available() > 0)
         {
            gps.encode(gpsSS.read());
         }

         t_centisecond = gps.time.centisecond();
         t_second = gps.time.second();
         t_minute = gps.time.minute();
         t_hour = gps.time.hour();
         d_day = gps.date.day();
         d_month = gps.date.month();
         d_year = gps.date.year();

         previous_second = t_second;

         if ((--gpsDelayCount == 0) || secondSync)
         {
            gpsDelayCount = useGPSToSetInternalClockIntervalInSeconds;

            if (secondSync)
            {
               if (debugGPStime)
               {
                  Serial.println("initiating sync to the nearest second process");
               }

               // sync to the nearest second
               while (notDone)
               {
                  // process all data from the GPS before/while attempting to sync time
                  while (gpsSS.available() > 0)
                  {
                     gps.encode(gpsSS.read());
                  }

                  t_centisecond = gps.time.centisecond();
                  t_second = gps.time.second();
                  t_minute = gps.time.minute();
                  t_hour = gps.time.hour();
                  d_day = gps.date.day();
                  d_month = gps.date.month();
                  d_year = gps.date.year();

                  if (previous_second != t_second)
                  {
                     notDone = false;

                     setTime(t_hour, t_minute, t_second, d_day, d_month, d_year);
                     clockTime = millis();
                  }
               }

               if (debugGPStime)
               {
                  Serial.println("completed sync to the nearest second process");
               }
            }
         }

         if (debugGPStime)
         {
            Serial.print("GPS: ");
            Serial.print(d_month);
            Serial.print("/");
            Serial.print(d_day);
            Serial.print("/");
            Serial.print(d_year);

            Serial.print(" ");
            if (t_hour < 10) Serial.print("0");
            Serial.print(t_hour);
            Serial.print(":");
            if (t_minute < 10) Serial.print("0");
            Serial.print(t_minute);
            Serial.print(":");
            if (t_second < 10) Serial.print("0");
            Serial.print(t_second);
            Serial.print(".");
            if (t_centisecond < 10) Serial.print("0");
            Serial.println(t_centisecond);
         }
      }
   } else {
      if (debugGPStime)
      {
         Serial.println("getGPSTimeUpdate() called while GPS is inactive");
      }
   }
}  // getGPSTimeUpdate()


void getSolarData(void)
{
   if ((getWiFiStatus() == WIFI_NET_CONNECTED) && (showMode & SHOW_MODE_SOLAR_BIT_MASK)) // only fetch solar data if WiFi is connected AND solar is currently selected for display
   {
      // if it's time to retrieve solar data
      if (--solarDelayCount <= 0)
      {
         updateDisplayFlag = true;

         solarDelayCount = SOLAR_INTERVAL_IN_SECONDS;

         if (debugSolar)
         {
            Serial.println("Calling Solar HTTPS begin()");
         }

         HTTPClient clientHttps;
         WiFiClient sClient;

         int responseCode = clientHttps.begin(sClient, SOLAR_URL);             // Open the URL connection

         if (responseCode)
         {
            if (debugSolar)
            {
               Serial.print("Sending Solar HTTPS request to: ");
               Serial.println(SOLAR_URL);
            }

            int httpCode = clientHttps.GET();                                  // Get the response code

            if (httpCode > 0)
            {
               if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
               {
                  xmlData = clientHttps.getString();                           // Get the XML data

                  if (debugSolar)
                  {
                     Serial.println(xmlData);                                  // For debugging
                  }
               }
            } else {
               if (debugSolar)
               {
                  Serial.print("Solar HTTPS GET... failed, error: ");
                  Serial.println(clientHttps.errorToString(httpCode).c_str());
               }

               // try again in how long
               solarDelayCount = SOLAR_FAIL_RETRY_INTERVAL_IN_SECONDS;

               xmlData = String();                                             // signify to try solar data again next pass
            }
         } else {
            if (debugSolar)
            {
               Serial.print("Solar HTTPS begin() failed, error: ");
               Serial.println(clientHttps.errorToString(responseCode).c_str());
            }

            // try again in how long
            solarDelayCount = SOLAR_FAIL_RETRY_INTERVAL_IN_SECONDS;

            xmlData = String();                                                // signify to try solar data again next pass
         }

         clientHttps.end();

         updateDisplayFlag = false;
      }
   } else {
      if (--solarDelayCount <= 0)
      {
         if (debugSolar)
         {
            if (getWiFiStatus() != WIFI_NET_CONNECTED)
            {
               Serial.println("Solar data unavailable - WiFi is not connected");
            } else {
               Serial.println("Solar data unavailable - Solar is not currently selected for display");
            }
         }
      }

      // try again in how long
      solarDelayCount = SOLAR_FAIL_RETRY_INTERVAL_IN_SECONDS;
   }
}  // getSolarData()


void getWeatherData(void)
{
   if ((getWiFiStatus() == WIFI_NET_CONNECTED) && (showMode & SHOW_MODE_WEATHER_BIT_MASK)) // only fetch weather data if WiFi is connected AND weather is currently selected for display
   {
      if (--weatherDelayCount <= 0)
      {
         const String baseURL        = "http://api.openweathermap.org/data/2.5/weather";
         const String latTag         = "?lat=";
         const String lonTag         = "&lon=";
         const String unitsImperial  = "&units=imperial";
         const String unitsMetric    = "&units=metric";
         const String modeTag        = "&mode=json";

         updateDisplayFlag = true;

         weatherDelayCount = weatherFetchIntervalInSeconds + 1;                // cause it to drift away from being right on every minute

         String WEATHER_URL = baseURL + latTag + gpsManualLat + lonTag + gpsManualLon;

         if (useMetric)
         {
            WEATHER_URL += unitsMetric + modeTag + appID;
         } else {
            WEATHER_URL += unitsImperial + modeTag + appID;
         }

         char text[200];

         if (debugWeather)
         {
            Serial.print("Sending Weather HTTP request to: ");
            Serial.println(WEATHER_URL);
         }

         HTTPClient clientHttps;
         WiFiClient sClient;

         if (clientHttps.begin(sClient, WEATHER_URL))                          // Open the URL connection
         {
            int httpResponseCode = clientHttps.GET();                          // Get the response code

            if (httpResponseCode > 0)
            {
               if (httpResponseCode == HTTP_CODE_OK)
               {
                  String payloadStr = clientHttps.getString();

                  if (payloadStr.length() > 0)
                  {
                     if (debugWeather)
                     {
                        Serial.println(payloadStr);
                     }

                     // Parse JSON
                     DynamicJsonDocument doc(768);
                     deserializeJson(doc, payloadStr);

                     weatherUnixTimestamp = (time_t)(doc["dt"]) + (time_t)(doc["timezone"]);

                     if (debugWeather)
                     {
                        strftime(text, sizeof(text), "Last updated: %Y-%m-%d %H:%M", gmtime(&weatherUnixTimestamp));
                        Serial.println(text);
                     }

                     const char* loc = doc["name"];
                     sprintf(weatherLocation, "%s", loc);

                     if (debugWeather)
                     {
                        Serial.print("Location: ");
                        Serial.println(weatherLocation);
                     }

                     const float lat = doc["coord"]["lat"];
                     sprintf(weatherLat, "%.6f", lat);

                     if (debugWeather)
                     {
                        Serial.print("Latitude: ");
                        Serial.print(weatherLat);
                        Serial.print("   ");
                     }

                     const float lon = doc["coord"]["lon"];
                     sprintf(weatherLon, "%.6f", lon);

                     if (debugWeather)
                     {
                        Serial.print("Longitude: ");
                        Serial.println(weatherLon);
                     }

                     const char* w_main = doc["weather"][0]["main"];
                     sprintf(weatherConditions, "%s", w_main);

                     if (debugWeather)
                     {
                        Serial.print("Weather: ");
                        Serial.print(w_main);
                     }

                     const char* w_description = doc["weather"][0]["description"];
                     sprintf(weatherDetails, "%s", w_description);

                     if (debugWeather)
                     {
                        Serial.print(" (");
                        Serial.print(w_description);
                        Serial.println(")");
                     }

                     const float temp = doc["main"]["temp"];
                     weatherTemp = (int)round(temp);

                     if (debugWeather)
                     {
                        Serial.print("Temperature: ");
                        Serial.print((int)round(temp));
                        if (useMetric)
                        {
                           Serial.print("°C   ");
                        } else {
                           Serial.print("°F   ");
                        }
                     }

                     const float feels = doc["main"]["feels_like"];
                     weatherFeels = (int)round(feels);

                     if (debugWeather)
                     {
                        Serial.print("Feels like: ");
                        Serial.print((int)round(feels));
                        if (useMetric)
                        {
                           Serial.print("°C   ");
                        } else {
                           Serial.print("°F   ");
                        }
                     }

                     const int humidity = doc["main"]["humidity"];
                     weatherHumidity = (int)round(humidity);

                     if (debugWeather)
                     {
                        Serial.print("Humidity: ");
                        Serial.print(humidity);
                        Serial.println("%");
                     }

                     float wind_speed = doc["wind"]["speed"];
                     float wind_deg = doc["wind"]["deg"];
                     float wind_gust = doc["wind"]["gust"];
                     weatherWind = (int)round(wind_speed);
                     weatherDirection = (int)round(wind_deg);
                     weatherGust = (int)round(wind_gust);

                     if (debugWeather)
                     {
                        Serial.print("Winds: ");
                        Serial.print((int)round(wind_speed));
                        if ((int)round(wind_gust) > 0)
                        {
                           Serial.print("-");
                           Serial.print((int)round(wind_gust));
                        }
                        if (useMetric)
                        {
                           Serial.print(" kph at ");
                        } else {
                           Serial.print(" mph at ");
                        }
                        Serial.print((int)round(wind_deg));
                        Serial.println("°");
                     }

                     weatherUnixSunrise = (time_t)(doc["sys"]["sunrise"]) + (time_t)(doc["timezone"]);
                     weatherUnixSunset = (time_t)(doc["sys"]["sunset"]) + (time_t)(doc["timezone"]);

                     if (debugWeather)
                     {
                        strftime(text, sizeof(text), "Sunrise: %H:%M", gmtime(&weatherUnixSunrise));
                        strftime(&text[strlen(text)], sizeof(text) - strlen(text), "   Sunset: %H:%M", gmtime(&weatherUnixSunset));
                        Serial.println(text);

                        Serial.println("");
                     }
                  } else {
                     Serial.println("clientHttps.getString() return payload was empty");
                  }
               } else {
                  if (httpResponseCode == 401)
                  {
                     if (debugWeather)
                     {
                        Serial.println("Error on Weather HTTP request: Invalid or missing apiID token");
                     }
                  }

                  // reset indicators that data is no longer valid
                  weatherLocation[0] = 0x00;
                  weatherTemp = 0;
               }
            } else {
               if (debugWeather)
               {
                  Serial.print("Error on Weather HTTP request: ");
                  Serial.println(httpResponseCode);
               }

               // reset indicators that data is no longer valid
               weatherLocation[0] = 0x00;
               weatherTemp = 0;

               // try again in how long
               weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;
            }
         } else {
            if (debugWeather)
            {
               Serial.print("Error on Weather HTTP begin()");
            }

            // reset indicators that data is no longer valid
            weatherLocation[0] = 0x00;
            weatherTemp = 0;

            // try again in how long
            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;
         }

         clientHttps.end();

         updateDisplayFlag = false;
      }
   } else {
      if (--weatherDelayCount <= 0)
      {
         if (debugWeather)
         {
            if (getWiFiStatus() != WIFI_NET_CONNECTED)
            {
               Serial.println("Weather data unavailable - WiFi is not connected");
            } else {
               Serial.println("Weather data unavailable - Weather is not currently selected for display");
            }
         }
      }

      // reset indicators that data is no longer valid
      weatherLocation[0] = 0x00;
      weatherTemp = 0;

      // try again in how long
      weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;
   }
}  // getWeatherData()


uint8_t getWiFiStatus(void)
{
   return ((WiFi.status() == WL_CONNECTED) ? WIFI_NET_CONNECTED : WiFi.softAPgetStationNum() ? WIFI_CONNECTED : WIFI_NOT_CONNECTED);
}  // getWifiStatus()


String getXmlData(String xml, String tag)
{
   uint16_t i = xml.indexOf("<" + tag + ">");                                  // Where the beginning tag is
   uint16_t j = xml.indexOf("</" + tag + ">");                                 // and the ending tag

   String val = String();                                                      // Data value

   if (i > 0 && j > i && j < xml.length())                                     // Sanity check
   {
      val = xml.substring(i + tag.length() + 2, j);                            // Get the data between the tags
      val.trim();                                                              // Eliminate whitespace

      return val;
   } else {
      return ("??");                                                           // If we didn't find anything then use this
   }
}  // getXmlData()


void loop(void)
{
   uint32_t currentTime = millis();

   // Peridically see if we need to (re)connect to WiFi
   netTickTime();

   if (gpsActiveMode)
   {
      while (gpsSS.available() > 0)
      {
         gps.encode(gpsSS.read());
      }
   }

   if ((currentTime - clockTime) >= CLOCK_TIME_INTERVAL_IN_MILLIS)
   {
      // if GPS time updates and NTP time updates are both unavailable, then try to keep time as accurate as possible for as long as possible
      if ((!gpsActiveMode) && (getWiFiStatus() != WIFI_NET_CONNECTED))
      {
         uint32_t lastIntervalTime = currentTime - clockTime;

         // adjust the next cycle to accomodate any potential difference in precise CLOCK_TIME_INTERVAL_IN_MILLIS periods
         clockTime = currentTime;
         clockTime += CLOCK_TIME_INTERVAL_IN_MILLIS - lastIntervalTime;

         if (debugIsEnabled)
         {
            Serial.print("next 1000-millisecond period has been adjusted to be ");
            Serial.print((CLOCK_TIME_INTERVAL_IN_MILLIS * 2) - lastIntervalTime);
            Serial.println(" milliseconds long");
         }
      } else {
         clockTime = currentTime;
      }

      if (getWiFiStatus() == WIFI_NET_CONNECTED)                               // only try to update NTP time if WiFi is connected
      {
         events();                                                             // Get periodic NTP updates
      }

      getSolarData();
      getWeatherData();

      if (gpsActiveMode != previousGPSActiveMode)
      {
         previousGPSActiveMode = gpsActiveMode;

         if (gpsActiveMode)
         {
            startGPS(true);
         } else {
            gpsFixState = GPS_NO_FIX;
            gpsSatelliteCount = 0;
         }

         newDualScreen();
      }

      printTime();                                                             // Send updated time to serial port

      if (--tzCountdownInterval == 0)                                          // Time for next timezone?
      {
         tzCountdownInterval = tzCountdownIntervalSetting;

         tzIndex = SHOW_TZ_TYPE(tzIndex + 1);
         if (tzIndex >= SHOW_TZ_END_MARKER)
         {
            tzIndex = SHOW_TZ_EST_EDT;
         }

         boolean foundNext = false;

         while (foundNext == false)
         {
            switch (tzIndex)
            {
               case SHOW_TZ_EST_EDT:
                  {
                     if (showESTEDT)
                     {
                        foundNext = true;
                     }
                  }
                  break;

               case SHOW_TZ_CST_CDT:
                  {
                     if (showCSTCDT)
                     {
                        foundNext = true;
                     }
                  }
                  break;

               case SHOW_TZ_MST_MDT:
                  {
                     if (showMSTMDT)
                     {
                        foundNext = true;
                     }
                  }
                  break;

               case SHOW_TZ_MST_NO_MDT:
                  {
                     if (showMSTNOMDT)
                     {
                        foundNext = true;
                     }
                  }
                  break;

               case SHOW_TZ_PST_PDT:
                  {
                     if (showPSTPDT)
                     {
                        foundNext = true;
                     }
                  }
                  break;

               case SHOW_TZ_AWST:
                  {
                     if (showAWST)
                     {
                        foundNext = true;
                     }
                  }
                  break;

               case SHOW_TZ_ACST_ACDT:
                  {
                     if (showACSTACDT)
                     {
                        foundNext = true;
                     }
                  }
                  break;

               case SHOW_TZ_AEST_AEDT:
                  {
                     if (showAESTAEDT)
                     {
                        foundNext = true;
                     }
                  }
                  break;

               case SHOW_TZ_GMT_BST:
                  {
                     if (showGMTBST)
                     {
                        foundNext = true;
                     }
                  }
                  break;
            }

            if (!foundNext)
            {
               tzIndex = SHOW_TZ_TYPE(tzIndex + 1);
               if (tzIndex >= SHOW_TZ_END_MARKER)
               {
                  tzIndex = SHOW_TZ_EST_EDT;
               }
            }
         }

         local.setPosix(timeZones[int(tzIndex)]);                              // Set new local time zone by rule
      }

      show5VDC(225, 16);
      show12VDC(274, 16);

      static int8_t previousGPSFixState = GPS_NO_FIX;
      static int8_t previousGPSSatelliteCount = 0;

      if (gpsActiveMode)
      {
         // check the current fix state
         getGPSFixState();

         // check GPS time
         getGPSTimeUpdate();

         // check GPS altitude
         getGPSAltitudeUpdate();

         // check GPS position
         getGPSPositionUpdate();

         // if GPS fix state changes, or if the number of satellites increases (to more than 3), then re-sync time
         if ((previousGPSFixState != gpsFixState) || ((gpsSatelliteCount > previousGPSSatelliteCount) && (gpsSatelliteCount > 3)))
         {
            // sync time to the nearest second
            getGPSTimeUpdate(true);

            getGPSPositionUpdate();

            previousGPSFixState = gpsFixState;
            previousGPSSatelliteCount = gpsSatelliteCount;

            // and anytime fix mode changes, 60 seconds later, set internal clock from GPS again, just to be sure
            gpsDelayCount = INITIAL_USE_GPS_TO_SET_INTERNAL_CLOCK_INTERVAL_IN_SECONDS;
         }
      }

      if (debugStackUsage)
      {
         uint32_t freeStack = uxTaskGetStackHighWaterMark(NULL);
         Serial.print("Minimum remaining stack space: ");
         Serial.println(freeStack);
         Serial.print("Heap available: ");
         Serial.println(ESP.getFreeHeap());
      }

      if (BMP280_available || BME280_available)
      {
         printValues();                                                        // Display on serial monitor
      }
   }

   if (configButton.wasPressed())                                              // If the button was pressed & released
   {
      switch (configButton.read())
      {
         case single_click:
            {
               if (brightButton.isPressed())
               {
                  while (brightButton.isPressed())
                  {
                     delay(10);
                  }

                  configButton.resetPressedState();
                  brightButton.resetPressedState();

                  showButtonDefinitions();
               } else {
                  dataIndex = SHOW_DATA_SFI;                                   // (re)start the data sequence at the beginning

                  showMode = (SHOW_MODE_TYPE)(showMode + 1);
                  if (showMode >= SHOW_MODE_END_MARKER)
                  {
                     showMode = SHOW_MODE_NONE;
                  }

                  showSolarSelection =   (boolean)(showMode & SHOW_MODE_SOLAR_BIT_MASK);
                  showEnvSelection =     (boolean)(showMode & SHOW_MODE_ENV_BIT_MASK);
                  showWeatherSelection = (boolean)(showMode & SHOW_MODE_WEATHER_BIT_MASK);
                  showMLSSelection =     (boolean)(showMode & SHOW_MODE_MLS_BIT_MASK);
                  showGPSSelection =     (boolean)(showMode & SHOW_MODE_GPS_BIT_MASK);

                  showModeFlag = true;
                  showModeDelayCount = CYCLE_TIME_IN_SECONDS;

                  switch (showMode)
                  {
                     case SHOW_MODE_NONE:
                        {
                        }
                        break;

                     case SHOW_MODE_SOLAR_ONLY:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update
                        }
                        break;

                     case SHOW_MODE_ENV_ONLY:
                        {
                           dataIndex = SHOW_DATA_ENV;
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_ENV:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update
                        }
                        break;

                     case SHOW_MODE_WEATHER_ONLY:
                        {
                           dataIndex = SHOW_FIRST_WEATHER_DATA;

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // force a change in the weather fetch countdown & apply the new interval
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_WEATHER:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_ENV_PLUS_WEATHER:
                        {
                           dataIndex = SHOW_DATA_ENV;
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_MLS_ONLY:
                        {
                           dataIndex = SHOW_DATA_MLS;
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_MLS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update
                        }
                        break;

                     case SHOW_MODE_ENV_PLUS_MLS:
                        {
                           dataIndex = SHOW_DATA_ENV;
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update
                        }
                        break;

                     case SHOW_MODE_WEATHER_PLUS_MLS:
                        {
                           dataIndex = SHOW_FIRST_WEATHER_DATA;

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // force a change in the weather fetch countdown & apply the new interval
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS:
                        {
                           dataIndex = SHOW_DATA_ENV;

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_GPS_ONLY:
                        {
                           dataIndex = SHOW_FIRST_GPS_DATA;
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update
                        }
                        break;

                     case SHOW_MODE_ENV_PLUS_GPS:
                        {
                           dataIndex = SHOW_DATA_ENV;
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update
                        }
                        break;

                     case SHOW_MODE_WEATHER_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_WEATHER_DATA;

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // force a change in the weather fetch countdown & apply the new interval
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_GPS:
                        {
                           dataIndex = SHOW_DATA_ENV;

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_MLS_PLUS_GPS:
                        {
                           dataIndex = SHOW_DATA_MLS;
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_MLS_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update
                        }
                        break;

                     case SHOW_MODE_ENV_PLUS_MLS_PLUS_GPS:
                        {
                           dataIndex = SHOW_DATA_ENV;
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update
                        }
                        break;

                     case SHOW_MODE_WEATHER_PLUS_MLS_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_WEATHER_DATA;

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // force a change in the weather fetch countdown & apply the new interval
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
                        {
                           dataIndex = SHOW_DATA_ENV;

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;

                     case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;

                           solarDelayCount = 5;                                // trigger a solar update

                           weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS; // trigger a weather update
                        }
                        break;
                  }

                  prefs.begin("config", false);
                  prefs.putInt("showMode", int(showMode));
                  prefs.end();
               }
            }
            break;

         case double_click:
            {
               useMetric = !useMetric;                                         // Toggle metric/US ENV display

               solarDelayCount = 5;                                            // trigger a solar update
               weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;     // force a change in the weather fetch countdown & apply the new interval

               prefs.begin("settings", false);
               prefs.putBool("useMetric", useMetric);
               prefs.end();
            }
            break;

         case triple_click:
            {
               wifiRetryAuto = !wifiRetryAuto;
               showWiFiStatus();
            }
            break;

         case long_click:
            {
               if (brightButton.isPressed())
               {
                  while (brightButton.isPressed())
                  {
                     delay(10);
                  }

                  configButton.resetPressedState();
                  brightButton.resetPressedState();

                  showButtonDefinitions();
               } else {
                  int32_t pressedDuration = configButton.wasPressedFor();

                  if (pressedDuration >= EXTRA_LONG_BUTTON_PRESS_TIME_IN_MS)
                  {
                     loginPassword = String();

                     // Start modifying network preferences
                     prefs.begin("network", false);

                     prefs.putString("loginpassword", loginPassword);

                     // Done with modifying network preferences
                     prefs.end();

                     Serial.println("Network login password cleared...");
                  }

                  // in any case (long click or extra long click), choose whether to reboot the whole NTPclock or not
                  if (selectReboot())
                  {
                     ESP.restart();                                            // Reboot the ESP32
                  } else {
                     showWiFiInfo();                                           // display the WiFi info screen
                  }

                  switchDisplayMode();
                  newDualScreen();
               }
            }
            break;
      }
   }

   if (brightButton.wasPressed())
   {
      switch (brightButton.read())
      {
         case single_click:
            {
               if (configButton.isPressed())
               {
                  while (configButton.isPressed())
                  {
                     delay(10);
                  }

                  configButton.resetPressedState();
                  brightButton.resetPressedState();

                  showButtonDefinitions();
               } else {
                  brightness++;
                  if (brightness > 7)
                  {
                     brightness = 0;
                  }

                  prefs.begin("settings", false);
                  prefs.putInt("brightness", brightness);
                  prefs.end();

                  tft.setTextDatum(TC_DATUM);

                  tft.setTextColor(labelFGColor, labelBGColor);
                  tft.drawString("B:", 216, 2, 2);

                  if (brightness == 0)                                         // Auto mode?
                  {
                     tft.drawString("A", 228, 2, 2);
                  } else {
                     tft.drawNumber(brightness, 228, 2, 2);
                  }

                  tft.setTextDatum(TL_DATUM);
               }
            }
            break;

         case double_click:
            {
               nightMode = !nightMode;                                         // Switch between normal & night modes
               switchDisplayMode();
               prefs.begin("settings", false);
               prefs.putBool("nightMode", nightMode);
               prefs.end();
               newDualScreen();                                                // Repaint the whole screen
            }
            break;

         case triple_click:
            {
               forceDefaults(true);
               readSettings();
            }
            break;

         case long_click:
            {
               if (configButton.isPressed())
               {
                  while (configButton.isPressed())
                  {
                     delay(10);
                  }

                  configButton.resetPressedState();
                  brightButton.resetPressedState();

                  showButtonDefinitions();
               } else {
                  int32_t pressedDuration = brightButton.wasPressedFor();

                  if (pressedDuration >= EXTRA_LONG_BUTTON_PRESS_TIME_IN_MS)
                  {
                     debugIsEnabled = true;

                     Serial.println("DEBUG control via the AP hosted webpage is now enabled...");
                  } else {
                     switch (selectGPSResetType())
                     {
                        case 0:
                           {
                              gpsActiveMode = !gpsActiveMode;

                              // if the GPS was just disabled, then save whatever the current LAT/LON/ALT may be for future use
                              if (!gpsActiveMode)
                              {
                                 prefs.begin("config", false);

                                 prefs.putString("gpsManualLat", gpsManualLat);
                                 prefs.putString("gpsManualLon", gpsManualLon);
                                 prefs.putString("qthaltft", qthAltitudeInFeet);

                                 prefs.end();

#if !defined(ESP32S3_MJC_TESTBED) && !defined(ESP32S3_MINI) && !defined(ESP32S3_SUPERMINI)
                              } else {
                                 // if GPS was just enabled, then disable Weather & Solar fetching (since they are very likely to start failing)
                                 showMode = (SHOW_MODE_TYPE)(showMode & ~SHOW_MODE_SOLAR_BIT_MASK);
                                 showMode = (SHOW_MODE_TYPE)(showMode & ~SHOW_MODE_WEATHER_BIT_MASK);

#endif

                              }
                           }
                           break;

                        case 1:
                           {
                              if (gpsFixState != GPS_3D_FIX)
                              {
                                 gpsActiveMode = false;
                                 gpsFixState = GPS_NO_FIX;
                                 gpsSatelliteCount = 0;

                                 // initialize the GPS before resetting it, to increase the chances of talking to it quickly
                                 startGPS(true);
                              }

                              // try with current pin definitons...
                              if (!gpsFactoryReset())
                              {
                                 // if that fails, try with the opposite pin definitions
                                 gpsSSreversed = !gpsSSreversed;

                                 if (gpsSSreversed)
                                 {
                                    gps_tx_pin = GPS_DATA_IN_PIN;
                                    gps_rx_pin = GPS_DATA_OUT_PIN;
                                 } else {
                                    gps_tx_pin = GPS_DATA_OUT_PIN;
                                    gps_rx_pin = GPS_DATA_IN_PIN;
                                 }

                                 if (gpsFactoryReset())
                                 {
                                    startGPS();
                                 }
                              } else {
                                 startGPS();
                              }

                              newDualScreen();
                           }
                           break;

                        default:
                           {
                              newDualScreen();
                           }
                           break;
                     }
                  }
               }
            }
            break;
      }
   }

   delay(10);

   updateDisplay();                                                            // update clock every second
}  // loop()


void netTickTime(void)
{
   if (getWiFiStatus() == WIFI_NET_CONNECTED)
   {
      reconnectTime = millis();
   } else {
      if ((millis() - reconnectTime) >= RECONNECT_TIME)
      {
         if (wifiRetryAuto)                                                    // if we're in auto reconnect mode, then request WiFi reconnect
         {
            // request a WiFi restart
            itIsTimeToWiFi = true;
         }

         reconnectTime = millis();
      }
   }

   // Connect to WiFi if requested
   if (itIsTimeToWiFi && ((millis() - connectTime) >= CONNECT_TIME))
   {
      itIsTimeToWiFi = false;

      netInit();

      startupScreen();
      newDualScreen();
      updateDisplay();

      connectTime = millis();

      solarDelayCount = 5;                                                     // trigger a solar update
      weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;              // force a change in the weather fetch countdown & apply the new interval
   }
}  // netTickTime()


void netInit(void)
{
   WiFi.disconnect(true);

   WiFi.softAPdisconnect(true);

   WiFi.disconnect(true, true);

   // Start WiFi access point + Station mode
   WiFi.mode(WIFI_AP_STA);

   wifiInitAP();

   // Initialize web server for remote configuration
   webInit();

   // Initialize WiFi and try connecting to a network
   if (wifiConnect())
   {
      // Let user see connection status if successful
      delay(2000);

      xmlData = String();                                                      // signify to try solar data again next pass

      // We're on the home network now - the local Access Point is no longer needed,
      // so shut it down and drop out of AP_STA into plain Station mode. If the STA
      // connection is later lost, netTickTime()/netInit() will bring the AP back.
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
   }
}  // netInit()


void newDualScreen(void)                                                       // Displays the fixed parts
{
   String verStr = String("rev: ") + VERSION_TIMESTAMP;

   tft.setTextDatum(TC_DATUM);

   tft.fillScreen(screenBGColor);

   if (labelFGColor != screenBGColor)
   {
      tft.setTextColor(labelFGColor, screenBGColor);
   } else {
      tft.setTextColor(TFT_BLACK, screenBGColor);
   }

   tft.drawString(verStr, 160, 226, 2);                                        // put version info on the screen

   if (nightMode)
   {
      tft.drawRoundRect(1, 1, 318, 34, 10, edgeColor);                         // Title bar for local time
      tft.drawRoundRect(1, 116, 318, 34, 10, edgeColor);                       // Title bar for UTC
   } else {
      tft.fillRoundRect(1, 1, 318, 33, 10, labelBGColor);                      // Title bar for local time
      tft.drawRoundRect(1, 1, 318, 34, 10, edgeColor);                         // Title block
      tft.fillRoundRect(1, 116, 318, 33, 10, labelBGColor);                    // Title bar for UTC
      tft.drawRoundRect(1, 116, 318, 34, 10, edgeColor);                       // Title block
   }

   tft.setTextColor(labelFGColor, labelBGColor);
   tft.drawString(bannerTitle, 140, 7, 4);
   tft.drawRoundRect(1, 1, 318, 110, 10, edgeColor);                           // Draw edge around local time
   tft.drawRoundRect(1, 116, 318, 110, 10, edgeColor);                         // Draw edge around UTC

   tft.setTextDatum(TC_DATUM);

   tft.setTextColor(labelFGColor, labelBGColor);
   tft.drawString("B:", 216, 2, 2);

   if (brightness == 0)                                                        // Auto mode?
   {
      tft.drawString("A", 228, 2, 2);
   } else {
      tft.drawNumber(brightness, 228, 2, 2);
   }

   tft.setTextDatum(TC_DATUM);

   tft.setTextColor(labelFGColor, labelBGColor);
   tft.drawString("B:", 216, 2, 2);

   if (brightness == 0)                                                        // Auto mode?
   {
      tft.drawString("A", 228, 2, 2);
   } else {
      tft.drawNumber(brightness, 228, 2, 2);
   }

   tft.setTextDatum(TL_DATUM);

   showGPSStatus(248, 2);
}  // newDualScreen()


void printTime(void)                                                           // Print time to serial port
{
   switch (printedTime)
   {
      case PRINTED_TIME_UTC:
         {
            Serial.println(dateTime(EZTIME_TIME_FORMAT));
         }
         break;

      case PRINTED_TIME_LOCAL:
         {
            Serial.println(local.dateTime(EZTIME_TIME_FORMAT));
         }
         break;

      case PRINTED_TIME_BOTH:
         {
            Serial.println(dateTime(EZTIME_TIME_FORMAT));
            Serial.println(local.dateTime(EZTIME_TIME_FORMAT));
         }
         break;
   }
}  // printTime()


void printValues(void)                                                         // Send environmental data to serial monitor
{
   if ((debugEnvSensor) && (BME280_available))
   {
      if (useMetric)
      {
         Serial.print("Temperature = ");
         Serial.print(bme.readTemperature () - 1.9);
         Serial.println(" *C");

         Serial.print("Pressure = ");
         Serial.print(bme.readPressure() / 100.0F);
         Serial.println(" hPa");
      } else {
         Serial.print("Temperature = ");
         Serial.print(1.8 * (bme.readTemperature() - 1.9) + 32);
         Serial.println(" *F");

         Serial.print("Pressure = ");
         Serial.print(bme.readPressure() / 100.0F * 0.02953 + (DEFAULT_METRIC_PRESSURE_OFFSET / 33));
         Serial.println(" in");
      }
      Serial.print("Humidity = ");
      Serial.print(bme.readHumidity());
      Serial.println(" %");
      Serial.println();
   }

   if ((debugEnvSensor) && (BMP280_available))
   {
      if (useMetric)
      {
         Serial.print("Temperature = ");
         Serial.print(bmp.readTemperature() - 1.9);
         Serial.println(" *C");

         Serial.print("Pressure = ");
         Serial.print(bmp.readPressure() / 100.0F);
         Serial.println(" hPa");
      } else {
         Serial.print("Temperature = ");
         Serial.print(1.8 * (bmp.readTemperature() - 1.9) + 32);
         Serial.println(" *F");

         Serial.print("Pressure = ");
         Serial.print(bmp.readPressure() / 100.0F * 0.02953 + (DEFAULT_METRIC_PRESSURE_OFFSET / 33));
         Serial.println(" in");
      }
      Serial.println();
   }
}  // printValues()


void readSettings(void)
{
   prefs.begin("settings", false);

   if (prefs.isKey("brightness"))
   {
      brightness = prefs.getInt("brightness", DEFAULT_BRIGHTNESS);
      if (!BMP280_available && !BME280_available && (brightness == 0))         // if sensor not installed, don't let brightness be "auto"
      {
         brightness = DEFAULT_BRIGHTNESS;

         if (brightness == 0)                                                  // if the default happened to be "auto", then force to max
         {
            brightness = 7;
         }

         prefs.putInt("brightness", brightness);
      }
   } else {
      brightness = DEFAULT_BRIGHTNESS;

      prefs.putInt("brightness", brightness);
   }

   if (prefs.isKey("nightMode"))
   {
      nightMode = prefs.getBool("nightMode", DEFAULT_NIGHTMODE);
   } else {
      nightMode = DEFAULT_NIGHTMODE;
      prefs.putBool("nightMode", nightMode);
   }

   if (prefs.isKey("useMetric"))
   {
      useMetric = prefs.getBool("useMetric", DEFAULT_USEMETRIC);
   } else {
      useMetric = DEFAULT_USEMETRIC;
      prefs.putBool("useMetric", useMetric);
   }

   if (prefs.isKey("showDataHeader"))
   {
      showDataHeaderBetweenDataGroups = prefs.getBool("showDataHeader", false);
   } else {
      showDataHeaderBetweenDataGroups = false;
      prefs.putBool("showDataHeader", false);
   }

   prefs.end();

   prefs.begin("config", false);

   if (prefs.isKey("showMode"))
   {
      showMode = (SHOW_MODE_TYPE)(prefs.getInt("showMode", SHOW_MODE_NONE));

      if (showMode >= SHOW_MODE_END_MARKER)
      {
         showMode = SHOW_MODE_NONE;
      }
   } else {
      showMode = SHOW_MODE_NONE;
      prefs.putInt("showMode", int(showMode));
   }

   showSolarSelection =   (boolean)(showMode & SHOW_MODE_SOLAR_BIT_MASK);
   showEnvSelection =     (boolean)(showMode & SHOW_MODE_ENV_BIT_MASK);
   showWeatherSelection = (boolean)(showMode & SHOW_MODE_WEATHER_BIT_MASK);
   showMLSSelection =     (boolean)(showMode & SHOW_MODE_MLS_BIT_MASK);
   showGPSSelection =     (boolean)(showMode & SHOW_MODE_GPS_BIT_MASK);

   switch (showMode)
   {
      case SHOW_MODE_NONE:
         {
         }
         break;

      case SHOW_MODE_SOLAR_ONLY:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_ENV_ONLY:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_WEATHER_ONLY:
         {
            dataIndex = SHOW_FIRST_WEATHER_DATA;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_WEATHER:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_ENV_PLUS_WEATHER:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_MLS_ONLY:
         {
            dataIndex = SHOW_DATA_MLS;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_ENV_PLUS_MLS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_WEATHER_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_WEATHER_DATA;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_GPS_ONLY:
         {
            dataIndex = SHOW_FIRST_GPS_DATA;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_ENV_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_WEATHER_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_WEATHER_DATA;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_MLS;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_ENV_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_WEATHER_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_WEATHER_DATA;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;

      case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }
         break;
   }

   if (prefs.isKey("title"))
   {
      bannerTitle = prefs.getString("title", bannerTitle);
   } else {
      prefs.putString("title", bannerTitle);
   }

   if (prefs.isKey("appID"))
   {
      weatherAppID = prefs.getString("appID", weatherAppID);

      appID = "&appid=" + weatherAppID;
   } else {
      prefs.putString("appID", weatherAppID);
   }

   if (!(((weatherAppID.charAt(0) >= '0') && (weatherAppID.charAt(0) <= '9')) || ((weatherAppID.charAt(0) >= 'a') && (weatherAppID.charAt(0) <= 'f'))))
   {
      needAppIDFlag = true;
   } else {
      needAppIDFlag = false;
   }

   if (prefs.isKey("gpsManualLat"))
   {
      gpsManualLat = prefs.getString("gpsManualLat", latDefault);
   } else {
      gpsManualLat = latDefault;
      prefs.putString("gpsManualLat", gpsManualLat);
   }

   if (prefs.isKey("gpsManualLon"))
   {
      gpsManualLon = prefs.getString("gpsManualLon", lonDefault);
   } else {
      gpsManualLon = lonDefault;
      prefs.putString("gpsManualLon", gpsManualLon);
   }

   if (prefs.isKey("decimals"))
   {
      weatherLatLonDecimalPlaces = prefs.getInt("decimals", DEFAULT_LAT_LON_DECIMAL_PLACES);
   } else {
      weatherLatLonDecimalPlaces = DEFAULT_LAT_LON_DECIMAL_PLACES;
      prefs.putInt("decimals", weatherLatLonDecimalPlaces);
   }

   if (prefs.isKey("tzInterval"))
   {
      tzCountdownIntervalSetting = prefs.getInt("tzInterval", DEFAULT_TZ_INTERVAL);
   } else {
      tzCountdownIntervalSetting = DEFAULT_TZ_INTERVAL;
      prefs.putInt("tzInterval", tzCountdownIntervalSetting);

   }

   if (prefs.isKey("showest"))
   {
      showESTEDT = prefs.getBool("showest", DEFAULT_SHOW_EST_EDT_TZ);
   } else {
      showESTEDT = DEFAULT_SHOW_EST_EDT_TZ;
      prefs.putBool("showest", showESTEDT);
   }

   if (prefs.isKey("showcst"))
   {
      showCSTCDT = prefs.getBool("showcst", DEFAULT_SHOW_CST_CDT_TZ);
   } else {
      showCSTCDT = DEFAULT_SHOW_CST_CDT_TZ;
      prefs.putBool("showcst", showCSTCDT);
   }

   if (prefs.isKey("showmst"))
   {
      showMSTMDT = prefs.getBool("showmst", DEFAULT_SHOW_MST_MDT_TZ);
   } else {
      showMSTMDT = DEFAULT_SHOW_MST_MDT_TZ;
      prefs.putBool("showmst", showMSTMDT);
   }

   if (prefs.isKey("showmstonly"))
   {
      showMSTNOMDT = prefs.getBool("showmstonly", DEFAULT_SHOW_MST_NO_MDT_TZ);
   } else {
      showMSTNOMDT = DEFAULT_SHOW_MST_NO_MDT_TZ;
      prefs.putBool("showmstonly", showMSTNOMDT);
   }

   if (prefs.isKey("showpst"))
   {
      showPSTPDT = prefs.getBool("showpst", DEFAULT_SHOW_PST_PDT_TZ);
   } else {
      showPSTPDT = DEFAULT_SHOW_PST_PDT_TZ;
      prefs.putBool("showpst", showPSTPDT);
   }

   if (prefs.isKey("showawst"))
   {
      showAWST = prefs.getBool("showawst", DEFAULT_SHOW_AWST_TZ);
   } else {
      showAWST = DEFAULT_SHOW_AWST_TZ;
      prefs.putBool("showawst", showAWST);
   }

   if (prefs.isKey("showacst"))
   {
      showACSTACDT = prefs.getBool("showacst", DEFAULT_SHOW_ACST_ACDT_TZ);
   } else {
      showACSTACDT = DEFAULT_SHOW_ACST_ACDT_TZ;
      prefs.putBool("showacst", showACSTACDT);
   }

   if (prefs.isKey("showaest"))
   {
      showAESTAEDT = prefs.getBool("showaest", DEFAULT_SHOW_AEST_AEDT_TZ);
   } else {
      showAESTAEDT = DEFAULT_SHOW_AEST_AEDT_TZ;
      prefs.putBool("showaest", showAESTAEDT);
   }

   if (prefs.isKey("showgmt"))
   {
      showGMTBST = prefs.getBool("showgmt", DEFAULT_SHOW_GMT_BST_TZ);
   } else {
      showGMTBST = DEFAULT_SHOW_GMT_BST_TZ;
      prefs.putBool("showgmt", showGMTBST);
   }

   // make sure that at least one timezone is selected (you choose which it is)
   if (!showESTEDT && !showCSTCDT && !showMSTMDT && !showMSTNOMDT && !showPSTPDT && !showAWST && !showACSTACDT && !showAESTAEDT && !showGMTBST)
   {
      showCSTCDT = true;
      prefs.putBool("showcst", showCSTCDT);
   }

   if (prefs.isKey("showAMPM"))
   {
      showAMPMMode = (SHOW_AMPM_TYPE)prefs.getInt("showAMPM", DEFAULT_SHOW_AMPM_MODE);
   } else {
      showAMPMMode = DEFAULT_SHOW_AMPM_MODE;
      prefs.putInt("showAMPM", showAMPMMode);
   }

   if (prefs.isKey("local24hr"))
   {
      localFormat24hr = prefs.getBool("local24hr", DEFAULT_LOCAL_FORMAT_24HR);
   } else {
      localFormat24hr = DEFAULT_LOCAL_FORMAT_24HR;
      prefs.putBool("local24hr", localFormat24hr);
   }

   if (prefs.isKey("utc24hr"))
   {
      utcFormat24hr = prefs.getBool("utc24hr", DEFAULT_UTC_FORMAT_24HR);
   } else {
      utcFormat24hr = DEFAULT_UTC_FORMAT_24HR;
      prefs.putBool("utc24hr", utcFormat24hr);
   }

   if (prefs.isKey("hrlead0"))
   {
      hourLeadingZero = prefs.getBool("hrlead0", DEFAULT_HOUR_LEADING_ZERO);
   } else {
      hourLeadingZero = DEFAULT_HOUR_LEADING_ZERO;
      prefs.putBool("hrlead0", hourLeadingZero);
   }

   if (prefs.isKey("dtlead0"))
   {
      dateLeadingZero = prefs.getBool("dtlead0", DEFAULT_DATE_LEADING_ZERO);
   } else {
      dateLeadingZero = DEFAULT_DATE_LEADING_ZERO;
      prefs.putBool("dtlead0", dateLeadingZero);
   }

   if (prefs.isKey("dtovermo"))
   {
      dateAboveMonth = prefs.getBool("dtovermo", DEFAULT_DATE_ABOVE_MONTH);
   } else {
      dateAboveMonth = DEFAULT_DATE_ABOVE_MONTH;
      prefs.putBool("dtovermo", dateAboveMonth);
   }

   if (prefs.isKey("show5vdc"))
   {
      fiveVoltDisplay = prefs.getBool("show5vdc", DEFAULT_5VDC_DISPLAY);
   } else {
      fiveVoltDisplay = DEFAULT_5VDC_DISPLAY;
      prefs.putBool("show5vdc", fiveVoltDisplay);
   }

   if (prefs.isKey("fivevoltcorr"))
   {
      fiveVoltCorrection = prefs.getString("fivevoltcorr", "1.0");
   } else {
      fiveVoltCorrection = "1.0";
      prefs.putString("fivevoltcorr", fiveVoltCorrection);
   }

   if (prefs.isKey("show12vdc"))
   {
      twelveVoltDisplay = prefs.getBool("show12vdc", DEFAULT_12VDC_DISPLAY);
   } else {
      twelveVoltDisplay = DEFAULT_12VDC_DISPLAY;
      prefs.putBool("show12vdc", twelveVoltDisplay);
   }

   if (prefs.isKey("twelvevoltcorr"))
   {
      twelveVoltCorrection = prefs.getString("twelvevoltcorr", "1.0");
   } else {
      twelveVoltCorrection = "1.0";
      prefs.putString("twelvevoltcorr", twelveVoltCorrection);
   }

   if (prefs.isKey("gpsInterval"))
   {
      useGPSToSetInternalClockIntervalInSeconds = prefs.getInt("gpsInterval", DEFAULT_USE_GPS_TO_SET_INTERNAL_CLOCK_INTERVAL_IN_SECONDS);
   } else {
      useGPSToSetInternalClockIntervalInSeconds = DEFAULT_USE_GPS_TO_SET_INTERNAL_CLOCK_INTERVAL_IN_SECONDS;
      prefs.putInt("gpsInterval", useGPSToSetInternalClockIntervalInSeconds);
   }

   if (prefs.isKey("wfInterval"))
   {
      weatherFetchIntervalInSeconds = prefs.getInt("wfInterval", DEFAULT_WEATHER_FETCH_INTERVAL_IN_SECONDS);
   } else {
      weatherFetchIntervalInSeconds = DEFAULT_WEATHER_FETCH_INTERVAL_IN_SECONDS;
      prefs.putInt("wfInterval", weatherFetchIntervalInSeconds);
   }

   if (prefs.isKey("printtime"))
   {
      printedTime = prefs.getInt("printtime", DEFAULT_PRINTED_TIME);
   } else {
      printedTime = DEFAULT_PRINTED_TIME;
      prefs.putInt("printtime", (int)printedTime);
   }

   if (prefs.isKey("blinktime"))
   {
      blinkTimeUpdate = prefs.getBool("blinktime", false);
   } else {
      blinkTimeUpdate = false;
      prefs.putBool("blinktime", blinkTimeUpdate);
   }

   if (prefs.isKey("showDataHeader"))
   {
      showDataHeaderBetweenDataGroups = prefs.getBool("showDataHeader", false);
   } else {
      showDataHeaderBetweenDataGroups = false;
      prefs.putBool("showDataHeader", showDataHeaderBetweenDataGroups);
   }

   if (prefs.isKey("qthaltft"))
   {
      qthAltitudeInFeet = prefs.getString("qthaltft", DEFAULT_QTH_ALTITUDE_IN_FEET);
   } else {
      qthAltitudeInFeet = DEFAULT_QTH_ALTITUDE_IN_FEET;
      prefs.putString("qthaltft", qthAltitudeInFeet);
   }

   if (prefs.isKey("tempoffset"))
   {
      temperatureOffsetInCelsius = prefs.getString("tempoffset", DEFAULT_TEMPERATURE_OFFSET_IN_CELSIUS);
   } else {
      temperatureOffsetInCelsius = DEFAULT_TEMPERATURE_OFFSET_IN_CELSIUS;
      prefs.putString("tempoffset", temperatureOffsetInCelsius);
   }

   if (prefs.isKey("humoffset"))
   {
      humidityOffset = prefs.getInt("humoffset", DEFAULT_HUMIDITY_OFFSET);
   } else {
      humidityOffset = DEFAULT_HUMIDITY_OFFSET;
      prefs.putInt("humoffset", humidityOffset);
   }

   prefs.end();

   prefs.begin("network", false);

   if (prefs.isKey("wifiRetryAuto"))
   {
      wifiRetryAuto = prefs.getBool("wifiRetryAuto", DEFAULT_WIFI_RETRY_MODE);
   } else {
      wifiRetryAuto = DEFAULT_WIFI_RETRY_MODE;
      prefs.putBool("wifiRetryAuto", wifiRetryAuto);
   }

   prefs.end();

   prefs.begin("colors", false);

   if (prefs.isKey("sBGColorIdx"))
   {
      screenBGColorIdx = prefs.getInt("sBGColorIdx", DEFAULT_SCREEN_BG_COLOR_IDX);
   } else {
      screenBGColorIdx = DEFAULT_SCREEN_BG_COLOR_IDX;
      prefs.putInt("sBGColorIdx", screenBGColorIdx);
   }

   if (prefs.isKey("tColorIdx"))
   {
      timeColorIdx = prefs.getInt("tColorIdx", DEFAULT_TIME_COLOR_IDX);
   } else {
      timeColorIdx = DEFAULT_TIME_COLOR_IDX;
      prefs.putInt("tColorIdx", timeColorIdx);
   }

   if (prefs.isKey("dColorIdx"))
   {
      dateColorIdx = prefs.getInt("dColorIdx", DEFAULT_DATE_COLOR_IDX);
   } else {
      dateColorIdx = DEFAULT_DATE_COLOR_IDX;
      prefs.putInt("dColorIdx", dateColorIdx);
   }

   if (prefs.isKey("lFGColorIdx"))
   {
      labelFGColorIdx = prefs.getInt("lFGColorIdx", DEFAULT_LABEL_FGCOLOR_IDX);
   } else {
      labelFGColorIdx = DEFAULT_LABEL_FGCOLOR_IDX;
      prefs.putInt("lFGColorIdx", labelFGColorIdx);
   }

   if (prefs.isKey("lBGColorIdx"))
   {
      labelBGColorIdx = prefs.getInt("lBGColorIdx", DEFAULT_LABEL_BGCOLOR_IDX);
   } else {
      labelBGColorIdx = DEFAULT_LABEL_BGCOLOR_IDX;
      prefs.putInt("lBGColorIdx", labelBGColorIdx);
   }

   if (prefs.isKey("eColorIdx"))
   {
      edgeColorIdx = prefs.getInt("eColorIdx", DEFAULT_EDGE_COLOR_IDX);
   } else {
      edgeColorIdx = DEFAULT_EDGE_COLOR_IDX;
      prefs.putInt("eColorIdx", edgeColorIdx);
   }

   if (prefs.isKey("nColorIdx"))
   {
      normalColorIdx = prefs.getInt("nColorIdx", DEFAULT_NORMAL_COLOR_IDX);
   } else {
      normalColorIdx = DEFAULT_NORMAL_COLOR_IDX;
      prefs.putInt("nColorIdx", normalColorIdx);
   }

   if (prefs.isKey("mColorIdx"))
   {
      mediumColorIdx = prefs.getInt("mColorIdx", DEFAULT_MEDIUM_COLOR_IDX);
   } else {
      mediumColorIdx = DEFAULT_MEDIUM_COLOR_IDX;
      prefs.putInt("mColorIdx", mediumColorIdx);
   }

   if (prefs.isKey("hColorIdx"))
   {
      highColorIdx = prefs.getInt("hColorIdx", DEFAULT_HIGH_COLOR_IDX);
   } else {
      highColorIdx = DEFAULT_HIGH_COLOR_IDX;
      prefs.putInt("hColorIdx", highColorIdx);
   }

   if (prefs.isKey("lColorIdx"))
   {
      lostColorIdx = prefs.getInt("lColorIdx", DEFAULT_LOST_COLOR_IDX);
   } else {
      lostColorIdx = DEFAULT_LOST_COLOR_IDX;
      prefs.putInt("lColorIdx", lostColorIdx);
   }

   if (prefs.isKey("wColorIdx"))
   {
      warningColorIdx = prefs.getInt("wColorIdx", DEFAULT_WARNING_COLOR_IDX);
   } else {
      warningColorIdx = DEFAULT_WARNING_COLOR_IDX;
      prefs.putInt("wColorIdx", warningColorIdx);
   }

   prefs.end();

   prefs.begin("debug", false);

   if (prefs.isKey("debugstack"))
   {
      debugStackUsage = prefs.getBool("debugstack", false);
   } else {
      debugStackUsage = false;
      prefs.putBool("debugstack", debugStackUsage);
   }

   if (prefs.isKey("debugenv"))
   {
      debugEnvSensor = prefs.getBool("debugenv", false);
   } else {
      debugEnvSensor = false;
      prefs.putBool("debugenv", debugEnvSensor);
   }

   if (prefs.isKey("debuglight"))
   {
      debugLightSensor = prefs.getBool("debuglight", false);
   } else {
      debugLightSensor = false;
      prefs.putBool("debuglight", debugLightSensor);
   }

   if (prefs.isKey("debugvolts"))
   {
      debugSupplyVoltage = prefs.getBool("debugvolts", false);
   } else {
      debugSupplyVoltage = false;
      prefs.putBool("debugvolts", debugSupplyVoltage);
   }

   if (prefs.isKey("debugsolar"))
   {
      debugSolar = prefs.getBool("debugsolar", false);
   } else {
      debugSolar = false;
      prefs.putBool("debugsolar", debugSolar);
   }

   if (prefs.isKey("debugweather"))
   {
      debugWeather = prefs.getBool("debugweather", false);
   } else {
      debugWeather = false;
      prefs.putBool("debugweather", debugWeather);
   }

   if (prefs.isKey("debugwifi"))
   {
      debugWiFiConnect = prefs.getBool("debugwifi", false);
   } else {
      debugWiFiConnect = false;
      prefs.putBool("debugwifi", debugWiFiConnect);
   }

   if (prefs.isKey("debuggpsinit"))
   {
      debugGPSinitialization = prefs.getBool("debuggpsinit", false);
   } else {
      debugGPSinitialization = false;
      prefs.putBool("debuggpsinit", debugGPSinitialization);
   }

   if (prefs.isKey("debuggpspos"))
   {
      debugGPSposition = prefs.getBool("debuggpspos", false);
   } else {
      debugGPSposition = false;
      prefs.putBool("debuggpspos", debugGPSposition);
   }

   if (prefs.isKey("debuggpstime"))
   {
      debugGPStime = prefs.getBool("debuggpstime", false);
   } else {
      debugGPStime = false;
      prefs.putBool("debuggpstime", debugGPStime);
   }

   if (prefs.isKey("debuggpsalt"))
   {
      debugGPSaltitude = prefs.getBool("debuggpsalt", false);
   } else {
      debugGPSaltitude = false;
      prefs.putBool("debuggpsalt", debugGPSaltitude);
   }

   if (prefs.isKey("debuggpsfix"))
   {
      debugGPSfix = prefs.getBool("debuggpsfix", false);
   } else {
      debugGPSfix = false;
      prefs.putBool("debuggpsfix", debugGPSfix);
   }

   prefs.end();
}  // readSettings()


boolean selectForceDefaults(void)
{
   tft.fillScreen(screenBGColor);                                              // Start with empty screen
   tft.setTextDatum(TC_DATUM);
   tft.setFreeFont(&FreeSerifBold18pt7b);

   tft.setTextColor(normalColor);

   tft.drawString("NTP Clock", 160, 5);

   tft.setTextColor(warningColor);

   tft.setFreeFont(&FreeSerifBold9pt7b);
   tft.drawString("** FORCE ALL SAVED SETTINGS **", 160, 70);
   tft.drawString("** BACK TO DEFAULTS **", 160, 100);

   tft.setTextColor(labelFGColor);

   tft.drawString("RIGHT (WIFI/CONFIG) = CONFIRM", 160, 170);
   tft.drawString("LEFT (BRIGHT) = CANCEL", 160, 140);

   String verStr = String("rev: ") + VERSION_TIMESTAMP;
   tft.drawString(verStr, 160, 226, 2);                                        // put version info on the screen

   tft.setTextColor(warningColor);

#define TIMEOUT_PERIOD 20000                                                   // timeout after 20 seconds of inactivity
   uint32_t startTime = millis();

   while (true)
   {
      if (brightButton.wasPressed())
      {
         if (brightButton.read() == single_click)
         {
            tft.drawString("-- CANCELLED --", 160, 210);

            return (false);
         }
      }

      if (configButton.wasPressed())
      {
         if (configButton.read() == single_click)
         {
            tft.drawString("-- CONFIRMED --", 160, 210);

            return (true);
         }
      }

      if ((millis() - startTime) >= TIMEOUT_PERIOD)
      {
         tft.drawString("--  CANCELLED (TIMED OUT) --", 160, 210);

         return (false);
      }

      delay(10);
   }

   tft.setTextDatum(TL_DATUM);
}  // selectForceDefaults()


int8_t selectGPSResetType(void)
{
   tft.fillScreen(screenBGColor);                                              // Start with empty screen
   tft.setTextDatum(TC_DATUM);
   tft.setFreeFont(&FreeSerifBold18pt7b);

   tft.setTextColor(normalColor);

   tft.drawString("NTP Clock", 160, 5);

   tft.setTextColor(warningColor);

   tft.setFreeFont(&FreeSerifBold9pt7b);
   tft.drawString("** CONTROL GPS **", 160, 70);

   tft.setTextColor(labelFGColor);

   tft.drawString("PRESS LEFT (BRIGHT) TO", 160, 100);
   tft.drawString("ENABLE/DISABLE THE GPS", 160, 120);
   tft.drawString("PRESS RIGHT (WIFI/CONFIG) TO", 160, 150);
   tft.drawString("FACTORY RESET THE GPS", 160, 170);

   String verStr = String("rev: ") + VERSION_TIMESTAMP;
   tft.drawString(verStr, 160, 226, 2);                                        // put version info on the screen

   tft.setTextColor(warningColor);

#define TIMEOUT_PERIOD 20000                                                   // timeout after 20 seconds of inactivity
   uint32_t startTime = millis();

   while (true)
   {
      if (brightButton.wasPressed())
      {
         if (brightButton.read() == single_click)
         {
            if (gpsActiveMode)
            {
               tft.drawString("GPS DISABLE", 160, 210);
            } else {
               tft.drawString("GPS ENABLE", 160, 210);
            }

            for (int i = 0; i < 200; i++)
            {
               delay(10);
            }

            return (0);
         }
      }

      if (configButton.wasPressed())
      {
         if (configButton.read() == single_click)
         {
            tft.drawString("GPS FACTORY RESET", 160, 210);

            for (int i = 0; i < 200; i++)
            {
               delay(10);
            }

            return (1);
         }
      }

      if ((millis() - startTime) >= TIMEOUT_PERIOD)
      {
         tft.drawString("--  CANCELLED (TIMED OUT) --", 160, 210);

         for (int i = 0; i < 200; i++)
         {
            delay(10);
         }

         return (-1);
      }

      delay(10);
   }

   tft.setTextDatum(TL_DATUM);
}  // selectGPSResetType()


boolean selectReboot(void)
{
   tft.fillScreen(screenBGColor);                                              // Start with empty screen
   tft.setTextDatum(TC_DATUM);
   tft.setFreeFont(&FreeSerifBold18pt7b);

   tft.setTextColor(normalColor);

   tft.drawString("NTP Clock", 160, 5);

   tft.setTextColor(warningColor);

   tft.setFreeFont(&FreeSerifBold9pt7b);
   tft.drawString("** FORCE A REBOOT **", 160, 60);
   tft.drawString("- OR -", 160, 85);
   tft.drawString("** SHOW WIFI INFO **", 160, 110);

   tft.setTextColor(labelFGColor);

   tft.drawString("LEFT (BRIGHT) = REBOOT", 160, 150);
   tft.drawString("RIGHT (WIFI/CONFIG) = WIFI INFO", 160, 180);

   String verStr = String("rev: ") + VERSION_TIMESTAMP;
   tft.drawString(verStr, 160, 226, 2);                                        // put version info on the screen

   tft.setTextColor(warningColor);

#define TIMEOUT_PERIOD 20000                                                   // timeout after 20 seconds of inactivity
   uint32_t startTime = millis();

   while (true)
   {
      if (configButton.wasPressed())
      {
         if (configButton.read() == single_click)
         {
            tft.drawString("-- CANCELLED --", 160, 210);

            return (false);
         }
      }

      if (brightButton.wasPressed())
      {
         if (brightButton.read() == single_click)
         {
            tft.drawString("-- CONFIRMED --", 160, 210);

            return (true);
         }
      }

      if ((millis() - startTime) >= TIMEOUT_PERIOD)
      {
         tft.drawString("--  CANCELLED (TIMED OUT) --", 160, 210);

         return (false);
      }

      delay(10);
   }

   tft.setTextDatum(TL_DATUM);
}  // selectReboot()


void setup(void)
{
   uint32_t startWaitTime = millis();

   Serial.begin(115200);

   while ((!Serial) && ((millis() - startWaitTime) <= 3000))                   // wait a maximum of 3 seconds for serial to become active
   {
      delay(10);
   }

   analogReadResolution(10);

   Serial.println("NTP Dual Clock v4.x");

   strcpy(apSSID, DEFAULT_AP_NAME);

   // uncomment this line to reset all settings to defaults (e.g. WiFi definitions, etc.)
   //   forceDefaults(false);

   readSettings();                                                             // Read saved settings (except WiFi info)

   startButtonLoopTask();

   colorUpdateNeeded = false;

   screenBGColor_day = Colors[screenBGColorIdx].color;
   screenBGColor = screenBGColor_day;

   timeColor_day = Colors[timeColorIdx].color;
   timeColor = timeColor_day;

   dateColor_day = Colors[dateColorIdx].color;
   dateColor = dateColor_day;

   labelFGColor_day = Colors[labelFGColorIdx].color;
   labelFGColor = labelFGColor_day;

   labelBGColor_day = Colors[labelBGColorIdx].color;
   labelBGColor = labelBGColor_day;

   edgeColor_day = Colors[edgeColorIdx].color;
   edgeColor = edgeColor_day;

   normalColor_day = Colors[normalColorIdx].color;
   normalColor = normalColor_day;

   mediumColor_day = Colors[mediumColorIdx].color;
   mediumColor = mediumColor_day;

   highColor_day = Colors[highColorIdx].color;
   highColor = highColor_day;

   lostColor_day = Colors[lostColorIdx].color;
   lostColor = lostColor_day;

   warningColor_day = Colors[warningColorIdx].color;
   warningColor = warningColor_day;

   switchDisplayMode();
   newDualScreen();

   pinMode(BACKLIGHT_PIN, OUTPUT);
   analogWrite(BACKLIGHT_PIN, (brightness * 32) + 15);                         // Set brightness

   pinMode(LED_PIN, OUTPUT);
   digitalWrite(LED_PIN, LOW);

#ifdef RGB_BUILTIN
   rgbLedWrite(RGB_BUILTIN, 0, 0, 0);                                          // blink OFF
#endif

#ifdef LED_BUILTIN
   digitalWrite(LED_BUILTIN, LOW);                                             // blink OFF
#endif

#if defined(ESP32S3_SUPERMINI) || defined(ESP32S3_MJC_TESTBED)

   // Start I2C with defined SDA/SCL pins
   Wire.begin(I2C_SDA, I2C_SCL);

#endif

   tft.init();
   tft.setRotation(SCREEN_ORIENTATION);

   configButton.setDoubleClickTime(500);
   configButton.setLongClickTime(2000);
   configButton.setDebounceTime(10);
   configButton.begin(CONFIG_PIN);

   brightButton.setDoubleClickTime(500);
   brightButton.setLongClickTime(2000);
   brightButton.setDebounceTime(10);
   brightButton.begin(BRIGHT_PIN);

   showSplash();
   for (uint8_t i = 0; i < 200; i++)
   {
      delay(10);                                                               // Time to read the SPLASH screen info
   }

   if (needAppIDFlag)
   {
      showAppIDNagScreen();

      for (int i = 0; i < 1000; i++)
      {
         delay(10);
      }

      switchDisplayMode();
      newDualScreen();
   }

   setDebug(EZTIME_DEBUGLEVEL);                                                // enable NTP debug info

   netInit();

   configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");                   // Set time via NTP, as required for x.509 validation

   // determine which TZ to display first
   if (showESTEDT)
   {
      tzIndex = SHOW_TZ_EST_EDT;
   } else {
      if (showCSTCDT)
      {
         tzIndex = SHOW_TZ_CST_CDT;
      } else {
         if (showMSTMDT)
         {
            tzIndex = SHOW_TZ_MST_MDT;
         } else {
            if (showMSTNOMDT)
            {
               tzIndex = SHOW_TZ_MST_NO_MDT;
            } else {
               if (showPSTPDT)
               {
                  tzIndex = SHOW_TZ_PST_PDT;
               } else {
                  if (showAWST)
                  {
                     tzIndex = SHOW_TZ_AWST;
                  } else {
                     if (showACSTACDT)
                     {
                        tzIndex = SHOW_TZ_ACST_ACDT;
                     } else {
                        if (showAESTAEDT)
                        {
                           tzIndex = SHOW_TZ_AEST_AEDT;
                        } else {
                           if (showGMTBST)
                           {
                              tzIndex = SHOW_TZ_GMT_BST;
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }

   local.setPosix(timeZones[int(tzIndex)]);                                    // Set local time zone

   Serial.println("");
   Serial.println("BME280 test");

#if defined(ESP32S3_SUPERMINI) || defined(ESP32S3_MJC_TESTBED)

   Wire.setPins(I2C_SDA, I2C_SCL);

#endif

   BME280_available = bme.begin();                                             // Attempt to start it up on the default address

   if (!BME280_available)
   {
      BME280_available = bme.begin(BME280_ADDRESS_ALTERNATE);                  // Attempt to start it up on the alternate address

      if (!BME280_available)
      {
         Serial.println("BME280 sensor NOT found at either address");
      } else {
         Serial.print("BME280 sensor found at 0x");
         Serial.println(BME280_ADDRESS_ALTERNATE, HEX);
      }
   } else {
      Serial.println("BME280 sensor found at ");
      Serial.println(BME280_ADDRESS, HEX);
   }

   Serial.println("");

   Serial.println("");
   Serial.println("BMP280 test");

   BMP280_available = bmp.begin();                                             // Attempt to start it up on the default address

   if (!BMP280_available)
   {
      BMP280_available = bmp.begin(BMP280_ADDRESS_ALT);                        // Attempt to start it up on the alternate address

      if (!BMP280_available)
      {
         Serial.println("BMP280 sensor NOT found at either address");
      } else {
         Serial.print("BMP280 sensor found at 0x");
         Serial.println(BMP280_ADDRESS_ALT, HEX);
      }
   } else {
      Serial.print("BMP280 sensor found at 0x");
      Serial.println(BMP280_ADDRESS, HEX);
   }

   Serial.println("");

   if (debugStackUsage)
   {
      uint32_t freeStack = uxTaskGetStackHighWaterMark(NULL);
      Serial.print("Minimum remaining stack space: ");
      Serial.println(freeStack);
      Serial.print("Heap available: ");
      Serial.println(ESP.getFreeHeap());
   }

   showButtonDefinitions();
} // setup()


void show12VDC(uint16_t x, uint16_t y)
{
   static char ps_twelve_dc[8] = { 0x00 };
   char *endPtr;
   float twelveVoltCorrectionFloat = strtof(twelveVoltCorrection.c_str(), &endPtr);

   if (INPUT_12VDC_PIN != -1)
   {
      tft.setTextDatum(TC_DATUM);

      float twelveVoltAverage = 0.0;

      for (int i = 0; i < NUMBER_OF_VOLTAGE_AVERAGES; i++)
      {
         twelveVoltAverage += ((((float)analogRead(INPUT_12VDC_PIN) / 1024.0) * (110.0 / 10.0) * 3.3) * twelveVoltCorrectionFloat);
         delay(10);
      }

      twelveVoltAverage /= (float)NUMBER_OF_VOLTAGE_AVERAGES;

      tft.fillRect(x - 22, y + 3, 45, 10, labelBGColor);

      if (twelveVoltDisplay)
      {
         tft.setTextColor(labelFGColor, labelBGColor);

         sprintf(ps_twelve_dc, "%.1f%c", ((round(twelveVoltAverage * 10.0)) / 10.0), 'V');

         tft.drawString(ps_twelve_dc, x, y, 2);

         if (debugSupplyVoltage)
         {
            sprintf(ps_twelve_dc, "%.3f%c", ((round((twelveVoltAverage / twelveVoltCorrectionFloat) * 1000.0)) / 1000.0), 'V');

            Serial.print("12VDC supply measured:  ");
            Serial.println(ps_twelve_dc);

            sprintf(ps_twelve_dc, "%.3f%c", ((round(twelveVoltAverage * 1000.0)) / 1000.0), 'V');

            Serial.print("12VDC supply displayed: ");
            Serial.println(ps_twelve_dc);
         }
      }

      tft.setTextDatum(TL_DATUM);
   }
}  // show12VDC()


void show5VDC(uint16_t x, uint16_t y)
{
   static char ps_five_dc[8] = { 0x00 };
   char *endPtr;
   float fiveVoltCorrectionFloat = strtof(fiveVoltCorrection.c_str(), &endPtr);

   if (INPUT_5VDC_PIN != -1)
   {
      tft.setTextDatum(TC_DATUM);

      float fiveVoltAverage = 0.0;

      for (int i = 0; i < NUMBER_OF_VOLTAGE_AVERAGES; i++)
      {
         fiveVoltAverage += ((((float)analogRead(INPUT_5VDC_PIN) / 1024.0) * (127.0 / 27.0) * 3.3) * fiveVoltCorrectionFloat);
         delay(10);
      }

      fiveVoltAverage /= (float)NUMBER_OF_VOLTAGE_AVERAGES;

      tft.fillRect(x - 17, y + 3, 35, 10, labelBGColor);

      if (fiveVoltDisplay)
      {
         sprintf(ps_five_dc, "%.1f%c", ((round(fiveVoltAverage * 10.0)) / 10.0), 'V');

         tft.setTextColor(labelFGColor, labelBGColor);

         tft.drawString(ps_five_dc, x, y, 2);

         if (debugSupplyVoltage)
         {
            sprintf(ps_five_dc, "%.3f%c", ((round((fiveVoltAverage / fiveVoltCorrectionFloat) * 1000.0)) / 1000.0), 'V');

            Serial.print("5VDC supply measured:   ");
            Serial.println(ps_five_dc);

            sprintf(ps_five_dc, "%.3f%c", ((round(fiveVoltAverage * 1000.0)) / 1000.0), 'V');

            Serial.print("5VDC supply displayed:  ");
            Serial.println(ps_five_dc);
         }
      }

      tft.setTextDatum(TL_DATUM);
   }
}  // show5VDC()


void showAltitude(void)
{
   String headings;
   String altitudeStr;

   if (useMetric)
   {
      char *endPtr;
      float altitudeInFeetFloat = strtof(qthAltitudeInFeet.c_str(), &endPtr);
      float altitudeInMetersFloat = altitudeInFeetFloat * 12.0 / NUMBER_OF_INCHES_PER_METER;

      char altitudeInMetersStr[16] = { 0x00 };
      sprintf(altitudeInMetersStr, "%.2f", (float)((round(altitudeInMetersFloat * 100.0)) / 100.0));

      headings    = "Alt (m):";
      altitudeStr = (String)altitudeInMetersStr;
   } else {
      headings    = "Alt (ft):";
      altitudeStr = qthAltitudeInFeet;
   }

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (altitudeStr, 160, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showAltitude()


void showAppIDNagScreen(void)
{
   tft.fillScreen(screenBGColor);
   tft.setTextDatum(TC_DATUM);
   tft.setFreeFont(&FreeSerifBold18pt7b);

   tft.setTextColor(labelFGColor);

   tft.drawString(F("NTP Clock"), 160, 5);
   tft.drawString(F("Accessing Internet Weather"), 160, 40, 2);
   tft.drawString(F("requires a (free) API Key."), 160, 60, 2);
   tft.drawString(F("Create an account (free) at:"), 160, 80, 2);
   tft.drawString(F("home.openweathermap.org/users/sign_up"), 160, 110, 2);
   tft.drawString(F("Login to your new account"), 160, 140, 2);
   tft.drawString(F("and request your (free) API Key,"), 160, 160, 2);
   tft.drawString(F("then use this clock's web interface"), 160, 180, 2);
   tft.drawString(F("to confgure it with your new API Key"), 160, 200, 2);

   String verStr = String("rev: ") + VERSION_TIMESTAMP;
   tft.drawString(verStr, 160, 226, 2);                                        // put version info on the screen

   tft.setTextDatum(TL_DATUM);
}  // showAppIDNagScreen()


void showAUR(void)                                                             // Aurora level
{
   String headings = "AUR:          BZ:";

   String aur = getXmlData(xmlData, "aurora");
   String bz  = getXmlData(xmlData, "magneticfield");

   tft.setTextColor(labelFGColor, labelBGColor);
   tft.drawString(headings, 80, 123, 4);

   tft.setTextColor(normalColor, labelBGColor);

   tft.drawString(aur, 150, 123, 4);
   tft.drawString(bz,  235, 123, 4);
}  // showAUR()


void showButtonDefinitions(void)
{
   tft.fillScreen(screenBGColor);                                              // Start with empty screen
   tft.setTextDatum(TC_DATUM);
   tft.setFreeFont(&FreeSerifBold18pt7b);

   tft.setTextColor(normalColor);

   tft.drawString(F("NTP Clock"), 160, 0);

   tft.setTextColor(warningColor);

   tft.drawString(F("Left = BRIGHT"), 160, 35, 2);
   tft.drawString(F("Right = WIFI/CONFIG"), 160, 50, 2);

   tft.setTextDatum(TL_DATUM);

   tft.setTextColor(labelFGColor);

   tft.drawString(F("Single-click"), 50, 70, 2);
   tft.drawString(F("Brightness Level"), 160, 70, 2);
   tft.drawString(F("Double-click"), 50, 85, 2);
   tft.drawString(F("Day/Night Mode"), 160, 85, 2);
   tft.drawString(F("Triple-click"), 50, 100, 2);
   tft.drawString(F("Force Defaults"), 160, 100, 2);
   tft.drawString(F("Long-click"), 50, 115, 2);
   tft.drawString(F("GPS Control"), 160, 115, 2);

   tft.drawString(F("Single-click"), 50, 140, 2);
   tft.drawString(F("Data Display Select"), 160, 140, 2);
   tft.drawString(F("Double-click"), 50, 155, 2);
   tft.drawString(F("Metric/Imperial Units"), 160, 155, 2);
   tft.drawString(F("Triple-click"), 50, 170, 2);
   tft.drawString(F("WiFi Auto/Manual Mode"), 160, 170, 2);
   tft.drawString(F("Long-click"), 50, 185, 2);
   tft.drawString(F("Reboot/WiFi Info"), 160, 185, 2);

   String verStr = String("rev: ") + VERSION_TIMESTAMP;
   tft.setTextDatum(TC_DATUM);
   tft.drawString(verStr, 160, 226, 2);                                        // put version info on the screen

   tft.setTextColor(warningColor);

   tft.setTextDatum(TL_DATUM);
   tft.drawString(F("Left"), 10, 70, 2);
   tft.drawString(F("Right"), 10, 140, 2);

   tft.setTextDatum(TC_DATUM);
   tft.drawString(F("Press either button to resume..."), 160, 210, 2);

#define TIMEOUT_PERIOD 20000                                                   // timeout after 20 seconds of inactivity
   uint32_t startTime = millis();

   while (true)
   {
      if (configButton.isPressed())
      {
         while (configButton.isPressed())
         {
            delay(10);
         }

         configButton.resetPressedState();
         brightButton.resetPressedState();

         break;
      }

      if (brightButton.isPressed())
      {
         while (brightButton.isPressed())
         {
            delay(10);
         }

         configButton.resetPressedState();
         brightButton.resetPressedState();

         break;
      }

#ifndef DISABLE_BUTTON_DEF_TIMEOUT

      if ((millis() - startTime) >= TIMEOUT_PERIOD)
      {
         break;
      }

#endif

      delay(10);
   }

   newDualScreen();

   tft.setTextDatum(TL_DATUM);
}  // showButtonDefinitions()


void showCond(void)
{
   String headings = "Cond:";

   String cond = weatherConditions;

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (cond, 160, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showCond()


void showDataItems()
{
   if (showModeFlag)
   {
      showModeFlag = false;

      switch (showMode)
      {
         case SHOW_MODE_NONE:
            {
               tft.drawString("Data: None", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_ONLY:
            {
               tft.drawString("Data: S", 80, 123, 4);
            }
            break;

         case SHOW_MODE_ENV_ONLY:
            {
               tft.drawString("Data: E", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_ENV:
            {
               tft.drawString("Data: S+E", 80, 123, 4);
            }
            break;

         case SHOW_MODE_WEATHER_ONLY:
            {
               tft.drawString("Data: W", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_WEATHER:
            {
               tft.drawString("Data: S+W", 80, 123, 4);
            }
            break;

         case SHOW_MODE_ENV_PLUS_WEATHER:
            {
               tft.drawString("Data: E+W", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER:
            {
               tft.drawString("Data: S+E+W", 80, 123, 4);
            }
            break;

         case SHOW_MODE_MLS_ONLY:
            {
               tft.drawString("Data: M", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_MLS:
            {
               tft.drawString("Data: S+M", 80, 123, 4);
            }
            break;

         case SHOW_MODE_ENV_PLUS_MLS:
            {
               tft.drawString("Data: E+M", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS:
            {
               tft.drawString("Data: S+E+M", 80, 123, 4);
            }
            break;

         case SHOW_MODE_WEATHER_PLUS_MLS:
            {
               tft.drawString("Data: W+M", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS:
            {
               tft.drawString("Data: S+W+M", 80, 123, 4);
            }
            break;

         case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS:
            {
               tft.drawString("Data: E+W+M", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS:
            {
               tft.drawString("Data: S+E+W+M", 80, 123, 4);
            }
            break;

         case SHOW_MODE_GPS_ONLY:
            {
               tft.drawString("Data: G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_GPS:
            {
               tft.drawString("Data: S+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_ENV_PLUS_GPS:
            {
               tft.drawString("Data: E+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_GPS:
            {
               tft.drawString("Data: S+E+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_WEATHER_PLUS_GPS:
            {
               tft.drawString("Data: W+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_GPS:
            {
               tft.drawString("Data: S+W+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_GPS:
            {
               tft.drawString("Data: E+W+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_GPS:
            {
               tft.drawString("Data: S+E+W+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_MLS_PLUS_GPS:
            {
               tft.drawString("Data: M+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_MLS_PLUS_GPS:
            {
               tft.drawString("Data: S+M+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_ENV_PLUS_MLS_PLUS_GPS:
            {
               tft.drawString("Data: E+M+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS_PLUS_GPS:
            {
               tft.drawString("Data: S+E+M+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_WEATHER_PLUS_MLS_PLUS_GPS:
            {
               tft.drawString("Data: W+M+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
            {
               tft.drawString("Data: S+W+M+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
            {
               tft.drawString("Data: E+W+M+G", 80, 123, 4);
            }
            break;

         case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
            {
               tft.drawString("Data: S+E+W+M+G", 80, 123, 4);
            }
            break;
      }
   } else {
      if (showMode != SHOW_MODE_NONE)
      {
         switch (dataIndex)
         {
            case SHOW_DATA_SFI:
               {
                  showSFI();
               }
               break;

            case SHOW_DATA_GMF:
               {
                  showGMF();
               }
               break;

            case SHOW_DATA_S2N:
               {
                  showS2N();
               }
               break;

            case SHOW_DATA_AUR:
               {
                  showAUR();
               }
               break;

            case SHOW_DATA_SSN:
               {
                  showSSN();
               }
               break;

            case SHOW_DATA_ENV:
               {
                  showEnv();

                  if (showMode == SHOW_MODE_ENV_ONLY)
                  {
                     dataIndex = SHOW_DATA_TYPE(dataIndex - 1);
                  }
               }
               break;

            case SHOW_DATA_LOC:
               {
                  showLoc();
               }
               break;

            case SHOW_DATA_COND:
               {
                  showCond();
               }
               break;

            case SHOW_DATA_TEMP:
               {
                  showTemp();
               }
               break;

            case SHOW_DATA_RH:
               {
                  showRH();
               }
               break;

            case SHOW_DATA_WIND:
               {
                  showWind();
               }
               break;

            case SHOW_DATA_SUNRISE:
               {
                  showSunrise();
               }
               break;

            case SHOW_DATA_SUNSET:
               {
                  showSunset();
               }
               break;

            case SHOW_DATA_MLS:
               {
                  showMLS();

                  if (showMode == SHOW_MODE_MLS_ONLY)
                  {
                     dataIndex = SHOW_DATA_TYPE(dataIndex - 1);
                  }
               }
               break;

            case SHOW_DATA_LAT:
               {
                  showLat();
               }
               break;

            case SHOW_DATA_LON:
               {
                  showLon();
               }
               break;

            case SHOW_DATA_ALTITUDE:
               {
                  showAltitude();
               }
               break;

            default:
               {
               }
               break;
         }

         dataIndex = SHOW_DATA_TYPE(dataIndex + 1);
         if (dataIndex >= SHOW_DATA_END_MARKER)
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;
         }

         switch (showMode)
         {
            case SHOW_MODE_NONE:
               {
               }
               break;

            case SHOW_MODE_SOLAR_ONLY:
               {
                  if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_SOLAR_DATA))
                  {
                     dataIndex = SHOW_FIRST_SOLAR_DATA;
                  }
               }
               break;

            case SHOW_MODE_ENV_ONLY:
               {
                  dataIndex = SHOW_DATA_ENV;
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_ENV:
               {
                  if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_ENV))
                  {
                     dataIndex = SHOW_DATA_ENV;
                  } else {
                     if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_DATA_ENV))
                     {
                        dataIndex = SHOW_FIRST_SOLAR_DATA;
                     }
                  }
               }
               break;

            case SHOW_MODE_WEATHER_ONLY:
               {
                  if ((dataIndex < SHOW_FIRST_WEATHER_DATA) || (dataIndex > SHOW_LAST_WEATHER_DATA))
                  {
                     dataIndex = SHOW_FIRST_WEATHER_DATA;
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_WEATHER:
               {
                  if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                  {
                     dataIndex = SHOW_FIRST_WEATHER_DATA;
                  } else {
                     if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_WEATHER_DATA))
                     {
                        dataIndex = SHOW_FIRST_SOLAR_DATA;
                     }
                  }
               }
               break;

            case SHOW_MODE_ENV_PLUS_WEATHER:
               {
                  if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                  {
                     dataIndex = SHOW_FIRST_WEATHER_DATA;
                  } else {
                     if ((dataIndex < SHOW_DATA_ENV) || (dataIndex > SHOW_LAST_WEATHER_DATA))
                     {
                        dataIndex = SHOW_DATA_ENV;
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER:
               {
                  if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_ENV))
                  {
                     dataIndex = SHOW_DATA_ENV;
                  } else {
                     if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                     {
                        dataIndex = SHOW_FIRST_WEATHER_DATA;
                     } else {
                        if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_WEATHER_DATA))
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_MLS_ONLY:
               {
                  dataIndex = SHOW_DATA_MLS;
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_MLS:
               {
                  if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_MLS))
                  {
                     dataIndex = SHOW_DATA_MLS;
                  } else {
                     if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_FIRST_SOLAR_DATA;
                     }
                  }
               }
               break;

            case SHOW_MODE_ENV_PLUS_MLS:
               {
                  if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_DATA_MLS))
                  {
                     dataIndex = SHOW_DATA_MLS;
                  } else {
                     if ((dataIndex < SHOW_DATA_ENV) || (dataIndex > SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_ENV;
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS:
               {
                  if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_ENV))
                  {
                     dataIndex = SHOW_DATA_ENV;
                  } else {
                     if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_DATA_MLS))
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_WEATHER_PLUS_MLS:
               {
                  if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_MLS))
                  {
                     dataIndex = SHOW_DATA_MLS;
                  } else {
                     if ((dataIndex < SHOW_FIRST_WEATHER_DATA) || (dataIndex > SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_FIRST_WEATHER_DATA;
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS:
               {
                  if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                  {
                     dataIndex = SHOW_FIRST_WEATHER_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_DATA_MLS))
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS:
               {
                  if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                  {
                     dataIndex = SHOW_FIRST_WEATHER_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex < SHOW_DATA_ENV) || (dataIndex > SHOW_DATA_MLS))
                        {
                           dataIndex = SHOW_DATA_ENV;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS:
               {
                  if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_MLS))
                  {
                     dataIndex = SHOW_DATA_MLS;
                  } else {
                     if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                     {
                        dataIndex = SHOW_FIRST_WEATHER_DATA;
                     } else {
                        if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_MLS))
                        {
                           dataIndex = SHOW_DATA_MLS;
                        } else {
                           if ((dataIndex < SHOW_DATA_ENV) || (dataIndex > SHOW_DATA_MLS))
                           {
                              dataIndex = SHOW_DATA_ENV;
                           }
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_GPS_ONLY:
               {
                  if ((dataIndex < SHOW_FIRST_GPS_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_FIRST_SOLAR_DATA;
                     }
                  }
               }
               break;

            case SHOW_MODE_ENV_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex < SHOW_DATA_ENV) || (dataIndex > SHOW_LAST_GPS_DATA))
                     {
                        dataIndex = SHOW_DATA_ENV;
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_ENV))
                  {
                     dataIndex = SHOW_DATA_ENV;
                  } else {
                     if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_GPS_DATA))
                     {
                        dataIndex = SHOW_FIRST_GPS_DATA;
                     } else {
                        if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_WEATHER_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex < SHOW_FIRST_WEATHER_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                     {
                        dataIndex = SHOW_FIRST_WEATHER_DATA;
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                     {
                        dataIndex = SHOW_FIRST_WEATHER_DATA;
                     } else {
                        if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                     {
                        dataIndex = SHOW_FIRST_WEATHER_DATA;
                     } else {
                        if ((dataIndex < SHOW_DATA_ENV) || (dataIndex > SHOW_LAST_GPS_DATA))
                        {
                           dataIndex = SHOW_DATA_ENV;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                     {
                        dataIndex = SHOW_FIRST_WEATHER_DATA;
                     } else {
                        if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_ENV))
                        {
                           dataIndex = SHOW_DATA_ENV;
                        } else {
                           if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                           {
                              dataIndex = SHOW_FIRST_SOLAR_DATA;
                           }
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_MLS_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_MLS) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex < SHOW_DATA_MLS) || (dataIndex > SHOW_LAST_GPS_DATA))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_MLS_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_MLS) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                        {
                           dataIndex = SHOW_FIRST_SOLAR_DATA;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_ENV_PLUS_MLS_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_MLS) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex < SHOW_DATA_ENV) || (dataIndex > SHOW_LAST_GPS_DATA))
                        {
                           dataIndex = SHOW_DATA_ENV;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_MLS) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_ENV))
                        {
                           dataIndex = SHOW_DATA_ENV;
                        } else {
                           if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                           {
                              dataIndex = SHOW_FIRST_SOLAR_DATA;
                           }
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_WEATHER_PLUS_MLS_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_MLS) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_ENV))
                     {
                        dataIndex = SHOW_DATA_ENV;
                     } else {
                        if ((dataIndex < SHOW_FIRST_WEATHER_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                        {
                           dataIndex = SHOW_FIRST_WEATHER_DATA;
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_MLS) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                        {
                           dataIndex = SHOW_FIRST_WEATHER_DATA;
                        } else {
                           if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                           {
                              dataIndex = SHOW_FIRST_SOLAR_DATA;
                           }
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_MLS) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                        {
                           dataIndex = SHOW_FIRST_WEATHER_DATA;
                        } else {
                           if ((dataIndex < SHOW_DATA_ENV) || (dataIndex > SHOW_LAST_GPS_DATA))
                           {
                              dataIndex = SHOW_DATA_ENV;
                           }
                        }
                     }
                  }
               }
               break;

            case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
               {
                  if ((dataIndex > SHOW_DATA_MLS) && (dataIndex < SHOW_FIRST_GPS_DATA))
                  {
                     dataIndex = SHOW_FIRST_GPS_DATA;
                  } else {
                     if ((dataIndex > SHOW_LAST_WEATHER_DATA) && (dataIndex < SHOW_DATA_MLS))
                     {
                        dataIndex = SHOW_DATA_MLS;
                     } else {
                        if ((dataIndex > SHOW_DATA_ENV) && (dataIndex < SHOW_FIRST_WEATHER_DATA))
                        {
                           dataIndex = SHOW_FIRST_WEATHER_DATA;
                        } else {
                           if ((dataIndex > SHOW_LAST_SOLAR_DATA) && (dataIndex < SHOW_DATA_ENV))
                           {
                              dataIndex = SHOW_DATA_ENV;
                           } else {
                              if ((dataIndex < SHOW_FIRST_SOLAR_DATA) || (dataIndex > SHOW_LAST_GPS_DATA))
                              {
                                 dataIndex = SHOW_FIRST_SOLAR_DATA;
                              }
                           }
                        }
                     }
                  }
               }
               break;
         }

         showModeFlag = false;

         if (showDataHeaderBetweenDataGroups)
         {
            if ((dataIndex == SHOW_DATA_ENV) && (showMode != SHOW_MODE_ENV_ONLY))
            {
               showModeFlag = true;
            }

            if ((dataIndex == SHOW_FIRST_SOLAR_DATA) && (showMode != SHOW_MODE_SOLAR_ONLY))
            {
               showModeFlag = true;
            }

            if ((dataIndex == SHOW_FIRST_WEATHER_DATA) && (showMode != SHOW_MODE_WEATHER_ONLY))
            {
               showModeFlag = true;
            }

            if ((dataIndex == SHOW_DATA_MLS) && (showMode != SHOW_MODE_MLS_ONLY))
            {
               showModeFlag = true;
            }

            if ((dataIndex == SHOW_FIRST_GPS_DATA) && (showMode != SHOW_MODE_GPS_ONLY))
            {
               showModeFlag = true;
            }
         }
      }
   }
}  // showDataItems()


void showEnv(void)
{
   char *endPtr;
   float tempOffsetFloat = strtof(temperatureOffsetInCelsius.c_str(), &endPtr);
   float qthAltitudeInFeetFloat = strtof(qthAltitudeInFeet.c_str(), &endPtr);

   if (BME280_available)
   {
      float tempC = bme.readTemperature() + tempOffsetFloat;

      float tempF;
      float humidity = bme.readHumidity();
      float pressure = bme.readPressure() / 100.0;

      String heading = "E: ";

      pressure += qthAltitudeInFeetFloat / 33.864;

      tft.setTextColor(labelFGColor, labelBGColor);

      tft.drawString(heading, 80, 123, 4);

      tft.drawString("o", 156, 119, 2);
      tft.drawString("%H", 208, 122, 2);

      if (useMetric)
      {
         tft.drawString("C", 166, 122, 2);
         tft.drawString("hPa", 291, 122, 2);
         tft.setTextColor(normalColor, labelBGColor);

         tft.drawFloat(tempC, 0, 125, 123, 4);

         pressure = pressure + DEFAULT_METRIC_PRESSURE_OFFSET;

         if (pressure < 1000)
         {
            tft.drawNumber(pressure, 240, 123, 4);
         } else {
            tft.drawNumber(pressure, 230, 123, 4);
         }
      } else {
         tft.drawString("F", 166, 122, 2);
         tft.drawString("in", 300, 122, 2);
         tft.setTextColor(normalColor, labelBGColor);

         tempF = ((tempC * 1.8) + 32);
         pressure *= 0.02953;
         pressure += (DEFAULT_METRIC_PRESSURE_OFFSET / 30);

         tft.drawFloat(pressure, 2, 235, 123, 4);

         if (tempF < 100)
         {
            tft.drawFloat(tempF, 0, 125, 123, 4);
         } else
         {
            tft.drawFloat(tempF, 0, 113, 123, 4);
         }
      }

      humidity += humidityOffset;
      if (humidity > 99)
      {
         humidity = 99;                                                        // limit to 2 digits max
      }
      tft.drawNumber(humidity, 176, 123, 4);
   }

   if (BMP280_available)
   {
      float tempC = bmp.readTemperature() + tempOffsetFloat;

      float tempF;
      float pressure = bmp.readPressure() / 100.0F;

      String  heading = "E: ";

      pressure += qthAltitudeInFeetFloat / 33.864;

      tft.setTextColor(labelFGColor, labelBGColor);

      tft.drawString(heading, 80, 123, 4);

      tft.drawString("o", 156, 119, 2);

      if (useMetric)
      {
         tft.drawString("C", 166, 122, 2);
         tft.drawString("hPa", 291, 122, 2);
         tft.setTextColor(normalColor, labelBGColor);

         tft.drawFloat(tempC, 0, 125, 123, 4);

         pressure += DEFAULT_METRIC_PRESSURE_OFFSET;

         if (pressure < 1000)
         {
            tft.drawNumber(pressure, 240, 123, 4);
         } else
         {
            tft.drawNumber(pressure, 230, 123, 4);
         }
      } else {
         tft.drawString("F", 166, 122, 2);
         tft.drawString("in", 300, 122, 2);
         tft.setTextColor(normalColor, labelBGColor);

         tempF = ((tempC  * 1.8) + 32);
         pressure *= 0.02953;
         pressure += (DEFAULT_METRIC_PRESSURE_OFFSET / 30);

         tft.drawFloat(pressure, 2, 235, 123, 4);

         if (tempF < 100)
         {
            tft.drawFloat(tempF, 0, 125, 123, 4);
         } else {
            tft.drawFloat(tempF, 0, 113, 123, 4);
         }
      }
   }

   if (!BME280_available && !BMP280_available)
   {
      tft.drawString("(ENV unavailable)", 80, 123, 4);
   }
}  // showEnv()


void showGMF(void)
{
   String headings = "GMF:";

   String gmf = getXmlData(xmlData, "geomagfield");

   tft.setTextColor(labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (gmf, 150, 123, 4);
}  // showGMF()


void showGPSStatus(uint16_t x, uint16_t y)
{
   if (gpsActiveMode)
   {
      tft.setTextDatum(TC_DATUM);

      tft.fillRect(x, y + 1, 50, 15, labelBGColor);                            // erase any previous status

      tft.setTextColor(labelFGColor, labelBGColor);

      tft.drawString("G:", x, y, 2);

      switch (gpsFixState)
      {
         case 2:   // 2D fix
            {
               tft.drawString("2D", x + 15, y, 2);
            }
            break;

         case 3:   // 3D fix
            {
               tft.setTextColor(normalColor, labelBGColor);
               tft.drawString("3D", x + 15, y, 2);
            }
            break;

         default:
            {
               tft.drawString("--", x + 15, y, 2);
            }
            break;
      }

      if ((gpsSatelliteCount > 0) && (gpsActiveMode))
      {
         char countStr[8];
         sprintf(countStr, "/%02d", gpsSatelliteCount);
         tft.drawString(countStr, x + 35, y, 2);
      }

      tft.setTextDatum(TL_DATUM);
   }
} // showGPSStatus()


void showMLS(void)
{
   char *endPtr;
   float gpsLatFloat = strtof(gpsManualLat.c_str(), &endPtr);
   float gpsLonFloat = strtof(gpsManualLon.c_str(), &endPtr);

   char mls[16] = { 0x00 };

   String headings = "MLS:";


   int latZone = (int)((gpsLatFloat + 90) / 10);
   int lonZone = (int)((gpsLonFloat + 180) / 20);

   mls[0] = 'A' + lonZone;
   mls[1] = 'A' + latZone;

   float latSub = (gpsLatFloat + 90) - (latZone * 10);
   float lonSub = (gpsLonFloat + 180) - (lonZone * 20);

   mls[2] = '0' + (int)(lonSub / 2);
   mls[3] = '0' + (int)(latSub);

   mls[4] = 'a' + (int)((lonSub - (mls[2] - '0') * 2) * 12);
   mls[5] = 'a' + (int)((latSub - (mls[3] - '0')) * 24);

   mls[6] = '\0';

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (mls, 150, 123, 4);
}  // showMLS()


void showLat(void)
{
   String headings = "Lat:";

   String lat = gpsManualLat;

   if ((lat.indexOf('.') > 0) && (lat.length() > (lat.indexOf('.') + (weatherLatLonDecimalPlaces + 1))))
   {
      lat.setCharAt(lat.indexOf('.') + (weatherLatLonDecimalPlaces + 1), 0x00);
   }

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (lat, 140, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showLat()


void showLoc(void)
{
   String loc = String();

   if (weatherLocation[0])
   {
      loc = weatherLocation;
   } else {
      if (getWiFiStatus() != WIFI_NET_CONNECTED)
      {
         loc = "(Wx unavailable)";
      } else {
         loc = "(Wx updating)";
      }
      dataIndex = SHOW_LAST_WEATHER_DATA;
   }

   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (loc, 80, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showLoc()


void showLon(void)
{
   String headings = "Lon:";

   String lon = gpsManualLon;

   if ((lon.indexOf('.') > 0) && (lon.length() > (lon.indexOf('.') + (weatherLatLonDecimalPlaces + 1))))
   {
      lon.setCharAt(lon.indexOf('.') + (weatherLatLonDecimalPlaces + 1), 0x00);
   }

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (lon, 140, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showLon()


void showNextData(void)
{
   if (++showModeDelayCount >= CYCLE_TIME_IN_SECONDS)
   {
      showModeDelayCount = 0;

      clearDataArea();                                                         // Erase any previous data
      tft.setTextColor (labelFGColor, labelBGColor);

      showDataItems();
   }
}  // showNextData()


void showRH(void)
{
   String headings = "RH:";

   String rh = String(weatherHumidity) + "%";

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (rh, 140, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showRH()


void showS2N(void)
{
   String headings = "S2N:";

   String s2n = getXmlData(xmlData, "signalnoise");

   tft.setTextColor(labelFGColor, labelBGColor);
   tft.drawString(headings, 80, 123, 4);

   tft.setTextColor(normalColor, labelBGColor);

   tft.drawString (s2n, 150, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showS2N()


void showSFI(void)
{
   String sflux = getXmlData(xmlData, "solarflux");
   String kindx = getXmlData(xmlData, "kindex");
   String aindx = getXmlData(xmlData, "aindex");

   String headings = "SFI:           A:           K:";

   //   if ((sflux.charAt(0) != '?') && (WiFi.softAPgetStationNum() == 0) && (!gpsActiveMode))
   if (sflux.charAt(0) != '?')
   {
      tft.setTextColor(labelFGColor, labelBGColor);
      tft.drawString(headings, 80, 123, 4);
      tft.setTextColor(normalColor, labelBGColor);

      if (sflux.toInt() >= MEDIUM_SFI)
      {
         tft.setTextColor(mediumColor, labelBGColor);
      }

      if (sflux.toInt() >= HIGH_SFI)
      {
         tft.setTextColor(highColor, labelBGColor);
      }

      tft.drawString(sflux, 125, 123, 4);

      tft.setTextColor(normalColor, labelBGColor);

      if (aindx.toInt() >= MEDIUM_A)
      {
         tft.setTextColor(mediumColor, labelBGColor);
      }

      if (aindx.toInt() >= HIGH_A)
      {
         tft.setTextColor(highColor, labelBGColor);
      }

      tft.drawString(aindx, 205, 123, 4);

      tft.setTextColor(normalColor, labelBGColor);

      if (kindx.toInt() >= MEDIUM_K)
      {
         tft.setTextColor(mediumColor, labelBGColor);
      }

      if (kindx.toInt() >= HIGH_K)
      {
         tft.setTextColor(highColor, labelBGColor);
      }

      tft.drawString(kindx, 284, 123, 4);
      tft.setTextColor(labelFGColor, labelBGColor);
   } else {
      tft.setTextColor(normalColor, labelBGColor);

      if (getWiFiStatus() != WIFI_NET_CONNECTED)
      {
         tft.drawString("(Solar unavailable)", 80, 123, 4);
      } else {
         tft.drawString("(Solar updating)", 80, 123, 4);
      }
      dataIndex = SHOW_LAST_SOLAR_DATA;
   }
}  // showSFI()


void showSplash(void)
{
   String versionStr = String("dated: ") + VERSION_TIMESTAMP;

   tft.fillScreen(screenBGColor);
   tft.setTextDatum(TC_DATUM);
   tft.setFreeFont(&FreeSerifBold18pt7b);

   tft.setTextColor(normalColor);

   tft.drawString("NTP Clock", 160, 5);
   tft.setFreeFont(&FreeSerifBold9pt7b);
   tft.drawString("Original Design by: W8BH", 160, 60);
   tft.drawString("Cowtown ARC Version 4.x", 160, 120);
   tft.drawString(versionStr, 160, 140, 2);

   tft.setTextColor(labelFGColor);

   tft.drawString("Solar data from: N0NBH", 160, 85);
   tft.drawString("Software Modified by:", 160, 170);
   tft.drawString("VK2ARH, KD5RXT", 160, 190);
   tft.drawString("WA2FZW, AI6P", 160, 210);

   tft.setTextColor(labelFGColor);
   tft.setTextDatum(TL_DATUM);
}  // showSplash()


void showSSN(void)
{
   String headings = "SSN:";

   String ssn = getXmlData(xmlData, "sunspots");

   tft.setTextColor(labelFGColor, labelBGColor);
   tft.drawString(headings, 80, 123, 4);

   tft.setTextColor(normalColor, labelBGColor);

   tft.drawString(ssn, 150, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showSSN()


void showSunrise(void)
{
   String headings = "Sunrise:";
   char text[8] = { 0x00 };

   strftime(text, sizeof(text), "%H:%M", gmtime(&weatherUnixSunrise));
   String sunrise = text;

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (sunrise, 180, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showSunrise()


void showSunset(void)
{
   String headings = "Sunset:";
   char text[8] = { 0x00 };

   strftime(text, sizeof(text), "%H:%M", gmtime(&weatherUnixSunset));
   String sunset = text;

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (sunset, 180, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showSunset()


void showTemp(void)
{
   String headings = "T:";
   String tmp = String();

   if (useMetric)
   {
      tmp = String(weatherTemp) + "C (feels " + String(weatherFeels) + "C)";
   } else {
      tmp = String(weatherTemp) + "F (feels " + String(weatherFeels) + "F)";
   }

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (tmp, 120, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showTemp()


void showTime(time_t t, boolean hr24, uint16_t x, uint16_t y)
{
   tft.setTextColor(timeColor, screenBGColor);

   uint16_t h = hour(t);
   uint16_t m = minute(t);
   uint16_t s = second(t);

   if (!((h >= 0) && (h < 24)))
   {
      h = 0;
   }

   if (!((m >= 0) && (m < 60)))
   {
      m = 0;
   }

   if (!((s >= 0) && (s < 60)))
   {
      s = 0;
   }

   char ampm;

   if (h <= 11)
   {
      ampm = 'A';
   } else {
      ampm = 'P';
   }

   tft.setTextColor(screenBGColor, screenBGColor);

   tft.drawChar(ampm, x + 220, y + 34, 2);                                     // erase any old AM/PM display
   tft.drawChar('M', x + 230, y + 34, 2);
   tft.drawChar (ampm, x + 222, y + 2, 4);
   tft.drawChar ('M', x + 220, y + 26, 4);

   tft.setTextColor(timeColor, screenBGColor);

   if (!hr24)
   {
      if (showAMPMMode == SHOW_AMPM_SMALL)                                     // Display small AM/PM indicator
      {
         tft.drawChar(ampm, x + 220, y + 34, 2);
         tft.drawChar('M', x + 230, y + 34, 2);
      }

      if (showAMPMMode == SHOW_AMPM_LARGE)
      {  // Display large AM/PM
         tft.drawChar (ampm, x + 222, y + 2, 4);
         tft.drawChar ('M', x + 220, y + 26, 4);
      }
   }

   if (!hr24)
   {
      if (h == 0)
      {
         h = 12;
      }

      if (h > 12)
      {
         h -= 12;
      }
   }

   if (h < 10)
   {
      if ((hr24) || (hourLeadingZero))                                         // 24hr format: always use leading 0
      {
         x += tft.drawChar ('0', x, y, 7);
      } else {
         tft.setTextColor(screenBGColor, screenBGColor);                       // erase old digit
         x += tft.drawChar('8', x, y, 7);
         tft.setTextColor(timeColor, screenBGColor);
      }
   }

   x += tft.drawNumber(h, x, y, 7);
   x += tft.drawChar(':', x, y, 7);

   if (m < 10)
   {
      x += tft.drawChar('0', x, y, 7);                                         // Always a leading zero for minutes
   }

   x += tft.drawNumber(m, x, y, 7);
   x += tft.drawChar(':', x, y, 7);

   if (s < 10)
   {
      x += tft.drawChar('0', x, y, 7);                                         // Always a leading zero for seconds
   }

   x += tft.drawNumber(s, x, y, 7);
}  // showTime()


void showTimeDate(boolean useLocalTime, time_t t, boolean hr24, uint16_t x, uint16_t y)
{
   showTime(t, hr24, x, y);                                                    // Display time HH:MM:SS

   showTimeZone(useLocalTime, x - 2, y - 42);

   const uint16_t yspacing = 23;
   const char* months[] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };

   uint16_t i = 0;
   uint16_t m = month(t) - 1;
   uint16_t d = day(t);
   uint16_t xOffset = 249;

   if (!((m >= 0) && (m <= 12)))
   {
      m = 0;
   }

   if (!((d >= 0) && (d <= 31)))
   {
      d = 0;
   }

   const char* dotw[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };

   tft.setTextColor(dateColor, screenBGColor);
   tft.fillRect(x + xOffset, y - 9, 57, 65, screenBGColor);                    // Erase any previous date

   tft.drawString(dotw[weekday(t) - 1], x + xOffset, y - 9, 4);
   y += yspacing;

   if (dateAboveMonth)
   {
      if ((dateLeadingZero) && (d < 10))
      {
         i = tft.drawNumber(0, x + xOffset, y - 9, 4);
      }

      tft.drawNumber(d, x + xOffset + i, y - 9, 4);
      y += yspacing;
      tft.drawString(months[m], x + xOffset, y - 9, 4);
   } else {
      tft.drawString(months[m], x + xOffset, y - 9, 4);
      y += yspacing;

      if ((dateLeadingZero) && (d < 10))
      {
         x += tft.drawNumber(0, x + xOffset, y - 9, 4);
      }

      tft.drawNumber(d, x + xOffset, y - 9, 4);
   }
}  // showTimeDate()


void showTimeZone(boolean useLocalTime, uint16_t x, uint16_t y)
{
   tft.setTextColor(labelFGColor, labelBGColor);

   if (!useLocalTime)
   {
      tft.drawString("UTC", x, y + 3, 4);
   } else {
      tft.fillRoundRect(2, 2, 76, 30, 10, labelBGColor);
      tft.drawString(local.getTimezoneName(), x, y + 2, 4);
   }
}  // showTimeZone()


void showWiFiInfo(void)
{
   tft.fillScreen(screenBGColor);                                              // Start with empty screen
   tft.setTextDatum(TC_DATUM);
   tft.setFreeFont(&FreeSerifBold18pt7b);

   tft.setTextColor(normalColor);

   tft.drawString(F("NTP Clock"), 160, 0);

   String wifi_ssid = String();

   if (getWiFiStatus() == WIFI_NET_CONNECTED)
   {
      wifi_ssid = WiFi.SSID();
   }
   else
   {
      wifi_ssid = String("(not connected)");
   }

   tft.setTextDatum(TL_DATUM);

   tft.setTextColor(warningColor);

   tft.drawString(String("AP SSID:"), 10, 60, 2);
   tft.drawString(String("AP IP:"), 10, 80, 2);
   tft.drawString(String("WiFi SSID:"), 10, 100, 2);
   tft.drawString(String("WiFi IP:"), 10, 120, 2);
   tft.drawString(String("AP chan:"), 10, 140, 2);
   tft.drawString(String("MAC:"), 10, 160, 2);

   tft.setTextColor(labelFGColor);

   tft.drawString(String(apSSID), 90, 60, 2);
   tft.drawString(WiFi.softAPIP().toString(), 90, 80, 2);
   tft.drawString(wifi_ssid, 90, 100, 2);
   tft.drawString(WiFi.localIP().toString(), 90, 120, 2);
   tft.drawString(String(apChannel), 90, 140, 2);
   tft.drawString(String(WiFi.macAddress()), 90, 160, 2);

   tft.setTextDatum(TC_DATUM);

   String verStr = String("rev: ") + VERSION_TIMESTAMP;
   tft.drawString(verStr, 160, 226, 2);                                        // put version info on the screen

   tft.setTextColor(warningColor);

   tft.drawString(F("Press either button to resume..."), 160, 210, 2);

#define TIMEOUT_PERIOD 20000                                                   // timeout after 20 seconds of inactivity
   uint32_t startTime = millis();

   while (true)
   {
      if (configButton.isPressed())
      {
         while (configButton.isPressed())
         {
            delay(10);
         }

         configButton.resetPressedState();
         brightButton.resetPressedState();

         break;
      }

      if (brightButton.isPressed())
      {
         while (brightButton.isPressed())
         {
            delay(10);
         }

         configButton.resetPressedState();
         brightButton.resetPressedState();

         break;
      }

#ifndef DISABLE_BUTTON_DEF_TIMEOUT

      if ((millis() - startTime) >= TIMEOUT_PERIOD)
      {
         break;
      }

#endif

      delay(10);
   }

   newDualScreen();

   tft.setTextDatum(TL_DATUM);
}  // showWiFiInfo()


void showWiFiStatus(void)
{
   uint16_t color;

   uint16_t syncAge = now() - lastNtpUpdateTime();                             // How long has it been since last sync?

   if (syncAge < SYNC_MARGINAL)                                                // time is good & in sync
   {
      color = normalColor;
   } else {
      if (syncAge < SYNC_LOST)                                                 // sync is 1-24 hours old
      {
         color = mediumColor;
      }
   }

   if (getWiFiStatus() != WIFI_NET_CONNECTED)
   {
      color = lostColor;                                                       // WiFi connection lost
   }

   tft.drawSmoothArc(305, 18, 14, 13, 150, 210, color, screenBGColor);
   tft.drawSmoothArc(305, 18, 9, 8, 150, 210, color, screenBGColor);
   tft.drawSmoothArc(305, 18, 4, 3, 150, 210, color, screenBGColor);

   if ((nightMode) && (getWiFiStatus() != WIFI_NET_CONNECTED))
   {
      tft.drawLine(301, 3, 310, 15, color);
      tft.drawLine(300, 3, 309, 15, color);

      tft.drawLine(310, 3, 301, 15, color);
      tft.drawLine(309, 3, 300, 15, color);
   }

   // erase any previous indication (truth be told, writing only the "M" is sufficient)
   tft.setTextColor(labelBGColor, labelBGColor);
   tft.drawString("A", 302, 17, 2);
   tft.drawString("M", 301, 17, 2);

   tft.setTextColor(labelFGColor, labelBGColor);

   if (wifiRetryAuto)
   {
      tft.drawString("A", 302, 17, 2);
   } else {
      tft.drawString("M", 301, 17, 2);
   }
}  // showWiFiStatus()


void showWind(void)
{
   String headings = "W:";

   String wind = String();

   if ((weatherDirection >= 348) || (weatherDirection < 11))
   {
      wind = "N";
   }
   if ((weatherDirection >= 11) && (weatherDirection < 33))
   {
      wind = "NNE";
   }
   if ((weatherDirection >= 33) && (weatherDirection < 56))
   {
      wind = "NE";
   }
   if ((weatherDirection >= 56) && (weatherDirection < 78))
   {
      wind = "ENE";
   }
   if ((weatherDirection >= 78) && (weatherDirection < 101))
   {
      wind = "E";
   }
   if ((weatherDirection >= 101) && (weatherDirection < 123))
   {
      wind = "ESE";
   }
   if ((weatherDirection >= 123) && (weatherDirection < 146))
   {
      wind = "SE";
   }
   if ((weatherDirection >= 146) && (weatherDirection < 168))
   {
      wind = "SSE";
   }
   if ((weatherDirection >= 168) && (weatherDirection < 191))
   {
      wind = "S";
   }
   if ((weatherDirection >= 191) && (weatherDirection < 213))
   {
      wind = "SSW";
   }
   if ((weatherDirection >= 213) && (weatherDirection < 236))
   {
      wind = "SW";
   }
   if ((weatherDirection >= 236) && (weatherDirection < 258))
   {
      wind = "WSW";
   }
   if ((weatherDirection >= 258) && (weatherDirection < 281))
   {
      wind = "W";
   }
   if ((weatherDirection >= 281) && (weatherDirection < 303))
   {
      wind = "WNW";
   }
   if ((weatherDirection >= 303) && (weatherDirection < 326))
   {
      wind = "NW";
   }
   if ((weatherDirection >= 326) && (weatherDirection < 348))
   {
      wind = "NNW";
   }

   wind += " " + String(weatherWind);

   if (weatherGust > weatherWind)
   {
      wind += "-" + String(weatherGust);
   }

   if (useMetric)
   {
      wind += "kph";
   } else {
      wind += "mph";
   }

   tft.setTextColor (labelFGColor, labelBGColor);
   tft.drawString (headings, 80, 123, 4);
   tft.setTextColor(normalColor, labelBGColor);
   tft.drawString (wind, 115, 123, 4);

   tft.setTextColor(labelFGColor, labelBGColor);
}  // showWind()


void startButtonLoopTask(void)
{
#define TASK_PRIORITY 2                                                        // NOTE: loop() is running at priority level 1
#define CPU_CORE 1                                                             // NOTE: ESP32-S2 is a single-core processor, whereas the ESP32-S3 is a dual-core processor

   // Start a permanent thread to manage the button timing & detection

#if defined(ESP32S3_SUPERMINI) || defined(ESP32S3_MINI) || defined(ESP32S3_MJC_TESTBED)

   xTaskCreatePinnedToCore(buttonLoopTask, "ButtonLoopTask", 4096, NULL, TASK_PRIORITY, NULL, CPU_CORE);

#else

   xTaskCreate(buttonLoopTask, "ButtonLoopTask", 2048, NULL, TASK_PRIORITY, NULL);

#endif

}  // startButtonLoopTask()


void startGPS(boolean tryBothPinConfigs)
{
   boolean triedBothPinConfigs = true;                                         // starts TRUE, togggled each pass, then will cause an exit when it toggles back to TRUE
   boolean gnssFound = false;

   gpsFixState = GPS_NO_FIX;

   while (true)
   {
      startupScreen();

      tft.setTextDatum(TC_DATUM);

      if (gpsSSreversed)
      {
         tft.drawString("Initializing GPS (reversed RX/TX pins)", 160, 50, 2);
      } else {
         tft.drawString("Initializing GPS (non-reversed RX/TX pins)", 160, 50, 2);
      }

      tft.setFreeFont(&FreeSansBold9pt7b);

      if (debugGPSinitialization)
      {
         Serial.println("GNSS: attempting to initialize at 38400 baud");
      }

      tft.fillRect(5, 80, 310, 20, screenBGColor);

      tft.drawString("GNSS: checking at 38400 baud", 160, 80, 2);

      if (gpsSSreversed)
      {
         gps_tx_pin = GPS_DATA_IN_PIN;
         gps_rx_pin = GPS_DATA_OUT_PIN;
      } else {
         gps_tx_pin = GPS_DATA_OUT_PIN;
         gps_rx_pin = GPS_DATA_IN_PIN;
      }

      gpsSS.end();
      gpsSS.begin(38400, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

      if (myGNSS.begin(gpsSS))
      {
         if (debugGPSinitialization)
         {
            Serial.println("GNSS: connected at 38400 baud");
         }

         tft.fillRect(5, 80, 310, 20, screenBGColor);

         tft.drawString("GNSS: connected at 38400 baud", 160, 80, 2);
         tft.drawString("waiting for GPS to respond...", 160, 100, 2);
         tft.drawString("NOTE: may take up to one minute or more.", 160, 130, 2);
         tft.drawString("If you see this screen for more than two", 160, 160, 2);
         tft.drawString("minutes, then a power cycle is recommended.", 160, 180, 2);

         if (debugGPSinitialization)
         {
            Serial.println("GNSS: successfully initialized the GPS serial interface");
         }

         gnssFound = true;
      } else {
         if (debugGPSinitialization)
         {
            Serial.println("GNSS: attempting to initialize at  9600 baud");
         }

         tft.fillRect(5, 80, 310, 20, screenBGColor);

         tft.drawString("GNSS: checking at 9600 baud", 160, 80, 2);

         gpsSS.end();
         gpsSS.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

         if (myGNSS.begin(gpsSS))
         {
            if (debugGPSinitialization)
            {
               Serial.println("GNSS: connected at 9600 baud, switching to 38400");
               Serial.println("waiting for the GPS to respond...");
               Serial.println("(NOTE: may take up to one minute or more.)");
               Serial.println("(If this report hangs for more than two)");
               Serial.println("(minutes, then a power cycle is needed)");
            }

            tft.fillRect(5, 80, 310, 20, screenBGColor);

            tft.drawString("GNSS: connected at 9600 baud", 160, 80, 2);
            delay(500);

            tft.fillRect(5, 80, 310, 20, screenBGColor);

            tft.drawString("GNSS: switching to 38400", 160, 80, 2);

            myGNSS.setSerialRate(38400);

            gpsSS.end();
            gpsSS.begin(38400, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

            tft.fillRect(5, 100, 310, 20, screenBGColor);

            tft.drawString("waiting for GPS to respond...", 160, 100, 2);

            tft.fillRect(5, 120, 310, 20, screenBGColor);

            tft.drawString("(NOTE: may take up to one minute or more.)", 160, 120, 2);
            tft.drawString("(If you see this screen for more than two)", 160, 140, 2);
            tft.drawString("(minutes, then a power cycle is needed)", 160, 160, 2);

            delay(500);

            gnssFound = true;
         }
      }

      if (gnssFound)
      {
         myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
         myGNSS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
         myGNSS.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
         myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
         myGNSS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
         myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);

         myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_USB);
         myGNSS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_USB);
         myGNSS.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_USB);
         myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_USB);
         myGNSS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_USB);
         myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_USB);

         myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C);
         myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
         myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
         myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);
         myGNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
         myGNSS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C);

         myGNSS.setI2COutput(0);                                               // Turn off sentences on the I2C interface
         myGNSS.setUSBOutput(COM_TYPE_NMEA);                                   // Set the USB port to output only NMEA
         myGNSS.setNMEAOutputPort(gpsSS);                                      // Send standard NMEA output to the software serial port that we've initialized

         myGNSS.setUART1Output(COM_TYPE_NMEA);                                 // Set the UART1 port output type
         myGNSS.saveConfiguration();                                           // Save the current settings to flash and BBR

         if (debugGPSinitialization)
         {
            Serial.println("GNSS: initialization complete");
         }

         getGPSTimeUpdate(true);
      }

      if (tryBothPinConfigs)
      {
         triedBothPinConfigs = !triedBothPinConfigs;
      }

      if ((triedBothPinConfigs) || (gnssFound))
      {
         // allow time to read the final status
         delay(1500);

         break;
      } else {
         gpsSSreversed = !gpsSSreversed;
      }

      if (gpsSSreversed)
      {
         gps_tx_pin = GPS_DATA_IN_PIN;
         gps_rx_pin = GPS_DATA_OUT_PIN;
      } else {
         gps_tx_pin = GPS_DATA_OUT_PIN;
         gps_rx_pin = GPS_DATA_IN_PIN;
      }

      gpsSS.end();
      gpsSS.begin(38400, EspSoftwareSerial::SWSERIAL_8N1, gps_rx_pin, gps_tx_pin, false, BYTE_BUFFER_SIZE, ISR_BIT_BUFFER_SIZE);

      if (!gpsSS)
      {
         Serial.println("gpsSS initialization at 38400 FAILED...");
      }
   }

   if (!gnssFound)
   {
      gpsActiveMode = false;
      gpsFixState = GPS_NO_FIX;
      gpsSatelliteCount = 0;
   }

   tft.setTextDatum(TL_DATUM);
}  // startGPS()


void startupScreen(void)
{
   tft.fillScreen(screenBGColor);
   tft.fillRoundRect(1, 1, 318, 33, 10, labelBGColor);
   tft.drawRoundRect(1, 1, 318, 34, 10, edgeColor);
   tft.drawRoundRect(1, 1, 318, 238, 10, edgeColor);
   tft.setTextColor(labelFGColor, labelBGColor);
   tft.drawCentreString(bannerTitle, 160, 7, 4);
   tft.setTextColor(labelFGColor, screenBGColor);
}  // startupScreen()


// insert a call to this function for troubleshooting
void stopAndWaitForever(void)
{
   boolean toggle = false;

   Serial.println("...entered stopAndWaitForever()");

   while (true)
   {
      toggle = !toggle;

      if (toggle)
      {
         digitalWrite(LED_PIN, HIGH);
      } else {
         digitalWrite(LED_PIN, LOW);
      }

      delay(250);
   }
}  // stopAndWaitForever()


void switchDisplayMode(void)
{
   if (nightMode)
   {  screenBGColor = NIGHTMODE_BGCOLOR;
      timeColor     = NIGHTMODE_COLOR;
      dateColor     = NIGHTMODE_COLOR;
      labelFGColor  = NIGHTMODE_COLOR;
      labelBGColor  = NIGHTMODE_BGCOLOR;
      edgeColor     = NIGHTMODE_COLOR;
      normalColor   = NIGHTMODE_COLOR;
      mediumColor   = NIGHTMODE_COLOR;
      highColor     = NIGHTMODE_COLOR;
      lostColor     = NIGHTMODE_COLOR;
      warningColor  = NIGHTMODE_COLOR;
   } else {
      screenBGColor = screenBGColor_day;
      timeColor     = timeColor_day;
      dateColor     = dateColor_day;
      labelFGColor  = labelFGColor_day;
      labelBGColor  = labelBGColor_day;
      edgeColor     = edgeColor_day;
      normalColor   = normalColor_day;
      mediumColor   = mediumColor_day;
      highColor     = highColor_day;
      lostColor     = lostColor_day;
      warningColor  = warningColor_day;
   }
}  // switchDisplayMode()


void updateBrightness(void)
{
   static int16_t previous_bright_level;                                       // previous setting
   int16_t bright_level;                                                       // new level

   if (brightness == 0)                                                        // in auto mode?
   {
      bright_level = analogRead(LIGHT_SENSOR_PIN) / 4;

      if (bright_level < 10)
      {
         bright_level = 10;
      }

      if (bright_level > 255)
      {
         bright_level = 255;
      }
      bright_level = ((previous_bright_level * 3) + bright_level) / 4;
   } else {
      bright_level = (brightness * 32) + 15;
   }

   previous_bright_level = bright_level;

   analogWrite(BACKLIGHT_PIN, bright_level);

   if (debugLightSensor)
   {
      Serial.print("analogRead = ");
      Serial.print(analogRead(LIGHT_SENSOR_PIN));
      Serial.print("   bright level = ");
      Serial.println(bright_level);
   }
}  // updateBrightness()


void updateDisplay(void)
{
   static uint8_t previousSecond;
   time_t t;                                                                   // Current & displayed UTC time
   time_t lt;                                                                  // Current & displayed local time

   t = now();
   lt = local.now();

   if (second(t) != previousSecond)
   {
      previousSecond = second(t);

      if (colorUpdateNeeded)
      {
         colorUpdateNeeded = false;

         screenBGColor_day = Colors[screenBGColorIdx].color;
         screenBGColor = screenBGColor_day;

         timeColor_day = Colors[timeColorIdx].color;
         timeColor = timeColor_day;

         dateColor_day = Colors[dateColorIdx].color;
         dateColor = dateColor_day;

         labelFGColor_day = Colors[labelFGColorIdx].color;
         labelFGColor = labelFGColor_day;

         labelBGColor_day = Colors[labelBGColorIdx].color;
         labelBGColor = labelBGColor_day;

         edgeColor_day = Colors[edgeColorIdx].color;
         edgeColor = edgeColor_day;

         normalColor_day = Colors[normalColorIdx].color;
         normalColor = normalColor_day;

         mediumColor_day = Colors[mediumColorIdx].color;
         mediumColor = mediumColor_day;

         highColor_day = Colors[highColorIdx].color;
         highColor = highColor_day;

         lostColor_day = Colors[lostColorIdx].color;
         lostColor = lostColor_day;

         warningColor_day = Colors[warningColorIdx].color;
         warningColor = warningColor_day;

         switchDisplayMode();
         newDualScreen();
      }

      showTimeDate(true, lt, localFormat24hr, 10, 47);
      showTimeDate(false, t, utcFormat24hr, 10, 162);

      showWiFiStatus();

      showGPSStatus(248, 2);

      showNextData();

      if (blinkTimeUpdate)
      {
         activateLED = true;
      }
   }
}  // updateDisplay()


const String webAMPMModeSelector(void)
{
   String result = String();

   char text[100];

   sprintf(text,
           "<OPTION VALUE='%d' % s > % s</OPTION>",
           (int)SHOW_AMPM_NONE, showAMPMMode == SHOW_AMPM_NONE ? " SELECTED" : "", "Don't Show AM/PM"
          );

   result += text;

   sprintf(text,
           "<OPTION VALUE='%d' % s > % s</OPTION>",
           (int)SHOW_AMPM_SMALL, showAMPMMode == SHOW_AMPM_SMALL ? " SELECTED" : "", "Show Small AM/PM"
          );

   result += text;

   sprintf(text,
           "<OPTION VALUE='%d' % s > % s</OPTION>",
           (int)SHOW_AMPM_LARGE, showAMPMMode == SHOW_AMPM_LARGE ? " SELECTED" : "", "Show Large AM/PM"
          );

   result += text;

   return (result);
}  // webAMPMModeSelector


const String webCheckbox(boolean isSelected, const String & idname, const String & text)
{
   return (
             "<INPUT STYLE = 'TEXT-ALIGN:CENTER; VERTICAL-ALIGN: MIDDLE' TYPE = 'CHECKBOX' ID = '" + idname + "' NAME = '" + idname + "' " + String(isSelected ? "CHECKED" : "") + ">"
             "<LABEL FOR = '" + idname + "'>" + text + " </LABEL>"
          );
}  // webCheckbox()


const String webColorSelector(uint16_t index)
{
   String result = String();

   for (int i = 0 ; i < (sizeof(Colors) / sizeof(Colors[0])); i++)
   {
      char text[200];

      sprintf(text,
              "<OPTION VALUE='%d'%s>%s</OPTION>",
              i, index == i ? " SELECTED" : "", Colors[i].name
             );

      result += text;
   }

   return (result);
}  // webColorSelector()


const String webColorsPage(void)
{
   return webPage(
             "<H1>NTPclock [COLORS]</H1>"
             "<P ALIGN='CENTER'>"
             "<A HREF='/'>[ STATUS ]</A>&nbsp;&nbsp;<A HREF='/network'>[ NETWORK ]</A>&nbsp;&nbsp;<A HREF='/colors'>[ COLORS ]</A>&nbsp;&nbsp;<A HREF='/config'>[ CONFIG ]</A>"
             + String(debugIsEnabled ? "<BR><BR><A HREF='/scan'>[ SCAN ]</A>&nbsp;&nbsp;<A HREF='/debug'>[ DEBUG ]</A>" : "<BR><BR><A HREF='/scan'>[ SCAN ]</A>") +
             "</P>"
             "<FORM ACTION='/setcolors' METHOD='POST'>"
             "<TABLE>"
             "<TR><TH CLASS='HEADING'>(NOTE: A pause may occur when going to/from [SCAN])</TH></TR>"
             "</TABLE>"
             "<BR>"
             "<TABLE COLUMNS=2>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Color Selections</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Screen Background</TD>"
             "<TD>"
             "<SELECT NAME='sBGColorIdx'>" + webColorSelector(screenBGColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Clock Digits</TD>"
             "<TD>"
             "<SELECT NAME='tColorIdx'>" + webColorSelector(timeColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Date</TD>"
             "<TD>"
             "<SELECT NAME='dColorIdx'>" + webColorSelector(dateColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Label Foreground</TD>"
             "<TD>"
             "<SELECT NAME='lFGColorIdx'>" + webColorSelector(labelFGColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Label Background</TD>"
             "<TD>"
             "<SELECT NAME='lBGColorIdx'>" + webColorSelector(labelBGColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Edge</TD>"
             "<TD>"
             "<SELECT NAME='eColorIdx'>" + webColorSelector(edgeColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Normal Readings</TD>"
             "<TD>"
             "<SELECT NAME='nColorIdx'>" + webColorSelector(normalColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Medium Readings</TD>"
             "<TD>"
             "<SELECT NAME='mColorIdx'>" + webColorSelector(mediumColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>High Readings</TD>"
             "<TD>"
             "<SELECT NAME='hColorIdx'>" + webColorSelector(highColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Lost WiFi</TD>"
             "<TD>"
             "<SELECT NAME='lColorIdx'>" + webColorSelector(lostColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Warnings</TD>"
             "<TD>"
             "<SELECT NAME='wColorIdx'>" + webColorSelector(warningColorIdx) + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Activate To Restore Default Colors</TH></TR>"
             "<TD><CLASS='LABEL'>Return All Colors To Defaults</TD>"
             "<TD>" + webCheckbox(false, "colordefaults", "Default Colors") + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>"
             "<INPUT TYPE='SUBMIT' VALUE='Save / Apply'>"
             "</TH></TR>"
             "</TABLE>"
             "</FORM>"
          );
}  // webColorsPage()


const String webConfigPage()
{
   char tzcStr[8];

   char gpsStr[8];
   char wfiStr[8];
   char decStr[8];

   char ptStr[8];
   char humStr[8];

   sprintf(tzcStr, "%d", tzCountdownIntervalSetting);

   sprintf(gpsStr, "%d", useGPSToSetInternalClockIntervalInSeconds);
   sprintf(wfiStr, "%d", weatherFetchIntervalInSeconds);
   sprintf(decStr, "%d", weatherLatLonDecimalPlaces);

   sprintf(ptStr, "%d", printedTime);
   sprintf(humStr, "%d", humidityOffset);

   return webPage(
             "<H1>NTPclock [CONFIG]</H1>"
             "<P ALIGN='CENTER'>"
             "<A HREF='/'>[ STATUS ]</A>&nbsp;&nbsp;<A HREF='/network'>[ NETWORK ]</A>&nbsp;&nbsp;<A HREF='/colors'>[ COLORS ]</A>&nbsp;&nbsp;<A HREF='/config'>[ CONFIG ]</A>"
             + String(debugIsEnabled ? "<BR><BR><A HREF='/scan'>[ SCAN ]</A>&nbsp;&nbsp;<A HREF='/debug'>[ DEBUG ]</A>" : "<BR><BR><A HREF='/scan'>[ SCAN ]</A>") +
             "</P>"
             "<FORM ACTION='/setconfig' METHOD='POST'>"
             "<TABLE>"
             "<TR><TH CLASS='HEADING'>(NOTE: A pause may occur when going to/from [SCAN])</TH></TR>"
             "</TABLE>"
             "<BR>"
             "<TABLE COLUMNS=2>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Data Display Selections</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Show Solar Data</TD>"
             "<TD>" + webCheckbox(showSolarSelection, "showSolar", "Show Solar (S)") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Show Environmental Data</TD>"
             "<TD>" + webCheckbox(showEnvSelection, "showEnv", "Show Environmental (E)") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Show Weather Data</TD>"
             "<TD>" + webCheckbox(showWeatherSelection, "showWeather", "Show Weather (W)") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Show Maidenhead Grid Square (MLS) Data</TD>"
             "<TD>" + webCheckbox(showMLSSelection, "showMLS", "Show MLS (M)") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Show GPS Data</TD>"
             "<TD>" + webCheckbox(showGPSSelection, "showGPS", "Show GPS (G)") + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Banner Title</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Banner Title (8 chars max)</TD>"
             "<TD>" + webInputField("title", bannerTitle) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>GPS Mode</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>GPS Mode</TD>"
             "<TD>"
             "<SELECT NAME='gpsmode'>" + webGPSModeSelector() + "</SELECT>"
             "</TD>"
             "</TR>"
             + String(gpsActiveMode ? "" :
                      "<TR>"
                      "<TD><CLASS='LABEL'>GPS Latitude</TD>"
                      "<TD>" + webInputField("gpslat", gpsManualLat) + "</TD>"
                      "</TR>"
                      "<TR>"
                      "<TD><CLASS='LABEL'>GPS Longitude</TD>"
                      "<TD>" + webInputField("gpslon", gpsManualLon) + "</TD>"
                      "</TR>") +
             "<TR><TH COLSPAN=2 CLASS='HEADING'># Lat/Lon Decimal Places</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Max Number of Decimal<BR>Places To Show In Lat/Lon<BR>For Weather Display (1-6)</TD>"
             "<TD>" + webInputField("decimals", decStr) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Timezone Display Interval</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Number of Seconds<BR>To Display Each<BR>Active Timezone (1-30)</TD>"
             "<TD>" + webInputField("tzcountdown", tzcStr) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Timezones to Display</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the EST/EDT<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showESTEDT, "showest", "Show EST/EDT") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the CST/CDT<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showCSTCDT, "showcst", "Show CST/CDT") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the MST/MDT<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showMSTMDT, "showmst", "Show MST/MDT") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the EST/EDT<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showMSTNOMDT, "showmstonly", "Show MST (only)") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the PST/PDT<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showPSTPDT, "showpst", "Show PST/PDT") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the AWST<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showAWST, "showawst", "Show AWST") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the ACST/ACDT<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showACSTACDT, "showacst", "Show ACST/ACDT") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the AEST/AEDT<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showAESTAEDT, "showaest", "Show AEST/AEDT") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display the GMT/BST<BR>Timezone</TD>"
             "<TD>" + webCheckbox(showGMTBST, "showgmt", "Show GMT/BST") + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Time & Date Display Format</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Show AM/PM Mode</TD>"
             "<TD>"
             "<SELECT NAME='showampm'>" + webAMPMModeSelector() + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display Local Time<BR>In 24 Hour Format</TD>"
             "<TD>" + webCheckbox(localFormat24hr, "local24hr", "Local 24 Hour") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display UTC Time<BR>In 24 Hour Format</TD>"
             "<TD>" + webCheckbox(utcFormat24hr, "utc24hr", "UTC 24 Hour") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Time (hour) Is Displayed<BR>With A Leading Zero</TD>"
             "<TD>" + webCheckbox(hourLeadingZero, "hrlead0", "Hour Leading Zero") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Date (day) Is Displayed<BR>With A Leading Zero</TD>"
             "<TD>" + webCheckbox(dateLeadingZero, "dtlead0", "Date Leading Zero") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Date (day) Is Displayed<BR>Above The Month</TD>"
             "<TD>" + webCheckbox(dateAboveMonth, "dtovermo", "Date Above Month") + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Voltage Measurement Display</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>5VDC Supply Measurement<BR>Is Displayed</TD>"
             "<TD>" + webCheckbox(fiveVoltDisplay, "show5vdc", "Show 5VDC Display") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>5VDC Supply Measurement<BR>Correction/Calibration Factor<BR>(Default Value = 1.0)</TD>"
             "<TD>" + webInputField("fivevoltcorr", fiveVoltCorrection) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>12VDC Supply Measurement<BR>Is Displayed</TD>"
             "<TD>" + webCheckbox(twelveVoltDisplay, "show12vdc", "Show 12VDC Display") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>12VDC Supply Measurement<BR>Correction/Calibration Factor<BR>(Default Value = 1.0)</TD>"
             "<TD>" + webInputField("twelvevoltcorr", twelveVoltCorrection) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Printed Time Selection</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Printed Time</TD>"
             "<TD>"
             "<SELECT NAME='printtime'>" + webPrintedTimeSelector() + "</SELECT>"
             "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Blink Time Update For Visual Syncing</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Blink the (RGB) LED<BR>With Each Time Update</TD>"
             "<TD>" + webCheckbox(blinkTimeUpdate, "blinktime", "Blink Time Update") + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Show Selected Data Header Between Data Groups</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Show Selected Data Header<BR>Between Data Groups</TD>"
             "<TD>" + webCheckbox(showDataHeaderBetweenDataGroups, "showDataHeader", "Show Header") + "</TD>"
             "</TR>"
             + String(gpsActiveMode ? "" :
                      "<TR><TH COLSPAN=2 CLASS='HEADING'>QTH Altitude In Feet</TH></TR>"
                      "<TR>"
                      "<TD><CLASS='LABEL'><BR>Altitude At Current QTH<BR>(Entered In Feet)</TD>"
                      "<TD>" + webInputField("qthaltft", qthAltitudeInFeet) + "</TD>"
                      "</TR>") +
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Temperature Offset</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'><BR>Temperature Offset<BR>(Entered In Degrees Celsius)</TD>"
             "<TD>" + webInputField("tempoffset", temperatureOffsetInCelsius) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'><BR>Humidity Offset<BR>(Entered In Percent)</TD>"
             "<TD>" + webInputField("humoffset", humStr) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Weather Fetch Interval</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Number of Seconds Between<BR>Weather Fetches From<BR>The Internet (10-3600)</TD>"
             "<TD>" + webInputField("wfInterval", wfiStr) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Personal API Key From openweathermap.org</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Weather API Key</TD>"
             "<TD>" + webInputField("appID", weatherAppID) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>"
             "<INPUT TYPE='SUBMIT' VALUE='Save / Apply'>"
             "</TH></TR>"
             "</TABLE>"
             "</FORM>"
          );
}  // webConfigPage()


const String webDebugPage()
{
   return webPage(
             "<H1>NTPclock [DEBUG]</H1>"
             "<P ALIGN='CENTER'>"
             "<A HREF='/'>[ STATUS ]</A>&nbsp;&nbsp;<A HREF='/network'>[ NETWORK ]</A>&nbsp;&nbsp;<A HREF='/colors'>[ COLORS ]</A>&nbsp;&nbsp;<A HREF='/config'>[ CONFIG ]</A>"
             "<BR><BR><A HREF='/scan'>[ SCAN ]</A>&nbsp;&nbsp;<A HREF='/debug'>[ DEBUG ]</A>"
             "</P>"
             "<FORM ACTION='/setdebug' METHOD='POST'>"
             "<TABLE>"
             "<TR><TH CLASS='HEADING'>(NOTE: A pause may occur when going to/from [SCAN])</TH></TR>"
             "</TABLE>"
             "<BR>"
             "<TABLE COLUMNS=2>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Stack Usage Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugStackUsage, "debugstack", "Debug Stack Usage") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display ENV Sensor Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugEnvSensor, "debugenv", "Debug ENV Sensor") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display Light Sensor Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugLightSensor, "debuglight", "Debug Light Sensor") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display Supply Voltage Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugSupplyVoltage, "debugvolts", "Debug Supply Voltage") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display Solar Data Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugSolar, "debugsolar", "Debug Solar Data") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display Weather Data Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugWeather, "debugweather", "Debug Weather Data") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display WiFi Connect Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugWiFiConnect, "debugwifi", "Debug WiFi Connect") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display GPS Init Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugGPSinitialization, "debuggpsinit", "Debug GPS Init") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display GPS Position Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugGPSposition, "debuggpspos", "Debug GPS Position") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display GPS Time Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugGPStime, "debuggpstime", "Debug GPS Time") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display GPS Altitude Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugGPSaltitude, "debuggpsalt", "Debug GPS Altitude") + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Display GPS Fix State Debug<BR>Info In Serial Console</TD>"
             "<TD>" + webCheckbox(debugGPSfix, "debuggpsfix", "Debug GPS Fix State") + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>"
             "<INPUT TYPE='SUBMIT' VALUE='Save / Apply'>"
             "</TH></TR>"
             "</TABLE>"
             "</FORM>"
          );
}  // webDebugPage()


const String webGPSModeSelector(void)
{
   String result = String();

   if (gpsActiveMode)
   {
      result += "<OPTION VALUE='1' SELECTED> GPS Active Mode</OPTION>";
      result += "<OPTION VALUE='0'> GPS Manual Mode</OPTION>";
   } else {
      result += "<OPTION VALUE='1'> GPS Active Mode</OPTION>";
      result += "<OPTION VALUE='0' SELECTED> GPS Manual Mode</OPTION>";
   }

   return (result);
}  // webGPSModeSelector()


void webInit(void)
{
   server.on("/", HTTP_ANY, [] (AsyncWebServerRequest * request) {
      request->send(200, "text/html", webStatusPage());
   });

   server.on("/colors", HTTP_ANY, [] (AsyncWebServerRequest * request) {
      request->send(200, "text/html", webColorsPage());
   });

   server.on("/config", HTTP_ANY, [] (AsyncWebServerRequest * request) {
      request->send(200, "text/html", webConfigPage());
   });

   server.on("/network", HTTP_ANY, [] (AsyncWebServerRequest * request) {
      if (loginUsername != "" && loginPassword != "")
         if (!request->authenticate(loginUsername.c_str(), loginPassword.c_str()))
            return request->requestAuthentication();
      request->send(200, "text/html", webNetworkPage());
   });

   server.on("/scan", HTTP_ANY, [] (AsyncWebServerRequest * request) {
      request->send(200, "text/html", webScanPage());
   });

   server.on("/debug", HTTP_ANY, [] (AsyncWebServerRequest * request) {
      request->send(200, "text/html", webDebugPage());
   });

   server.onNotFound([] (AsyncWebServerRequest * request) {
      request->send(404, "text/plain", "Not found");
   });

   // This method saves network form contents
   server.on("/setnetwork", HTTP_ANY, webSetNetwork);

   // This method saves colors form contents
   server.on("/setcolors", HTTP_ANY, webSetColors);

   // This method saves configuration form contents
   server.on("/setconfig", HTTP_ANY, webSetConfig);

   // This method saves debug form contents
   server.on("/setdebug", HTTP_ANY, webSetDebug);

   // Start web server
   server.begin();
}  // webInit()


const String webInputField(const String & name, const String & value, boolean pass /* = false*/)
{
   String newValue(value);

   newValue.replace("\"", "&quot;");
   newValue.replace("'", "&apos;");

   return (
             "<INPUT TYPE='" + String(pass ? hidePasswords ? "PASSWORD" : "TEXT" : "TEXT") + "' NAME='" +
             name + "' VALUE='" + newValue + "'>"
          );
}  // webInputField()


const String webNetworkPage(void)
{
   String ssid1 = String();
   String pass1 = String();
   String ssid2 = String();
   String pass2 = String();
   String ssid3 = String();
   String pass3 = String();
   String ssid4 = String();
   String pass4 = String();
   String ssid5 = String();
   String pass5 = String();
   String ssid6 = String();
   String pass6 = String();

   prefs.begin("network", true);

   if (prefs.isKey("wifissid1"))
   {
      ssid1 = prefs.getString("wifissid1", "");
   }
   if (prefs.isKey("wifipass1"))
   {
      pass1 = prefs.getString("wifipass1", "");
   }
   if (prefs.isKey("wifissid2"))
   {
      ssid2 = prefs.getString("wifissid2", "");
   }
   if (prefs.isKey("wifipass2"))
   {
      pass2 = prefs.getString("wifipass2", "");
   }
   if (prefs.isKey("wifissid3"))
   {
      ssid3 = prefs.getString("wifissid3", "");
   }
   if (prefs.isKey("wifipass3"))
   {
      pass3 = prefs.getString("wifipass3", "");
   }
   if (prefs.isKey("wifissid4"))
   {
      ssid4 = prefs.getString("wifissid4", "");
   }
   if (prefs.isKey("wifipass4"))
   {
      pass4 = prefs.getString("wifipass4", "");
   }
   if (prefs.isKey("wifissid5"))
   {
      ssid5 = prefs.getString("wifissid5", "");
   }
   if (prefs.isKey("wifipass5"))
   {
      pass5 = prefs.getString("wifipass5", "");
   }
   if (prefs.isKey("wifissid6"))
   {
      ssid6 = prefs.getString("wifissid6", "");
   }
   if (prefs.isKey("wifipass6"))
   {
      pass6 = prefs.getString("wifipass6", "");
   }

   prefs.end();

   return webPage(
             "<H1>NTPclock [NETWORK]</H1>"
             "<P ALIGN='CENTER'>"
             "<A HREF='/'>[ STATUS ]</A>&nbsp;&nbsp;<A HREF='/network'>[ NETWORK ]</A>&nbsp;&nbsp;<A HREF='/colors'>[ COLORS ]</A>&nbsp;&nbsp;<A HREF='/config'>[ CONFIG ]</A>"
             + String(debugIsEnabled ? "<BR><BR><A HREF='/scan'>[ SCAN ]</A>&nbsp;&nbsp;<A HREF='/debug'>[ DEBUG ]</A>" : "<BR><BR><A HREF='/scan'>[ SCAN ]</A>") +
             "</P>"
             "<FORM ACTION='/setnetwork' METHOD='POST'>"
             "<TABLE>"
             "<TR><TH CLASS='HEADING'>(NOTE: A pause may occur when going to/from [SCAN])</TH></TR>"
             "</TABLE>"
             "<BR>"
             "<TABLE COLUMNS=2>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>Show/Hide WiFi Passwords</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Show/Hide Passwords</TD>"
             "<TD>" + webCheckbox(hidePasswords, "hidepass", "Hide Passwords") + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>This Web UI</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>AP Name</TD>"
             "<TD>" + webInputField("apname", String(apSSID)) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>WiFi/AP Channel (1-13)</TD>"
             "<TD>" + webInputField("apchannel", String(apChannel)) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Username</TD>"
             "<TD>" + webInputField("username", loginUsername) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Password</TD>"
             "<TD>" + webInputField("password", loginPassword, true) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 1</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>SSID</TD>"
             "<TD>" + webInputField("wifissid1", ssid1) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Password</TD>"
             "<TD>" + webInputField("wifipass1", pass1, true) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 2</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>SSID</TD>"
             "<TD>" + webInputField("wifissid2", ssid2) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Password</TD>"
             "<TD>" + webInputField("wifipass2", pass2, true) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 3</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>SSID</TD>"
             "<TD>" + webInputField("wifissid3", ssid3) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Password</TD>"
             "<TD>" + webInputField("wifipass3", pass3, true) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 4</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>SSID</TD>"
             "<TD>" + webInputField("wifissid4", ssid4) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Password</TD>"
             "<TD>" + webInputField("wifipass4", pass4, true) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 5</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>SSID</TD>"
             "<TD>" + webInputField("wifissid5", ssid5) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Password</TD>"
             "<TD>" + webInputField("wifipass5", pass5, true) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 6</TH></TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>SSID</TD>"
             "<TD>" + webInputField("wifissid6", ssid6) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Password</TD>"
             "<TD>" + webInputField("wifipass6", pass6, true) + "</TD>"
             "</TR>"
             "<TR><TH COLSPAN=2 CLASS='HEADING'>"
             "<INPUT TYPE='SUBMIT' VALUE='Save / Apply'>"
             "</TH></TR>"
             "</TABLE>"
             "</FORM>"
          );
}  // webNetworkPage()


const String webPage(const String & body)
{
   return
      "<!DOCTYPE HTML>"
      "<HTML>"
      "<HEAD>"
      "<META CHARSET='UTF-8'>"
      "<META NAME='viewport' CONTENT='width=device-width, initial-scale=1.0'>"
      "<TITLE>NTPclock [Config/Status]</TITLE>"
      "<STYLE>" + webStyleSheet() + "</STYLE>"
      "</HEAD>"
      "<BODY STYLE='font-family: sans-serif;'>" + body + " </BODY>"
      "</HTML>"
      ;
}  // webPage()


const String webPrintedTimeSelector(void)
{
   String result = String();

   char text[200];

   sprintf(text,
           "<OPTION VALUE='%d' % s > % s</OPTION>",
           0, printedTime == PRINTED_TIME_NONE ? " SELECTED" : "", "No Printed Time"
          );

   result += text;

   sprintf(text,
           "<OPTION VALUE='%d' % s > % s</OPTION>",
           1, printedTime == PRINTED_TIME_UTC ? " SELECTED" : "", "Print UTC Time"
          );

   result += text;

   sprintf(text,
           "<OPTION VALUE='%d' % s > % s</OPTION>",
           2, printedTime == PRINTED_TIME_LOCAL ? " SELECTED" : "", "Print Local Time"
          );

   result += text;

   sprintf(text,
           "<OPTION VALUE='%d' % s > % s</OPTION>",
           3, printedTime == PRINTED_TIME_BOTH ? " SELECTED" : "", "Print UTC & Local Time"
          );

   result += text;

   return (result);
}  // webPrintedTimeSelector



const String webScanPage(void)
{
   return webPage(
             "<META HTTP-EQUIV='refresh' CONTENT='1'>"
             "<H1>NTPclock [SCAN]</H1>"
             "<P ALIGN='CENTER'>"
             "<A HREF='/'>[ STATUS ]</A>&nbsp;&nbsp;<A HREF='/network'>[ NETWORK ]</A>&nbsp;&nbsp;<A HREF='/colors'>[ COLORS ]</A>&nbsp;&nbsp;<A HREF='/config'>[ CONFIG ]</A>"
             + String(debugIsEnabled ? "<BR><BR><A HREF='/scan'>[ SCAN ]</A>&nbsp;&nbsp;<A HREF='/debug'>[ DEBUG ]</A>" : "<BR><BR><A HREF='/scan'>[ SCAN ]</A>") +
             "</P>"
             "<TABLE>"
             "<TR><TH CLASS='HEADING'>(NOTE: A pause may occur when going to/from [SCAN])</TH></TR>"
             "</TABLE>"
             "<BR>"
             "<TABLE COLUMNS=4>"
             + webShowNetworkScan() +
             "<TR><TH COLSPAN=4 CLASS='HEADING'>(NOTE: Automatic rescan/refresh is active)</TH></TR>"
             "</TABLE>"
             "</FORM>"
          );
}  // webScanPage()


void webSetColors(AsyncWebServerRequest * request)
{
   // Save colors
   if (request->hasParam("sBGColorIdx", true))
   {
      String screenBGColorIdxStr = request->getParam("sBGColorIdx", true)->value();
      screenBGColorIdx = screenBGColorIdxStr.toInt();
   }

   if (request->hasParam("tColorIdx", true))
   {
      String timeColorIdxStr = request->getParam("tColorIdx", true)->value();
      timeColorIdx = timeColorIdxStr.toInt();
   }

   if (request->hasParam("dColorIdx", true))
   {
      String dateColorIdxStr = request->getParam("dColorIdx", true)->value();
      dateColorIdx = dateColorIdxStr.toInt();
   }

   if (request->hasParam("lFGColorIdx", true))
   {
      String labelFGColorIdxStr = request->getParam("lFGColorIdx", true)->value();
      labelFGColorIdx = labelFGColorIdxStr.toInt();
   }

   if (request->hasParam("lBGColorIdx", true))
   {
      String labelBGColorIdxStr = request->getParam("lBGColorIdx", true)->value();
      labelBGColorIdx = labelBGColorIdxStr.toInt();
   }

   if (request->hasParam("eColorIdx", true))
   {
      String edgeColorIdxStr = request->getParam("eColorIdx", true)->value();
      edgeColorIdx = edgeColorIdxStr.toInt();
   }

   if (request->hasParam("nColorIdx", true))
   {
      String normalColorIdxStr = request->getParam("nColorIdx", true)->value();
      normalColorIdx = normalColorIdxStr.toInt();
   }

   if (request->hasParam("mColorIdx", true))
   {
      String mediumColorIdxStr = request->getParam("mColorIdx", true)->value();
      mediumColorIdx = mediumColorIdxStr.toInt();
   }

   if (request->hasParam("hColorIdx", true))
   {
      String highColorIdxStr = request->getParam("hColorIdx", true)->value();
      highColorIdx = highColorIdxStr.toInt();
   }

   if (request->hasParam("lColorIdx", true))
   {
      String lostColorIdxStr = request->getParam("lColorIdx", true)->value();
      lostColorIdx = lostColorIdxStr.toInt();
   }

   if (request->hasParam("wColorIdx", true))
   {
      String warningColorIdxStr = request->getParam("wColorIdx", true)->value();
      warningColorIdx = warningColorIdxStr.toInt();
   }

   // check the set colors to defaults selection
   colorDefaults = (boolean)(request->hasParam("colordefaults", true));

   if (colorDefaults)
   {
      forceColorDefaults();
   }

   prefs.begin("colors", false);

   prefs.putInt("sBGColorIdx", screenBGColorIdx);
   prefs.putInt("tColorIdx", timeColorIdx);
   prefs.putInt("dColorIdx", dateColorIdx);
   prefs.putInt("lFGColorIdx", labelFGColorIdx);
   prefs.putInt("lBGColorIdx", labelBGColorIdx);
   prefs.putInt("eColorIdx", edgeColorIdx);
   prefs.putInt("nColorIdx", normalColorIdx);
   prefs.putInt("mColorIdx", mediumColorIdx);
   prefs.putInt("hColorIdx", highColorIdx);
   prefs.putInt("lColorIdx", lostColorIdx);
   prefs.putInt("wColorIdx", warningColorIdx);

   prefs.end();

   colorUpdateNeeded = true;

   // always return to false
   colorDefaults = false;

   // show config page again
   request->redirect("/colors");
}  // webSetColors()


void webSetConfig(AsyncWebServerRequest * request)
{
   String new_banner_title = String();
   int showModeInt;

   prefs.begin("config", false);

   // Save data display selections
   showSolarSelection =   (boolean)(request->hasParam("showSolar", true));
   showEnvSelection =     (boolean)(request->hasParam("showEnv", true));
   showWeatherSelection = (boolean)(request->hasParam("showWeather", true));
   showMLSSelection =     (boolean)(request->hasParam("showMLS", true));
   showGPSSelection =     (boolean)(request->hasParam("showGPS", true));

   showModeInt = 0;
   if (showSolarSelection)
   {
      showModeInt += SHOW_MODE_SOLAR_BIT_MASK;
   }
   if (showEnvSelection)
   {
      showModeInt += SHOW_MODE_ENV_BIT_MASK;
   }
   if (showWeatherSelection)
   {
      showModeInt += SHOW_MODE_WEATHER_BIT_MASK;
   }
   if (showMLSSelection)
   {
      showModeInt += SHOW_MODE_MLS_BIT_MASK;
   }
   if (showGPSSelection)
   {
      showModeInt += SHOW_MODE_GPS_BIT_MASK;
   }

   showMode = (SHOW_MODE_TYPE)showModeInt;

   showModeFlag = true;
   showModeDelayCount = CYCLE_TIME_IN_SECONDS;

   switch (showMode)
   {
      case SHOW_MODE_NONE:
         {
         }
         break;

      case SHOW_MODE_SOLAR_ONLY:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update
         }
         break;

      case SHOW_MODE_ENV_ONLY:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update
         }
         break;

      case SHOW_MODE_WEATHER_ONLY:
         {
            dataIndex = SHOW_FIRST_WEATHER_DATA;

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_WEATHER:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_ENV_PLUS_WEATHER:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_MLS_ONLY:
         {
            dataIndex = SHOW_DATA_MLS;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update
         }
         break;

      case SHOW_MODE_ENV_PLUS_MLS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update
         }
         break;

      case SHOW_MODE_WEATHER_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_WEATHER_DATA;

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS:
         {
            dataIndex = SHOW_DATA_ENV;

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_GPS_ONLY:
         {
            dataIndex = SHOW_FIRST_GPS_DATA;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update
         }
         break;

      case SHOW_MODE_ENV_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update
         }
         break;

      case SHOW_MODE_WEATHER_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_WEATHER_DATA;

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_ENV;

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_MLS;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update
         }
         break;

      case SHOW_MODE_ENV_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_ENV;
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update
         }
         break;

      case SHOW_MODE_WEATHER_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_WEATHER_DATA;

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_DATA_ENV;

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;

      case SHOW_MODE_SOLAR_PLUS_ENV_PLUS_WEATHER_PLUS_MLS_PLUS_GPS:
         {
            dataIndex = SHOW_FIRST_SOLAR_DATA;

            solarDelayCount = 5;                                               // trigger a solar update

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // trigger a weather update
         }
         break;
   }

   prefs.putInt("showMode", int(showMode));

   // Save Banner Title
   if (request->hasParam("title", true))
   {
      new_banner_title = request->getParam("title", true)->value();

      // if the new banner title is not empty
      if (new_banner_title.length())
      {
         if (new_banner_title.length() > 8)
         {
            new_banner_title = new_banner_title.substring(0, 8);
         }

         // if the new banner title is different
         if (bannerTitle != new_banner_title)
         {
            bannerTitle = new_banner_title;

            prefs.putString("title", bannerTitle);

            newDualScreen();
         }
      }
   }

   // Save GPS active mode
   if (request->hasParam("gpsmode", true))
   {
      String gpsmode = request->getParam("gpsmode", true)->value();
      gpsActiveMode = gpsmode.toInt();
   }

   // Save GPS manual latitude
   if (request->hasParam("gpslat", true))
   {
      String gpslat = request->getParam("gpslat", true)->value();
      gpsManualLat = gpslat;

      prefs.putString("gpsManualLat", gpsManualLat);                           // GPS manual mode latitude
   }

   // Save GPS manual longitude
   if (request->hasParam("gpslon", true))
   {
      String gpslon = request->getParam("gpslon", true)->value();
      gpsManualLon = gpslon;

      prefs.putString("gpsManualLon", gpsManualLon);                           // GPS manual mode longitude
   }

   // Save # Decimal Places to Use for Weather Lat/Lon
   if (request->hasParam("decimals", true))
   {
      String decimals = request->getParam("decimals", true)->value();
      int8_t decplaces = decimals.toInt();

      if (decplaces < 1)
      {
         decplaces = 1;
      }

      if (decplaces > 6)
      {
         decplaces = 6;
      }

      weatherLatLonDecimalPlaces = decplaces;

      prefs.putInt("decimals", weatherLatLonDecimalPlaces);
   }

   // Save TZ Countdown Interval
   if (request->hasParam("tzcountdown", true))
   {
      String tzc = request->getParam("tzcountdown", true)->value();

      tzCountdownIntervalSetting = tzc.toInt();

      if (tzCountdownIntervalSetting < 1)
      {
         tzCountdownIntervalSetting = 1;
      }

      if (tzCountdownIntervalSetting > 30)
      {
         tzCountdownIntervalSetting = 30;
      }

      tzCountdownInterval = 1;                                                 // force a change in the TZ display in the next second & apply the new interval

      prefs.putInt("tzInterval", tzCountdownIntervalSetting);
   }

   // save the show EST/EDT timezone selection
   showESTEDT = (boolean)(request->hasParam("showest", true));
   prefs.putBool("showest", showESTEDT);

   // save the show CST/CDT timezone selection
   showCSTCDT = (boolean)(request->hasParam("showcst", true));
   prefs.putBool("showcst", showCSTCDT);

   // save the show MST/MDT timezone selection
   showMSTMDT = (boolean)(request->hasParam("showmst", true));
   prefs.putBool("showmst", showMSTMDT);

   showMSTNOMDT = (boolean)(request->hasParam("showmstonly", true));
   prefs.putBool("showmstonly", showMSTNOMDT);

   // save the show PST/PDT timezone selection
   showPSTPDT = (boolean)(request->hasParam("showpst", true));
   prefs.putBool("showpst", showPSTPDT);

   // save the show AWST timezone selection
   showAWST = (boolean)(request->hasParam("showawst", true));
   prefs.putBool("showawst", showAWST);

   // save the show ACST/ACDT timezone selection
   showACSTACDT = (boolean)(request->hasParam("showacst", true));
   prefs.putBool("showacst", showACSTACDT);

   // save the show AEST/AEDT timezone selection
   showAESTAEDT = (boolean)(request->hasParam("showaest", true));
   prefs.putBool("showaest", showAESTAEDT);

   // save the show GMT/BST timezone selection
   showGMTBST = (boolean)(request->hasParam("showgmt", true));
   prefs.putBool("showgmt", showGMTBST);

   // make sure that at least one timezone is selected (you choose which it is)
   if (!showESTEDT && !showCSTCDT && !showMSTMDT && !showMSTNOMDT && !showPSTPDT && !showAWST && !showACSTACDT && !showAESTAEDT && !showGMTBST)
   {
      showCSTCDT = true;
      prefs.putBool("showcst", showCSTCDT);
   }

   // Save AM/PM mode
   if (request->hasParam("showampm", true))
   {
      String ampmmode = request->getParam("showampm", true)->value();
      showAMPMMode = (SHOW_AMPM_TYPE)ampmmode.toInt();

      prefs.putInt("showAMPM", (int)showAMPMMode);
   }

   // save the local 24 hour format selection
   localFormat24hr = (boolean)(request->hasParam("local24hr", true));
   prefs.putBool("local24hr", localFormat24hr);

   // save the UTC 24 hour format selection
   utcFormat24hr = (boolean)(request->hasParam("utc24hr", true));
   prefs.putBool("utc24hr", utcFormat24hr);

   // save the hour leading zero format selection
   hourLeadingZero = (boolean)(request->hasParam("hrlead0", true));
   prefs.putBool("hrlead0", hourLeadingZero);

   // save the date leading zero format selection
   dateLeadingZero = (boolean)(request->hasParam("dtlead0", true));
   prefs.putBool("dtlead0", dateLeadingZero);

   // save the date above month format selection
   dateAboveMonth = (boolean)(request->hasParam("dtovermo", true));
   prefs.putBool("dtovermo", dateAboveMonth);

   // save the show 5VDC measurement selection
   fiveVoltDisplay = (boolean)(request->hasParam("show5vdc", true));
   prefs.putBool("show5vdc", fiveVoltDisplay);

   // Save the 5VDC Supply Calibration
   if (request->hasParam("fivevoltcorr", true))
   {
      fiveVoltCorrection = request->getParam("fivevoltcorr", true)->value();
      prefs.putString("fivevoltcorr", fiveVoltCorrection);
   }

   // save the show 12VDC measurement selection
   twelveVoltDisplay = (boolean)(request->hasParam("show12vdc", true));
   prefs.putBool("show12vdc", twelveVoltDisplay);

   // Save the 12VDC Supply Calibration
   if (request->hasParam("twelvevoltcorr", true))
   {
      twelveVoltCorrection = request->getParam("twelvevoltcorr", true)->value();
      prefs.putString("twelvevoltcorr", twelveVoltCorrection);
   }

   // save the printed time selection
   if (request->hasParam("printtime", true))
   {
      String ptime = request->getParam("printtime", true)->value();
      printedTime = (PRINTED_TIME_TYPE)ptime.toInt();

      prefs.putInt("printtime", (int)printedTime);
   }

   // save the show blink time update selection
   blinkTimeUpdate = (boolean)(request->hasParam("blinktime", true));
   prefs.putBool("blinktime", blinkTimeUpdate);

   // save the show selected data header selection
   showDataHeaderBetweenDataGroups = (boolean)(request->hasParam("showDataHeader", true));
   prefs.putBool("showDataHeader", showDataHeaderBetweenDataGroups);

   // save the QTH altitude selection
   if (request->hasParam("qthaltft", true))
   {
      qthAltitudeInFeet = request->getParam("qthaltft", true)->value();

      prefs.putString("qthaltft", qthAltitudeInFeet);
   }

   // save the temperature offset selection
   if (request->hasParam("tempoffset", true))
   {
      String tempoffset = request->getParam("tempoffset", true)->value();
      temperatureOffsetInCelsius = tempoffset;

      prefs.putString("tempoffset", temperatureOffsetInCelsius);
   }

   // save the humidity offset selection
   if (request->hasParam("humoffset", true))
   {
      String humoffset = request->getParam("humoffset", true)->value();
      humidityOffset = (int32_t)humoffset.toInt();

      prefs.putInt("humoffset", (int)humidityOffset);
   }

   // Save The Set The Internal Clock from GPS Interval
   if (request->hasParam("gpsInterval", true))
   {
      String gps = request->getParam("gpsInterval", true)->value();

      useGPSToSetInternalClockIntervalInSeconds = gps.toInt();

      if (useGPSToSetInternalClockIntervalInSeconds < 60)
      {
         useGPSToSetInternalClockIntervalInSeconds = 60;
      }

      if (useGPSToSetInternalClockIntervalInSeconds > 3600)
      {
         useGPSToSetInternalClockIntervalInSeconds = 3600;
      }

      gpsDelayCount = 1;                                                       // force a change in the GPS time fetch interval in the next second & apply the new interval

      prefs.putInt("gpsInterval", useGPSToSetInternalClockIntervalInSeconds);
   }

   // Save Weather Fetch Interval
   if (request->hasParam("wfInterval", true))
   {
      String wfi = request->getParam("wfInterval", true)->value();

      weatherFetchIntervalInSeconds = wfi.toInt();

      if (weatherFetchIntervalInSeconds < 10)
      {
         weatherFetchIntervalInSeconds = 10;
      }

      if (weatherFetchIntervalInSeconds > 3600)
      {
         weatherFetchIntervalInSeconds = 3600;
      }

      weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;              // force a change in the weather fetch interval in the next second & apply the new interval

      prefs.putInt("wfInterval", weatherFetchIntervalInSeconds);
   }

   // Save Weather appID
   if (request->hasParam("appID", true))
   {
      weatherAppID = request->getParam("appID", true)->value();

      prefs.putString("appID", weatherAppID);

      appID = "&appid=" + weatherAppID;
   }

   // Done with the preferences
   prefs.end();

   // Show config page again
   request->redirect("/config");
}  // webSetConfig()


void webSetDebug(AsyncWebServerRequest * request)
{
   prefs.begin("debug", false);

   // save the show stack usage debug selection
   debugStackUsage = (boolean)(request->hasParam("debugstack", true));
   prefs.putBool("debugstack", debugStackUsage);

   // save the show ENV sensor debug selection
   debugEnvSensor = (boolean)(request->hasParam("debugenv", true));
   prefs.putBool("debugenv", debugEnvSensor);

   // save the show light sensor debug selection
   debugLightSensor = (boolean)(request->hasParam("debuglight", true));
   prefs.putBool("debuglight", debugLightSensor);

   // save the show supply voltage debug selection
   debugSupplyVoltage = (boolean)(request->hasParam("debugvolts", true));
   prefs.putBool("debugvolts", debugSupplyVoltage);

   // save the show solar debug selection
   debugSolar = (boolean)(request->hasParam("debugsolar", true));
   prefs.putBool("debugsolar", debugSolar);

   // save the show weather debug selection
   debugWeather = (boolean)(request->hasParam("debugweather", true));
   prefs.putBool("debugweather", debugWeather);

   // save the show WiFi connection debug selection
   debugWiFiConnect = (boolean)(request->hasParam("debugwifi", true));
   prefs.putBool("debugwifi", debugWiFiConnect);

   // save the show GPS time debug selection
   debugGPSinitialization = (boolean)(request->hasParam("debuggpsinit", true));
   prefs.putBool("debuggpsinit", debugGPSinitialization);

   // save the show GPS position debug selection
   debugGPSposition = (boolean)(request->hasParam("debuggpspos", true));
   prefs.putBool("debuggpspos", debugGPSposition);

   // save the show GPS time debug selection
   debugGPStime = (boolean)(request->hasParam("debuggpstime", true));
   prefs.putBool("debuggpstime", debugGPStime);

   // save the show GPS altitude debug selection
   debugGPSaltitude = (boolean)(request->hasParam("debuggpsalt", true));
   prefs.putBool("debuggpsalt", debugGPSaltitude);

   // save the show GPS fix state debug selection
   debugGPSfix = (boolean)(request->hasParam("debuggpsfix", true));
   prefs.putBool("debuggpsfix", debugGPSfix);

   prefs.end();

   // Show config page again
   request->redirect("/debug");
}  // webSetDebug()


void webSetNetwork(AsyncWebServerRequest * request)
{
   boolean haveSSID = false;
   String new_ap_ssid = String();
   String new_ap_channel = String();

   // Start modifying preferences
   prefs.begin("network", false);

   // Save AP name
   if (request->hasParam("apname", true))
   {
      new_ap_ssid = request->getParam("apname", true)->value();

      // if the new AP SSID is not empty
      if (new_ap_ssid.length())
      {
         // if the new AP SSID is different
         if (String(apSSID) != new_ap_ssid)
         {
            strcpy(apSSID, new_ap_ssid.c_str());

            prefs.putString("apName", String(apSSID));

            // request a WiFi restart
            itIsTimeToWiFi = true;
         }
      } else {
         strcpy(apSSID, DEFAULT_AP_NAME);

         prefs.putString("apName", String(apSSID));

         // request a WiFi restart
         itIsTimeToWiFi = true;
      }
   }

   // Save AP channel
   if (request->hasParam("apchannel", true))
   {
      new_ap_channel = request->getParam("apchannel", true)->value();

      // if the new AP channel is not empty
      if ((new_ap_channel.length()) && (new_ap_channel.toInt() >= 1) && (new_ap_channel.toInt() <= 13))
      {
         // if the new AP channel is different
         if (String(apChannel) != new_ap_channel)
         {
            apChannel = new_ap_channel.toInt();

            prefs.putInt("apChannel", apChannel);

            // request a WiFi restart
            itIsTimeToWiFi = true;
         }
      } else {
         apChannel = DEFAULT_AP_CHANNEL;

         prefs.putInt("apChannel", apChannel);

         // request a WiFi restart
         itIsTimeToWiFi = true;
      }


      // Save user name and password
      if (request->hasParam("username", true) && request->hasParam("password", true))
      {
         loginUsername = request->getParam("username", true)->value();
         loginPassword = request->getParam("password", true)->value();

         prefs.putString("loginusername", loginUsername);
         prefs.putString("loginpassword", loginPassword);
      }

      for (int j = 0 ; j < NUM_WIFI_SLOTS ; j++)
      {
         char nameSSID[16], namePASS[16];

         sprintf(nameSSID, "wifissid%d", j + 1);
         sprintf(namePASS, "wifipass%d", j + 1);

         if (request->hasParam(nameSSID, true) && request->hasParam(namePASS, true))
         {
            String ssid = request->getParam(nameSSID, true)->value();
            String pass = request->getParam(namePASS, true)->value();
            prefs.putString(nameSSID, ssid);
            prefs.putString(namePASS, pass);
            haveSSID |= ssid != "" && pass != "";
         }
      }
   }

   // Done with the preferences
   prefs.end();

   // save the hide passwords selection
   hidePasswords = (boolean)(request->hasParam("hidepass", true));

   // show config page again
   request->redirect("/network");

   // If there is at least one SSID / PASS pair, &we're not already connected, request network connection
   if (haveSSID && (getWiFiStatus() != WIFI_NET_CONNECTED))
   {
      // request a WiFi restart
      itIsTimeToWiFi = true;
   }
}  // webSetNetwork()


const String webShowNetworkScan(void)
{
   String result = String();
   String scan_ssid = String();
   String scan_rssi = String();
   String scan_channel = String();
   String scan_encryption = String();
   char str_rssi[16];
   char str_channel[8];

   if (getWiFiStatus() != WIFI_NET_CONNECTED)
   {
      WiFi.disconnect();
   }

   int scanResult = WiFi.scanNetworks();

   Serial.println("...completed WiFi scan");

   result += "<TR><TH COLSPAN=4 CLASS='HEADING'>WiFi Scan Found These 2.4GHz APs Within Range:</TH></TR>";
   result += "<TR><TH>SSID</TH><TH>RSSI</TH><TH>CH</TH><TH>CIPHER</TH></TR>";

   if (scanResult <= 0)
   {
      result += String("<TR><TD>(none found)</TD></TR>");
   } else {
      for (int i = 0; i < scanResult; ++i)
      {
         scan_ssid = String(WiFi.SSID(i).c_str());

         sprintf(str_rssi, "%d", WiFi.RSSI(i));
         scan_rssi = String(str_rssi);

         sprintf(str_channel, "%d", WiFi.channel(i));
         scan_channel = String(str_channel);

         switch (WiFi.encryptionType(i)) {
            case WIFI_AUTH_OPEN:            scan_encryption = String("OPEN"); break;
            case WIFI_AUTH_WEP:             scan_encryption = String("WEP"); break;
            case WIFI_AUTH_WPA_PSK:         scan_encryption = String("WPA"); break;
            case WIFI_AUTH_WPA2_PSK:        scan_encryption = String("WPA2"); break;
            case WIFI_AUTH_WPA_WPA2_PSK:    scan_encryption = String("WPA+WPA2"); break;
            case WIFI_AUTH_WPA2_ENTERPRISE: scan_encryption = String("WPA2-EAP"); break;
            case WIFI_AUTH_WPA3_PSK:        scan_encryption = String("WPA3"); break;
            case WIFI_AUTH_WPA2_WPA3_PSK:   scan_encryption = String("WPA2+WPA3"); break;
            case WIFI_AUTH_WAPI_PSK:        scan_encryption = String("WAPI"); break;
            default:                        scan_encryption = String("UNKNOWN");
         }

         result += "<TR>";
         result += "<TD>" + scan_ssid + "</TD>"
                   "<TD>" + scan_rssi + "</TD>"
                   "<TD>" + scan_channel + "</TD>"
                   "<TD>" + scan_encryption + "</TD>"
                   "</TR>";

         delay(10);
      }
   }

   // Delete the scan result to free memory for code below.
   WiFi.scanDelete();

   return (result);
}  // webShowNetworkScan()


const String webStatusPage()
{
   String ap_ip = String();
   String wifi_ip = String();
   String ap_ssid = String();
   String wifi_ssid = String();

   if (getWiFiStatus() == WIFI_NET_CONNECTED)
   {
      wifi_ssid = "connected to: " + WiFi.SSID();
   }
   else
   {
      wifi_ssid = String("not connected");
   }

   ap_ip = WiFi.softAPIP().toString();
   ap_ssid = String(apSSID);
   wifi_ip = WiFi.localIP().toString();

   return webPage(
             "<H1>NTPclock [STATUS]</H1>"
             "<P ALIGN='CENTER'>"
             "<A HREF='/'>[ STATUS ]</A>&nbsp;&nbsp;<A HREF='/network'>[ NETWORK ]</A>&nbsp;&nbsp;<A HREF='/colors'>[ COLORS ]</A>&nbsp;&nbsp;<A HREF='/config'>[ CONFIG ]</A>"
             + String(debugIsEnabled ? "<BR><BR><A HREF='/scan'>[ SCAN ]</A>&nbsp;&nbsp;<A HREF='/debug'>[ DEBUG ]</A>" : "<BR><BR><A HREF='/scan'>[ SCAN ]</A>") +
             "</P>"
             "<TABLE>"
             "<TR><TH CLASS='HEADING'>(NOTE: A pause may occur when going to/from [SCAN])</TH></TR>"
             "</TABLE>"
             "<BR>"
             "<TABLE COLUMNS=2>"
             "<TR>"
             "<TD><CLASS='LABEL'>Version:</TD>"
             "<TD>" + (String)VERSION_TIMESTAMP + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>AP:</TD>"
             "<TD><A HREF='http://" + ap_ip + "'>" + ap_ip + "</A>   (" + ap_ssid + ")</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>WiFi:</TD>"
             "<TD>(" + wifi_ssid + ")</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>WiFi IP:</TD>"
             "<TD>" + wifi_ip + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>Channel:</TD>"
             "<TD>" + String(apChannel) + "</TD>"
             "</TR>"
             "<TR>"
             "<TD><CLASS='LABEL'>MAC:</TD>"
             "<TD>" + String(WiFi.macAddress()) + "</TD>"
             "</TR>"
             "</TABLE>"
          );
}  // webStatusPage()


const String webStyleSheet(void)
{
   return
      "BODY"
      "{"
      "margin: 0;"
      "padding: 0;"
      "}"
      "H1"
      "{"
      "text-align: center;"
      "}"
      "TABLE"
      "{"
      "width: 100%;"
      "max-width: 768px;"
      "margin-left: auto;"
      "margin-right: auto;"
      "}"
      "TH, TD"
      "{"
      "border: 2px solid black;"
      "padding-top: 0.0em;"
      "padding-bottom: 0.0em;"
      "text-align: left;"
      "}"
      "TH.HEADING"
      "{"
      "background-color: #80A0FF;"
      "column-span: all;"
      "text-align: center;"
      "}"
      "TD.LABEL"
      "{"
      "text-align: right;"
      "}"
      "INPUT[type=text], INPUT[type=password], SELECT"
      "{"
      "width: 95%;"
      "padding: 0.5em;"
      "}"
      "INPUT[type=submit]"
      "{"
      "width: 50%;"
      "padding: 0.5em 0;"
      "}"
      ".CENTER"
      "{"
      "text-align: center;"
      "}"
      ;
}  // webStylePage()


void wifiInitAP(void)
{
   // These are our own access point (AP) addresses
   IPAddress ip(10, 1, 1, 1);
   IPAddress gateway(10, 1, 1, 1);
   IPAddress subnet(255, 255, 255, 0);

   // Get the preferences
   prefs.begin("network", true);

   if (prefs.isKey("apName"))
   {
      strcpy(apSSID, (prefs.getString("apName", String(DEFAULT_AP_NAME))).c_str());
   }

   if (prefs.isKey("apChannel"))
   {
      apChannel = prefs.getInt("apChannel", DEFAULT_AP_CHANNEL);
   }

   // Done with preferences
   prefs.end();

   startupScreen();

   tft.setTextDatum(TC_DATUM);
   tft.setFreeFont(&FreeSansBold9pt7b);

   tft.drawString("Activating Access Point", 160, 50, 2);

   tft.drawString(("Local AP: " + String(apSSID)).c_str(), 160, 140, 2);
   tft.drawString("For CONFIG/STATUS, connect to Local AP,", 160, 170, 2);
   tft.drawString("then open a browser to 10.1.1.1", 160, 200, 2);

   // Start as access point (AP)
   WiFi.softAP(apSSID, apPWD, apChannel, apHideMe, apClientsMax);
   WiFi.softAPConfig(ip, gateway, subnet);

   tft.drawString("-- COMPLETE --", 160, 100, 2);

   ajaxInterval = 2500;

   for (int k = 0; k < 100; k++)
   {
      delay(10);
   }

   tft.setTextDatum(TL_DATUM);
}  // wifiInitAP()


boolean wifiConnect(void)
{
   String status = String();
   String ssid = String();
   String password = String();

   boolean wifiConnected = false;

   // Get the preferences
   prefs.begin("network", true);

   if (prefs.isKey("loginusername"))
   {
      loginUsername = prefs.getString("loginusername", "");
   }
   if (prefs.isKey("loginpassword"))
   {
      loginPassword = prefs.getString("loginpassword", "");
   }

   // Try connecting to known WiFi networks
   for (int j = 0 ; (j < NUM_WIFI_SLOTS) && (getWiFiStatus() != WIFI_NET_CONNECTED) ; j++)
   {
      ssid = String();
      password = String();

      char nameSSID[16], namePASS[16];
      sprintf(nameSSID, "wifissid%d", j + 1);
      sprintf(namePASS, "wifipass%d", j + 1);

      if (prefs.isKey(nameSSID))
      {
         ssid = prefs.getString(nameSSID, "");
      }
      if (prefs.isKey(namePASS))
      {
         password = prefs.getString(namePASS, "");
      }

      if (ssid != "")  // if the SSID is not empty
      {
         if (debugWiFiConnect)
         {
            Serial.print("Attempting to connect to '");
            Serial.print(ssid);
            Serial.print("'");
         }

         status = "Connecting to WiFi network..";

         WiFi.disconnect(true, true);
         WiFi.mode(WIFI_AP_STA);

         startupScreen();

         tft.setTextDatum(TC_DATUM);

         tft.drawString(("Local AP: " + String(apSSID)).c_str(), 160, 140, 2);
         tft.drawString("For CONFIG/STATUS, connect to Local AP,", 160, 170, 2);
         tft.drawString("then open a browser to 10.1.1.1", 160, 200, 2);

         tft.setFreeFont(&FreeSansBold9pt7b);

         WiFi.begin(ssid, password);

         // try each WiFi definition for appriximately 8 seconds
         for (int k = 0 ; (getWiFiStatus() != WIFI_NET_CONNECTED) && (k <= 860) ; k++)
         {
            if (!(k % 80))
            {
               if (debugWiFiConnect)
               {
                  Serial.print(".");
               }

               status += ".";

               tft.drawString(status, 160, 50, 2);
               tft.drawString(ssid, 160, 80, 2);
            }
            delay(10);
         }

         // If failed connecting to WiFi network...
         if (getWiFiStatus() != WIFI_NET_CONNECTED)
         {
            if (debugWiFiConnect)
            {
               Serial.println("FAILED");
            }

            tft.drawString("-- FAILED --", 160, 100, 2);

            wifiConnected = false;
         }
         else
         {
            if (debugWiFiConnect)
            {
               Serial.println("CONNECTED");
            }

            tft.drawString("-- CONNECTED --", 160, 100, 2);

            ajaxInterval = 1000;
            wifiConnected = true;

            weatherDelayCount = WEATHER_FAIL_RETRY_INTERVAL_IN_SECONDS;        // force a change in the weather fetch countdown in the next second & apply the new interval
         }

         for (int k = 0; k < 10; k++)
         {
            delay(10);
         }
      }
   }

   // Done with preferences
   prefs.end();

   tft.setTextDatum(TL_DATUM);

   return (wifiConnected);
}  // wifiConnect()


// EOF placeholder
