/*
   You can put any string in for the 'TITLE' (e.g. your call sign). Best to keep
   the length to 8 characters or less to fit the allocated space. For now, we
   have the software version number in the 'TITLE' while still developing the code.
*/

#define DEFAULT_TITLE "ver 4.x"                       // Can be any (up to) 8 characters max


/*
   Time Zone rules in "Posix timezone string" format. For an explanation and
   a list of the strings appropriate for various locations, see the following
   web pages:

      https://support.cyberdata.net/portal/en/kb/articles/010d63c0cfce3676151e1f2d5442e311
      https://developer.ibm.com/articles/au-aix-posix/

   There must be at least one entry in the list.  Uncomment the timszones that you want
   as choices to be displayed.
*/

const char timeZones[][50] = {                        // List of timezones that can be potentially be displayed (configure using config webpage)
   "EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00",          // US Eastern time
   "CST6CDT,M3.2.0/2:00:00,M11.1.0/2:00:00",          // US Central time
   "MST7MDT,M3.2.0/2:00:00,M11.1.0/2:00:00",          // US Mountain time
   "MST7",                                            // Arizona time (no DST)
   "PST8PDT,M3.2.0/2:00:00,M11.1.0/2:00:00",          // US Pacific Time
   "AWST-8",                                          // AUS Western time
   "ACST-9:30ACDT,M10.1.0/2:00:00,M4.1.0/2:00:00",    // AUS Central time
   "AEST-10AEDT,M10.1.0/2:00:00,M4.1.0/2:00:00",      // AUS Eastern time
   "GMT1BST,M3.5.0/1:00:00,M10.5.0/2:00:00",          // UK time
};

#define DEFAULT_SHOW_EST_EDT_TZ         false
#define DEFAULT_SHOW_CST_CDT_TZ         true
#define DEFAULT_SHOW_MST_MDT_TZ         false
#define DEFAULT_SHOW_MST_NO_MDT_TZ      false
#define DEFAULT_SHOW_PST_PDT_TZ         false
#define DEFAULT_SHOW_AWST_TZ            false
#define DEFAULT_SHOW_ACST_ACDT_TZ       false
#define DEFAULT_SHOW_AEST_AEDT_TZ       true
#define DEFAULT_SHOW_GMT_BST_TZ         false
/*
   TZ_INTERVAL (should be a value between 1 and 30) defines how long each timezone
   will be displayed before moving onto the next one.
*/
#define DEFAULT_TZ_INTERVAL 3                         // default interval for how long each timezone is displayed

#define DEFAULT_WEATHER_FETCH_INTERVAL_IN_SECONDS 600 // default interval for how often to fetch internet weather

#define DEFAULT_WEATHER_APP_ID "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

#define DEFAULT_LAT_LON_DECIMAL_PLACES 6

typedef enum
{
   SHOW_AMPM_NONE = 0, SHOW_AMPM_SMALL, SHOW_AMPM_LARGE
} SHOW_AMPM_TYPE;

#define DEFAULT_SHOW_AMPM_MODE SHOW_AMPM_SMALL

/*
   These definitions control how the time and date data are displayed.
*/

#define DEFAULT_LOCAL_FORMAT_24HR false               // Local time format 12hr "11:34" vs 24hr "23:34"
#define DEFAULT_UTC_FORMAT_24HR    true               // UTC time format 12 hr "11:34" vs 24hr "23:34"
#define DEFAULT_HOUR_LEADING_ZERO  true               // "01:00" vs " 1:00"
#define DEFAULT_DATE_LEADING_ZERO  true               // "Feb 07" vs. "Feb 7"
#define DEFAULT_DATE_ABOVE_MONTH   true               // "12 Feb" vs. "Feb 12"

#define DEFAULT_5VDC_DISPLAY       true
#define DEFAULT_12VDC_DISPLAY      true

typedef enum
{
   PRINTED_TIME_NONE = 0, PRINTED_TIME_UTC, PRINTED_TIME_LOCAL, PRINTED_TIME_BOTH
} PRINTED_TIME_TYPE;

#define DEFAULT_PRINTED_TIME PRINTED_TIME_NONE


/*
    If the BME280 is installed, some fudge factors will need to be used
   for the readings. Most importantly, the altitude above mean sea level
   must be set, reflecting the clock's actual altitude, not just that of
   the ground level at the current location.
*/

#define DEFAULT_QTH_ALTITUDE_IN_FEET         "700.0"  // Altitude in feet at QTH - set this to current altitude in feet
#define DEFAULT_TEMPERATURE_OFFSET_IN_CELSIUS  "0.0"  // BME280 temperature offset in Celcius, to be added to reported value
#define DEFAULT_HUMIDITY_OFFSET                    0  // Humidity offset to calibrate BME280, if required


/*
   Set for the pressure reading from a nearby airport (METAR).
      Convert Metric to Imperial => Divide by 33
*/

#define DEFAULT_METRIC_PRESSURE_OFFSET     1.0        // Adjustment when using metric uints


#define DEFAULT_AP_NAME      "NTPCLOCK_AP"
#define DEFAULT_AP_CHANNEL   10


#define DEFAULT_WIFI_RETRY_MODE   true                // By default, WiFi reconnect will be automatic

#define DEFAULT_BRIGHTNESS           7                // Default brightness if prefs entry does not exist
#define DEFAULT_NIGHTMODE        false                // Default nightMode if prefs entry does not exist
#define DEFAULT_USEMETRIC        true                 // Default useMetric if prefs entry does not exist

/*
   The following items define the colors used to display the various things.
   The colors used by the Australian folks are currently enabled. The colors
   originally used by the California QRP group are included but commented
   out.

   You can use either set by uncommenting the ones you like and commenting
   out the ones you don't like. You can also make up your own!

   Colour Scheme used for the Cowtown ARC Clock:
*/

#define DEFAULT_SCREEN_BG_COLOR_IDX        0                  // index to the color of the screen background
#define DEFAULT_TIME_COLOR_IDX             4                  // index to the color of the 7-segment time display
#define DEFAULT_DATE_COLOR_IDX             8                  // index to the color of the month & day
#define DEFAULT_LABEL_FGCOLOR_IDX         10                  // index to the color of the label text
#define DEFAULT_LABEL_BGCOLOR_IDX          9                  // index to the color of the label background
#define DEFAULT_EDGE_COLOR_IDX            10                  // index to the color of the edge around the time
#define DEFAULT_NORMAL_COLOR_IDX           4                  // index to the color of normal readings
#define DEFAULT_MEDIUM_COLOR_IDX           2                  // index to the color of medium level readings
#define DEFAULT_HIGH_COLOR_IDX             1                  // index to the color of Maximum or High readings
#define DEFAULT_LOST_COLOR_IDX             1                  // index to the color of Wifi lost
#define DEFAULT_WARNING_COLOR_IDX          3                  // index to the color of warnings
/*
   The following colors are those used when the clock is switched to night
   mode (long press of the brightness button):
*/

#define TIMECOLOR_NIGHT        TFT_RED                // Color of 7-segment time display
#define DATECOLOR_NIGHT        TFT_RED                // Color of displayed month & day
#define LABEL_FGCOLOR_NIGHT    TFT_RED                // Color of label text
#define LABEL_BGCOLOR_NIGHT  TFT_BLACK                // Color of label background
#define NIGHTMODE_COLOR        TFT_RED                // Color to be used in Night Mode
#define NIGHTMODE_BGCOLOR    TFT_BLACK                // Color to be used for night background


/*
   More colors; these are used to indicate that various data elements have
   exceeded predefined limits. For example, When the 'K' index is 4 or higher,
   NOAA shows yellow bars on its graph. When 'K' exceeds 5, NOAA shows red
   bars. We will color code the numbers the same!

   NOAA website is at: https://www.swpc.noaa.gov/products/station-k-and-indices

   These variables define the breakpoints for the solar indicies that are color coded. Those
   for the 'A' and 'K' values correspond to the NOAA breakpoints. The SFI
   breakpoints are arbitrary.
*/

#define MEDIUM_K                     4                // Above 4 it's yellow
#define HIGH_K                       5                // Above 5, red

#define MEDIUM_A                    20                // Above 20 it's yellow
#define HIGH_A                      30                // Above 30, red

#define MEDIUM_SFI                 175                // Above 175 it's yellow
#define HIGH_SFI                   200                // Above 200, red


#define SYNC_MARGINAL             3600                // Orange status if no time sync for 1 hour in seconds
#define SYNC_LOST                86400                // Red status if no time sync for 1 day in seconds
