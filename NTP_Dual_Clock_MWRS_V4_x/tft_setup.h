
//#define ILI9341_DRIVER
#define ST7789_DRIVER
#define TFT_RGB_ORDER TFT_BGR
#define TFT_INVERSION_OFF
#define USE_HSPI_PORT

// activate/uncomment one of these build flags to build for a target different from the ESP32-S2-mini (default)
//    NOTE: don't forget to also select the correct/corresponding processor type in the Arduino IDE
//#define ESP32S3_SUPERMINI
//#define ESP32S3_MINI
//#define ESP32S3_MJC_TESTBED

#define TFT_MOSI  11   // pin  6
#define TFT_MISO  -1   // pin  5
#define TFT_SCLK   7   // pin  4
#define TFT_RST   -1   // not used
#define TFT_CS    12   // pin  7
#define TFT_DC    18   // pin 12

#define SCREEN_ORIENTATION           1                // Screen portrait mode:  use 1 or 3


#ifdef ESP32S3_MINI

#undef TFT_MOSI
#undef TFT_MISO
#undef TFT_SCLK
#undef TFT_RST
#undef TFT_CS
#undef TFT_DC
#undef SCREEN_ORIENTATION

#define TFT_MOSI  11   // pin  6
#define TFT_MISO  -1   // pin  5
#define TFT_SCLK  12   // pin  4
#define TFT_RST   -1   // not used
#define TFT_CS    10   // pin  7
#define TFT_DC    18   // pin 12

#define SCREEN_ORIENTATION           1                // Screen portrait mode:  use 1 or 3

#endif

#ifdef ESP32S3_MJC_TESTBED

#undef TFT_MOSI
#undef TFT_MISO
#undef TFT_SCLK
#undef TFT_RST
#undef TFT_CS
#undef TFT_DC
#undef SCREEN_ORIENTATION

#define TFT_MOSI  11
#define TFT_MISO  12
#define TFT_SCLK  13
#define TFT_RST    5
#define TFT_CS     6
#define TFT_DC     7

#define SCREEN_ORIENTATION           3                // Screen portrait mode:  use 1 or 3

#endif

#ifdef ESP32S3_SUPERMINI

#undef TFT_MOSI
#undef TFT_MISO
#undef TFT_SCLK
#undef TFT_RST
#undef TFT_CS
#undef TFT_DC
#undef SCREEN_ORIENTATION

#define TFT_MOSI   7
#define TFT_MISO  -1
#define TFT_SCLK   9
#define TFT_RST   -1
#define TFT_CS     6
#define TFT_DC    10

#define SCREEN_ORIENTATION           1                // Screen portrait mode:  use 1 or 3

#endif

#define CGRAM_OFFSET
#define TFT_RGB_ORDER TFT_BRG  // Colour order Blue-Red-Green

#define TFT_BACKLIGHT_ON HIGH

#define TFT_WIDTH 240
#define TFT_HEIGHT 320

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
