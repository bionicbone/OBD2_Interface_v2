// See SetupX_Template.h for all options available
#define USER_SETUP_ID 42

#define ILI9341_DRIVER

//// Setting for the OBD2_Interface_v1 on LOLIN32 ESP32
//#define TFT_MISO 12  // (leave TFT SDO disconnected if other SPI devices share MISO)
//#define TFT_MOSI 13
//#define TFT_SCLK 2
//#define TFT_CS   4   // Chip select control pin
//#define TFT_DC   15  // Data Command control pin
//#define TFT_RST  5   // Reset pin (could connect to RST pin)

//// Setting for the OBD2_Interface_v2 on ESP32
//#define TFT_MISO 35		// (leave TFT SDO disconnected if other SPI devices share MISO)
//#define TFT_MOSI 25
//#define TFT_SCLK 33
//#define TFT_CS   14		// Chip select control pin
//#define TFT_DC   26		// Data Command control pin
//#define TFT_RST  27   // Reset pin (could connect to RST pin)
//// FSPI (or VSPI) port (SPI2) used unless following defined. HSPI port is (SPI3) on S3.
//#define USE_HSPI_PORT

// Setting for the OBD2_Interface_v2 on ESP32-S3
#define TFT_MISO 38		// (leave TFT SDO disconnected if other SPI devices share MISO)
#define TFT_MOSI 41
#define TFT_SCLK 40
#define TFT_CS   1 		// Chip select control pin
#define TFT_DC   42		// Data Command control pin
#define TFT_RST  2    // Reset pin (could connect to RST pin)
// FSPI (or VSPI) port (SPI2) used unless following defined. HSPI port is (SPI3) on S3.
#define USE_HSPI_PORT // Use HSPI because the MCP2515 CAN Controllers are on VSPI

// Optional touch screen chip select
//#define TOUCH_CS 5 // Chip select pin (T_CS) of touch screen

// TODO  For the OBD2_Interfacce_v2 do I really need to load all these fonts ?
//#define LOAD_GLCD    // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
//#define LOAD_FONT2   // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
//#define LOAD_FONT4   // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
//#define LOAD_FONT6   // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
//#define LOAD_FONT7   // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
//#define LOAD_FONT8   // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_GFXFF   // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

#define SMOOTH_FONT

// FSPI (or VSPI) port (SPI2) used unless following defined. HSPI port is (SPI3) on S3.
//#define USE_HSPI_PORT

// TFT SPI clock frequency
// #define SPI_FREQUENCY  20000000
// #define SPI_FREQUENCY  27000000
//#define SPI_FREQUENCY  40000000
#define SPI_FREQUENCY  80000000

// Optional reduced SPI frequency for reading TFT
#define SPI_READ_FREQUENCY  16000000

// SPI clock frequency for touch controller
#define SPI_TOUCH_FREQUENCY  2500000
