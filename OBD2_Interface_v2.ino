/*
 Name:		    OBD2_Interface_v2.ino
 Description: Designed Specifically for connection to a Land Rover Freelander 2 VIN: >382339
 Created:	    Sep 2024
 Author:	    Kevin Guest AKA TheBionicBone

  THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  THIS SOFTWARE IS PROVIDED FOR EDUCATIONAL PURPOSES ONLY
*/

// Library: arduino-mcp2515 by autowp information, the follow license should be observed
/*
  This code uses the MCP2515 CANBUS Library:
  arduino-mcp2515 by autowp
  https://github.com/autowp/arduino-mcp2515
  The MIT License (MIT)
  Copyright (c) 2013 Seeed Technology Inc.
  Copyright (c) 2016 Dmitry

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <mcp2515.h>
const String LIBRARY_NAME = "arduino_mpc2515 (by autowp)";

// Library: TFT_ePSI by Bodmer information, the follow license should be observed
/*
  Software License Agreement (FreeBSD License)

  Copyright (c) 2023 Bodmer (https://github.com/Bodmer)

  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  The views and conclusions contained in the software and documentation are those
  of the authors and should not be interpreted as representing official policies,
  either expressed or implied, of the FreeBSD Project.
*/
/*  For the Display & Touch, I am using TFT_eSPI by Bodmer
    See Setup42_ILI9341_ESP32.h for more information
    External Setup Required:-
    * Ensure you are using the Library contained within this project
    * Key points are:
    *   User_Setup_Select.h is pointing to Setup42
    *   Setup42_ILI9341_ESP32.h from this project is being used because this has the correct pins and other requirements like HSPI
*/
#include <TFT_eSPI.h>
#include <FS.h>                                                       // TFT_eSPI requires this arduino-ESP32 core library
#include <SPIFFS.h>                                                   // TFT_eSPI requires this arduino-ESP32 core library
#include <vfs_api.h>                                                  // TFT_eSPI requires this arduino-ESP32 core library
#include <FSImpl.h>                                                   // TFT_eSPI requires this arduino-ESP32 core library
TFT_eSPI TFT_Rectangle_ILI9341 = TFT_eSPI();

// include the arduino-ESP32 core SPI library which is used for the MCP2515 controllers and the ILI9341 Display
#include <SPI.h>
// include the arduino-ESP32 core HardwareSerial library which is used for the SD Card Writer
#include <HardwareSerial.h>
//

// Connections to the ESP32-S3 Follow:
/*
  ESP32-S3 & two MCP2515 Breakout Board (HW-184) - 8 MHz Crystal
  MCP2515_0 for 500kbps CAN Bus
  SCK = pin 12
  SI = pin 11
  SO = pin 13
  CS = pin 14
  GND = GND
  VCC = 5v

  MCP2515_1 for 125kbps CAN Bus
  SCK = pin 12
  SI = pin 11
  SO = pin 13
  CS = pin 10
  GND = GND
  VCC = 5v

  ESP32-S3 OpenLager (SD Card) Connections ("OpenLager" device should not be confused with the slower "OpenLogger") 
  Rx = Pin 9
  GND = GND
  VCC = 5v

  ESP32-S3 ILI9341 Display (with touch) Connections
  T_IRQ = Not Connected
  T_DO = 38
  T_DIN = 41
  T_CS = 37
  T_CLK = 40
  MISO = 38
  LED = 39 (2.5mA)
  SCK = 40
  MOSI = 41
  DC = 42
  RESET = 2
  CS = 1
  GND = GND
  VCC = 3.3v

*/


// Debugging Options (ESP32 Version)
// DEBUG 0 = Debugging Serial Messages are switched off
// DEBUG 1 = Debugging Serial Messages are switched on
#define DEBUG 1

#if DEBUG > 0
#define debugLoop(fmt, ...) Serial.printf("%s: " fmt "\r\n", __func__, ##__VA_ARGS__) // Serial Debugging On
#else
#define debugLoop(fmt, ...)                                           // Serial Debugging Off
#endif


// Enums
enum          CAN_INTERFACES {
              CAN1 =    16,
              CAN2 =    32,
              CANBOTH = 48,
};
enum          MESSAGE_BOX_BUTTONS {
              BTN_OK =      1,
              BTN_IGNORE =  2,
              BTN_CANCEL =  4,
              BTN_STOP =    8, 
              BTN_CAN1 =    16,
              BTN_CAN2 =    32 };
enum          BUTTON_TYPES { 
              MENU, 
              MESSAGE_BOX, 
              DISPLAY_BUTTON };
enum          OUTPUT_TYPES { 
              OUTPUT_ANALYSE_CAN_BUS_RESULTS, 
              OUTPUT_FORMAT_CANDRIVE, 
              OUTPUT_FORMAT_SAVVYCAN, 
              OUTPUT_SD_CARD_SAVVYCAN, 
              OUTPUT_WIFI };


// Constants & ESP32-S3 pin declarations
const auto    STANDARD_SERIAL_OUTPUT_BAUD = 2000000;                  // Must be at least 2,000,000 to keep up with Land Rover Freelander 2 
const auto    SD_PORT_HARDWARE_SERIAL_NUMBER = 1;                     // The OpenLager will be connected to Hardware Serial Port 1
const auto    SD_CARD_ESP32_S3_TX_PIN = 9;                            // The OpenLager Rx pin will be connected to this ESP32-S3 Tx pin
const auto    CAN_BUS_0_CS_PIN = 14;                                  // The CS pin of the MCP2515 (High Speed CAN Bus 500kbps) will be connected to this ESP32-S3 pin
const auto    CAN_BUS_1_CS_PIN = 10;                                  // The CS pin of the MCP2515 (Medium Speed CAN Bus 125kbps) will be connected to this ESP32-S3 pin
const auto    TFT_LANDROVERGREEN = 12832;                             // A Land Rover Green RBG colour
const auto    TFT_Rectangle_ILI9341_LEDPIN = 39;                      // The ILI9341 display LED PIN will be connected to this ESP32-S3 pin
const auto    CALIBRATION_FILE = "/TouchCalData1";                    // This is the file name used to store the touch display calibration data, must start with /
const auto    REPEAT_CALIBRATION = false;                             // Set to true to always calibrate / recalibrate the touch display
const auto    TOP_MENU_FONT = 2;                                      // Font used for selection menu that is at the top of the display
const auto    TOP_MENU_Y_OFFSET = 10;                                 // Used to position the top menu just under the Program Title
const auto    TOP_MENU_PROGRAM_NAME = "OBD2 Interface by Bionicbone"; // Will be displayed at the top of the display
const auto    TOP_MENU_PROGRAM_VERSION = "v2.0.0(RC)";                    // Will be displayed at the top of the display
const auto    MENU_BOLD_FONT = FreeSansBoldOblique9pt7b;              // Font used for selection menus headers
const auto    MENU_FONT = FreeSansOblique9pt7b;                       // Font used for selection menus buttons
const auto    MENU_Y_OFFSET = 68;                                     // Used to position the menu just under the Menu Header

// Control variables
bool          sdCardFirstRun = false;                                 // Will set to true when SD Card initialises, trigger for SavvyCAN header
bool          CANBusFirstRun = false;                                 // Will set to true when CAN Bus mode changes, trigger for cfps timer to start
uint32_t      totalCANReceiveTimeTimer = 0;                           // Times how long we have been receiving CAN Frames
uint16_t      numberOfCANFramesReceived[2] = { 0,0 };                 // Counts the number of CAN Frames received
uint8_t       menuCurrentlyDisplayed = 0;                             // Tracks the menu displayed
const char*   btnText[] = { "","","","","" };                         // Text displayed in each valid button, used for the inversing
uint8_t       outputFormat = false;                                   // Tracks the required output type
uint32_t      upTimer = micros();                                     // Tracks how long the program has been running, used in the outputs


// Sets the new output requirements
void actionOutputChange(uint16_t arg) {
  debugLoop("arg = %d\n", arg);
  outputFormat = arg;
  switch (outputFormat) {
  case OUTPUT_ANALYSE_CAN_BUS_RESULTS:
    OutputAnalyseCANBusResults();
    break;

  // TODO we maybe able to use Menu structure to jump straight to StartReadingCanBus()
  case OUTPUT_FORMAT_CANDRIVE:
    StartReadingCanBus();
    break;

  default:
    break;
  }
}


// Allows user to configure the CAN BUS Settings
void changeCANSettings(uint16_t arg) {
  // TODO write the script
  // TODO I would like an "Auto Detect CAN Bus Speed" function, but first be 100% which CAN library I will be using
  debugLoop("arg = %d\n", arg);
}
                                                                      
     
// Create menus (each menu must have no more than 5 options to choose)
// The order of the menus in the code must consider the order of definitions
// Thus the structure is at the top and the highest menu hierarchy is at the bottom

// Menu Structure
enum { H, M, A };                                                     // H = Header, M = New Menu, A = Action Menu Option 
struct Menu {
  const char* text;
  uint16_t          action;
  Menu* menu;
  void (*func) (uint16_t);
  uint16_t          arg;
};

// Create the menus
// H = Header, M = New Menu, A = Action Menu Option 
Menu CANSettings[]{
  { "CAN Bus Settings", H },
  { "CAN Bus 0 Speed", A, 0, changeCANSettings, 0 },
  { },
};

Menu menuOutput[]{
  { "Select Required Output", H },
  { "Analyse CAN Bus Capacity", A, 0, actionOutputChange, OUTPUT_ANALYSE_CAN_BUS_RESULTS },
  { "Serial CanDrive", A, 0, actionOutputChange, OUTPUT_FORMAT_CANDRIVE },
  { "Serial SavvyCAN", A, 0, actionOutputChange, OUTPUT_FORMAT_SAVVYCAN },
  { "Save SavvyCAN", A, 0, actionOutputChange, OUTPUT_SD_CARD_SAVVYCAN },
  { "WiFi Data", A, 0, actionOutputChange, OUTPUT_WIFI },
  { },
}; 

Menu menuRoot[]{
  // TODO  WARNING: H (header) option is not available in the horizontal menu
  { "CAN Settings", M, CANSettings},
  { "Output Type", M, menuOutput},
  { },
};

// Set and track the corrent menu being displayed
Menu* menu = menuRoot;                                                // Start with the top most menu

// Add MCP2515 Modules
MCP2515 mcp2515_1(CAN_BUS_0_CS_PIN);                                  // Create MCP2515 controller for High Speed CAN Bus 500kbps
MCP2515 mcp2515_2(CAN_BUS_1_CS_PIN);                                  // Create MCP2515 controller for Medium Speed CAN Bus 125kbps
struct can_frame frame;

// Add SD Card Serial Port
HardwareSerial SD_Port(SD_PORT_HARDWARE_SERIAL_NUMBER);               // Connect OpenLager to Hardware Serial Port specified

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button btnMenu[5];



// Setup Serial, SD_CARD, CAN Bus, and Display
// Also check the Arduino CORE used to compile the code has been validated
void setup() {
  // Serial needs to be at least 2,000,000 baud otherwise lines will be dropped when outputting CAN lines to the standard serial output display.
  while (!Serial) { Serial.begin(STANDARD_SERIAL_OUTPUT_BAUD); delay(100); }

  // Display the Serial header
  bool versionMessageBoxRequired = false;
  Serial.printf("\nName        : OBD2_Interface_v2\nCreated     : Sep 2024\nAuthor      : Kevin Guest AKA TheBionicBone\n");
  Serial.printf("Program     : %s\n", TOP_MENU_PROGRAM_VERSION);
  Serial.printf("ESP-IDF     : %s\n",esp_get_idf_version());
  Serial.printf("Arduino Core: v%d.%d.%d\n", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  // Check the ESP32 Arduino Core used to compile the code has been validated 
  if (ESP_ARDUINO_VERSION != ESP_ARDUINO_VERSION_VAL(2, 0, 17)) {
    Serial.printf("\n\n");
    Serial.printf("This ESP Arduino Core Version has not been tested\n");
    Serial.printf("To ensure full compatibility use ESP32 Arduino Core v2.0.17\n");
    Serial.printf("***!!!*** PROCEED WITH CAUTION ***!!!***\n");
    versionMessageBoxRequired = true;
  }

  // Start SD Card
  SDCardStart(SD_CARD_ESP32_S3_TX_PIN);

  // Start the two CAN Bus, 500kbps for High Speed and 125kbps for the Medium Speed and both in ListenOnly Mode
  CANBusStart(mcp2515_1, CAN_500KBPS, 1); 
  CANBusStart(mcp2515_2, CAN_125KBPS, 1);

  // TFT_eSPI runs on HSPI bus, see Setup42_ILI9341_ESP32.h for pin definitions and more information
  TFT_Rectangle_ILI9341.init();
  pinMode(TFT_Rectangle_ILI9341_LEDPIN, OUTPUT);
  digitalWrite(TFT_Rectangle_ILI9341_LEDPIN, HIGH);

  // Calibrate the touch screen and retrieve the scaling factors, see Setup42_ILI9341_ESP32.h for more touch CS Pin definition
  TFT_Rectangle_ILI9341.setRotation(3);
  TFTRectangleILI9341TouchCalibrate();

  // Clear the display and reset the program header
  ClearDisplay();

  // Issue MessageBox about the ESP32 Arduino Core used to compile the code
  if (versionMessageBoxRequired) {
    MessageBox("WARNING", "To ensure full compatibility use ESP32 Arduino Core v2.0.17\n\n*!* USE WITH CAUTION *!*", BTN_OK);
  }
}


// At the end of each function we return here to reset and show the main root menu
void loop() {
  debugLoop("Called");

  // Reset required Global Control Variables
  // TODO - Review each Global Variable to see if it really needs to be Global
  totalCANReceiveTimeTimer = 0;                                       // Times how long we have been receiving CAN Frames
  numberOfCANFramesReceived[0] = 0;                                   // Counts the number of CAN Frames received
  numberOfCANFramesReceived[1] = 0;                                   // Counts the number of CAN Frames received
  menuCurrentlyDisplayed = 0;                                         // Tracks the menu displayed
  btnText[0] = "";                                                    // Text displayed in each valid button, used for the inversing
  btnText[1] = "";                                                    // Text displayed in each valid button, used for the inversing
  btnText[2] = "";                                                    // Text displayed in each valid button, used for the inversing
  btnText[3] = "";                                                    // Text displayed in each valid button, used for the inversing
  btnText[4] = "";                                                    // Text displayed in each valid button, used for the inversing
  outputFormat = false;                                               // Tracks the required output type

  // Clear display and redraw the Title and Menu
  menu = menuRoot;                                                    // After the code function returns jump back to the root menu
  DrawHorizontalMenu(TOP_MENU_Y_OFFSET, MENU_FONT);
}



// Main Program Functions

void StartReadingCanBus() {
    /*
      For information
      As part of the MCP2515 CAN Bus Controllers are 3 Rx buffers in each, thus 3 for the 500kbps, and 3 for the 125kbps CAN Bus.
      500kbps CAN Bus will fill in a minimum period of 666us
      125kbps CAN bus will fill in a minimum period of 2664us
    */
  
  debugLoop("Called");

  // Reset the display
  ClearDisplay();

  uint8_t interfaceNumber = 0;
  
  switch (outputFormat) {
  case OUTPUT_FORMAT_CANDRIVE:
    // CanDrive only supports one interface at a time, user must select one first
    interfaceNumber = MessageBox("Select Interface", "CanDrive only supports reading one CAN interface at a time. Please select which one.", BTN_CAN1 + BTN_CAN2);
    debugLoop("interfaceNumber = %d", interfaceNumber);

    TFT_Rectangle_ILI9341.setTextColor(TFT_GREEN);
    TFT_Rectangle_ILI9341.drawCentreString("Outputting CAN Bus data in CanDrive format", 160, 25, 2);
    TFT_Rectangle_ILI9341.drawCentreString("to the USB Port at 2,000,000 baud.", 160, 45, 2);
    TFT_Rectangle_ILI9341.drawCentreString("Connect PC with CanDrive running to see", 160, 65, 2);
    TFT_Rectangle_ILI9341.drawCentreString("and filter the Live CAN BUS Data.", 160, 85, 2);
    TFT_Rectangle_ILI9341.drawCentreString("Press STOP to end the output.", 160, 105, 2);
    TFT_Rectangle_ILI9341.setTextColor(TFT_BLACK);
    break;

  default:
    break;
  }

  // Create a STOP button to exit
  TFT_Rectangle_ILI9341.setFreeFont(&MENU_FONT);                    // Set the normal button font
  char handler[1] = "";
  uint16_t xButtonWidth = TFT_Rectangle_ILI9341.textWidth("A") * 5;
  uint16_t yButtonHeight = TFT_Rectangle_ILI9341.fontHeight() * 1.2;
  uint16_t xButtonMiddle = TFT_Rectangle_ILI9341.width() - (xButtonWidth / 2);
  uint16_t yButtonMiddle = TFT_Rectangle_ILI9341.height() - (yButtonHeight / 2);
  const char* messageBtnText[1] = { "STOP" };
  btnText[0] = messageBtnText[0];                                   // Must capture btnText for the ProcessButtons() function
  btnMenu[0].initButton(&TFT_Rectangle_ILI9341,
    xButtonMiddle,
    yButtonMiddle,
    xButtonWidth,
    yButtonHeight,
    TFT_YELLOW, TFT_BLUE, TFT_YELLOW, handler, 1);                  // initButton limits the amount of text drawn, draw text in the drawButton() function
  btnMenu[0].drawButton(false, messageBtnText[0]);                  // Specifiy the text for the button because initButton will not display the full text length
  btnMenu[0].press(false);                                          // Because I am reusing buttons it is important to tell the button it is NOT pressed



  while (true) {
    // Check the 500kbps bus as priority over 125kbps because 500kbps is faster
    // and the buffers fill significantly more quickly
    if (CANBusCheckRecieved(mcp2515_1) && (interfaceNumber & CAN1)) {
      if (CANBusReadCANData(mcp2515_1)) {
        debugLoop("mcp2515_1 read");
        CANFrameProcessing(1);
      }
    }
    // only check the 125kbps if there any no 500kbps messages in the MCP2515 buffers
    else if (CANBusCheckRecieved(mcp2515_2) && (interfaceNumber & CAN2)) {
      if (CANBusReadCANData(mcp2515_2)) {
        debugLoop("mcp2515_2 read");
        CANFrameProcessing(2);
      }
    }
    else {
      // If no CAN Bus data use the time to check the STOP Button
      uint8_t result = ProcessButtons(DISPLAY_BUTTON, 1, BTN_STOP, false);
      
      debugLoop("MessageBox result = %d (zero-based indexing)", result);
      
      if (result == BTN_STOP) return;
    }
  }
}


// Processes the CAN Frame and triggers the required output
// For Land Rover Freelander 2 set:
// whichBus = 0 for 500kpbs (High Speed) and 1 for 125kbps (Medium Speed)
void CANFrameProcessing(uint8_t whichCANBus) {
  
  // TODO - implement extend frame detection

  if (CANBusFirstRun) { 
    totalCANReceiveTimeTimer = micros();                              // Set the timer for calculating the CAN Frames per Second (cfps) 
    CANBusFirstRun = false; 
  }

  numberOfCANFramesReceived[whichCANBus]++;


  switch (outputFormat) {
  case OUTPUT_FORMAT_CANDRIVE:
    OutputFormatCanDrive(frame.can_id, frame.can_dlc, frame.data, whichCANBus);
      break;

  default:
    break;
  }
  // Trigger the correct output here

  //SDCardCANFrameSavvyCANOutput(whichCANBus);
}




// SD Card Functions

// Starts the SD Card Device (OpenLager) @ 2,000,000 baud on ESP32-S3 TxPin
void SDCardStart(uint8_t TxPin) {
  debugLoop("Called");

  // Due to the writing speed necessary I am using an STM32F411 based "OpenLager" (not to be confused with the slower "OpenLogger") 
  while (!SD_Port) {
    SD_Port.begin(2000000, SERIAL_8N1, -1, TxPin);                    // Rx pin in not required, we will not use any OpenLager read options
    delay(500);
  }
  delay(1000);                                                        // For start up stability (corruption was noticed if we try to write immediately)
  debugLoop("SD Card writer initialised\n");
  sdCardFirstRun = true;
}

// Writes a CAN Frame to the SD Card Device (OpenLager) in SavvyCAN compatible format
void SDCardCANFrameSavvyCANOutput(uint8_t whichCANBus) {
  if(sdCardFirstRun){ 
    SD_Port.printf("Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n"); 
    sdCardFirstRun = false; 
  }

  String helperString =
    String(micros()) + "," + String(frame.can_id, HEX) + ",false," + String(whichCANBus) + "," + String(frame.can_dlc) + ","
    + String(frame.data[0], HEX) + "," + String(frame.data[1], HEX) + "," + String(frame.data[2], HEX) + "," + String(frame.data[3], HEX) + ","
    + String(frame.data[4], HEX) + "," + String(frame.data[5], HEX) + "," + String(frame.data[6], HEX) + "," + String(frame.data[7], HEX) + "\n";
  SD_Port.printf(helperString.c_str());
}



// MCP2515 CAN Controller Functions
  /****************************************************************************************  
   *  This coding has been designed this way so I can change the MCP2515 CAN BUS Library  *
   *  to a different one without changing the main code.                                  *
   ****************************************************************************************/

// Resets all CAN related Program Contol Variables
// for example for calculating CAN Frames per Second (cfps)
void CANBusResetControlVariables() {
  CANBusFirstRun = true;
  totalCANReceiveTimeTimer = 0;
  numberOfCANFramesReceived[0] = 0;
  numberOfCANFramesReceived[1] = 0;
}

// Starts one of the two MCP2515 CAN controllers
// For Land Rover Freelander 2 set:
// mcp2515_1 to 500kpbs (High Speed) in Mode 1 (ListenOnly)
// mcp2515_2 to 125kbps (Medium Speed) in Mode 1 (ListenOnly)
// CANMode = 0 for Normal and 1 for ListenOnly
void CANBusStart(MCP2515 CANBusModule, CAN_SPEED CANSpeed, uint8_t CANMode) {
  while (CANBusModule.reset() != MCP2515::ERROR_OK) { delay(500); }
  debugLoop("MCP2515 Reset Successful");

  while (CANBusModule.setBitrate(CANSpeed) != MCP2515::ERROR_OK) { delay(500); }
  debugLoop("MCP2515 BitRate Successful");

  if (CANMode == 0) {
    CANbusSetNormalMode(CANBusModule);
  }
  else {                                                              
    CANbusSetListenOnlyMode(CANBusModule);
  }
}

// Set an MCP2515 CAN controller to Listen Only Mode
void CANbusSetListenOnlyMode(MCP2515 CANBusModule) {
  while (CANBusModule.setListenOnlyMode() != MCP2515::ERROR_OK) { delay(500); }
  debugLoop("MCP2515 Listen Only Mode Successful");
  CANBusResetControlVariables();
}

// Set an MCP2515 CAN controller to Normal Mode
void CANbusSetNormalMode(MCP2515 CANBusModule) {
  while (CANBusModule.setNormalMode() != MCP2515::ERROR_OK) { delay(500); }
  debugLoop("MCP2515 Normal Mode Successful");
  CANBusResetControlVariables();
}

// Checks the MCP2515 CAN controller RX Buffer for available data
bool CANBusCheckRecieved(MCP2515 CANBusModule) {
  return CANBusModule.checkReceive();
}

// Checks the MCP2515 CAN controller RX Buffer for available data
bool CANBusReadCANData(MCP2515 CANBusModule) {

  if (CANBusModule.readMessage(&frame) == MCP2515::ERROR_OK) {
    return true;
  }
  else {
    return false;
  }
}


// Writes satistics to Serial and SD Card outputs
void SDCardOutputResults() {
  debugLoop("Called");

  // Temporary - The results
  uint32_t totalCANReceiveTime = micros() - totalCANReceiveTimeTimer;
  debugLoop("Total Time = %d us", totalCANReceiveTime);
  debugLoop("500kbps Bus");
  debugLoop("numberOfFramesReceived = %d", numberOfCANFramesReceived[0]);
  debugLoop("Frames per Second = %.2f fps", (float)numberOfCANFramesReceived[0] / totalCANReceiveTime * 1000000);

  SD_Port.printf("Total Time = %d us\n", totalCANReceiveTime);
  SD_Port.printf("500kbps Bus\n");
  SD_Port.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[0]);
  SD_Port.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[0] / totalCANReceiveTime * 1000000);

  // Calculate CAN Bus Capacity Used
  debugLoop("Capacity %% calculation data");
  debugLoop("numberOfFramesReceived = %d", numberOfCANFramesReceived[0]);
  debugLoop("totalReceiveTime = %d", totalCANReceiveTime);

  uint8_t percentageOfBusCapacity = 0.00;
  float timeTaken = 0.000000;                                         // define high precision floating point math
  timeTaken = (((float)totalCANReceiveTime / (float)1000000));        // time taken to read numberOfFramesReceived
  uint32_t bitsTx = 119 * numberOfCANFramesReceived[0];               // 119 bits (average based on small sample of FL2 live CAN Data).
  uint32_t bitRate = bitsTx / timeTaken;                              // bit rate used on the CAN bus
  percentageOfBusCapacity = ((float)bitRate / 500000) * 100;          // the percentage used of the CAN bus

  // For DEBUGGING
  debugLoop("timeTaken = %d", (uint32_t)timeTaken);
  debugLoop("bitsTx = %d", bitsTx); 
  debugLoop("bitRate = %d", bitRate);

  debugLoop("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  debugLoop("125kbps Bus");
  debugLoop("numberOfFramesReceived = %d", numberOfCANFramesReceived[1]);
  debugLoop("Frames per Second = %.2f fps", (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000);

  SD_Port.printf("125kbps Bus\n");
  SD_Port.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[1]);
  SD_Port.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000);

  // Calculate CAN Bus Capacity Used
  debugLoop("Capacity %% calculation data");
  debugLoop("numberOfFramesReceived = %d", numberOfCANFramesReceived[1]);
  debugLoop("totalReceiveTime = %d", totalCANReceiveTime);

  percentageOfBusCapacity = 0.00;
  timeTaken = 0.000000;                                               // define high precision floating point math
  timeTaken = (((float)totalCANReceiveTime / (float)1000000));        // time taken to read numberOfFramesReceived
  bitsTx = 119 * numberOfCANFramesReceived[1];                        // 119 bits (average based on small sample of FL2 live CAN Data).
  bitRate = bitsTx / timeTaken;                                       // bit rate used on the CAN bus
  percentageOfBusCapacity = ((float)bitRate / 125000) * 100;          // the percentage used of the CAN bus

  debugLoop("timeTaken = %d", (uint32_t)timeTaken);
  debugLoop("bitsTx = %d", bitsTx);
  debugLoop("bitRate = %d", bitRate);

  debugLoop("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.flush();
  SD_Port.end();
}


// Calibrate the touch screen and retrieve the scaling factors, see Setup42_ILI9341_ESP32.h for more touch CS Pin definition
// To recalibrate set REPEAT_CALIBRATION = true 
void TFTRectangleILI9341TouchCalibrate() {
  debugLoop("Called\n");
  
  // This calibration code came for the TFT_eSPI library example Keypad_240x320
  // My thanks to Bodmer for this code, copywrite acknowledged in the header and library folder

  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CALIBRATION)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char*)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CALIBRATION) {
    // calibration data valid
    TFT_Rectangle_ILI9341.setTouch(calData);
  }
  else {
    // data not valid so recalibrate
    TFT_Rectangle_ILI9341.fillScreen(TFT_BLACK);
    TFT_Rectangle_ILI9341.setCursor(20, 0);
    //TFT_Rectangle_ILI9341.setTextFont(2);
    //TFT_Rectangle_ILI9341.setTextSize(1);
    TFT_Rectangle_ILI9341.setTextColor(TFT_WHITE, TFT_BLACK);

    TFT_Rectangle_ILI9341.println("Touch corners as indicated");

    //TFT_Rectangle_ILI9341.setTextFont(1);
    TFT_Rectangle_ILI9341.println();

    if (REPEAT_CALIBRATION) {
      TFT_Rectangle_ILI9341.setTextColor(TFT_RED, TFT_BLACK);
      TFT_Rectangle_ILI9341.println("Set REPEAT_CAL to false to stop this running again!");
    }

    TFT_Rectangle_ILI9341.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    TFT_Rectangle_ILI9341.setTextColor(TFT_GREEN, TFT_BLACK);
    TFT_Rectangle_ILI9341.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char*)calData, 14);
      f.close();
    }
  }
}


// Clears the display and places the Program Name and Version at the top of the display
// Sets the rotation to 3, textColor to TFT_WHITE, TFT_LANDROVERGREEN, true
// Sets font to MENU_FONT & TextDatum(MC_DATUM)
// Requires FONT_2 to be loaded (header font)
void ClearDisplay() {
  debugLoop("Called\n");
  TFT_Rectangle_ILI9341.setRotation(3);
  TFT_Rectangle_ILI9341.fillScreen(TFT_LANDROVERGREEN);
  // Ensure the user is reminded if in DEBUG mode
#if DEBUG > 0
  TFT_Rectangle_ILI9341.setTextColor(TFT_RED, TFT_YELLOW, true);
  TFT_Rectangle_ILI9341.setTextDatum(TL_DATUM);
  TFT_Rectangle_ILI9341.drawString(String("DEBUG"), TFT_Rectangle_ILI9341.width() - 120, 0, TOP_MENU_FONT);
#endif
  TFT_Rectangle_ILI9341.setTextColor(TFT_WHITE, TFT_LANDROVERGREEN, true);
  TFT_Rectangle_ILI9341.setTextDatum(TL_DATUM);
  TFT_Rectangle_ILI9341.drawString(TOP_MENU_PROGRAM_NAME, 0, 0, TOP_MENU_FONT);
  TFT_Rectangle_ILI9341.setTextDatum(TR_DATUM);
  TFT_Rectangle_ILI9341.drawString(TOP_MENU_PROGRAM_VERSION, TFT_Rectangle_ILI9341.width(), 0, TOP_MENU_FONT);
  TFT_Rectangle_ILI9341.setTextDatum(MC_DATUM);
  TFT_Rectangle_ILI9341.setFreeFont(&MENU_FONT);
}


// Draw a menu along the top of the TFT Display (320 x 240) Rotation 3
// Menu options are drawn in a line across the top of the screen
// Contents based on the current menu defined by menu structures
void DrawHorizontalMenu(int16_t yOffset, GFXfont menuFont) {
  debugLoop("Called\n");

  // Clear the display and reset the program header
  ClearDisplay();

  // Determine the maximum number of characters in a button so we can calculate the button width correctly
  TFT_Rectangle_ILI9341.setFreeFont(&menuFont);                       // Must set menu font for following calculations
  uint8_t maxChars = 0;
  uint8_t menuButtons = 0;
  while (menu[menuButtons].text) {
    if (maxChars < String(menu[menuButtons].text).length()) maxChars = String(menu[menuButtons].text).length();
    menuButtons++;
  }
  if (maxChars < 10) maxChars = 10;                                   // Draw a minimum button size so user can easily select it

  // Calculate the basic button positions
  uint16_t fontWidth = 10; // take a default value for now
  uint16_t yButtonMiddle = TFT_Rectangle_ILI9341.fontHeight() + yOffset;
  uint16_t xButtonWidth = fontWidth * maxChars;
  uint16_t yButtonHeight = TFT_Rectangle_ILI9341.fontHeight() * 1;

  // Draw the menu buttons
  TFT_Rectangle_ILI9341.setFreeFont(&menuFont);
  char handler[1] = "";
  uint8_t menuBtnStartPos = 0;
  menuButtons = 0;                                                    // Reset the number of buttons that will be active on the new menu
  while (menu[menuButtons].text) {
    btnText[menuButtons] = menu[menuButtons].text;                    // Must capture btnText for the ProcessButtons() function
    btnMenu[menuButtons].initButton(&TFT_Rectangle_ILI9341,
      (menuButtons * xButtonWidth) + (menuButtons * 5) + (xButtonWidth / 2),
      yButtonMiddle,
      xButtonWidth,
      yButtonHeight,
      TFT_YELLOW, TFT_BLUE, TFT_YELLOW, handler, 1);                  // initButton limits the amount of text drawn, draw text in the drawButton() function
    btnMenu[menuButtons].drawButton(false, menu[menuButtons].text);   // Specifiy the text for the button because initButton will not display the full text length
    btnMenu[menuButtons].press(false);                                // Because I am reusing buttons it is important to tell the button it is NOT pressed
    menuButtons++;                                                    // Add the button to the Global Variable that tracks the number of active buttons 
  }
  ProcessButtons(MENU, menuButtons, menuBtnStartPos, true);
}


// Draw a menu in middle of the TFT Display (320 x 240) Rotation 3
// Menu options are drawn in a line down the screen and presented as buttons
// Contents based on the current menu defined by menu structures
void DrawVerticalMenu(int16_t yOffset, GFXfont headerFont, GFXfont menuFont) {
  debugLoop("Called\n");
  
  // Clear the display and reset the program header
  ClearDisplay();

  // Determine the maximum number of characters in a button so we can calculate the 
  // button width correctly and starting position of the buttons in the menu structure
  TFT_Rectangle_ILI9341.setFreeFont(&menuFont);                       // Must set menu font for following calculations
  uint8_t maxChars = 0;
  uint8_t menuPosition = 0;
  uint8_t menuButtons = 0;
  uint8_t menuBtnStartPos = 99;
  while (menu[menuPosition].text) {
    if (A == menu[menuPosition].action) {
      if (maxChars < String(menu[menuPosition].text).length()) maxChars = String(menu[menuPosition].text).length();
      if (menuBtnStartPos == 99) menuBtnStartPos = menuPosition;      // Capture the position of the first button in the menu structure
      menuButtons++;
    }
    menuPosition++;
  }
  if (maxChars < 10) maxChars = 10;                                   // Minimum button size so user can easily select it

  // Calculate the basic button positions
  uint16_t fontWidth = 10; // take a default value for now
  uint16_t xButtonMiddle = TFT_Rectangle_ILI9341.width() / 2;
  uint16_t yButtonMiddle = TFT_Rectangle_ILI9341.fontHeight() * 1.8;
  uint16_t xButtonWidth = fontWidth * maxChars;
  uint16_t yButtonHeight = TFT_Rectangle_ILI9341.fontHeight() * 1.30;

  // Draw the menu buttons
  TFT_Rectangle_ILI9341.setFreeFont(&menuFont);
  char handler[1] = "";
  menuPosition = 0;
  menuButtons = 0;

  // Reset the Global Variable that tracks the number of buttons that will be active on the new menu
  while (menu[menuPosition].text) {
    if (H == menu[menuPosition].action) {
      // Draw the menu header
      TFT_Rectangle_ILI9341.setFreeFont(&headerFont);
      TFT_Rectangle_ILI9341.drawString(menu[menuPosition].text, xButtonMiddle, (yOffset - TFT_Rectangle_ILI9341.fontHeight()) / 2);
      TFT_Rectangle_ILI9341.setFreeFont(&menuFont);
    }
    else if (A == menu[menuPosition].action) {
      // Draw the action button
      btnText[menuButtons] = menu[menuPosition].text;                 // Must capture btnText for the ProcessButtons() function
      btnMenu[menuButtons].initButton(&TFT_Rectangle_ILI9341,
        xButtonMiddle,
        menuButtons * yButtonMiddle + yOffset,
        xButtonWidth,
        yButtonHeight,
        TFT_YELLOW, TFT_BLUE, TFT_YELLOW, handler, 1);                // initButton limits the amount of text drawn, draw text in the drawButton() function
      btnMenu[menuButtons].drawButton(false, menu[menuPosition].text);// Specifiy the text for the button because initButton will not display the full text length
      btnMenu[menuButtons].press(false);                              // Because I am reusing buttons it is important to tell the button it is NOT pressed
      menuButtons++;                                                  // Add the button to the Global Variable that tracks the number of active buttons
    }
    menuPosition++;
  }
  ProcessButtons(MENU, menuButtons, menuBtnStartPos, true);
}


// Checks if any active buttons on the display have been pressed
// Once called it will wait for a button to be pressed
// type: 0 = Menu, 1 = MessageBox
// NOTE: Overload automatically sets wait (for button press) = true
uint8_t ProcessButtons(uint8_t type, uint8_t numberOfButtons) {          // overload
  debugLoop("Called (without menuBtnStartPos overload)\n");
  uint8_t result = ProcessButtons(type, numberOfButtons, 0, true);
  return result;
}

// Checks if any active buttons on the display have been pressed
// Once called it will wait for a button to be pressed
// type: 0 = Menu, 1 = MessageBox
// wait (for button press): true or false
// Returns: number of button pressed or 99 if no button pressed
uint8_t ProcessButtons(uint8_t type, uint8_t numberOfButtons, uint8_t menuBtnStartPos, bool wait) {
  debugLoop("Called (with menuBtnStartPos overload)");
  debugLoop("wait = %d\n", wait);
  
  do {   // for now create an endless loop until a button is pressed

    uint16_t t_x = 0, t_y = 0;                                        // To store the touch coordinates

    // Pressed will be set true is there is a valid touch on the screen
    bool pressed = TFT_Rectangle_ILI9341.getTouch(&t_x, &t_y);

    // Check if any key coordinate boxes contain the touch coordinates
    for (uint8_t btnCounter = 0; btnCounter < numberOfButtons; btnCounter++) {
      if (pressed && btnMenu[btnCounter].contains(t_x, t_y)) {
        btnMenu[btnCounter].press(true);                              // tell the button it is pressed
      }
      else {
        btnMenu[btnCounter].press(false);                             // tell the button it is NOT pressed
      }
    }

    // Check if any key has changed state
    for (uint8_t btnCounter = 0; btnCounter < numberOfButtons; btnCounter++) {

      TFT_Rectangle_ILI9341.setFreeFont(&MENU_FONT);

      if (btnMenu[btnCounter].justPressed()) {
        btnMenu[btnCounter].drawButton(true, btnText[btnCounter]);    // draw inverted
        delay(100);                                                   // UI debouncing
      }

      if (btnMenu[btnCounter].justReleased()) {
        btnMenu[btnCounter].drawButton(false, btnText[btnCounter]);   // draw normal

        if (type == MENU) {
          ProcessMenu(btnCounter, menuBtnStartPos);                   // Jump to new menu
          return 99;
        }
        if (type == MESSAGE_BOX) return btnCounter;                   // Return message button pressed
        if (type == DISPLAY_BUTTON) return btnCounter + menuBtnStartPos; // Return display button pressed

        Serial.printf("ERROR: Button type not handled correctly, type = %d\n", type);
        return btnCounter;
      }
    }
  } while (wait);
  return 99;
}


// Controls the move to a new menu that will be displayed
// btnNumber = the button the user pressed
// menuBtnStartPos = the postion of the first menu option in the menu structure
void ProcessMenu(uint8_t btnNumber, uint8_t menuBtnStartPos) {
  debugLoop("Called\n");
  if (M == menu[btnNumber + menuBtnStartPos].action) {                // User selection required another menu
    menu = menu[btnNumber + menuBtnStartPos].menu;
    DrawVerticalMenu(MENU_Y_OFFSET, MENU_BOLD_FONT, MENU_FONT);
  }
  else if (A == menu[btnNumber + menuBtnStartPos].action) {           // User selection calls a code function
    menu[btnNumber + menuBtnStartPos].func(menu[btnNumber + menuBtnStartPos].arg);
  }
}


// Displays a message box on the display and allows the user to respond
// Use options to specify buttons, i.e. BTN_OK + BTN_IGNORE + BTN_CANCEL
uint8_t MessageBox(const char* title, const char* message, uint8_t options) {
  debugLoop("Called\n");
  uint16_t result = 0;
  uint16_t boxX = TFT_Rectangle_ILI9341.width() * 0.05;
  uint16_t boxY = TFT_Rectangle_ILI9341.height() * 0.2 + TOP_MENU_Y_OFFSET;
  uint16_t boxWidth = TFT_Rectangle_ILI9341.width() * 0.9;
  uint16_t boxHeight = TFT_Rectangle_ILI9341.height() * 0.6;
  uint16_t lineY = boxY;

  // Set up the display
  ClearDisplay();
  TFT_Rectangle_ILI9341.setTextColor(TFT_YELLOW, TFT_BLUE, true);

  // Draw the message box
  TFT_Rectangle_ILI9341.drawRect(boxX - 1, boxY - 1, boxWidth + 2, boxHeight + 2, TFT_YELLOW);
  TFT_Rectangle_ILI9341.fillRect(boxX, boxY, boxWidth, boxHeight, TFT_BLUE);

  // Draw the message title
  lineY += TFT_Rectangle_ILI9341.fontHeight() * 0.5;
  TFT_Rectangle_ILI9341.setFreeFont(&MENU_BOLD_FONT);
  TFT_Rectangle_ILI9341.drawString(String(title), TFT_Rectangle_ILI9341.width() / 2, lineY);
  TFT_Rectangle_ILI9341.setFreeFont(&MENU_FONT);
  lineY += TFT_Rectangle_ILI9341.fontHeight() * 1.5;

  // Split the message into lines and draw
  const uint16_t maxLineLength = 30;
  char lineBuffer[maxLineLength + 1];                                 // +1 for the null-terminator
  uint16_t lineLength = 0;
  const char* current = message;

  while (*current != '\0') {
    lineLength = 0;
    // Fill the buffer with characters until reaching the maximum length or newline
    while (lineLength < maxLineLength && *current != '\n' && *current != '\0') {
      lineBuffer[lineLength++] = *current++;
    }
    // Handle splitting lines at spaces if necessary
    if (lineLength == maxLineLength && *current != '\0' && *current != ' ' && *current != '\n') {
      // Walk backwards to find a space
      while (lineLength > 0 && lineBuffer[lineLength - 1] != ' ') {
        lineLength--;
        current--;
      }
    }
    // Null-terminate the string and print it
    lineBuffer[lineLength] = '\0';
    TFT_Rectangle_ILI9341.drawString(String(lineBuffer), TFT_Rectangle_ILI9341.width() / 2, lineY);
    lineY += TFT_Rectangle_ILI9341.fontHeight() * 0.8;
    // Skip the newline characters if present
    if (*current == '\n') {
      current++;
    }
    // Skip any leading spaces on the next line
    while (*current == ' ') {
      current++;
    }
  }

  // Work out how many buttons we will need and their text
  uint8_t numberOfOptionButtons = 0;
  const char* messageBtnText[3] = { "","","" };
  uint16_t messageButtonPos[3] = { 0,0,0 };

  if (options & BTN_OK) { messageBtnText[numberOfOptionButtons] = "OK"; numberOfOptionButtons++; }
  if (options & BTN_IGNORE) { messageBtnText[numberOfOptionButtons] = "IGNORE"; numberOfOptionButtons++; }
  if (options & BTN_CANCEL) { messageBtnText[numberOfOptionButtons] = "CANCEL"; numberOfOptionButtons++; }
  if (options & BTN_CAN1) { messageBtnText[numberOfOptionButtons] = "CAN 1"; numberOfOptionButtons++; }
  if (options & BTN_CAN2) { messageBtnText[numberOfOptionButtons] = "CAN 2"; numberOfOptionButtons++; }

  debugLoop("MessageBox numberOfOptionButtons = %d", numberOfOptionButtons);

  // Draw the buttons
  uint8_t menuButtons = 0;
  char handler[1] = "";
  uint16_t xButtonWidth = (boxWidth / 3) * 0.95;
  uint16_t yButtonHeight = TFT_Rectangle_ILI9341.fontHeight() * 1.2;
  uint16_t yButtonMiddle = boxY + boxHeight - yButtonHeight * 0.75;
  if (numberOfOptionButtons == 1) {
    messageButtonPos[0] = TFT_Rectangle_ILI9341.width() / 2;
  }
  else if (numberOfOptionButtons == 2) {
    messageButtonPos[0] = (TFT_Rectangle_ILI9341.width() / 2) - xButtonWidth - 5;
    messageButtonPos[1] = (TFT_Rectangle_ILI9341.width() / 2) + xButtonWidth + 5;
  }
  else if (numberOfOptionButtons == 3) {
    messageButtonPos[0] = (TFT_Rectangle_ILI9341.width() / 2) - xButtonWidth - 5;
    messageButtonPos[1] = TFT_Rectangle_ILI9341.width() / 2;
    messageButtonPos[2] = (TFT_Rectangle_ILI9341.width() / 2) + xButtonWidth + 5;
  }

  for (uint16_t i = 0; i < numberOfOptionButtons; i++) {
    btnText[menuButtons] = messageBtnText[menuButtons];               // Must capture btnText for the ProcessButtons() function
    btnMenu[menuButtons].initButton(&TFT_Rectangle_ILI9341,
      messageButtonPos[i],
      yButtonMiddle,
      xButtonWidth,
      yButtonHeight,
      TFT_YELLOW, TFT_BLUE, TFT_YELLOW, handler, 1);                  // initButton limits the amount of text drawn, draw text in the drawButton() function
    btnMenu[menuButtons].drawButton(false, messageBtnText[menuButtons]);// Specifiy the text for the button because initButton will not display the full text length
    btnMenu[menuButtons].press(false);                                // Because I am reusing buttons it is important to tell the button it is NOT pressed
    menuButtons++;
  }

  result = ProcessButtons(MESSAGE_BOX, menuButtons);

  debugLoop("MessageBox result = %d (zero-based indexing)", result);


  // Clear the display and reset the program header
  ClearDisplay();

  // Evaluate the user choice to the MESSAGE_BOX enum values
  // TODO link enum to result by value rather than button text, but not sure how :)
  const char* helper0 = "OK";
  const char* helper1 = "IGNORE";
  const char* helper2 = "CANCEL";
  const char* helper3 = "CAN 1";
  const char* helper4 = "CAN 2";
  if (messageBtnText[result] == helper0) result = BTN_OK;
  else if (messageBtnText[result] == helper1) result = BTN_IGNORE;
  else if (messageBtnText[result] == helper2) result = BTN_CANCEL;
  else if (messageBtnText[result] == helper3) result = BTN_CAN1;
  else if (messageBtnText[result] == helper4) result = BTN_CAN2;

  return result;
}


// Performs analysis on both CAN Interfaces to determine if the OBD2_Interface can sucessfully read both at the same time
void OutputAnalyseCANBusResults() {

  /*
    For information
    As part of the MCP2515 CAN Bus Controllers are 3 Rx buffers in each, thus 3 for the 500kbps, and 3 for the 125kbps CAN Bus.
    500kbps CAN Bus will fill in a minimum period of 666us
    125kbps CAN bus will fill in a minimum period of 2664us
  */

#define MEASURE_TIME 10000000                                         // 10000000 for live, 1000000 to TESTING
#if MEASURE_TIME != 10000000
  #warning "OutputAnalyseCANBusResults() IS NOT EQUAL to 10000000"
#endif

  debugLoop("Called");
  uint8_t result = MessageBox("Analyse CAN Bus Capacity", "Ensure the OBD2 device is connected to the car with the engine running.", BTN_OK + BTN_CANCEL);

  debugLoop("MessageBox Returned Button %d", result);

  if (result == BTN_OK) {
    numberOfCANFramesReceived[0] = 0;
    numberOfCANFramesReceived[1] = 0;
    uint8_t   percentageOfBusCapacity[2]{ 0,0 };
    uint16_t  cfps[2] = { 0,0 };
    uint32_t  totalCANReceiveTime = 0;
    CANBusFirstRun = false;

    // Clear the display and reset the program header
    ClearDisplay();

    TFT_Rectangle_ILI9341.setTextFont(2);
    TFT_Rectangle_ILI9341.setTextColor(TFT_BLACK);
    TFT_Rectangle_ILI9341.setTextDatum(TL_DATUM);

    // Draw Results table
    uint16_t tableX = 20;
    uint16_t tableY = 100;
    uint16_t tableW = 280;
    uint16_t tableH = 80;
    uint16_t tableFontH = TFT_Rectangle_ILI9341.fontHeight();
    uint16_t tableFontW = TFT_Rectangle_ILI9341.textWidth("A");
    // Draw table
    TFT_Rectangle_ILI9341.drawRect(tableX - 1, tableY - 1, tableW + 2, tableH + 2, TFT_LIGHTGREY);
    TFT_Rectangle_ILI9341.fillRect(tableX, tableY, tableW, tableH, TFT_GREEN);
    // Horizontal Lines
    TFT_Rectangle_ILI9341.drawLine(tableX, tableY + tableFontH + 2, tableX + tableW, tableY + tableFontH + 2, TFT_LIGHTGREY);
    TFT_Rectangle_ILI9341.drawLine(tableX, tableY + tableFontH + 22, tableX + tableW, tableY + tableFontH + 22, TFT_LIGHTGREY);
    TFT_Rectangle_ILI9341.drawLine(tableX, tableY + tableFontH + 42, tableX + tableW, tableY + tableFontH + 42, TFT_LIGHTGREY);
    // Vertical Lines
    TFT_Rectangle_ILI9341.drawLine(tableX + (tableFontW * 7), tableY, tableX + (tableFontW * 7), tableY + tableH, TFT_LIGHTGREY);
    TFT_Rectangle_ILI9341.drawLine(tableX + (tableFontW * 17), tableY, tableX + (tableFontW * 17), tableY + tableH, TFT_LIGHTGREY);
    TFT_Rectangle_ILI9341.drawLine(tableX + (tableFontW * 26), tableY, tableX + (tableFontW * 26), tableY + tableH, TFT_LIGHTGREY);

    TFT_Rectangle_ILI9341.drawString("Results", tableX + 2, tableY);
    TFT_Rectangle_ILI9341.drawCentreString("Capacity*", tableX + (tableFontW * 12), tableY, 2);
    TFT_Rectangle_ILI9341.drawCentreString("Single", tableX + (tableFontW * 22), tableY, 2);
    TFT_Rectangle_ILI9341.drawCentreString("Dual", tableX + (tableFontW * 31), tableY, 2);

    TFT_Rectangle_ILI9341.drawString("500kbps", tableX + 2, tableY + 20);
    TFT_Rectangle_ILI9341.drawString("125kbps", tableX + 2, tableY + 40);
    TFT_Rectangle_ILI9341.drawString("Passed", tableX + 2, tableY + 60);

    TFT_Rectangle_ILI9341.setTextColor(TFT_GREEN);
    TFT_Rectangle_ILI9341.drawCentreString("Performs analysis on both CAN Interfaces", 160, 25, 2);
    TFT_Rectangle_ILI9341.drawCentreString("to determine if the OBD2_Interface can", 160, 45, 2);
    TFT_Rectangle_ILI9341.drawCentreString("sucessfully read both at the same time.", 160, 65, 2);
    TFT_Rectangle_ILI9341.drawString("* Calculated as 119 bits per CAN Frame", 2, TFT_Rectangle_ILI9341.height() - 10, 1);
    TFT_Rectangle_ILI9341.setTextColor(TFT_BLACK);

    // Check CAN Interface 0 CAN frames per second (cfps)
    totalCANReceiveTimeTimer = micros();                              // Set the timer for calculating the CAN Frames per Second (cfps) 
    while (micros() - totalCANReceiveTimeTimer < MEASURE_TIME) {          // Measure for 10 seconds
      if (CANBusCheckRecieved(mcp2515_1)) {
        if (CANBusReadCANData(mcp2515_1)) {
          CANFrameProcessing(0);
        }
      }
    }
    // Calculate cfps
    totalCANReceiveTime = micros() - totalCANReceiveTimeTimer;
    cfps[0] = (float)numberOfCANFramesReceived[0] / totalCANReceiveTime * 1000000;
    // Calculate %
    float timeTaken = 0.000000;                                         // define high precision floating point math
    timeTaken = (((float)totalCANReceiveTime / (float)1000000));        // time taken to read numberOfFramesReceived
    uint64_t bitsTx = 119 * numberOfCANFramesReceived[0];               // 119 bits (average based on small sample of FL2 live CAN Data).
    uint64_t bitRate = bitsTx / timeTaken;                              // bit rate used on the CAN bus
    percentageOfBusCapacity[0] = ((float)bitRate / 500000) * 100;          // the percentage used of the CAN bus
    // Update table
    TFT_Rectangle_ILI9341.drawCentreString(String(percentageOfBusCapacity[0]) + "%", tableX + (tableFontW * 12), tableY + 20, 2);
    TFT_Rectangle_ILI9341.drawCentreString(String(cfps[0]) + "cfps", tableX + (tableFontW * 22), tableY + 20, 2);


    // Check CAN Interface 1 CAN frames per second (cfps)
    totalCANReceiveTimeTimer = micros();                              // Set the timer for calculating the CAN Frames per Second (cfps) 
    while (micros() - totalCANReceiveTimeTimer < MEASURE_TIME) {          // Measure for 10 seconds
      if (CANBusCheckRecieved(mcp2515_2)) {
        if (CANBusReadCANData(mcp2515_2)) {
          CANFrameProcessing(1);
        }
      }
    }
    // Calculate cfps
    totalCANReceiveTime = micros() - totalCANReceiveTimeTimer;
    cfps[1] = (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000;
    // Calculate %
    timeTaken = 0.000000;                                               // define high precision floating point math
    timeTaken = (((float)totalCANReceiveTime / (float)1000000));        // time taken to read numberOfFramesReceived
    bitsTx = 119 * numberOfCANFramesReceived[1];                    // 119 bits (average based on small sample of FL2 live CAN Data).
    bitRate = bitsTx / timeTaken;                                    // bit rate used on the CAN bus
    percentageOfBusCapacity[1] = ((float)bitRate / 125000) * 100;          // the percentage used of the CAN bus
    // Update table
    TFT_Rectangle_ILI9341.drawCentreString(String(percentageOfBusCapacity[1]) + "%", tableX + (tableFontW * 12), tableY + 40, 2);
    TFT_Rectangle_ILI9341.drawCentreString(String(cfps[1]) + "cfps", tableX + (tableFontW * 22), tableY + 40, 2);

    // If cfps[] is Zero then we should fail these tests (i.e. no CAN Data detected)
    if (cfps[0] != 0) {
      TFT_Rectangle_ILI9341.drawCentreString("Yes", tableX + (tableFontW * 12), tableY + 60, 2);
    }
    else {
      TFT_Rectangle_ILI9341.drawCentreString("No", tableX + (tableFontW * 12), tableY + 60, 2);
    }
    if (cfps[1] != 0) {
      TFT_Rectangle_ILI9341.drawCentreString("Yes", tableX + (tableFontW * 22), tableY + 60, 2);
    }
    else {
      TFT_Rectangle_ILI9341.drawCentreString("No", tableX + (tableFontW * 22), tableY + 60, 2);
    }

    // Check both CAN Interfaces at the same time for CAN frames per second (cfps)
    numberOfCANFramesReceived[0] = 0;
    numberOfCANFramesReceived[1] = 0;
    totalCANReceiveTimeTimer = micros();                              // Set the timer for calculating the CAN Frames per Second (cfps) 
    while (micros() - totalCANReceiveTimeTimer < MEASURE_TIME) {          // Measure for 10 seconds
      // Check the 500kbps bus as priority over 125kbps because 500kbps is faster and the buffers fill significantly more quickly
      if (CANBusCheckRecieved(mcp2515_1)) {
        if (CANBusReadCANData(mcp2515_1)) {
          CANFrameProcessing(0);
        }
      }
      // only check the 125kbps if there any no 500kbps messages in the MCP2515 buffers
      else if (CANBusCheckRecieved(mcp2515_2)) {
        if (CANBusReadCANData(mcp2515_2)) {
          CANFrameProcessing(1);
        }
      }
    }
    totalCANReceiveTime = micros() - totalCANReceiveTimeTimer;

    // Compare the results for CAN Interface 0
    bool passed = true;

    debugLoop("passed = %d", passed);

    uint16_t cfpsCompare = (float)numberOfCANFramesReceived[0] / totalCANReceiveTime * 1000000;

    debugLoop("numberOfCANFramesReceived[0] = %d", numberOfCANFramesReceived[0]);
    debugLoop("totalCANReceiveTime = %dus", totalCANReceiveTime);
    debugLoop("cfpsCompare = %d", cfpsCompare);
    debugLoop("cfps[0] = %d (%d%%)", cfps[0], percentageOfBusCapacity[0]);

    // If no CAN Bus data then fail the test
    if (cfps[0] == 0 || cfps[1] == 0) {
      passed = false;
      TFT_Rectangle_ILI9341.setTextColor(TFT_RED, TFT_YELLOW, true);
      TFT_Rectangle_ILI9341.drawString("ERROR: *!* NO CAN BUS DATA RECEIVED", 2, TFT_Rectangle_ILI9341.height() - 30, 2);
      TFT_Rectangle_ILI9341.setTextColor(TFT_BLACK, TFT_GREEN, true);
    }

    // If cfps for Dual CAN Bus reading is ont within 1% of the cfps for Single CAN Bus then fail the test 
    if ((cfpsCompare < cfps[0] * 0.99) || (cfpsCompare > cfps[0] * 1.01)) { passed = false; }
    debugLoop("passed = %d", passed);

    TFT_Rectangle_ILI9341.drawCentreString(String(cfpsCompare) + "cfps", tableX + (tableFontW * 31), tableY + 20, 2);

    // Compare the results for CAN Interface 1
    cfpsCompare = (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000;

    debugLoop("numberOfCANFramesReceived[1] = %d", numberOfCANFramesReceived[1]);
    debugLoop("totalCANReceiveTime = %dus", totalCANReceiveTime);
    debugLoop("cfpsCompare = %d", cfpsCompare);
    debugLoop("cfps[1] = %d (%d%%)", cfps[1], percentageOfBusCapacity[1]);

    // If cfps for Dual CAN Bus reading is ont within 1% of the cfps for Single CAN Bus then fail the test 
    if ((cfpsCompare < cfps[1] * 0.99) || (cfpsCompare > cfps[1] * 1.01)) { passed = false; }
    debugLoop("passed = %d", passed);

    TFT_Rectangle_ILI9341.drawCentreString(String(cfpsCompare) + "cfps", tableX + (tableFontW * 31), tableY + 40, 2);
    if (passed) {
      TFT_Rectangle_ILI9341.drawCentreString("Yes", tableX + (tableFontW * 31), tableY + 60, 2);
    }
    else {
      TFT_Rectangle_ILI9341.drawCentreString("No", tableX + (tableFontW * 31), tableY + 60, 2);
    }

    // Create an OK button to exit
    TFT_Rectangle_ILI9341.setFreeFont(&MENU_FONT);                    // Set the normal button font
    char handler[1] = "";
    uint16_t xButtonWidth = TFT_Rectangle_ILI9341.textWidth("A") * 3;
    uint16_t yButtonHeight = TFT_Rectangle_ILI9341.fontHeight() * 1.2;
    uint16_t xButtonMiddle = TFT_Rectangle_ILI9341.width() - (xButtonWidth / 2);
    uint16_t yButtonMiddle = TFT_Rectangle_ILI9341.height() - (yButtonHeight / 2);
    const char* messageBtnText[1] = { "OK" };
    btnText[0] = messageBtnText[0];                                   // Must capture btnText for the ProcessButtons() function
    btnMenu[0].initButton(&TFT_Rectangle_ILI9341,
      xButtonMiddle,
      yButtonMiddle,
      xButtonWidth,
      yButtonHeight,
      TFT_YELLOW, TFT_BLUE, TFT_YELLOW, handler, 1);                  // initButton limits the amount of text drawn, draw text in the drawButton() function
    btnMenu[0].drawButton(false, messageBtnText[0]);                  // Specifiy the text for the button because initButton will not display the full text length
    btnMenu[0].press(false);                                          // Because I am reusing buttons it is important to tell the button it is NOT pressed

    result = ProcessButtons(MESSAGE_BOX, 1);
    return;                                                           // Back to root menu
  }
  else if (result == BTN_CANCEL) {
    return;                                                           // Back to root menu
  }

  while (true) { Serial.printf("OutputAnalyseCANBusResults MessageBox result"); delay(1000); }
}


void OutputFormatCanDrive(ulong rxId, uint8_t len, uint8_t rxBuf[], uint8_t MCP2515number) {
  // canDrive Format - Validated as working Sep 2024
  // 151,0,0,025B8151025B8151
  // 152,0,0,025B8152025B8152
  // 153,0,0,025B8153025B8153
  // 154,0,0,025B8154025B8154
  // 155,0,0,025B8155025B8155
  debugLoop("called, MCP2515number = % d", MCP2515number);
  char msgString[128];                                                // Array to store serial string
  if ((rxId & 0x80000000) == 0x80000000)                              // Determine if ID is standard (11 bits) or extended (29 bits)
    sprintf(msgString, "%.8lX,0,0,", (rxId & 0x1FFFFFFF));
  else
    sprintf(msgString, "%.3lX,0,0,", rxId);


  Serial.print(msgString);

  if ((rxId & 0x40000000) == 0x40000000) {                            // Determine if message is a remote request frame.
    sprintf(msgString, " REMOTE REQUEST FRAME");
    Serial.print(msgString);
  }
  else {
    for (byte i = 0; i < len; i++) {
      sprintf(msgString, "%.2X", rxBuf[i]);
      Serial.print(msgString);
    }
  }
  Serial.print("\n");
}







