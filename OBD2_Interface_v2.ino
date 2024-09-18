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
// TODO  For the OBD2_Interfacce_v2 do I really need to load all the fonts, see Setup42_ILI9341_ESP32.h ?
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

// Constants & ESP32-S3 pin declarations
const auto          STANDARD_SERIAL_OUTPUT_BAUD = 2000000;            // Must be at least 2,000,000 to keep up with Land Rover Freelander 2 
const auto          SD_PORT_HARDWARE_SERIAL_NUMBER = 1;               // The OpenLager will be connected to Hardware Serial Port 1
const auto          SD_CARD_ESP32_S3_TX_PIN = 9;                      // The OpenLager Rx pin will be connected to this ESP32-S3 Tx pin
const auto          CAN_BUS_0_CS_PIN = 14;                            // The CS pin of the MCP2515 (High Speed CAN Bus 500kbps) will be connected to this ESP32-S3 pin
const auto          CAN_BUS_1_CS_PIN = 10;                            // The CS pin of the MCP2515 (Medium Speed CAN Bus 125kbps) will be connected to this ESP32-S3 pin
const auto          TFT_LANDROVERGREEN = 12832;                       // A Land Rover Green RBG colour
const auto          TFT_Rectangle_ILI9341_LEDPIN = 39;                // The ILI9341 display LED PIN will be connected to this ESP32-S3 pin
const auto          CALIBRATION_FILE = "/TouchCalData1";              // This is the file name used to store the touch display calibration data, must start with /
const auto          REPEAT_CALIBRATION = false;                       // Set to true to always calibrate / recalibrate the touch display
const auto          TOP_MENU_FONT = 2;                                // Font used for selection menu that is at the top of the display
const auto          TOP_MENU_Y_OFFSET = 10;                           // Used to position the top menu just under the Program Title
const auto          TOP_MENU_PROGRAM_NAME = "OBD2 Interface by Bionicbone"; // Will be displayed at the top of the display
const auto          TOP_MENU_PROGRAM_VERSION = "v2.0.0";              // Will be displayed at the top of the display
const auto          MENU_BOLD_FONT = FreeSansBoldOblique9pt7b;        // Font used for selection menus headers
const auto          MENU_FONT = FreeSansOblique9pt7b;                 // Font used for selection menus buttons
const auto          MENU_Y_OFFSET = 68;                               // Used to position the menu just under the Menu Header

// Control variables
bool                sdCardFirstRun = false;                           // Will set to true when SD Card initialises, trigger for SavvyCAN header
bool                CANBusFirstRun = false;                           // Will set to true when CAN Bus mode changes, trigger for cfps timer to start
unsigned long       totalCANReceiveTimeTimer = 0;                     // Times how long we have been receiving CAN Frames
unsigned int        numberOfCANFramesReceived[2] = { 0,0 };           // Counts the number of CAN Frames received
byte                menuCurrentlyDisplayed = 0;                       // Tracks the menu displayed
byte                menuButtons = 0;                                  // menuButtons controls how many buttons are currently in use, updated when menu is drawn
byte                menuButtonsPos = 0;                               // menuButtonsPos controls the line in the menu struct of the first button, updated when menu is drawn


// Sets the new output requirements
void actionOutputChange(int arg) {
  // TODO write the script
  Serial.print("action: ");
  Serial.println(arg);
}

// Allows user to configure the CAN BUS Settings
void changeCANSettings(int arg) {
  // TODO write the script
  // TODO I would like an "Auto Detect CAN Bus Speed" function, but first be 100% which CAN library I will be using
  Serial.print("changeCANSettings: ");
  Serial.println(arg);
}
                                                                      
     
// Create menus (each menu must have no more than 5 options to choose)
// The order of the menus in the code must consider the order of definitions
// Thus the structure is at the top and the highest menu hierarchy is at the bottom

// Menu Structure
enum { H, M, A };                                                     // H = Header, M = New Menu, A = Action Menu Option 
struct Menu {
  const char* text;
  int          action;
  Menu* menu;
  void (*func) (int);
  int          arg;
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
  { "Live CanDrive", A, 0, actionOutputChange, 1 },
  { "Live Display", A, 0, actionOutputChange, 2 },
  { "Serial SavvyCAN", A, 0, actionOutputChange, 3 },
  { "Save SavvyCAN", A, 0, actionOutputChange, 4 },
  { "WiFi Data", A, 0, actionOutputChange, 5 },
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
MCP2515 mcp2515_0(CAN_BUS_0_CS_PIN);                                  // Create MCP2515 controller for High Speed CAN Bus 500kbps
MCP2515 mcp2515_1(CAN_BUS_1_CS_PIN);                                  // Create MCP2515 controller for Medium Speed CAN Bus 125kbps
struct can_frame frame;

// Add SD Card Serial Port
HardwareSerial SD_Port(SD_PORT_HARDWARE_SERIAL_NUMBER);               // Connect OpenLager to Hardware Serial Port specified

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button btnMenu[5];


void setup() {
  // Serial needs to be at least 2,000,000 baud otherwise lines will be dropped when outputting CAN lines to the standard serial output display.
  while (!Serial) { Serial.begin(STANDARD_SERIAL_OUTPUT_BAUD); delay(100); }

  // Display the Serial header
  Serial.printf("\nName        : OBD2_Interface_v2\nCreated     : Sep 2024\nAuthor      : Kevin Guest AKA TheBionicBone\n");
  Serial.printf("Program     : %s\n", TOP_MENU_PROGRAM_VERSION);
  Serial.printf("ESP-IDF     : %s\n",esp_get_idf_version());
  Serial.printf("Arduino Core: v%d.%d.%d\n", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  // Check the ESP32 Arduino Core used to compile the code has been validated 
  if (ESP_ARDUINO_VERSION > ESP_ARDUINO_VERSION_VAL(2, 0, 17)) {
    Serial.printf("\n\n");
    Serial.printf("This ESP Arduino Core Version has not been tested\n");
    Serial.printf("To ensure full compatibility use ESP32 Arduino Core v2.0.17\n");
    Serial.printf("***!!!*** PROCEED WITH CAUTION ***!!!***\n");
    // TODO: Add a message box to the display so user has to accept the warning or quit with while(true) statement
  }

  // Start SD Card
  SDCardStart(SD_CARD_ESP32_S3_TX_PIN);

  // Start the two CAN Bus, 500kbps for High Speed and 125kbps for the Medium Speed and both in ListenOnly Mode
  CANBusStart(mcp2515_0, CAN_500KBPS, 1); 
  CANBusStart(mcp2515_1, CAN_125KBPS, 1);

  // TFT_eSPI runs on HSPI bus, see Setup42_ILI9341_ESP32.h for pin definitions and more information
  TFT_Rectangle_ILI9341.init();
  pinMode(TFT_Rectangle_ILI9341_LEDPIN, OUTPUT);
  digitalWrite(TFT_Rectangle_ILI9341_LEDPIN, HIGH);

  // Calibrate the touch screen and retrieve the scaling factors, see Setup42_ILI9341_ESP32.h for more touch CS Pin definition
  TFT_Rectangle_ILI9341.setRotation(3);
  TFTRectangleILI9341TouchCalibrate();

  // Prepare display after calibration
  TFT_Rectangle_ILI9341.setRotation(3);
  TFT_Rectangle_ILI9341.fillScreen(TFT_LANDROVERGREEN);
  TFT_Rectangle_ILI9341.setTextColor(TFT_WHITE, TFT_LANDROVERGREEN, true);

  // Draw the initial Title and Menu
  drawHorizontalMenu(TOP_MENU_PROGRAM_NAME, TOP_MENU_PROGRAM_VERSION, TOP_MENU_Y_OFFSET, MENU_FONT);
}



void loop() {

  processMenu();
  
  
  //while (numberOfCANFramesReceived[0] < 10000) {

  //  /*
  //      For information
  //      As part of the MCP2515 CAN Bus Controllers are 3 Rx buffers in each, thus 3 for the 500kbps, and 3 for the 125kbps CAN Bus.
  //      500kbps CAN Bus will fill in a minimum period of 666us
  //      125kbps CAN bus will fill in a minimum period of 2664us
  //  */

  //  // Check the 500kbps bus as priority over 125kbps because 500kbps is faster and the buffers fill significantly more quickly
  //  if (CANBusCheckRecieved(mcp2515_0)) {
  //    if (CANBusReadCANData(mcp2515_0)) {
  //      CANFrameProcessing(0);
  //    }
  //  }
  //  // only check the 125kbps if there any no 500kbps messages in the MCP2515 buffers
  //  else if (CANBusCheckRecieved(mcp2515_1)) {
  //    if (CANBusReadCANData(mcp2515_1)) {
  //      CANFrameProcessing(1);
  //    }
  //  }
  //}
 
  //TemporaryOutputResults();

}



// Main Program Functions

// Processes the CAN Frame and triggers the required output
// For Land Rover Freelander 2 set:
// whichBus = 0 for 500kpbs (High Speed) and 1 for 125kbps (Medium Speed)
void CANFrameProcessing(byte whichCANBus) {
  
  // TODO - implement extend frame detection

  if (CANBusFirstRun) { 
    totalCANReceiveTimeTimer = micros();                              // Set the timer for calculating the CAN Frames per Second (cfps) 
    CANBusFirstRun = false; 
  }

  numberOfCANFramesReceived[whichCANBus]++;


  SDCardCANFrameSavvyCANOutput(whichCANBus);
}




// SD Card Functions

// Starts the SD Card Device (OpenLager) @ 2,000,000 baud on ESP32-S3 TxPin
void SDCardStart(byte TxPin) {
  // Due to the writing speed necessary I am using an STM32F411 based "OpenLager" (not to be confused with the slower "OpenLogger") 
  while (!SD_Port) {
    SD_Port.begin(2000000, SERIAL_8N1, -1, TxPin);                    // Rx pin in not required, we will not use any OpenLager read options
    delay(500);
  }
  delay(1000);                                                        // For start up stability (corruption was noticed if we try to write immediately)
  Serial.printf("\nSD Card writer initialised\n");
  sdCardFirstRun = true;
}

// Writes a CAN Frame to the SD Card Device (OpenLager) in SavvyCAN compatible format
void SDCardCANFrameSavvyCANOutput(byte whichCANBus) {
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
// mcp2515_0 to 500kpbs (High Speed) in Mode 1 (ListenOnly)
// mcp2515_1 to 125kbps (Medium Speed) in Mode 1 (ListenOnly)
// CANMode = 0 for Normal and 1 for ListenOnly
void CANBusStart(MCP2515 CANBusModule, CAN_SPEED CANSpeed, byte CANMode) {
  while (CANBusModule.reset() != MCP2515::ERROR_OK) { delay(500); }
  Serial.printf("\nMCP2515 Reset Successful\n");

  while (CANBusModule.setBitrate(CANSpeed) != MCP2515::ERROR_OK) { delay(500); }
  Serial.printf("MCP2515 BitRate Successful\n");

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
  Serial.printf("\nMCP2515 Listen Only Mode Successful\n");
  CANBusResetControlVariables();
}

// Set an MCP2515 CAN controller to Normal Mode
void CANbusSetNormalMode(MCP2515 CANBusModule) {
  while (CANBusModule.setNormalMode() != MCP2515::ERROR_OK) { delay(500); }
  Serial.printf("\nMCP2515 Normal Mode Successful\n");
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
void TemporaryOutputResults() {

  // Temporary - The results
  unsigned int totalCANReceiveTime = micros() - totalCANReceiveTimeTimer;
  Serial.printf("Total Time = %d us\n", totalCANReceiveTime);
  Serial.printf("500kbps Bus\n");
  Serial.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[0]);
  Serial.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[0] / totalCANReceiveTime * 1000000);

  SD_Port.printf("Total Time = %d us\n", totalCANReceiveTime);
  SD_Port.printf("500kbps Bus\n");
  SD_Port.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[0]);
  SD_Port.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[0] / totalCANReceiveTime * 1000000);

  TFT_Rectangle_ILI9341.printf("Total Time = %d us\n", totalCANReceiveTime);
  TFT_Rectangle_ILI9341.printf("500kbps Bus\n");
  TFT_Rectangle_ILI9341.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[0]);
  TFT_Rectangle_ILI9341.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[0] / totalCANReceiveTime * 1000000);

  // Calculate CAN Bus Capacity Used

  //// For DEBUGGING
  //Serial.printf("\n\n");
  //Serial.println("Capacity % calculation data");
  //Serial.print("numberOfFramesReceived = "); Serial.println(numberOfCANFramesReceived[0]);
  //Serial.print("totalReceiveTime = "); Serial.println(totalCANReceiveTime);

  uint8_t percentageOfBusCapacity = 0.00;
  float timeTaken = 0.000000;                                         // define high precision floating point math
  timeTaken = (((float)totalCANReceiveTime / (float)1000000));        // time taken to read numberOfFramesReceived
  uint64_t bitsTx = 119 * numberOfCANFramesReceived[0];               // 119 bits (average based on small sample of FL2 live CAN Data).
  uint64_t bitRate = bitsTx / timeTaken;                              // bit rate used on the CAN bus

  percentageOfBusCapacity = ((float)bitRate / 500000) * 100;          // the percentage used of the CAN bus

  //// For DEBUGGING
  //Serial.print("timeTaken = "); Serial.println(timeTaken, 6);
  //Serial.print("bitsTx = "); Serial.println(bitsTx);
  //Serial.print("bitRate = "); Serial.println(bitRate);

  Serial.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  TFT_Rectangle_ILI9341.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  Serial.printf("125kbps Bus\n");
  Serial.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[1]);
  Serial.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000);

  SD_Port.printf("125kbps Bus\n");
  SD_Port.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[1]);
  SD_Port.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000);

  TFT_Rectangle_ILI9341.printf("125kbps Bus\n");
  TFT_Rectangle_ILI9341.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[1]);
  TFT_Rectangle_ILI9341.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000);

  // Calculate CAN Bus Capacity Used

  // For DEBUGGING
  //Serial.printf("\n\n");
  //Serial.println("Capacity % calculation data");
  //Serial.print("numberOfFramesReceived = "); Serial.println(numberOfCANFramesReceived[1]);
  //Serial.print("totalReceiveTime = "); Serial.println(totalCANReceiveTime);

  percentageOfBusCapacity = 0.00;
  timeTaken = 0.000000;                                               // define high precision floating point math
  timeTaken = (((float)totalCANReceiveTime / (float)1000000));        // time taken to read numberOfFramesReceived
  bitsTx = 119 * numberOfCANFramesReceived[1];                        // 119 bits (average based on small sample of FL2 live CAN Data).
  bitRate = bitsTx / timeTaken;                                       // bit rate used on the CAN bus

  percentageOfBusCapacity = ((float)bitRate / 125000) * 100;          // the percentage used of the CAN bus

  //// For DEBUGGING
  //Serial.print("timeTaken = "); Serial.println(timeTaken, 6);
  //Serial.print("bitsTx = "); Serial.println(bitsTx);
  //Serial.print("bitRate = "); Serial.println(bitRate);

  Serial.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  TFT_Rectangle_ILI9341.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.flush();
  SD_Port.end();

  while (true);
}


// Calibrate the touch screen and retrieve the scaling factors, see Setup42_ILI9341_ESP32.h for more touch CS Pin definition
// To recalibrate set REPEAT_CALIBRATION = true 
// TODO - Checkout tft.calibrateTouch();
void TFTRectangleILI9341TouchCalibrate() {
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


// Draw a menu along the top of the TFT Display (320 x 240) Rotation 3
// Menu options are drawn in a line across the top of the screen
// Contents based on the current menu defined by menu structures
void drawHorizontalMenu(String menuHeader, String codeVersion, int yOffset, GFXfont menuFont) {
  TFT_Rectangle_ILI9341.fillScreen(TFT_LANDROVERGREEN);
  TFT_Rectangle_ILI9341.setTextColor(TFT_WHITE, TFT_LANDROVERGREEN, true);
  TFT_Rectangle_ILI9341.setTextDatum(TL_DATUM);
  TFT_Rectangle_ILI9341.setFreeFont(&menuFont);                       // Must set menu font for following calculations

  // Determine the maximum number of characters in a button so we can calculate the button width correctly
  byte maxChars = 0;
  menuButtons = 0;
  while (menu[menuButtons].text) {
    if (maxChars < String(menu[menuButtons].text).length()) maxChars = String(menu[menuButtons].text).length();
    menuButtons++;
  }
  if (maxChars < 10) maxChars = 10;                                   // Minimum button size so user can easily select it

  // Calculate the basic button positions
  int fontWidth = 10; // take a default value for now
  int yButtonMiddle = TFT_Rectangle_ILI9341.fontHeight() + yOffset;
  int xButtonWidth = fontWidth * maxChars;
  int yButtonHeight = TFT_Rectangle_ILI9341.fontHeight() * 1;

  // Draw the menu header
  TFT_Rectangle_ILI9341.drawString(menuHeader, 0, 0);
  TFT_Rectangle_ILI9341.setTextDatum(TR_DATUM);
  TFT_Rectangle_ILI9341.drawString(codeVersion, TFT_Rectangle_ILI9341.width(), 0);
  TFT_Rectangle_ILI9341.setTextDatum(MC_DATUM);

  // Draw the menu buttons
  TFT_Rectangle_ILI9341.setFreeFont(&menuFont);
  char handler[1] = "";
  menuButtons = 0;                                                    // Reset the Global Variable that tracks the number of buttons that will be active on the new menu
  menuButtonsPos = 0;
  while (menu[menuButtons].text) {
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
}


// Draw a menu in middle of the TFT Display (320 x 240) Rotation 3
// Menu options are drawn in a line down the screen and presented as buttons
// Contents based on the current menu defined by menu structures
void drawVerticalMenu(int yOffset, GFXfont headerFont, GFXfont menuFont) {
  TFT_Rectangle_ILI9341.fillScreen(TFT_LANDROVERGREEN);
  TFT_Rectangle_ILI9341.setTextColor(TFT_WHITE, TFT_LANDROVERGREEN, true);
  TFT_Rectangle_ILI9341.setTextDatum(MC_DATUM);
  TFT_Rectangle_ILI9341.setFreeFont(&menuFont);                       // Must set menu font for following calculations

  // Determine the maximum number of characters in a button so we can calculate the 
  //button width correctly and starting position of the buttons in the menu structure
  byte maxChars = 0;
  byte menuPosition = 0;
  menuButtons = 0;
  menuButtonsPos = 99;
  while (menu[menuPosition].text) {
    if (A == menu[menuPosition].action) {
      if (maxChars < String(menu[menuPosition].text).length()) maxChars = String(menu[menuPosition].text).length();
      if (menuButtonsPos == 99) menuButtonsPos = menuPosition;        // Capture the position of the first button in the menu structure
      menuButtons++;
    }
    menuPosition++;
  }
  if (maxChars < 10) maxChars = 10;                                   // Minimum button size so user can easily select it

  // Calculate the basic button positions
  int fontWidth = 10; // take a default value for now
  int xButtonMiddle = TFT_Rectangle_ILI9341.width() / 2;
  int yButtonMiddle = TFT_Rectangle_ILI9341.fontHeight() * 1.8;
  int xButtonWidth = fontWidth * maxChars;
  int yButtonHeight = TFT_Rectangle_ILI9341.fontHeight() * 1.30;

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
}


// Processes the menu buttons currently shown on the display
// Depending on the menu structure contents this function may
// display another menu or call another function
void processMenu() {
  while (true) {
    uint16_t t_x = 0, t_y = 0;                                        // To store the touch coordinates

    // Pressed will be set true is there is a valid touch on the screen
    bool pressed = TFT_Rectangle_ILI9341.getTouch(&t_x, &t_y);

    // Check if any key coordinate boxes contain the touch coordinates
    for (uint8_t b = 0; b < menuButtons; b++) {
      if (pressed && btnMenu[b].contains(t_x, t_y)) {
        btnMenu[b].press(true);                                       // tell the button it is pressed
      }
      else {
        btnMenu[b].press(false);                                      // tell the button it is NOT pressed
      }
    }

    // Check if any key has changed state
    for (uint8_t b = 0; b < menuButtons; b++) {

      TFT_Rectangle_ILI9341.setFreeFont(&MENU_FONT);

      if (btnMenu[b].justPressed()) {
        btnMenu[b].drawButton(true, menu[b + menuButtonsPos].text);   // draw invert
        delay(100);                                                   // UI debouncing
      }

      if (btnMenu[b].justReleased()) {
        btnMenu[b].drawButton(false, menu[b + menuButtonsPos].text);  // draw normal

        // Process the button press
        if (M == menu[b + menuButtonsPos].action) {                   // User selection required another menu
          menu = menu[b + menuButtonsPos].menu;
          drawVerticalMenu(MENU_Y_OFFSET, MENU_BOLD_FONT, MENU_FONT);
        }
        else if(A == menu[b + menuButtonsPos].action) {               // User selection calls a code function
          menu[b + menuButtonsPos].func(menu[b + menuButtonsPos].arg);

          menu = menuRoot;                                            // After the code function returns jump back to the root menu
          drawHorizontalMenu(TOP_MENU_PROGRAM_NAME, TOP_MENU_PROGRAM_VERSION, TOP_MENU_Y_OFFSET, MENU_FONT);
        }
      }
    }
  }
}