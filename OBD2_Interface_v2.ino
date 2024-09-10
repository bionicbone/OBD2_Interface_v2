/*
 Name:		OBD2_Interface_v2.ino
 Created:	Sep 2024
 Author:	Kevin Guest AKA TheBionicBone

  THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  THIS SOFTWARE IS PROVIDED FOR EDUCATIONAL PURPOSES ONLY
*/

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

// include the arduino-ESP32 core SPI library
#include <SPI.h>
// include the arduino-ESP32 core HardwareSerial library which is used for the SD Card Writer
#include <HardwareSerial.h>

/*
ESP32 & two MCP2515 Breakout Board (HW-184) - 8 MHz Crystal

MCP2515_0 for 500kbps CAN Bus
INT = pin 22
SCK = pin 18
SI = pin 23
SO = pin 19
CS = pin 17
GND = GND
VCC = 5v

MCP2515_1 for 125kbps CAN Bus
INT = pin 21
SCK = pin 18
SI = pin 23
SO = pin 19
CS = pin 16
GND = GND
VCC = 5v

*/

// Add MCP2515 Modules
MCP2515 mcp2515_0(17);    // 500kbps
MCP2515 mcp2515_1(16);    // 125kbps
struct can_frame frame;

// Add SD Card Serial Port
HardwareSerial SD_Port(1);

// constants
const auto STANDARD_SERIAL_OUTPUT_BAUD = 115200;                      // Must be at least 2,000,000 to keep up with Land Rover Freelander 2 
const auto SD_CARD_ESP32_TX_PIN = 26;

// control variables
bool sdCardFirstRun = false;                                          // Will set to true when SD Card initialises, trigger for SavvyCAN header
bool CANBusFirstRun = false;                                          // Will set to true when CAN Bus mode changes, trigger for cfps timer to start
unsigned long       totalCANReceiveTimeTimer = 0;                     // Times how long we have been receiving CAN Frames
unsigned int        numberOfCANFramesReceived[2] = { 0,0 };           // Counts the number of CAN Frames received

void setup() {
  // Serial needs to be at least 2,000,000 baud otherwise lines will be dropped when outputting CAN lines to the standard serial output display.
  while (!Serial) { Serial.begin(STANDARD_SERIAL_OUTPUT_BAUD); delay(100); }

  Serial.printf("\nName: OBD2_Interface_v2\nCreated:	Sep 2024\nAuthor : Kevin Guest AKA TheBionicBone\n");

  // Start SD Card
  SDCardStart(SD_CARD_ESP32_TX_PIN);

  // Start the two CAN Bus, 500kbps for High Speed and 125kbps for the Medium Speed and both in ListenOnly Mode
  CANBusStart(mcp2515_0, CAN_500KBPS, 1); 
  CANBusStart(mcp2515_1, CAN_125KBPS, 1);
}


void loop() {

  while (numberOfCANFramesReceived[0] < 10000) {

    if (CANBusCheckRecieved(mcp2515_0)) {                               // Checks 500kbps bus as priority over 125kbps
      if (CANBusReadCANData(mcp2515_0)) {                               // because 500kbps is faster and has more frames per second to catch
        CANFrameProcessing(0);
      }
    }
    else if (CANBusCheckRecieved(mcp2515_1)) {
      if (CANBusReadCANData(mcp2515_1)) {
        CANFrameProcessing(1);
      }
    }

  }
 
  TemporaryOutputResults();

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

// Starts the SD Card Device (OpenLager) @ 2,000,000 baud on ESP32 TxPin
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
    SD_Port.printf("Time Stamp, ID, Extended, Bus, LEN, D1, D2, D3, D4, D5, D6, D7, D8\n"); 
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

  // Calculate CAN Bus Capacity Used

  //// For DEBUGGING
  //Serial.printf("\n\n");
  //Serial.println("Capacity % calculation data");
  //Serial.print("numberOfFramesReceived = "); Serial.println(numberOfCANFramesReceived[0]);
  //Serial.print("totalReceiveTime = "); Serial.println(totalCANReceiveTime);

  uint8_t percentageOfBusCapacity = 0.00;
  float timeTaken = 0.000000;                                     // define high precision floating point math
  timeTaken = (((float)totalCANReceiveTime / (float)1000000));       // time taken to read numberOfFramesReceived
  uint64_t bitsTx = 119 * numberOfCANFramesReceived[0];                 // 119 bits (average based on small sample of FL2 live CAN Data).
  uint64_t bitRate = bitsTx / timeTaken;                          // bit rate used on the CAN bus

  percentageOfBusCapacity = ((float)bitRate / 500000) * 100;    // the percentage used of the CAN bus

  //// For DEBUGGING
  //Serial.print("timeTaken = "); Serial.println(timeTaken, 6);
  //Serial.print("bitsTx = "); Serial.println(bitsTx);
  //Serial.print("bitRate = "); Serial.println(bitRate);

  Serial.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  Serial.printf("125kbps Bus\n");
  Serial.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[1]);
  Serial.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000);

  SD_Port.printf("125kbps Bus\n");
  SD_Port.printf("numberOfFramesReceived = %d\n", numberOfCANFramesReceived[1]);
  SD_Port.printf("Frames per Second = %.2f fps\n", (float)numberOfCANFramesReceived[1] / totalCANReceiveTime * 1000000);

  // Calculate CAN Bus Capacity Used

  // For DEBUGGING
  //Serial.printf("\n\n");
  //Serial.println("Capacity % calculation data");
  //Serial.print("numberOfFramesReceived = "); Serial.println(numberOfCANFramesReceived[1]);
  //Serial.print("totalReceiveTime = "); Serial.println(totalCANReceiveTime);

  percentageOfBusCapacity = 0.00;
  timeTaken = 0.000000;                                     // define high precision floating point math
  timeTaken = (((float)totalCANReceiveTime / (float)1000000));       // time taken to read numberOfFramesReceived
  bitsTx = 119 * numberOfCANFramesReceived[1];                 // 119 bits (average based on small sample of FL2 live CAN Data).
  bitRate = bitsTx / timeTaken;                          // bit rate used on the CAN bus

  percentageOfBusCapacity = ((float)bitRate / 125000) * 100;    // the percentage used of the CAN bus

  //// For DEBUGGING
  //Serial.print("timeTaken = "); Serial.println(timeTaken, 6);
  //Serial.print("bitsTx = "); Serial.println(bitsTx);
  //Serial.print("bitRate = "); Serial.println(bitRate);

  Serial.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.printf("Capacity used %d%% (limited accuracy)\n", percentageOfBusCapacity);

  SD_Port.flush();
  SD_Port.end();

  while (true);
}
