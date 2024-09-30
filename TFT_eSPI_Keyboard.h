// TFT_eSPI_Keyboard.h

#ifndef _TFT_ESPI_KEYBOARD_h
#define _TFT_ESPI_KEYBOARD_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "arduino.h"
#else
  #include "WProgram.h"
#endif

#include <TFT_eSPI.h>

class TFT_eSPI_Keyboard {
private:
  TFT_eSPI& tftDisplay;                                               // Reference to the TFT instance
  void drawKeyboard(uint8_t charSet);

public:
  TFT_eSPI_Keyboard(TFT_eSPI& tftDisplay);                            // Constructor
  String displayKeyboard();
};

#endif
