/*
 Name:		    TFT_eSPI_Keyboard Library
 Description: Designed Specifically for use with the TFT_eSPI by bodmer
 Created:	    Oct 2024
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

/*
  FONT 2 Characters
    Standard ASCII ASC(32) to ASC(126)
*/

#include "TFT_eSPI_Keyboard.h"
TFT_eSPI_Keyboard::TFT_eSPI_Keyboard(TFT_eSPI& tftDisplay):tftDisplay(tftDisplay) {}  // constructor definition with an initializer

const auto  RESULT_MAX_CHARACTERS = 32;
char        result[RESULT_MAX_CHARACTERS + 1] = "";
uint8_t     resultPos = 0;

// Setup the availabe keys
// Special buttons have more than one character
const auto  KEYBOARD_NUMBER_OF_KEYS = 42;
const char* KEYBOARD_KEYS[KEYBOARD_NUMBER_OF_KEYS]{
            "", "", "", "", "", "", "", "", "", "",
            "", "", "", "", "", "", "", "", "", "",
            "", "", "", "", "", "", "", "", "",
            "", "", "", "", "", "", "", "", "",
            "", "", "", "" };
const char* LOWER[KEYBOARD_NUMBER_OF_KEYS]{                           // charSet 0
            "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
            "q", "w", "e", "r", "t", "y", "u", "i", "o", "p",
            "a", "s", "d", "f", "g", "h", "j", "k", "l",
            "CAPS", "z", "x", "c", "v", "b", "n", "m", "<--",
            "EXIT", "!#@", " ", "OK" };
const char* UPPER[KEYBOARD_NUMBER_OF_KEYS]{                           // charSet 1
            "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
            "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P",
            "A", "S", "D", "F", "G", "H", "J", "K", "L",
            "CAPS", "Z", "X", "C", "V", "B", "N", "M", "<--",
            "EXIT", "!#@", " ", "OK" };
const char* SPECIAL[KEYBOARD_NUMBER_OF_KEYS]{                         // charSet 2
            "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
            "$", "#", "%", "&", "*", "_", "-", "/", "x", "+",
            "[", "|", "]", "(", "~", ")", "<", "=", ">",
            "@", "'", ";", ":", "!", "?", ",", ".", "<--",
            "EXIT", "ABC", " ", "OK" };

TFT_eSPI_Button btnKeyboard[KEYBOARD_NUMBER_OF_KEYS];

// Positioning and spacing of the keyboard
const auto KEYBOARD_START_X = 20;
const auto KEYBOARD_START_Y = 20;
const auto KEY_WIDTH = 24;
const auto KEY_HEIGHT = 24;
const auto KEY_HORIZONTAL_SPACING = 28;
const auto KEY_VERTICLE_SPACING = 28;
const auto KEY_FONT = FreeSerif9pt7b;
const auto TEXT_AREA_WIDTH = (KEY_HORIZONTAL_SPACING * 10) - (KEY_HORIZONTAL_SPACING - KEY_WIDTH);
const auto TEXT_AREA_FG_COLOUR = TFT_BLUE;
const auto TEXT_AREA_BG_COLOUR = TFT_LIGHTGREY;



// Displays a keyboard and requests input
String TFT_eSPI_Keyboard::displayKeyboard() {
  uint8_t charSet = 0;

  // Setup the display
  tftDisplay.setFreeFont(&KEY_FONT);
  tftDisplay.setTextDatum(TL_DATUM);
  tftDisplay.setTextColor(TEXT_AREA_FG_COLOUR, TEXT_AREA_BG_COLOUR, true);

  // Draw the Text Entry Area
  tftDisplay.fillSmoothRoundRect(KEYBOARD_START_X, KEYBOARD_START_Y, TEXT_AREA_WIDTH, KEY_HEIGHT, 3, TEXT_AREA_BG_COLOUR);

  // Draw the initial keyboard, cherSet 0 (Lower Case)
  drawKeyboard(charSet);

  // Handle the keyboard input
  int xwidth = 0;                                                     // Tracks the width of the text in the Text Entry Area
  while (true) {                                                      // Create an endless loop until either OK or Exit is pressed
    uint16_t t_x = 0, t_y = 0;                                        // To store the touch coordinates  

    // Pressed will be set true is there is a valid touch on the screen
    bool pressed = tftDisplay.getTouch(&t_x, &t_y);

    // Check if any key coordinate boxes contain the touch coordinates
    for (uint8_t btnCounter = 0; btnCounter < KEYBOARD_NUMBER_OF_KEYS; btnCounter++) {
      if (pressed && btnKeyboard[btnCounter].contains(t_x, t_y)) {
        btnKeyboard[btnCounter].press(true);                          // tell the button it is pressed
      }
      else {
        btnKeyboard[btnCounter].press(false);                         // tell the button it is NOT pressed
      }
    }
   
    // Check if any key has changed state
    for (uint8_t btnCounter = 0; btnCounter < KEYBOARD_NUMBER_OF_KEYS; btnCounter++) {
      if (btnKeyboard[btnCounter].justReleased()) {
        if (strcmp(KEYBOARD_KEYS[btnCounter], "CAPS") == 0) {         // Lower or Upper Case
          btnKeyboard[btnCounter].drawButton(false, "^");             // draw normal
        }
        else {
          btnKeyboard[btnCounter].drawButton(false, String(KEYBOARD_KEYS[btnCounter]));   // draw normal
        }
      }

      if (btnKeyboard[btnCounter].justPressed()) {
        
        // Handle special key presses
        if (strcmp(KEYBOARD_KEYS[btnCounter], "OK") == 0) {           // OK
          btnKeyboard[btnCounter].drawButton(true, String("OK"));     // draw inverted
          return result;
        }
        else if (strcmp(KEYBOARD_KEYS[btnCounter], "EXIT") == 0) {    // Exit
          btnKeyboard[btnCounter].drawButton(true, String("EXIT"));   // draw inverted
          return "";
        }
        else if (strcmp(KEYBOARD_KEYS[btnCounter], "<--") == 0) {     // Back Space
          btnKeyboard[btnCounter].drawButton(true, String("<--"));    // draw inverted
          result[resultPos] = 0;
          if (resultPos > 0) {
            resultPos--;
            result[resultPos] = 0;
          }
        }
        else if (strcmp(KEYBOARD_KEYS[btnCounter], "CAPS") == 0) {    // Lower or Upper Case
          // Letters or Special Characters
          if (charSet != 2) {
          btnKeyboard[btnCounter].drawButton(true, String("^"));      // draw inverted
          }
          else {
            btnKeyboard[btnCounter].drawButton(true, String("@"));    // draw inverted
          }
          if (charSet == 0) charSet = 1; else charSet = 0;
          drawKeyboard(charSet);
        }
        else if (strcmp(KEYBOARD_KEYS[btnCounter], "!#@") == 0) {     // Show Special Chars
          btnKeyboard[btnCounter].drawButton(true, String("!#@"));    // draw inverted
          charSet = 2;
          drawKeyboard(charSet);
        }
        else if (strcmp(KEYBOARD_KEYS[btnCounter], "ABC") == 0) {     // Back to Letters
          btnKeyboard[btnCounter].drawButton(true, String("ABC"));    // draw inverted
          charSet = 0;
          drawKeyboard(charSet);
        }
        // Handle normal key presses
        else {
          btnKeyboard[btnCounter].drawButton(true, String(KEYBOARD_KEYS[btnCounter]));    // draw inverted
          // Ensure another character will fit in the Text Entry Area
          if (resultPos < RESULT_MAX_CHARACTERS && xwidth < TEXT_AREA_WIDTH - tftDisplay.textWidth("A")) {
            result[resultPos] = KEYBOARD_KEYS[btnCounter][0];
            resultPos++;
            result[resultPos] = 0;
          }
        }

        // Draw the string, the value returned is the width in pixels
        tftDisplay.setTextColor(TEXT_AREA_FG_COLOUR, TEXT_AREA_BG_COLOUR, true);
        xwidth = tftDisplay.drawString(result, KEYBOARD_START_X + 4, KEYBOARD_START_Y + 4);

        // Cover the remain space in the the Text Entry Area
        tftDisplay.fillSmoothRoundRect(KEYBOARD_START_X + xwidth + 4, KEYBOARD_START_Y, TEXT_AREA_WIDTH - xwidth - 9, KEY_HEIGHT, 3, TEXT_AREA_BG_COLOUR);
        delay(30);                                                   // UI debouncing
      }
    }
  }
}


void TFT_eSPI_Keyboard::drawKeyboard(uint8_t charSet) {
  // The first thig we need to do is copy the required keyboard chars into the Array
  for (uint8_t i = 0; i < 42; i++) {
    if (charSet == 0) KEYBOARD_KEYS[i] = LOWER[i];
    else if (charSet == 1) KEYBOARD_KEYS[i] = UPPER[i];
    else if (charSet == 2) KEYBOARD_KEYS[i] = SPECIAL[i];
  }

  uint16_t keyPosX = KEYBOARD_START_X;
  uint16_t keyPosY = KEYBOARD_START_Y;
  uint8_t  keyWidth = KEY_WIDTH;
  const char* handler;
  
  // Draw Keyboard
  // For simplisity in conditions only check the UPPER array regardless of charSet currently displayed
  for (uint8_t i = 0; i < 42; i++) {
    if (strcmp(UPPER[i], "1") == 0 || strcmp(UPPER[i], "Q") == 0 || strcmp(UPPER[i], "A") == 0 || strcmp(UPPER[i], "CAPS") == 0 || strcmp(UPPER[i], "EXIT") == 0) {
      if (strcmp(UPPER[i], "A") != 0) {
        keyPosX = KEYBOARD_START_X;
      }
      else {
        keyPosX = KEYBOARD_START_X + (KEY_HORIZONTAL_SPACING / 2);
      }
      keyPosY += KEY_VERTICLE_SPACING;
    }
    else {
      keyPosX += KEY_HORIZONTAL_SPACING;
    }

    // Handle the Special Keys, size and spacing of the next button
    handler = KEYBOARD_KEYS[i];                                       // handler maybe over written
    if (strcmp(UPPER[i], "CAPS") == 0) {
      if (charSet != 2) handler = "^"; else handler = "@";
      keyWidth = KEY_WIDTH * 1.62;
    }
    else if (strcmp(UPPER[i], "Z") == 0) {
      keyPosX += KEY_HORIZONTAL_SPACING / 2;
      keyWidth = KEY_WIDTH;
    }
    else if (strcmp(UPPER[i], "<--") == 0) {
      keyWidth = KEY_WIDTH * 1.5;
    }
    else if (strcmp(UPPER[i], "EXIT") == 0) {
      keyWidth = KEY_WIDTH * 2.75;
    }
    else if (strcmp(UPPER[i], "!#@") == 0) {
      keyWidth = KEY_WIDTH * 2.18;
      keyPosX += KEY_HORIZONTAL_SPACING * 1.5;
    }
    else if (strcmp(UPPER[i], "ABC") == 0) {
      keyWidth = KEY_WIDTH * 2.18;
      keyPosX += KEY_HORIZONTAL_SPACING * 1.5;
    }
    else if (strcmp(UPPER[i], " ") == 0) {
      keyWidth = KEY_WIDTH * 4.5;
      keyPosX += KEY_HORIZONTAL_SPACING;
    }
    else if (strcmp(UPPER[i], "OK") == 0) {
      keyWidth = KEY_WIDTH * 1.5;
      keyPosX += KEY_HORIZONTAL_SPACING * 3;
    }
    else {
      keyWidth = KEY_WIDTH;
    }

    // Draw the Key and set up the button
    btnKeyboard[i].initButtonUL(&tftDisplay,
      keyPosX,
      keyPosY,
      keyWidth,
      KEY_HEIGHT,
      TFT_YELLOW, TFT_BLUE, TFT_YELLOW, (char*)handler, 1);           // initButton limits the amount of text drawn, draw text in the drawButton() function
    btnKeyboard[i].drawButton(false, handler);                        // Specifiy the text for the button because initButton will not display the full text length
    //btnKeyboard[i].press(false);                                      // I've had issues in the past when buttons are used more than once or reallocated
  }
}
