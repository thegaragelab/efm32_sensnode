/*--------------------------------------------------------------------------*
* SensNode - Nokia LCD Driver
*---------------------------------------------------------------------------*
* 07-Nov-2015 ShaneG
*
* Simple driver for the Nokia LCD display.
---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "sensnode.h"
#include "nokialcd.h"

// The font to use
extern const uint8_t BASE_FONT[];

//---------------------------------------------------------------------------
// Helper functions
//---------------------------------------------------------------------------

static PIN g_dc;
static PIN g_reset;
static PIN g_sel;

/** Send data to the LCD
 */
static void sendData(uint8_t data) {
  pinWrite(g_dc, true);
  spiWrite(&data, 1);
  }

/** Send a command to the LCD
 */
static void sendCommand(uint8_t data) {
  pinWrite(g_dc, false);
  spiWrite(&data, 1);
  }

/** Write a glyph to the display
 *
 * This is used to write graphics and text. Assumes the LCD is in it's selected state.
 */
static void writeGlyph(int x, int y, const uint8_t *glyph, int width, int height, bool invert) {
  // Write the bytes of the glyph
  int offset = 0;
  while(height) {
    // Set the position
    sendCommand(0x80 | (x % WIDTH));
    sendCommand(0x40 | (y % LINES));
    for(int dx=0; dx<width; dx++,offset++)
      sendData(invert?~glyph[offset]:glyph[offset]);
    // Move to the next line
    height--;
    y++;
    }
  }

//---------------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------------

/** Initialise the LCD display
 *
 * @param dc the comand/data pin
 * @param reset the reset pin
 * @param sel device select pin
 */
void lcdInit(PIN dc, PIN reset, PIN sel) {
  // Save the pins
  pinConfig(dc, DIGITAL_OUTPUT, 0);
  g_dc = dc;
  pinConfig(reset, DIGITAL_OUTPUT, 0);
  g_reset = reset;
  pinConfig(sel, DIGITAL_OUTPUT, 1);
  g_sel = sel;
  // Reset the LCD
  pinWrite(g_reset, true);
  // Initialise the LCD
  spiConfig(false, true, true);
  pinWrite(g_sel, false);
  sendCommand(0x21);  // LCD Extended Commands.
  sendCommand(0xB1);  // Set LCD Vop (Contrast) 0xB1/0xA1.
  sendCommand(0x04);  // Set Temp coefficent. //0x04
  sendCommand(0x14);  // LCD bias mode 1:48. //0x13
  sendCommand(0x0C);  // Normal display, horizontal addressing
  sendCommand(0x20);  // LCD Normal commands
  sendCommand(0x0C);  // Normal display, horizontal addressing
  // Deselect the display
  pinWrite(g_sel, true);
  }

/** Clear the display
 */
void lcdClear(bool invert) {
  // Select the display
  spiConfig(false, true, true);
  pinWrite(g_sel, false);
  // Set the position
  sendCommand(0x80);
  sendCommand(0x40);
  // Fill in the whole display
  for(int index = 0; index < (WIDTH * LINES); index++)
    sendData(invert?0xff:0x00);
  // Deselect the display
  pinWrite(g_sel, true);
  }

/** Clear a region of the display
 */
void lcdClearRect(int y, int height, bool invert) {
  // Select the display
  spiConfig(false, true, true);
  pinWrite(g_sel, false);
  // Do the clear
  while(height) {
    // Set the position
    sendCommand(0x80);
    sendCommand(0x40 | (y % LINES));
    // Fill the line
    for(int index=0; index<WIDTH; index++)
      sendData(invert?0xff:0x00);
    // Step to the next line
    height--;
    }
  // Deselect the display
  pinWrite(g_sel, true);
  }

/** Write a character to the display
 */
void lcdWriteCh(int x, int y, char ch, bool invert) {
  // Select the display
  spiConfig(false, true, true);
  pinWrite(g_sel, false);
  // Make sure the character is valid
  if((ch<0x20)||(ch>0x7f))
    ch = 0x20;
  // Set the position
  sendCommand(0x80 | (x % WIDTH));
  sendCommand(0x40 | (y % LINES));
  // Write the character
  sendData(invert?0xff:0x00);
  writeGlyph(x + 1, y, BASE_FONT + (ch - 0x20) * (FONT_WIDTH - 2), FONT_WIDTH - 2, 1, invert);
  sendData(invert?0xff:0x00);
  // Deselect the display
  pinWrite(g_sel, true);
  }

/** Write a string to the display
 */
void lcdWriteStr(int x, int y, const char *str, bool invert) {
  // Select the display
  spiConfig(false, true, true);
  pinWrite(g_sel, false);
  while (*str) {
    // Make sure the character is valid
    char ch = *str;
    if((ch<0x20)||(ch>0x7f))
      ch = 0x20;
    // Set the position
    sendCommand(0x80 | (x % WIDTH));
    sendCommand(0x40 | (y % LINES));
    // Write the character
    sendData(invert?0xff:0x00);
    writeGlyph(x + 1, y, BASE_FONT + (ch - 0x20) * (FONT_WIDTH - 2), FONT_WIDTH - 2, 1, invert);
    sendData(invert?0xff:0x00);
    // Move to the next character (and position)
    str++;
    x += FONT_WIDTH;
    }
  // Deselect the display
  pinWrite(g_sel, true);
  }

/** Write an arbitrary glyph to the display
 */
void lcdWriteGlyph(int x, int y, const uint8_t *glyph, int width, int height, bool invert) {
  // Select the display
  spiConfig(false, true, true);
  pinWrite(g_sel, false);
  // Do the write
  writeGlyph(x, y, glyph, width, height, invert);
  // Deselect the display
  pinWrite(g_sel, true);
  }
