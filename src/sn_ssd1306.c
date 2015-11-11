/*--------------------------------------------------------------------------*
* SensNode - SSD1306 OLED Driver
*---------------------------------------------------------------------------*
* 11-Nov-2015 ShaneG
*
* Simple driver for SSD1306 based I2C OLED displays.
---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "sensnode.h"
#include "ssd1306.h"

// The font to use
extern const uint8_t BASE_FONT[];

// Size of write buffer to use
#define BUFFER_SIZE 16

// Address of the target (either 0x3C or 0x3D)
#define OLED_ADDRESS 0x3C

// Control byte values
#define BYTE_CMD 0x80
#define BYTE_DTA 0xC0

//---------------------------------------------------------------------------
// Helper functions
//---------------------------------------------------------------------------

static uint8_t g_buffer[BUFFER_SIZE]; // Output buffer
static uint8_t g_index;               // Current index

/** Start a write sequence
 *
 * This sets up the write buffer to collect data intended for the display.
 * Data will be queued until the buffer is full or writeEnd() is called
 * when it will be transmitted to the target.
 */
static void writeBegin() {
  g_index = 0;
  }

/** Flush the buffer contents (if any) to the OLED
 *
 */
static void writeEnd() {
  if(g_index>0) {
    i2cSendTo(OLED_ADDRESS, g_buffer, g_index);
    g_index = 0;
    }
  }

/** Add a command byte to the output buffer
 *
 * @param cmd the command byte to send
 */
static void writeCommand(uint8_t cmd) {
  // Flush the buffer if needed
  if(g_index==BUFFER_SIZE)
    writeEnd();
  // Add the command value
  g_buffer[g_index++] = BYTE_CMD;
  g_buffer[g_index++] = cmd;
  }

/** Add a data byte to the output buffer
 *
 * @param data the data byte to send
 */
static void writeData(uint8_t data) {
  // Flush the buffer if needed
  if(g_index==BUFFER_SIZE)
    writeEnd();
  // Add the command value
  g_buffer[g_index++] = BYTE_DTA;
  g_buffer[g_index++] = data;
  }

/** Move to the specified row and column
 *
 * @param row the row number to move to (0 to LINES - 1)
 * @param col the column number to move to (0 to WIDTH - 1)
 */
static void setPosition(uint8_t row, uint8_t col) {
  writeCommand(0xB0 | (row % LINES));
  writeCommand(0x00 | ((col % WIDTH) & 0x0f));
  writeCommand(0x10 | ((col % WIDTH) >> 4));
  }

/** Write a glyph to the display
 *
 * This is used to write graphics and text. Assumes that writeBegin() has
 * already been called.
 */
static void writeGlyph(int x, int y, const uint8_t *glyph, int width, int height, bool invert) {
  // Write the bytes of the glyph
  int offset = 0;
  while(height) {
    // Set the position
    setPosition(y, x);
    for(int dx=0; dx<width; dx++,offset++)
      writeData(invert?~glyph[offset]:glyph[offset]);
    // Move to the next line
    height--;
    y++;
    }
  }

//---------------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------------

/** Initialise the OLED display
 *
 */
void oledInit() {
  i2cConfig();
  // Initialise the LCD
  writeBegin();
  writeCommand(0xAF); // Turn display on
  writeEnd();
  }

/** Clear the display
 */
void oledClear(bool invert) {
  writeBegin();
  setPosition(0, 0);
  // Fill in the whole display
  for(int index = 0; index < (WIDTH * LINES); index++)
    writeData(invert?0xff:0x00);
  writeEnd();
  }

/** Clear a region of the display
 */
void oledClearRect(int y, int height, bool invert) {
  // Do the clear
  writeBegin();
  while(height) {
    setPosition(y, 0);
    // Fill the line
    for(int index=0; index<WIDTH; index++)
      writeData(invert?0xff:0x00);
    // Step to the next line
    height--;
    }
  writeEnd();
  }

/** Write a character to the display
 */
void oledWriteCh(int x, int y, char ch, bool invert) {
  // Make sure the character is valid
  if((ch<0x20)||(ch>0x7f))
    ch = 0x20;
  // Do the write
  writeBegin();
  setPosition(y, x);
  // Write the character
  writeData(invert?0xff:0x00);
  writeGlyph(x + 1, y, BASE_FONT + (ch - 0x20) * (FONT_WIDTH - 2), FONT_WIDTH - 2, 1, invert);
  writeData(invert?0xff:0x00);
  writeEnd();
  }

/** Write a string to the display
 */
void oledWriteStr(int x, int y, const char *str, bool invert) {
  writeBegin();
  while (*str) {
    // Make sure the character is valid
    char ch = *str;
    if((ch<0x20)||(ch>0x7f))
      ch = 0x20;
    // Set the position
    setPosition(y, x);
    // Write the character
    writeData(invert?0xff:0x00);
    writeGlyph(x + 1, y, BASE_FONT + (ch - 0x20) * (FONT_WIDTH - 2), FONT_WIDTH - 2, 1, invert);
    writeData(invert?0xff:0x00);
    // Move to the next character (and position)
    str++;
    x += FONT_WIDTH;
    }
  writeEnd();
  }

/** Write an arbitrary glyph to the display
 */
void oledWriteGlyph(int x, int y, const uint8_t *glyph, int width, int height, bool invert) {
  writeBegin();
  writeGlyph(x, y, glyph, width, height, invert);
  writeEnd();
  }
