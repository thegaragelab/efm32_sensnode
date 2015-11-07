/*--------------------------------------------------------------------------*
* SensNode - Nokia LCD Driver
*---------------------------------------------------------------------------*
* 07-Nov-2015 ShaneG
*
* Simple driver for the Nokia LCD display.
---------------------------------------------------------------------------*/
#ifndef _NOKIALCD_H
#define _NOKIALCD_H

// Display dimensions
#define WIDTH       84
#define HEIGHT      48
#define LINES       6
#define FONT_WIDTH  7
#define FONT_HEIGHT 8

/** Initialise the LCD display
 *
 * @param dc the comand/data pin
 * @param reset the reset pin
 * @param sel device select pin
 */
void lcdInit(PIN dc, PIN reset, PIN sel);

/** Clear the display
 */
void lcdClear(bool invert);

/** Clear a region of the display
 */
void lcdClearRect(int y, int height, bool invert);

/** Write a character to the display
 */
void lcdWriteCh(int x, int y, char ch, bool invert);

/** Write a string to the display
 */
void lcdWriteStr(int x, int y, const char *str, bool invert);

/** Write an arbitrary glyph to the display
 */
void lcdWriteGlyph(int x, int y, const uint8_t *glyph, int width, int height, bool invert);

#endif /* __NOKIALCD_H */
