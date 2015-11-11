/*--------------------------------------------------------------------------*
* SensNode - SSD1306 OLED I2C Driver
*---------------------------------------------------------------------------*
* 11-Nov-2015 ShaneG
*
* Simple driver for SSD1306 OLED displays.
---------------------------------------------------------------------------*/
#ifndef _SSD1306_H
#define _SSD1306_H

// Display dimensions
#define WIDTH       128
#define HEIGHT      64
#define LINES       8
#define FONT_WIDTH  7
#define FONT_HEIGHT 8

/** Initialise the OLED display
 */
void oledInit();

/** Clear the display
 */
void oledClear(bool invert);

/** Clear a region of the display
 */
void oledClearRect(int y, int height, bool invert);

/** Write a character to the display
 */
void oledWriteCh(int x, int y, char ch, bool invert);

/** Write a string to the display
 */
void oledWriteStr(int x, int y, const char *str, bool invert);

/** Write an arbitrary glyph to the display
 */
void oledWriteGlyph(int x, int y, const uint8_t *glyph, int width, int height, bool invert);

#endif /* __SSD1306_H */
