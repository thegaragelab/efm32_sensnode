/*--------------------------------------------------------------------------*
* SensNode - String Formatting Functions
*---------------------------------------------------------------------------*
* 2015-Nov-09 ShaneG
*
* Simple 'printf()' like string formatting for debug output.
*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "sensnode.h"

// Insertion character
#define INSERT_CHAR '#'

// Hex digits
static const char *HEXCHARS = "0123456789ABCDEF";

//---------------------------------------------------------------------------
// Helper functions
//---------------------------------------------------------------------------

/** Structure maintaining information about an output buffer
 */
typedef struct _SFORMAT_DATA {
  char *m_szOutput; //! Buffer to contain the output
  int   m_length;   //! Size of output buffer
  int   m_index;    //! Index of next character to write
  } SFORMAT_DATA;

/** Write a single character to a buffer
 *
 * @param ch the character to write.
 *
 * @return true if the write was successful
 */
static bool sformat_putc(char ch, SFORMAT_DATA *pData) {
  if(pData->m_index==pData->m_length)
    return false;
  pData->m_szOutput[pData->m_index++] = ch;
  return true;
  }

/** Output a NUL terminated string
 *
 * Uses the given writing function to output a NUL terminated string a
 * character at a time.
 *
 * @param pfnPutC pointer to the function to write output with.
 * @param pData pointer to user data to pass to the function.
 * @param cszValue the string to write.
 *
 * @return the number of characters written.
 */
static int _writeStr(FN_PUTC pfnPutC, void *pData, const char *cszValue) {
  int written = 0;
  // If we get a NULL string just write "(null)"
  if(cszValue==NULL)
    cszValue = "(null)";
  // Output the string
  while(*cszValue) {
    if((*pfnPutC)(*cszValue, pData))
      written++;
    cszValue++;
    }
  return written;
  }

/** Write an unsigned integer
 *
 * Emit an unsigned integer one character at a time using the writing function.
 * This is not the most efficient implementation (requires support for long
 * division) but does eliminate the need for an internal buffer.
 *
 * @param pfnPutC pointer to the function to write output with.
 * @param pData pointer to user data to pass to the function.
 * @param value the value to write.
 *
 * @return the number of characters written.
 */
static int _writeUnsigned(FN_PUTC pfnPutC, void *pData, unsigned long value) {
  int written = 0;
  unsigned long divisor = 1000000000L;
  unsigned long remain = value;
  while(divisor) {
    value = remain / divisor;
    remain = remain % divisor;
    divisor = divisor / 10;
    if((value>0)||(written>0)) {
      if((*pfnPutC)('0' + (int)value, pData))
        written++;
      }
    }
  return written;
  }

/** Write a signed integer
 *
 * Emit an signed integer one character at a time using the writing function.
 * This function simply takes care of the sign of the value and depends on
 * '_writeUnsigned()' to emit the digits.
 *
 * @param pfnPutC pointer to the function to write output with.
 * @param pData pointer to user data to pass to the function.
 * @param value the value to write.
 *
 * @return the number of characters written.
 */
static int _writeInt(FN_PUTC pfnPutC, void *pData, long value) {
  int written = 0;
  // Handle negative values
  if(value<0) {
    if((*pfnPutC)('-', pData))
      written++;
    value = -value;
    }
  // Emit the digits
  return written + _writeUnsigned(pfnPutC, pData, (unsigned long)value);
  }

/** Write a hexadecimal value
 *
 * Emit an unsigned value in hexadecimal one character at a time using the
 * writing function.
 *
 * @param pfnPutC pointer to the function to write output with.
 * @param pData pointer to user data to pass to the function.
 * @param value the value to write.
 *
 * @return the number of characters written.
 */
static int _writeHex(FN_PUTC pfnPutC, void *pData, unsigned long value, int digits) {
  int written = 0;
  int shift = (digits - 1) * 4;
  while(digits>0) {
    if((*pfnPutC)(HEXCHARS[(value >> shift) & 0x0f], pData))
      written++;
    digits--;
    shift -= 4;
    }
  return written;
  }

/** Do the actual formatting
 *
 * This function uses the current two characters of the input string to
 * determine what to print.
 *
 * @param pStream pointer to the output stream
 * @param ch1 the current character of the format string
 * @param ch2 the next character of the format string
 * @param args the argument list containing the embedded items
 *
 * @return true if both characters should be skipped, false if we only need
 *              to move ahead by one.
 */
static bool _format(FN_PUTC pfnPutC, void *pData, int *written, char ch1, char ch2, va_list *args) {
  bool skip = true;
  // Fail fast
  if(ch1==INSERT_CHAR) {
    // Use the second character to determine what is requested
    if((ch2==INSERT_CHAR)||(ch2=='\0')) {
      if((*pfnPutC)(INSERT_CHAR, pData))
        *written += 1;
      }
    else if(ch2=='c') {
      if((*pfnPutC)(va_arg(*args, int), pData))
        *written += 1;
      }
    else {
      switch(ch2) {
        case 's': // Insert string
          *written += _writeStr(pfnPutC, pData, va_arg(*args, char *));
          break;
        case 'i': // Insert integer
          *written += _writeInt(pfnPutC, pData, va_arg(*args, int));
          break;
        case 'u': // Insert unsigned
          *written += _writeUnsigned(pfnPutC, pData, va_arg(*args, unsigned));
          break;
        case 'l': // Insert long
          *written += _writeInt(pfnPutC, pData, va_arg(*args, long));
          break;
        case 'U': // Insert unsigned long
          *written += _writeUnsigned(pfnPutC, pData, va_arg(*args, unsigned long));
          break;
        case 'b': // Insert hex byte
          *written += _writeHex(pfnPutC, pData, va_arg(*args, int), 2);
          break;
        case 'w': // Insert hex word
          *written += _writeHex(pfnPutC, pData, va_arg(*args, unsigned), 4);
          break;
        case 'd': // Insert hex dword
          *written += _writeHex(pfnPutC, pData, va_arg(*args, unsigned long), 8);
          break;
        default: // Just emit the character
          if((*pfnPutC)(ch2, pData))
            *written += 1;
          break;
        }
      }
    }
  else {
    if((*pfnPutC)(ch1, pData))
      *written += 1;
    skip = false;
    }
  return skip;
  }

//---------------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------------

/** Generate a formatted string
 *
 * This function is used to generate strings from a format. This implementation
 * uses a user provided function pointer to output the data as it is generated.
 * Be aware that this is not a direct replacement for 'printf()' but provides
 * similar (but limited) functionality.
 *
 * A format string uses the # character to indicate insertions, the character
 * immediately following the # determines the size and output format of the
 * insertion value as follows:
 *
 *   ## - insert a # character, no value parameter is required.
 *   #c - insert a single ASCII character. Expects a matching 'char' parameter.
 *   #s - insert a NUL terminated string. Expects a matching 'const char *' parameter.
 *   #i - insert a decimal signed integer. Expects a matching 'int' parameter.
 *   #u - insert a decimal unsigned integer. Expects a matching 'unsigned' parameter.
 *   #l - insert a decimal signed long. Expects a matching 'long' parameter.
 *   #U - insert a decimal unsigned long. Expects a matching 'unsigned long' parameter.
 *   #b - insert a two digit hex byte. Expects a matching 'char' parameter.
 *   #w - insert a four digit hex byte. Expects a matching 'unsigned' parameter.
 *   #d - insert a eight digit hex byte. Expects a matching 'unsigned long' parameter.
 *
 * @param pfnPutC pointer the character output function
 * @param pData pointer to a user provided data block. This is passed to the
 *              character output function with each character.
 * @param cszFormat pointer to a NUL terminated format string.
 * @param args the variadic argument list.
 *
 * @return the number of characters generated.
 */
int vformat(FN_PUTC pfnPutC, void *pData, const char *cszString, va_list args) {
  char ch1, ch2 = *cszString;
  int written = 0;
  for(int index=1; ch2!='\0'; index++) {
    ch1 = ch2;
    ch2 = cszString[index];
    if(_format(pfnPutC, pData, &written, ch1, ch2, &args)) {
      // Move ahead an extra character so we wind up jumping by two
      ch1 = ch2;
      ch2 = cszString[++index];
      }
    }
  return written;
  }

/** Generate a formatted string
 *
 * This function uses the @see vformat function to generate a formatted string
 * in memory.
 *
 * @param szBuffer pointer to the buffer to place the string in
 * @param length the size of the buffer.
 * @param cszString pointer to a nul terminated format string in RAM.
 *
 * @return the number of characters (excluding the terminating NUL) that
 *         were written. If this value is equal to length the resulting
 *         string will not be NUL terminated.
 */
int sformat(char *szBuffer, int length, const char *cszString, ...) {
  va_list args;
  va_start(args, cszString);
  if(length<=0)
    return length;
  SFORMAT_DATA data;
  data.m_szOutput = szBuffer;
  data.m_length = length;
  data.m_index = 0;
  int result = vformat((FN_PUTC)&sformat_putc, &data, cszString, args);
  va_end(args);
  // Add a terminating NUL if there is room
  if(data.m_index!=data.m_length)
    data.m_szOutput[data.m_index] = '\0';
  return result;
  }

