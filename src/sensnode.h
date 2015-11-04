/*--------------------------------------------------------------------------*
* SensNode Core Library
*---------------------------------------------------------------------------*
* 31-Aug-2015 ShaneG
*
* Defines a common set of functions and structures to use with SensNode
* compatible boards. Although not mandatory to use for SensNode development
* the goal is to make it easy to switch between boards and have a common API.
---------------------------------------------------------------------------*/
#ifndef __SENSNODE_H
#define __SENSNODE_H

// Bring in required definitions
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>

// All library functions are C callable
#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------
// Timing and delays
//---------------------------------------------------------------------------

/** Time units
 */
typedef enum {
  TICK = 0,    //!< Raw ticks (processor dependent)
  MICROSECOND, //!< 1/1000000 of a second
  MILLISECOND, //!< 1/1000 of a second
  SECOND,      //!< Whole seconds
  } TIMEUNIT;

/** Get the current system tick count
 *
 * Each processor board maintains a count of system ticks since power up, this
 * function returns the current value of that count. The duration of a single
 * tick is processor independent - use the 'timeExpired()' function to determine
 * if a time period has been exceeded or 'timeElapsed()' to determine the
 * amount of time between two tick counts.
 *
 * @return the current system tick count.
 */
uint32_t getTicks();

/** Calculate the time difference between two tick counts.
 *
 * This function will convert the difference between two tick count values into
 * an actual time period. Tick count wrap around is accounted for. The return
 * value is the number of whole units rounded down.
 *
 * @param start the tick count at the start of the period
 * @param end the tick count at the end of the period
 * @param units the time units to calculate the difference in.
 *
 * @return the amount of elapsed time in whole units.
 */
uint32_t timeElapsed(uint32_t start, uint32_t end, TIMEUNIT units);

/** Determine if the specified amount of time has expired.
 *
 * This function compares the current tick count with a previously stored
 * start point and determines if the requested amount of time has expired
 * yet.
 *
 * @param reference the starting reference
 * @param duration the time period we are waiting to expire
 * @param units the units the duration is expressed in
 *
 * @return true if the specified duration has expired since the reference
 *              point.
 */
bool timeExpired(uint32_t reference, uint32_t duration, TIMEUNIT units);

/** Delay the invocation of the application loop for a specified duration
 *
 * This function will block until the specified time period has elapsed but
 * background tasks (network operations, battery monitoring, etc) will continue.
 *
 * @param duration the amount of time to delay for
 * @param units the units the duration is specified in
 */
void delay(uint32_t duration, TIMEUNIT units);

//---------------------------------------------------------------------------
// Time of Day functions
//
// These functions are used to measure longer time periods related to the
// current time of day. Each processor board features a real time clock which
// is initialised to a common network time when the network link is
// established. The accuracy of this time value depends on the network.
//---------------------------------------------------------------------------

/** Complete date time record.
 */
typedef struct _DATETIME {
  uint16_t m_year;   // 4 digit year
  uint8_t  m_month;  // Month of year (1 to 12)
  uint8_t  m_day;    // Day of month (1 to 31)
  uint8_t  m_hour;   // Hour of day (0 to 23)
  uint8_t  m_minute; // Minute of hour (0 to 59)
  uint8_t  m_second; // Second of minute (0 to 59)
  } DATETIME;

/** Get the current date and time according to the RTC
 *
 * @param pDateTime pointer to a structure to receive the date and time data.
 *
 * @return true on success, false on failure.
 */
bool getDateTime(DATETIME *pDateTime);

/** Set the current date and time in the RTC
 *
 * @param pDateTime pointer to a structure containing the new date and time.
 *
 * @return true on success, false on failure.
 */
bool setDateTime(DATETIME *pDateTime);

/** Set an alarm
 *
 * This function is used internally by the 'sleep()' function to set a wake
 * up event for the processor. Calling it from application code will have
 * no effect (the interrupt is essentially ignored).
 *
 * @param pDateTime pointer to a structure containing date and time for the alarm.
 *
 * @return true on success, false on failure.
 */
bool setAlarm(DATETIME *pDateTime);

/** Determine if the date and time are valid
 *
 * Helper function to validate the values in a DATETIME structure.
 *
 * @param pDateTime pointer to a structure containing the new date and time.
 *
 * @return true if the values are valid, false otherwise.
 */
bool isDateTimeValid(DATETIME *pDateTime);

/** Convert a date time structure to a timestamp
 *
 * A timestamp is an integer value representing the number of seconds since
 * 1/1/1970. Using a timestamp makes conversion and comparison of DATETIME
 * values easier.
 *
 * @param pDateTime pointer to a structure containing the date and time.
 *
 * @return the number of seconds since 1/1/1970 to the date and time specified
 *         in the structure. If the structure represents a time prior to the
 *         epoch or contains invalid information the return value will be 0.
 */
uint32_t toTimestamp(DATETIME *pDateTime);

/** Convert a timestamp to a date time structure
 *
 * A timestamp is an integer value representing the number of seconds since
 * 1/1/1970. Using a timestamp makes conversion and comparison of DATETIME
 * values easier.
 *
 * @param pDateTime pointer to a structure to receive the date and time.
 * @param timestamp the timestamp value to convert.
 */
void fromTimestamp(DATETIME *pDateTime, uint32_t timestamp);

//---------------------------------------------------------------------------
// System management
//---------------------------------------------------------------------------

/** Indicate why the processor woke up
 */
typedef enum {
  WAKE_UNKNOWN = 0, //!< Unknown wakeup reason
  WAKE_TIMEOUT,     //!< Sleep period expired
  WAKE_PINCHANGE,   //!< A wakeup pin changed state
  } WAKE_REASON;

/** Power down the device
 *
 * This function will completely power down the device. It is usually only
 * called in situations where continuing to operate would be dangerous or
 * possibly damage the sensor.
 */
void shutdown();

/** Put the processor into sleep mode
 *
 * This function puts the CPU into sleep mode for the specified period of
 * time to conserve power. While the processor is sleeping other background
 * tasks (network operations, battery monitoring, etc) will also be paused.
 *
 * Note that the timeout is triggered by the RTC so has a maximum resolution
 * of 1s.
 *
 * If the processor does not support sleep mode (or doesn't have an RTC to
 * trigger waking) this function will behave like the 'delay()' function using
 * SECONDS as the time unit.
 *
 * @param seconds the amount of time (in seconds) to sleep for
 */
WAKE_REASON sleep(uint32_t seconds);

/** Set the output indication sequence
 *
 * Every power adapter has an indication LED which is used to provide visual
 * feedback. This function sets an indication pattern to run on the LED. A
 * pattern consists of a sequence of 16 LED states (1 = on, 0 = off) which
 * are stepped through every 125ms giving a total of 2s for each pattern.
 *
 * If a pattern is already running it will be terminated at the current step
 * and the new pattern started instead.
 *
 * @param pattern the 16 bit pattern to start.
 * @param repeat if true the pattern will be repeated until a new pattern is
 *               set.
 */
void indicate(uint16_t pattern, bool repeat);

//--- Some standard patterns
#define PATTERN_FULL 0xffff

//---------------------------------------------------------------------------
// GPIO interface
//---------------------------------------------------------------------------

/** GPIO pins available on the board
 */
typedef enum {
  PIN0 = 0,
  PIN1,
  PIN2,
  PIN3,
  PIN4,
  // Pins used internally
  PIN_ACTION,     //!< Action button input
  PIN_LATCH,      //!< Power latch output
  PIN_INDICATOR,  //!< Indicator LED output
  PIN_BATTERY,    //!< Battery voltage analog input
  PIN_CE,         //!< Select pin for NRF24L01 module
  PIN_CSN,        //!< Transmitter enable pin for NRF24L01 module
  PIN_SCK,        //!< Pin for SPI clock
  PIN_MISO,       //!< Pin for SPI input
  PIN_MOSI,       //!< Pin for SPI output
  PINMAX
  } PIN;

/** Pin function modes
 */
typedef enum {
  DISABLED = 0,
  ANALOG,
  DIGITAL_INPUT,
  DIGITAL_OUTPUT,
  } PIN_MODE;

/** Pin flags
 */
typedef enum {
  PULLUP   = 0x01, // Enable internal pullup (DIGITAL_INPUT only)
  PULLDOWN = 0x02, // Enable internal pulldown (DIGITAL_INPUT only)
  WAKEUP   = 0x04, // Enable wake from sleep (DIGITAL_INPUT only)
  } PIN_FLAG;

/** Configure a GPIO pin
 *
 * @param pin the pin to configure
 * @param mode the requested mode for the pin
 * @param flags optional flags for the pin.
 *
 * @return true if the pin was configured as requested.
 */
bool pinConfig(PIN pin, PIN_MODE mode, uint8_t flags);

/** Read the value of a digital pin.
 *
 * To use this function the pin must be configured as DIGITAL_INPUT. If the pin
 * was configured for a different mode the result will always be false.
 *
 * @param pin the pin to read
 *
 * @return the current state of the pin.
 */
bool pinRead(PIN pin);

/** Change the state of a digital pin.
 *
 * To use this function the pin must be configured as DIGITAL_OUTPUT. If the
 * pin was configured for a different mode the function will have no effect.
 *
 * @param pin the pin to change the state of
 * @param value the value to set the pin to (true = high, false = low)
 */
void pinWrite(PIN pin, bool value);

/** Sample the value of a analog input
 *
 * To use this function the pin must be configured as ANALOG. If the pin was
 * configured for a different mode the function will always return 0.
 *
 * The value returned by this function is always scaled to a full 16 bit value
 * regardless of the resolution of the underlying ADC.
 *
 * The function allows the caller to sample and discard a number of samples
 * before reading and to take a group of samples and return the average. This
 * can improve the accuracy of the final result.
 *
 * @param pin the pin to sample the input from.
 * @param average the number of samples to average to get the final result.
 * @param skip the number of samples to skip before averaging.
 *
 * @return the sample read from the pin. This will be shifted left if needed
 *         to fully occupy a 16 bit value.
 */
uint16_t pinSample(PIN pin, int average, int skip);

//---------------------------------------------------------------------------
// SPI Operations
//
// Each board provides a dedicated SPI interface that is also used internally
// to control the NRF24L01 transceiver. These pins are dedicated to SPI and
// cannot be repurposed. It is essential that user code ensures that all SPI
// devices are deselected prior to exiting the 'loop()' function or before
// any call to 'sleep()' or 'delay()'.
//---------------------------------------------------------------------------

/** Configure the SPI interface
 *
 * This function sets the operating mode for the SPI interface for future
 * data transfer calls.
 *
 * @param polarity the polarity of the SPI clock - true = HIGH, false = LOW
 * @param phase the phase of the SPI clock - true = HIGH, false = LOW
 * @param msbFirst true if data should be sent MSB first, false if LSB first.
 */
void spiConfig(bool polarity, bool phase, bool msbFirst);

/** Write a sequence of bytes to the SPI interface
 *
 * This function assumes the target device has been selected by the caller.
 *
 * @param pData the buffer containing the data to write
 * @param count the number of bytes to write
 */
void spiWrite(const uint8_t *pData, int count);

/** Read a sequence of bytes from the SPI interface
 *
 * This function assumes the target device has been selected by the caller.
 * During the read the call will keep MOSI at 0.
 *
 * @param pData pointer to a buffer to receive the data
 * @param count the number of bytes to read.
 */
void spiRead(uint8_t *pData, int count);

/** Read and write to the SPI interface
 *
 * This function assumes the target device has been selected by the caller.
 *
 * @param pOutput a buffer containing the bytes to write to the SPI port
 * @param pInput a buffer to receive the bytes read from the SPI port
 * @param count the number of bytes to transfer. Both buffers must be at
 *        least this size.
 */
void spiTransfer(const uint8_t *pOutput, uint8_t *pInput, int count);

//---------------------------------------------------------------------------
// I2C Operations
//
// Each board allows for an I2C interface using pins PIN0 and PIN1. When I2C
// is being used these pins are no longer available for general use.
//---------------------------------------------------------------------------

/** Initialise the I2C interface
 *
 * This function sets up the I2C interface on PIN0/PIN1 and disables the use
 * of those pins for general purpose IO. Future calls to pinConfig() for
 * either pin will fail.
 */
void i2cConfig();

/** Write a bit stream to the I2C device
 *
 * This function is used to send a sequence of bits to the i2c slave device
 * identified by the address.
 *
 * @param address the address of the slave device
 * @param data the data to send, the lowest 'count' bits will be sent in
 *             MSB order.
 * @param count the number of bits to write.
 */
void i2cWriteBits(uint8_t address, uint32_t data, int count);

/** Read a bit stream from the I2C device
 *
 * This function is used to read a sequence of bits from the i2c slave
 * identified by the address.
 *
 * @param address the address of the slave device
 * @param count the number of bits to read
 *
 * @return a 32 bit value containing the bits read in the least significant
 *         bits.
 */
uint32_t i2cReadBits(uint8_t address, int count);

/** Write a sequence of byte values to the i2c slave
 *
 * @param address the address of the slave device
 * @param pData pointer to a buffer containing the data to send
 * @param count the number of bytes to transmit
 */
void i2cWriteBytes(uint8_t address, const uint8_t *pData, int count);

/** Read a sequence of bytes from the i2c slave
 *
 * @param address the address of the slave device
 * @param pData a pointer to a buffer to receive the data read
 * @param count the maximum number of bytes to read
 *
 * @return the number of bytes read from the slave.
 */
int i2cReadBytes(uint8_t address, uint8_t *pData, int count);

//---------------------------------------------------------------------------
// Serial port operations
//
// Each SensNode supports a single UART that is connected to the debug port.
// Although the intention is to use the port purely for debugging purposes
// it is also available for communication with serial peripherals.
//---------------------------------------------------------------------------

/** Supported baud rates
 */
typedef enum {
  B9600,
  B19200,
  B38400,
  B57600,
  B115200
  } BAUDRATE;

/** Initialise the serial port
 *
 * The serial port is always operated in 8 bit mode with a single stop bit
 * (8N1). The core initialisation will set the initial baudrate to 57600 but
 * user code may reconfigure the port to a different baud rate if required.
 *
 * @param rate the requested baud rate
 */
void serialInit(BAUDRATE rate);

/** Write a single character to the serial port
 *
 * @param ch the character to write
 */
void serialWrite(uint8_t ch);

/** Print a sequence of characters to the serial port.
 *
 * This function may be used to print a NUL terminated string (if the length
 * parameter < 0) or a fixed sequence of bytes (if length >= 0).
 *
 * The function is blocking and will not return until all characters have been
 * transmitted.
 *
 * @param cszString pointer to a buffer containing the data to be transmitted.
 * @param length the number of bytes to send. If length < 0 the buffer is treated
 *               as a NUL terminated string and sending will terminate with the
 *               first 0 byte.
 *
 * @return the number of bytes sent.
 */
int serialPrint(const char *cszString, int length);

/** Print a formatted string to the serial port.
 *
 * This function utilises the @see vprintf function to transmit a formatted
 * string to the serial port.
 *
 * The function is blocking and will not return until all characters have been
 * transmitted.
 *
 * @param cszString the format string to use to generate the output.
 *
 * @return the number of bytes sent.
 */
int serialPrintF(const char *cszString, ...);

/** Get the number of bytes available in the input buffer.
 *
 * This function determines how much data is available to read from the serial
 * port.
 *
 * @return the number of bytes available to read immediately.
 */
int serialAvailable();

/** Read a single byte from the serial port
 *
 * This function is non-blocking, if no data is available to read the function
 * will return a value < 0.
 *
 * @return the value of the byte read or a value < 0 if no data is available.
 */
int serialRead();

// Simple macro to output debug information on the serial port.
#ifdef DEBUG
#  define DBG(msg, ...) \
     serialPrintF("DEBUG: "), serialPrintF(msg, ## __VA_ARGS__), serialWrite('\n')
#else
#  define DBG(msg, ...)
#endif

//---------------------------------------------------------------------------
// Helper and utility functions
//
// This is a set of common functions that are useful for sensor based
// applications.
//---------------------------------------------------------------------------

/** Function prototype for writing a single byte
 *
 * This function prototype is used by the @see vprintf function to output
 * formatted data.
 *
 * @param ch the character to write
 * @param pData pointer to a user data block.
 *
 * @return true if the data was written.
 */
typedef bool (*FN_PUTC)(char ch, void *pData);

// SWG: SiLabs library does provide sprintf/vprintf implementations. Don't use ours for now
#if defined(PRINTF)

/** Generate a formatted string
 *
 * This function is used to generate strings from a format. This implementation
 * uses a user provided function pointer to output the data as it is generated.
 *
 * The function supports a subset of the 'printf' string formatting syntax.
 * Allowable insertion types are:
 *
 *  %% - Display a % character. There should be no entry in the variable
 *       argument list for this entry.
 *  %u - Display an unsigned integer in decimal. The matching argument may
 *       be any 16 bit value.
 *  %U - Display an unsigned integer in decimal. The matching argument may
 *       be any 32 bit value.
 *  %x - Display an unsigned integer in hexadecimal. The matching argument may
 *       be any 16 bit value.
 *  %X - Display an unsigned integer in hexadecimal. The matching argument may
 *       be any 32 bit value.
 *  %c - Display a single ASCII character. The matching argument may be any 8
 *       bit value.
 *  %s - Display a NUL terminated string from RAM. The matching argument must
 *       be a pointer to a RAM location.
 *
 * @param pfnPutC pointer the character output function
 * @param pData pointer to a user provided data block. This is passed to the
 *              character output function with each character.
 * @param cszFormat pointer to a nul terminated format string.
 * @param args the variadic argument list.
 *
 * @return the number of characters generated.
 */
int vprintf(FN_PUTC pfnPutC, void *pData, const char *cszFormat, va_list args);

/** Generate a formatted string
 *
 * This function uses the @see xprintf function to generate a formatted string
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
int sprintf(char *szBuffer, int length, const char *cszString, ...);

#endif // defined(PRINTF)

/** Initialise the CRC calculation
 *
 * Initialises the CRC value prior to processing data.
 *
 * @return the initial CRC value.
 */
uint16_t crcInit();

/** Update the CRC value with an additional data byte.
 *
 * @param crc the current CRC value
 * @param data the data byte to add to the calculation
 *
 * @return the updated CRC value.
 */
uint16_t crcByte(uint16_t crc, uint8_t data);

/** Add a sequence of bytes from a buffer.
 *
 * @param crc the current CRC value
 * @param pData pointer to the memory buffer
 * @param length the number of bytes to process.
 *
 * @return the updated CRC value.
 */
uint16_t crcData(uint16_t crc, const uint8_t *pData, int length);

/** Shift data out using clocked transfer
 *
 * @param polarity the polarity of the SPI clock - true = HIGH, false = LOW
 * @param phase the phase of the SPI clock - true = HIGH, false = LOW
 * @param msbFirst true if data should be sent MSB first, false if LSB first.
 * @param clock the pin to use for the clock
 * @param output the pin to use for the output data
 * @param value the value to send
 * @param bits the number of bits to send
 */
void shiftOut(bool polarity, bool phase, bool msbFirst, PIN clock, PIN output, uint32_t value, int bits);

/** Shift data in using clocked transfer
 *
 * @param polarity the polarity of the SPI clock - true = HIGH, false = LOW
 * @param phase the phase of the SPI clock - true = HIGH, false = LOW
 * @param msbFirst true if data should be sent MSB first, false if LSB first.
 * @param clock the pin to use for the clock
 * @param input the pin to use for the input data
 * @param bits the number of bits to read
 *
 * @return the data read
 */
uint32_t shiftIn(bool polarity, bool phase, bool msbFirst, PIN clock, PIN input, int bits);

/** Exchange data using a clocked transfer
 *
 * @param polarity the polarity of the SPI clock - true = HIGH, false = LOW
 * @param phase the phase of the SPI clock - true = HIGH, false = LOW
 * @param msbFirst true if data should be sent MSB first, false if LSB first.
 * @param clock the pin to use for the clock
 * @param input the pin to use for the input data
 * @param output the pin to use for the output data
 * @param value the value to send
 * @param bits the number of bits to transfer
 *
 * @return the data read
 */
uint32_t shiftInOut(bool polarity, bool phase, bool msbFirst, PIN clock, PIN input, PIN output, uint32_t data, int bits);

//---------------------------------------------------------------------------
// Application interface
//---------------------------------------------------------------------------

/** User application initialisation
 *
 * The library will call this function once at startup to allow the user
 * application to do any initialisation it needs. At the time this function
 * is called all IO pins will have been set to their default states and the
 * network subsystem initialised (if not yet connected).
 */
void setup();

/** User application loop
 *
 * The library repeatedly calls this function in an endless loop. The function
 * will generally be implemented as a state machine and take care to minimise
 * the amount of time spent in the function itself.
 */
void loop();

#ifdef __cplusplus
}
#endif

#endif /* __SENSNODE_H */

