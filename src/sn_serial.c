/*--------------------------------------------------------------------------*
* SensNode - Serial port implementation
*---------------------------------------------------------------------------*
* 2015-Nov-08 ShaneG
*
* Provides the interface to the serial port.
*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "sensnode.h"

// Buffer size
#define INPUT_BUFFER_SIZE 16

// Ring buffer
static uint8_t g_buffer[INPUT_BUFFER_SIZE];
static volatile uint8_t g_read = 0;
static volatile uint8_t g_write = 0;

// Baud rates
static uint32_t BAUDRATES[] = {
  9600,   // B9600,
  19200,  // B19200,
  38400,  // B38400,
  57600,  // B57600,
  115200, // B115200
  };

#define DEFAULT_BAUD B57600

//---------------------------------------------------------------------------
// Internal helpers
//---------------------------------------------------------------------------

/** Initialise the serial hardware
 */
void initSERIAL() {
  USART_TypeDef *usart = USART1;
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_USART1, true);
  // Use default location 0: TX - Pin C0, RX - Pin C1
  // To avoid false start, configure output as high
  GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);
  // Enable pins at default location
  usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;
  // Start with the default baud rate
  serialConfig(DEFAULT_BAUD);
  }

/** USART input interrupt handler
 *
 */
void USART1_RX_IRQHandler(void) {
  USART_TypeDef *uart = USART1;
  /* Checking that RX-flag is set*/
  if (uart->STATUS & USART_STATUS_RXDATAV) {
    // Now put it in the buffer and advance (if we have room)
    g_buffer[g_write] = uart->RXDATA;
    uint8_t write = (g_write + 1) % INPUT_BUFFER_SIZE;
    if(write!=g_read)
      g_write = write;
    }
  USART_IntClear(uart, USART_IF_RXDATAV);
  }

/** Write a single character to the serial port
 *
 * @param ch the character to write.
 * @param pData user defined data for the function
 *
 * @return true if the write was successful
 */
static bool serial_putc(char ch, void *pData) {
  serialWrite(ch);
  return true;
  }

//---------------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------------

/** Configure the serial port
 *
 * The serial port is always operated in 8 bit mode with a single stop bit
 * (8N1). The core initialisation will set the initial baudrate to 57600 but
 * user code may reconfigure the port to a different baud rate if required.
 *
 * @param rate the requested baud rate
 */
void serialConfig(BAUDRATE rate) {
  // Disable it and disable interrupts
  USART_Enable(USART1, usartDisable);
  // Set up the configuration
  USART_InitAsync_TypeDef init =
   {
     .enable = usartDisable,      // wait to enable transmitter and receiver
     .refFreq = 0,                // setting refFreq to 0 will invoke the CMU_ClockFreqGet() function and measure the HFPER clock
     .baudrate = BAUDRATES[rate], // desired baud rate
     .oversampling = usartOVS16,  // set oversampling to x16
     .databits = usartDatabits8,  // 8 data bits
     .parity = usartNoParity,     // no parity bits
     .stopbits = usartStopbits1,  // 1 stop bit
     .mvdis = false,              // use majority voting
     .prsRxEnable = false,        // not using PRS input
     .prsRxCh = usartPrsRxCh0,    // doesn't matter what channel we select
   };
  USART_InitAsync(USART1, &init);      // apply configuration to USART1
  USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | _USART_ROUTE_LOCATION_LOC0; // clear buffers, enable transmitter and receiver pins
  USART_IntClear(USART1, _USART_IF_MASK);  // clear all USART interrupt flags
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);    // clear pending RX interrupt flag in NVIC
  NVIC_ClearPendingIRQ(USART1_TX_IRQn);    // clear pending TX interrupt flag in NVIC
  USART_IntEnable(USART1, USART_IF_RXDATAV); // Enable RX interrupt
  NVIC_EnableIRQ(USART1_RX_IRQn);
  USART_Enable(USART1, usartEnable);       // enable transmitter and receiver
  }

/** Write a single character to the serial port
 *
 * @param ch the character to write
 */
void serialWrite(uint8_t ch) {
  USART_Tx(USART1, ch);
  }

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
int serialPrint(const char *cszString, int length) {
  if(cszString==NULL)
    cszString = "(null)";
  if(length<0) {
    length = 0;
    while(*cszString) {
      serialWrite(*cszString);
      cszString++;
      length++;
      }
    }
  else {
    for(int i=0; i<length; i++)
      serialWrite(cszString[i]);
    }
  return length;
  }

/** Print a formatted string to the serial port.
 *
 * This function utilises the @see vformat function to transmit a formatted
 * string to the serial port.
 *
 * The function is blocking and will not return until all characters have been
 * transmitted.
 *
 * @param cszString the format string to use to generate the output.
 *
 * @return the number of bytes sent.
 */
int serialFormat(const char *cszString, ...) {
  va_list args;
  va_start(args, cszString);
  int result = vformat((FN_PUTC)&serial_putc, NULL, cszString, args);
  va_end(args);
  return result;
  }

/** Determines if data is available to be read
 *
 * @return the number of bytes available to read immediately.
 */
bool serialAvailable() {
  return g_write!=g_read;
  }

/** Read a single byte from the serial port
 *
 * If no data is available this function will block until the next character
 * is received. Use 'serialAvailable()' to determine if data can be read
 * without blocking.
 *
 * @return the value of the byte read
 */
int serialRead() {
  // Wait for data to become available
  while(g_write==g_read);
  // Take the oldest value from the buffer and advance
  uint8_t ch = g_buffer[g_read];
  g_read = (g_read + 1) % INPUT_BUFFER_SIZE;
  return ch;
  }
