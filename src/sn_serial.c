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


/** Initialise the serial hardware
 */
void initSERIAL() {
  /* Initialize USART */
    USART_TypeDef *usart = USART1;
    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_USART1, true);

    /* Use default location 0: TX - Pin C0, RX - Pin C1 */
    /* To avoid false start, configure output as high */
    GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1);
    /* Define input, no filtering */
    GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);

    /* Enable pins at default location */
    usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;

    /* Configure USART for basic async operation */
    init.enable = usartDisable;
    USART_InitAsync(usart, &init);

    /* Clear previous RX interrupts */
    USART_IntClear((USART_TypeDef *) cmuClock_USART1, USART_IF_RXDATAV);
    USART_IntEnable((USART_TypeDef *) cmuClock_USART1, USART_IF_RXDATAV);
    NVIC_ClearPendingIRQ( USART1_RX_IRQn);
    NVIC_EnableIRQ(USART1_RX_IRQn);

    /* Finally enable it */
    USART_Enable(usart, usartEnable);
}

/** USART input interrupt handler
 *
 */
void USART1_RX_IRQHandler(void) {
  USART_TypeDef *uart = USART1;
  uint8_t       rxdata;
  /* Checking that RX-flag is set*/
  if (uart->STATUS & USART_STATUS_RXDATAV) {
    // TODO: Store data in buffer
    rxdata = uart->RXDATA;
    }
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
  }

/** Write a single character to the serial port
 *
 * @param ch the character to write
 */
void serialWrite(uint8_t ch) {
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
  return -1;
  }

/** Get the number of bytes available in the input buffer.
 *
 * This function determines how much data is available to read from the serial
 * port.
 *
 * @return the number of bytes available to read immediately.
 */
int serialAvailable() {
  return 0;
  }

/** Read a single byte from the serial port
 *
 * This function is non-blocking, if no data is available to read the function
 * will return a value < 0.
 *
 * @return the value of the byte read or a value < 0 if no data is available.
 */
int serialRead() {
  return 0;
  }

