/*--------------------------------------------------------------------------*
* SensNode - GPIO implementation
*---------------------------------------------------------------------------*
* 2015-Nov-03 ShaneG
*
* Provides the general GPIO configuration and access functions using the
* CMSIS library for EFM32 processors.
*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "sensnode.h"

typedef struct _PININFO {
  GPIO_Port_TypeDef m_port;
  unsigned int m_pin;
  unsigned int m_modes;
  } PININFO;

// Convert a bit number into a flag bit
#define SETBIT(n) (1<<(n))

/** Map SensNode pin functions to CPU pins
 */
static const PININFO g_pinInfo[] = {
  { gpioPortE, 13, SETBIT(DIGITAL_INPUT)|SETBIT(DIGITAL_OUTPUT) },                // PIN0
  { gpioPortE, 12, SETBIT(DIGITAL_INPUT)|SETBIT(DIGITAL_OUTPUT) },                // PIN1
  { gpioPortD, 4,  SETBIT(DIGITAL_INPUT)|SETBIT(DIGITAL_OUTPUT)|SETBIT(ANALOG) }, // PIN2
  { gpioPortD, 5,  SETBIT(DIGITAL_INPUT)|SETBIT(DIGITAL_OUTPUT)|SETBIT(ANALOG) }, // PIN3
  { gpioPortD, 6,  SETBIT(DIGITAL_INPUT)|SETBIT(DIGITAL_OUTPUT)|SETBIT(ANALOG) }, // PIN4
  { gpioPortC, 8,  SETBIT(DIGITAL_INPUT) },                                       // PIN_ACTION
  { 0, 0, 0 },                                                                    // PIN_LATCH (unused)
  { gpioPortC, 10, SETBIT(DIGITAL_OUTPUT) },                                      // PIN_INDICATOR
  { 0, 0, 0 },                                                                    // PIN_BATTERY (unused)
  { gpioPortA, 10, SETBIT(DIGITAL_OUTPUT) },                                      // PIN_CE
  { gpioPortA, 9,  SETBIT(DIGITAL_OUTPUT) },                                      // PIN_CSN
  { gpioPortA, 2,  SETBIT(DIGITAL_OUTPUT) },                                      // PIN_SCK
  { gpioPortA, 0,  SETBIT(DIGITAL_INPUT) },                                       // PIN_MISO
  { gpioPortA, 1,  SETBIT(DIGITAL_OUTPUT) },                                              // PIN_MOSI
  };

/** Remember which pins have been configured */
static uint16_t g_pinsUsed = 0;

/** Mask to clean all even numbered IO pins
 */
#define GPIO_PIN_EVEN \
  SETBIT(0)  | \
  SETBIT(2)  | \
  SETBIT(4)  | \
  SETBIT(6)  | \
  SETBIT(8)  | \
  SETBIT(10) | \
  SETBIT(12)

/** Mask to clean all even numbered IO pins
 */
#define GPIO_PIN_ODD \
  SETBIT(1) | \
  SETBIT(5) | \
  SETBIT(9) | \
  SETBIT(13)

/** Initialise the GPIO subsystem
 */
void initGPIO() {
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);
  // Set all pins to disabled state
  for (PIN pin = PIN0; pin < PINMAX; pin++) {
    if(g_pinInfo[pin].m_modes)
      GPIO_PinModeSet(g_pinInfo[pin].m_port, g_pinInfo[pin].m_pin, gpioModeDisabled, 0);
    }
  }

/** GPIO interrupt handler (even numbered pins)
 *
 * The interrupt handler doesn't actually do anything, enabling interrupts
 * allows pin change events to trigger wake from sleep.
 */
void GPIO_EVEN_IRQHandler(void) {
  /* Acknowledge interrupt */
  GPIO_IntClear(GPIO_PIN_EVEN);
  }

/** GPIO interrupt handler (even numbered pins)
 *
 * The interrupt handler doesn't actually do anything, enabling interrupts
 * allows pin change events to trigger wake from sleep.
 */
void GPIO_ODD_IRQHandler(void) {
  /* Acknowledge interrupt */
  GPIO_IntClear(GPIO_PIN_ODD);
  }

//---------------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------------

/** Configure a GPIO pin
 *
 * @param pin the pin to configure
 * @param mode the requested mode for the pin
 * @param flags optional flags for the pin.
 *
 * @return true if the pin was configured as requested.
 */
bool pinConfig(PIN pin, PIN_MODE mode, uint8_t flags) {
  // Is it a valid pin?
  if((pin<0)||(pin>=PINMAX))
    return false;
  // Has the pin already been configured?
  if(g_pinsUsed&SETBIT(pin))
    return false;
  // Does the pin support the requested mode?
  if(!(g_pinInfo[pin].m_modes&SETBIT(mode)))
    return false;
  // Do the configuration
  switch(mode) {
    case ANALOG:
      // TODO: Not yet supported
      break;
    case DIGITAL_INPUT:
      // Digital input with optional pull up/pull down
      GPIO_PinModeSet(
        g_pinInfo[pin].m_port,
        g_pinInfo[pin].m_pin,
        ((flags&PULLUP)||(flags&PULLDOWN))?gpioModeInputPull:gpioModeInput,
        (flags&PULLUP)?1:0
        );
      // If wake on change was requested enable interrupts for the pin
      if(flags&WAKEUP) {
        GPIO_IntConfig(g_pinInfo[pin].m_port, g_pinInfo[pin].m_pin, true, true, true);
        NVIC_ClearPendingIRQ((g_pinInfo[pin].m_pin&1)?GPIO_ODD_IRQn:GPIO_EVEN_IRQn);
        NVIC_EnableIRQ((g_pinInfo[pin].m_pin&1)?GPIO_ODD_IRQn:GPIO_EVEN_IRQn);
        }
      // Mark the pin as configured
      g_pinsUsed |= SETBIT(pin);
      break;
    case DIGITAL_OUTPUT:
      // Digital output (flags value determines initial output state)
      GPIO_PinModeSet(
        g_pinInfo[pin].m_port,
        g_pinInfo[pin].m_pin,
        gpioModePushPull,
        flags
        );
      // Mark the pin as configured
      g_pinsUsed |= SETBIT(pin);
      break;
    default:
      return false;
    }
  // Pin configured
  return true;
  }

/** Read the value of a digital pin.
 *
 * To use this function the pin must be configured as DIGITAL_INPUT. If the pin
 * was configured for a different mode the result will always be false.
 *
 * @param pin the pin to read
 *
 * @return the current state of the pin.
 */
bool pinRead(PIN pin) {
  // Is it a valid pin?
  if((pin<0)||(pin>=PINMAX))
    return false;
  // Has the pin been configured?
  if(!(g_pinsUsed&SETBIT(pin)))
    return false;
  // Read the pin input state
  return GPIO_PinInGet(g_pinInfo[pin].m_port, g_pinInfo[pin].m_pin) == 1;
  }

/** Change the state of a digital pin.
 *
 * To use this function the pin must be configured as DIGITAL_OUTPUT. If the
 * pin was configured for a different mode the function will have no effect.
 *
 * @param pin the pin to change the state of
 * @param value the value to set the pin to (true = high, false = low)
 */
void pinWrite(PIN pin, bool value) {
  // Is it a valid pin?
  if((pin<0)||(pin>=PINMAX))
    return;
  // Has the pin been configured?
  if(!(g_pinsUsed&SETBIT(pin)))
    return;
  // Set the output state of the pin
  if(value)
    GPIO_PinOutSet(g_pinInfo[pin].m_port, g_pinInfo[pin].m_pin);
  else
    GPIO_PinOutClear(g_pinInfo[pin].m_port, g_pinInfo[pin].m_pin);
  }

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
uint16_t pinSample(PIN pin, int average, int skip) {
	return 0;
}
