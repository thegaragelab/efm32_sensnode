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
#define FLAG(n) (1<<(n))

/** Map SensNode pin functions to CPU pins
 */
static const PININFO g_pinInfo[] = {
  { gpioPortE, 13, FLAG(DIGITAL_INPUT)|FLAG(DIGITAL_OUTPUT) },               // PIN0
  { gpioPortE, 12, FLAG(DIGITAL_INPUT)|FLAG(DIGITAL_OUTPUT) },               // PIN1
  { gpioPortD, 4,  FLAG(DIGITAL_INPUT)|FLAG(DIGITAL_OUTPUT)|FLAG(ANALOG) },  // PIN2
  { gpioPortD, 5,  FLAG(DIGITAL_INPUT)|FLAG(DIGITAL_OUTPUT)|FLAG(ANALOG) },  // PIN3
  { gpioPortD, 6,  FLAG(DIGITAL_INPUT)|FLAG(DIGITAL_OUTPUT)|FLAG(ANALOG) },  // PIN4
  { gpioPortC, 8,  FLAG(DIGITAL_INPUT) },                                    // PIN_ACTION
  { 0, 0, 0 },                                                               // PIN_LATCH (unused)
  { gpioPortC, 10, FLAG(DIGITAL_OUTPUT) },                                   // PIN_INDICATOR
  { 0, 0, 0 },                                                               // PIN_BATTERY (unused)
  { gpioPortA, 10, FLAG(DIGITAL_OUTPUT) },                                   // PIN_CE
  { gpioPortA, 9,  FLAG(DIGITAL_OUTPUT) },                                   // PIN_CSN
  { gpioPortA, 2,  FLAG(DIGITAL_OUTPUT) },                                   // PIN_SCK
  { gpioPortA, 0,  FLAG(DIGITAL_INPUT) },                                    // PIN_MISO
  { gpioPortA, 1,  DIGITAL_OUTPUT },                                         // PIN_MOSI
  };

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
	return false;
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
	return false;
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
