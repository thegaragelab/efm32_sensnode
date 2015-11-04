/*--------------------------------------------------------------------------*
* SensNode - Main loop implementation
*---------------------------------------------------------------------------*
* 2015-Nov-04 ShaneG
*
* Implements the 'main()' function - this does the necessary internal setup
* and then allows the user application to do it's setup. It then enters
* the main processing loop.
*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "sensnode.h"
#include "hardware.h"

static void mainLoop(bool callApp) {
  // TODO: Internal loop (network processing, action button events, indicator updates)
  if(callApp)
    loop();
  }

/** Delay the invocation of the application loop for a specified duration
 *
 * This function will block until the specified time period has elapsed but
 * background tasks (network operations, battery monitoring, etc) will continue.
 *
 * @param duration the amount of time to delay for
 * @param units the units the duration is specified in
 */
void delay(uint32_t duration, TIMEUNIT units) {
  uint32_t start = getTicks();
  while(!timeElapsed(start, duration, units))
    mainLoop(false);
  }

/** Program entry point
 */
int main(void) {
  // EFM32 specific setup
  CHIP_Init(); // Chip errata configuration
  // Set 21MHz clock
  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO  );
  CMU_HFRCOBandSet( cmuHFRCOBand_21MHz );
  // HAL setup
  initTICK();
  initGPIO();
  initSPI();
  // Configure our standard pins
  pinConfig(PIN_ACTION, DIGITAL_INPUT, WAKEUP); // Action button input
  pinConfig(PIN_LATCH, DIGITAL_OUTPUT, 1);      // Power latch output
  pinConfig(PIN_INDICATOR, DIGITAL_OUTPUT, 0);  // Indicator LED output
  pinConfig(PIN_BATTERY, ANALOG, 0);            // Battery voltage analog input
  // TODO: SPI configuration
  // TODO: NRF24L01 configuration
  // TODO: Network configuration
  // Application configuration
  setup();
  // Main loop
  while(true) {
    mainLoop(true);
    }
  }
