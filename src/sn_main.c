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

/** Program entry point
 */
int main(void) {
  // EFM32 specific setup
  CHIP_Init(); // Chip errata configuration
  // Set 21MHz clock
  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO  );
  CMU_HFRCOBandSet( cmuHFRCOBand_21MHz );
  // HAL setup
  initGPIO();
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
    // TODO: Internal loop (network processing, action button events, indicator updates)
    loop();
    }
  }
