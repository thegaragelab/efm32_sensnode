/**************************************************************************//**
 * @file main.c
 * @brief Clock example for EFM32ZG-STK3200
 * @version 3.20.5
 *
 * This example shows how to optimize your code in order to drive
 * a graphical display in an energy friendly way.
 *
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "sensnode.h"

/** User application initialisation
 *
 * The library will call this function once at startup to allow the user
 * application to do any initialisation it needs. At the time this function
 * is called all IO pins will have been set to their default states and the
 * network subsystem initialised (if not yet connected).
 */
void setup() {

  }

/** User application loop
 *
 * The library repeatedly calls this function in an endless loop. The function
 * will generally be implemented as a state machine and take care to minimise
 * the amount of time spent in the function itself.
 */
void loop() {
  // EMU_EnterEM2(false);
  // Just make indicator state follow the action button
  pinWrite(PIN_INDICATOR, pinRead(PIN_ACTION));
  }
