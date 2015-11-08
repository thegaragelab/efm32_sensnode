/*--------------------------------------------------------------------------*
* SensNode Hardware Abstraction Layer
*---------------------------------------------------------------------------*
* 03-Nov-2015 ShaneG
*
* These functions and types are used internally by the library to provide
* and interface between the SensNode abstraction and the underlying CPU.
*---------------------------------------------------------------------------*/
#ifndef __HARDWARE_H
#define __HARDWARE_H

/** Initialise the GPIO subsystem
 */
void initGPIO();

/** Initialise the SPI subsystem
 */
void initSPI();

/** Initialise the timer subsystem
 */
void initTICK();

/** Initialise the serial hardware
 */
void initSERIAL();

#endif /* __HARDWARE_H */
