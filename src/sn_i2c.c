/*--------------------------------------------------------------------------*
* SensNode - I2C Implementation
*---------------------------------------------------------------------------*
* 2015-Nov-15 ShaneG
*
* Implements the I2C interface on the Zero Gecko.
*--------------------------------------------------------------------------*/
#include "sensnode.h"

/** Initialise the I2C interface
 *
 * This function sets up the I2C interface on PIN0/PIN1 and disables the use
 * of those pins for general purpose IO. Future calls to pinConfig() for
 * either pin will fail.
 *
 * @return true if the configuration succeeded, false if I2C is not available
 *         or PIN0/PIN1 have already been configured.
 */
bool i2cConfig() {
  return false;
  }

/** Write a sequence of byte values to the i2c slave
 *
 * @param address the address of the slave device
 * @param pData pointer to a buffer containing the data to send
 * @param count the number of bytes to transmit
 */
void i2cSendTo(uint8_t address, const uint8_t *pData, int count) {
  return -1;
  }

/** Read a sequence of bytes from the i2c slave
 *
 * @param address the address of the slave device
 * @param pData a pointer to a buffer to receive the data read
 * @param count the maximum number of bytes to read
 *
 * @return the number of bytes read from the slave.
 */
int i2cReadFrom(uint8_t address, uint8_t *pData, int count) {
  return -1;
  }
