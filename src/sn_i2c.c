/*--------------------------------------------------------------------------*
* SensNode - I2C Implementation
*---------------------------------------------------------------------------*
* 2015-Nov-15 ShaneG
*
* Implements the I2C interface on the Zero Gecko.
*--------------------------------------------------------------------------*/
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "sensnode.h"

// Configuration flag
static bool g_i2cActive = false;

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
  // Have we already done the configuration ?
  if(g_i2cActive)
    return true;
  // Are the pins we want available?
  if(!(pinAvailable(PIN0)&&pinAvailable(PIN1)))
    return false;
  // Mark them as being used
  pinMarkUsed(PIN0);
  pinMarkUsed(PIN1);
  // TODO: Do the actual peripheral initialisation
  CMU_ClockEnable(cmuClock_I2C0, true);                        // Enable the peripheral
  GPIO_PinModeSet(gpioPortE, 12, gpioModeWiredAnd, 0); // configure SDA pin as open drain output
  GPIO_PinModeSet(gpioPortE, 13, gpioModeWiredAnd, 0); // configure SCL pin as open drain output
  I2C_Init_TypeDef i2c_init = I2C_INIT_DEFAULT;
  I2C_Init(I2C0, &i2c_init);    // apply configuration to I2C0
  I2C0->CTRL |= (1 << 2);       // enable AUTO-ACK feature
  I2C0->ROUTE = (6 << 8) | 0x3; // use location #6 (PE11, PE12), enable SDA and SCL
  g_i2cActive = true;
  // Done
  return true;
  }

/** Write a sequence of byte values to the i2c slave
 *
 * @param address the address of the slave device
 * @param pData pointer to a buffer containing the data to send
 * @param count the number of bytes to transmit
 *
 * @return number of bytes sent
 */
int i2cSendTo(uint8_t address, uint8_t *pData, int count) {
  // Do some basic parameter checking
  if((!g_i2cActive)||(pData==NULL))
    return -1;
  // Set up the message block and initialise the write
  I2C_TransferSeq_TypeDef message = {
    .addr = (address << 1),  // set slave address
    .flags = I2C_FLAG_WRITE, // indicate basic write
    .buf[0].data = pData,    // point to buffer
    .buf[0].len = count,     // specify number of bytes
    };
  I2C_TransferReturn_TypeDef result = I2C_TransferInit(I2C0, &message);
  while(result == i2cTransferInProgress) // continue until all data has been sent
    result = I2C_Transfer(I2C0);
  // Done
  return (result==i2cTransferDone)?count:-1;
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
  // Do some basic parameter checking
  if((!g_i2cActive)||(pData==NULL))
    return -1;
  // Set up the message block and initialise the read
  I2C_TransferSeq_TypeDef message = {
    .addr = (address << 1),  // set sensor slave address
    .flags = I2C_FLAG_READ, // indicate basic read
    .buf[0].data = pData,    // point to buffer
    .buf[0].len = count,     // specify number of bytes
    };
  I2C_TransferReturn_TypeDef result = I2C_TransferInit(I2C0, &message);
  while(result == i2cTransferInProgress) // continue until all data has been read
   result = I2C_Transfer(I2C0);
  // Done
  return (result==i2cTransferDone)?count:-1;
  }
