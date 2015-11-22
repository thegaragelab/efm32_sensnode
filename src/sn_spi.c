/*--------------------------------------------------------------------------*
* SensNode - SPI Implementation
*---------------------------------------------------------------------------*
* 2015-Nov-04 ShaneG
*
* On the EFM32 the SPI interface is bit banged (the USART is used for serial
* comms).
*--------------------------------------------------------------------------*/
#include "sensnode.h"

// SPI configuration
static bool g_spiPolarity = false;
static bool g_spiPhase = false;
static bool g_spiMSBFirst = false;

/** Initialise the SPI subsystem
 */
void initSPI() {
  // Set up the pins
  pinConfig(PIN_MISO, DIGITAL_INPUT, 0);
  pinConfig(PIN_MOSI, DIGITAL_OUTPUT, 0);
  pinConfig(PIN_SCK, DIGITAL_OUTPUT, 0);
  }

//---------------------------------------------------------------------------
// Helper functions
//---------------------------------------------------------------------------

/** Transfer a byte over SPI MSB first
 *
 * Uses the currently configured phase and polarity.
 *
 * @param data the data to transfer out
 *
 * @return the data received on the SPI bus.
 */
static uint8_t clockInOutMSB(uint8_t data) {
  uint8_t mask = 0x80;
  uint8_t input = 0;
  while(mask) {
    pinWrite(PIN_MOSI, data & mask);
    // Change to active state
    if(!g_spiPhase)
      input |= (pinRead(PIN_MISO)?mask:0);
    pinWrite(PIN_SCK, g_spiPolarity?0:1);
    // Change to idle state
    pinWrite(PIN_SCK, g_spiPolarity?1:0);
    if(g_spiPhase)
      input |= (pinRead(PIN_MISO)?mask:0);
    // Move to next bit
    mask = mask >> 1;
    }
  return input;
  }

/** Transfer a byte over SPI LSB first
 *
 * Uses the currently configured phase and polarity.
 *
 * @param data the data to transfer out
 *
 * @return the data received on the SPI bus.
 */
static uint8_t clockInOutLSB(uint8_t data) {
  uint8_t mask = 0x01;
  uint8_t input = 0;
  while(mask) {
    if(!g_spiPhase)
      input |= (pinRead(PIN_MISO)?mask:0);
    // Change to active state
    pinWrite(PIN_SCK, g_spiPolarity?0:1);
    if(!g_spiPhase) {
      input |= (pinRead(PIN_MISO)?mask:0);
      pinWrite(PIN_MOSI, data & mask);
      }
    // Change to idle state
    pinWrite(PIN_SCK, g_spiPolarity?1:0);
    if(g_spiPhase)
      input |= (pinRead(PIN_MISO)?mask:0);
    // Move to next bit
    mask = mask << 1;
    }
  return input;
  }

//---------------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------------

/** Configure the SPI interface
 *
 * This function sets the operating mode for the SPI interface for future
 * data transfer calls.
 *
 * @param polarity the polarity of the SPI clock - true = HIGH, false = LOW
 * @param phase the phase of the SPI clock - true = HIGH, false = LOW
 * @param msbFirst true if data should be sent MSB first, false if LSB first.
 */
void spiConfig(bool polarity, bool phase, bool msbFirst) {
  g_spiPolarity = polarity;
  g_spiPhase = phase;
  g_spiMSBFirst = msbFirst;
  // Set the clock to the idle state for the polarity
  pinWrite(PIN_SCK, g_spiPolarity?1:0);
  }

/** Write a sequence of bytes to the SPI interface
 *
 * This function assumes the target device has been selected by the caller.
 *
 * @param pData the buffer containing the data to write
 * @param count the number of bytes to write
 */
void spiWrite(const uint8_t *pData, int count) {
  for(int i=0; i<count; i++) {
    if(g_spiMSBFirst)
      clockInOutMSB(pData[i]);
    else
      clockInOutLSB(pData[i]);
    }
  }

/** Read a sequence of bytes from the SPI interface
 *
 * This function assumes the target device has been selected by the caller.
 * During the read the call will keep MOSI at 0.
 *
 * @param pData pointer to a buffer to receive the data
 * @param count the number of bytes to read.
 */
void spiRead(uint8_t *pData, int count) {
  for(int i=0; i<count; i++) {
    if(g_spiMSBFirst)
      pData[i] = clockInOutMSB(0);
    else
      pData[i] = clockInOutLSB(0);
    }
  }

/** Read and write to the SPI interface
 *
 * This function assumes the target device has been selected by the caller.
 *
 * @param pOutput a buffer containing the bytes to write to the SPI port
 * @param pInput a buffer to receive the bytes read from the SPI port
 * @param count the number of bytes to transfer. Both buffers must be at
 *        least this size.
 */
void spiTransfer(const uint8_t *pOutput, uint8_t *pInput, int count) {
  for(int i=0; i<count; i++) {
    if(g_spiMSBFirst)
      pInput[i] = clockInOutMSB(pOutput[i]);
    else
      pInput[i] = clockInOutLSB(pOutput[i]);
    }
  }
