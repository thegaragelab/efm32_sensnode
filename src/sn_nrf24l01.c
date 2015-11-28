/*--------------------------------------------------------------------------*
* SensNode - NRF24L01+ Driver
*---------------------------------------------------------------------------*
* 21-Nov-2015 ShaneG
*
* Simple driver for NRF24L01+ wireless modules.
---------------------------------------------------------------------------*/
#include "sensnode.h"
#include "nrf24l01.h"
#include "nrfconst.h"

/** Internal modes.
 */
enum Mode {
  Startup,  //!< Pre-initialisation
  Idle,     //!< Transceiver idle
  Transmit, //!< Transmitting data
  Receive,  //!< Receiving data
  };

// Network configuration
#define TX_ADDRESS (const uint8_t *)"hub  "
#define RX_ADDRESS (const uint8_t *)"node "

// State information
static enum Mode g_mode = Startup;
static PIN       g_cePin;
static PIN       g_csnPin;

//---------------------------------------------------------------------------
// Internal helpers
//---------------------------------------------------------------------------

static void pause() {
  long val = 0;
  while(val<10000)
    val++;
  }

/** Write a new value to the specified register
 */
static void writeRegister(uint8_t reg, const uint8_t *data, int length) {
  spiConfig(false, true, true);
  pinWrite(g_csnPin, 0);
  reg = NRF_W_REGISTER | (reg & NRF_W_REGISTER_DATA);
  spiWrite(&reg, 1);
  pause();
  spiWrite(data, length);
  pinWrite(g_csnPin, 1);
  }

/** Read a value from the specified register
 */
static void readRegister(uint8_t reg, uint8_t *data, int length) {
  spiConfig(false, false, true);
  pinWrite(g_csnPin, 0);
  reg = NRF_R_REGISTER | (reg & NRF_R_REGISTER_DATA);
  spiWrite(&reg, 1);
  pause();
  spiRead(data, length);
  pinWrite(g_csnPin, 1);
  }

/** Write a single command to the module
 */
static void writeCommand(uint8_t cmd) {
  spiConfig(false, false, true);
  pinWrite(g_csnPin, 0);
  spiWrite(&cmd, 1);
  pinWrite(g_csnPin, 1);
  }

/** Read a packet from the FIFO buffer
 *
 */
static void readPacket(uint8_t *packet) {
  spiConfig(false, true, true);
  pinWrite(g_csnPin, 0);
  writeCommand(NRF_R_RX_PAYLOAD);
  spiRead(packet, 32);
  pinWrite(g_csnPin, 1);
  }

/** Write a packet to the FIFO buffer
 *
 */
static void writePacket(const uint8_t *packet) {
  spiConfig(false, true, true);
  pinWrite(g_csnPin, 0);
  writeCommand(NRF_W_TX_PAYLOAD);
  spiWrite(packet, 32);
  pinWrite(g_csnPin, 1);
  }

/** Set the adapter to the appropriate mode
 */
static void setMode(enum Mode mode) {
  // Are we already in the requested mode ?
  if(g_mode==mode)
    return;
  // We can't set, or change from, Startup - use init to do that
  if((mode==Startup)||(g_mode==Startup)) {
    return;
    }
  // Change mode
  pinWrite(g_cePin, 0);
  uint8_t config;
  readRegister(NRF_CONFIG, &config, 1);
  if(mode==Idle) {
    config &= ~NRF_CONFIG_PWR_UP;
    }
  else if(mode==Receive) {
    config |= (NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX);
    }
  else if(mode==Transmit) {
    config |= NRF_CONFIG_PWR_UP;
    config &= ~NRF_CONFIG_PRIM_RX;
    }
  writeRegister(NRF_CONFIG, &config, 1);
  if(mode!=Idle) // Activate radio
    pinWrite(g_cePin, 1);
  g_mode = mode;
  }

//---------------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------------

/** Initialise the NRF24L01 module
 *
 * @param cePin the chip enable (radio enable) pin to use.
 * @param csnPin the chip select (SPI comms) pin to use.
 */
void nrfInit(PIN cePin, PIN csnPin) {
  if(g_mode==Startup) {
    // Set up the pins
    g_cePin = cePin;
    pinConfig(g_cePin, DIGITAL_OUTPUT, 0);
    pinWrite(g_cePin, 0);
    g_csnPin = csnPin;
    pinConfig(g_csnPin, DIGITAL_OUTPUT, 1);
    pinWrite(g_csnPin, 1);
    }
  else if(g_mode!=Idle) {
    // Make sure we are in idle mode
    setMode(Idle);
    }
  // DEBUG BEGIN
  uint8_t debug[5];
  int len = 1;
  for(uint8_t reg = NRF_CONFIG; reg <= NRF_FIFO_STATUS; reg++) {
    switch(reg) {
      case NRF_RX_ADDR_P0:
      case NRF_RX_ADDR_P1:
      case NRF_TX_ADDR:
        len = 5;
        break;
      default:
        len = 1;
      }
    readRegister(reg, debug, len);
    serialFormat("#b: ", reg);
    for(int i=0; i<len; i++)
      serialFormat("#b", debug[len]);
    serialWrite('\r');
    serialWrite('\n');
    }
  // DEBUG END
  // Set up the module
  uint8_t data;
  // CONFIG register (no interrupts, 2 byte CRC, power down
  data = NRF_CONFIG_MASK_RX_DR |
         NRF_CONFIG_MASK_TX_DS |
         NRF_CONFIG_MASK_MAX_RT |
         NRF_CONFIG_EN_CRC |
         NRF_CONFIG_CRCO;
  writeRegister(NRF_CONFIG, &data, 1);
  // Disable shockburst (auto ack)
  data = 0;
  writeRegister(NRF_EN_AA, &data, 1);
  // Only 1 data pipe
  data = NRF_EN_RXADDR_ERX_P0;
  writeRegister(NRF_EN_RXADDR, &data, 1);
  // 5 byte addresses
  data = NRF_SETUP_AW_5BYTES;
  writeRegister(NRF_SETUP_AW, &data, 1);
  // RX address (our address)
  writeRegister(NRF_RX_ADDR_P0, RX_ADDRESS, 5);
  // TX address (target address)
  writeRegister(NRF_TX_ADDR, TX_ADDRESS, 5);
  // Payload size
  data = 32;
  writeRegister(NRF_RX_PW_P0, &data, 1);
  // Flush any left over payloads
  writeCommand(NRF_FLUSH_TX);
  writeCommand(NRF_FLUSH_RX);
  // All done
  g_mode = Idle;
  }

/** Enable or disable the radio.
 *
 * The NRF24L01 must be enabled to allow wireless communication. When disabled
 * it remains in a low power mode and no communications can be performed.
 *
 * @param enable true to enable the transceiver, false to disable it.
 */
void nrfEnable(bool enable) {
  if(enable)
    setMode(Receive);
  else
    setMode(Idle);
  }

/** Receive a packet
 *
 * This function checks for an available packet and, if one is available
 * populates the given buffer with the packet payload.
 *
 * @param packet a pointer to the payload for the packet. This buffer
 *               must be at least @see PAYLOAD_SIZE bytes in length.
 *
 * @return a pointer to packet if a packet was read, NULL if no data is
 *         available.
 */
uint8_t *nrfReceive(uint8_t *packet) {
  if(g_mode!=Receive)
    return NULL;
  // Check FIFO status
  uint8_t fifo;
  readRegister(NRF_FIFO_STATUS, &fifo, 1);
  if(fifo&NRF_FIFO_STATUS_RX_EMPTY)
    return NULL;
  // At least one packet available, copy it in
  readPacket(packet);
  return packet;
  }

/** Send a packet
 *
 * @param packet a pointer to the payload for the packet. This buffer
 *               must be at least @see PAYLOAD_SIZE bytes in length.
 */
void nrfSend(uint8_t *packet) {
  if(g_mode!=Receive)
    return; // No transmission in Idle or Startup modes
  // Switch to idle mode
  setMode(Idle);
  writePacket(packet);
  setMode(Transmit);
  // Wait for packet to leave
  uint8_t fifo = 0;
  while(!(fifo&NRF_FIFO_STATUS_TX_EMPTY)) {
    delay(1, MILLISECOND);
    readRegister(NRF_FIFO_STATUS, &fifo, 1);
    }
  // Return to receive mode
  setMode(Receive);
  }
