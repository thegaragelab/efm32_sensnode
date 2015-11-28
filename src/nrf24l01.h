/*--------------------------------------------------------------------------*
* SensNode - NRF24L01+ Driver
*---------------------------------------------------------------------------*
* 21-Nov-2015 ShaneG
*
* Simple driver for NRF24L01+ wireless modules.
---------------------------------------------------------------------------*/
#ifndef _NRF24L01_H
#define _NRF24L01_H

//! Size of a single packet payload
#define PAYLOAD_SIZE 32

/** Initialise the NRF24L01 module
 *
 * @param cePin the chip enable (radio enable) pin to use.
 * @param csnPin the chip select (SPI comms) pin to use.
 */
void nrfInit(PIN cePin, PIN csnPin);

/** Enable or disable the radio.
 *
 * The NRF24L01 must be enabled to allow wireless communication. When disabled
 * it remains in a low power mode and no communications can be performed.
 *
 * @param enable true to enable the transceiver, false to disable it.
 */
void nrfEnable(bool enable);

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
uint8_t *nrfReceive(uint8_t *packet);

/** Send a packet
 *
 * @param packet a pointer to the payload for the packet. This buffer
 *               must be at least @see PAYLOAD_SIZE bytes in length.
 */
void nrfSend(uint8_t *packet);

#endif /* _NRF24L01_H */
