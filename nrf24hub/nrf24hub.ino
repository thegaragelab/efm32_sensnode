#include <SPI.h>
#include "nrfconst.h"

/*--------------------------------------------------------------------------*
* Simple NRF24L01 to Serial adapter.
*--------------------------------------------------------------------------*/

// Pin assignments
#define NRF24L01_CE  12
#define NRF24L01_CSN 13

// Network configuration
#define TX_ADDRESS "hub  "
#define RX_ADDRESS "node "

// Maximum payload size
#define MAX_PAYLOAD 32

// Serial input (allow for 64 hex digits + command char + EOL)
#define MAX_INPUT_LINE 70

// SPI configuration
#define SPI_SPEED 10000000
#define SPI_MODE  SPI_MODE0
#define SPI_ORDER MSBFIRST

// Forward definitions so we can report messages from the NRF24L01 class
void sendMessage(const char *cszMessage);
void writeHex(const uint8_t *data, int length);

/** Wrapper class for the NRF24L01
 */
class NRF24L01 {
  private:
    /** Internal modes.
     */
    enum Mode {
      Startup,  //!< Pre-initialisation
      Idle,     //!< Transceiver idle
      Transmit, //!< Transmitting data
      Receive,  //!< Receiving data
      };

  private:
    int     m_cePin;                   //!< Chip enable pin
    int     m_csnPin;                  //!< Chip select pin
    uint8_t m_buffer[MAX_PAYLOAD + 1]; //!< SPI transfer buffer
    Mode    m_mode;                    //!< Transceiver mode
    Mode    m_next;                    //!< Next mode after transmission

  private:
    /** Write a new value to the specified register
     */
    void writeRegister(uint8_t reg, uint8_t *data, int length) {
      // DEBUG BEGIN: Show what we are doing
      Serial.write(';');
      Serial.print("Write ");
      Serial.print(reg, HEX);
      Serial.print(" = ");
      writeHex(data, length);
      Serial.write('\n');
      // DEBUG END
      SPI.beginTransaction(SPISettings(SPI_SPEED, SPI_ORDER, SPI_MODE));
      digitalWrite(m_csnPin, 0);
      // Transfer the data into the SPI buffer first
      m_buffer[0] = NRF_W_REGISTER | (reg & NRF_W_REGISTER_DATA);
      memcpy(&m_buffer[1], data, length);
      SPI.transfer(m_buffer, length + 1);
      digitalWrite(m_csnPin, 1);
      SPI.endTransaction();
      }

    /** Read a value from the specified register
     */
    void readRegister(uint8_t reg, uint8_t *data, int length) {
      SPI.beginTransaction(SPISettings(SPI_SPEED, SPI_ORDER, SPI_MODE));
      digitalWrite(m_csnPin, 0);
      // Transfer the data into the SPI buffer first
      m_buffer[0] = NRF_R_REGISTER | (reg & NRF_R_REGISTER_DATA);
      SPI.transfer(m_buffer, length + 1);
      memcpy(data, &m_buffer[1], length);
      digitalWrite(m_csnPin, 1);
      SPI.endTransaction();
      // DEBUG BEGIN: Show what we are doing
      Serial.write(';');
      Serial.print("Read ");
      Serial.print(reg, HEX);
      Serial.print(" = ");
      writeHex(data, length);
      Serial.write('\n');
      // DEBUG END
      }

    /** Set the adapter to the appropriate mode
     */
    void setMode(Mode mode) {
      // Are we already in the requested mode ?
      if(m_mode==mode)
        return;
      // We can't set, or change from, Mode::Startup
      if((mode==Mode::Startup)||(m_mode==Mode::Startup)) {
        sendMessage("Warning: Mode change not supported.");
        return;
        }
      // TODO: Change mode
      digitalWrite(m_cePin, 0);
      }

  public:
    /** Constructor
     */
    NRF24L01(int cePin, int csnPin) {
      m_cePin = cePin;
      m_csnPin = csnPin;
      m_mode = Mode::Startup;
      }

    /** Initialise the module
     *
     * This does the basic initialisation (setting addresses, packet sizes,
     * etc).
     */
    void init() {
      if(m_mode==Mode::Startup) {
        // Set up the pins
        pinMode(m_cePin, OUTPUT);
        digitalWrite(m_cePin, 0);
        pinMode(m_csnPin, OUTPUT);
        digitalWrite(m_csnPin, 1);
        }
      else if(m_mode!=Mode::Idle) {
        // Make sure we are in idle mode
        setMode(Mode::Idle);
        }
      // DEBUG BEGIN: Read and display all register values
      uint8_t data[5];
      for(uint8_t reg = NRF_CONFIG; reg <= NRF_FIFO_STATUS; reg++) {
        switch(reg) {
          case NRF_RX_ADDR_P0:
          case NRF_RX_ADDR_P1:
          case NRF_TX_ADDR:
            readRegister(reg, data, 5);
            break;
          default:
            readRegister(reg, data, 1);
            break;
          }
        }
      // DEBUG END
      // TODO: Configure the addresses and other settings
      //m_mode = Mode::Idle;
      }

    /** Enable or disable the adapter
     *
     * When enabled the module will be put into listening mode and any incoming
     * packets will be copied to an internal buffer and made available to the
     * client. When disabled the adapter is put into low power mode and no
     * transmission or reception will take place.
     *
     * @param enabled true if the adapter should be enabled, false otherwise.
     */
    void enable(bool enabled) {
      if(enabled&&(m_mode==Mode::Idle))
        setMode(Mode::Receive);
      if((!enabled)&&(m_mode!=Mode::Idle)) {
        if(m_mode==Mode::Transmit)
          m_next = Mode::Idle;
        else
          setMode(Mode::Idle);
        }
      }

    /** Check for received packets
     *
     * If a packet is available a pointer to the 32 byte payload will be
     * returned. You must copy or do something with this packet before the
     * next call to 'update()' as the packet contents may be overwritten.
     *
     * @return a pointer to the received packet payload or NULL if no packet
     *         has been received.
     */
    uint8_t *receive() {
      if(m_mode!=Mode::Receive)
        return NULL;
      // TODO: Check FIFO status
      return NULL;
      }

    /** Send a packet
     *
     * Transmit a packet. This puts the adapter into transmission mode until
     * the packet has been sent (or an error occurs).
     *
     * @param payload pointer to the payload data
     */
    void send(const uint8_t *payload) {
      if((m_mode!=Mode::Transmit)&&(m_mode!=Mode::Receive))
        return; // No transmission in Idle or Startup modes
      if(m_mode==Mode::Receive) {
        m_next = m_mode; // Remember to come back to receive mode
        }
      }

    /** Update the module state
     *
     * This method must be called in the main application loop. It will check
     * for incoming packets, handle transmission and update the module state.
     */
    void update() {
      if(m_mode==Mode::Receive) {
        // TODO: Look for new packets
        }
      else if(m_mode==Mode::Transmit) {
        // TODO: If transmission is complete revert to previous mode
        }
      }
  };

// Globals
NRF24L01 g_nrf(NRF24L01_CE, NRF24L01_CSN);
uint8_t g_input[MAX_INPUT_LINE];
int g_inputIndex = 0;

//---------------------------------------------------------------------------
// Helper functions
//---------------------------------------------------------------------------

/** Send a message on the serial link
 *
 * Message lines start with the ';' character and are generally used for
 * informational and debug purposes.
 */
void sendMessage(const char *cszMessage) {
  Serial.write(';');
  Serial.print(cszMessage);
  Serial.write('\n');
  }

void writeHex(const uint8_t *data, int length) {
  for(int i=0; i<length; i++)
    Serial.print(data[i], HEX);
  }

/** Send a packet on the serial link
 */
void sendPacket(const uint8_t *packet) {
  Serial.write('<');
  writeHex(packet, MAX_PAYLOAD);
  Serial.write('\n');
  }

uint8_t hexValue(char ch) {
  if((ch>='0')&&(ch<='9'))
    return ch - '0';
  if((ch>='a')&&(ch<='f'))
    return 10 + (ch - 'a');
  if((ch>='A')&&(ch<='F'))
    return 10 + (ch - 'A');
  // TODO: Should handle errors nicely
  return 0;
  }

/** Read a packet from the input buffer
 */
uint8_t *readPacket() {
  if(g_inputIndex&0x01)
    return NULL; // Not valid
  // Reuse the input buffer
  uint8_t value;
  int index = 1;
  int output = 0;
  while((index<(g_inputIndex-1))&&(output<MAX_PAYLOAD)) {
    value = hexValue(g_input[index++]);
    value = (value << 4) | hexValue(g_input[index++]);
    g_input[output++] = value;
    }
  // Pad with zeros
  while(output<MAX_PAYLOAD)
    g_input[output++] = '\0';
  return (uint8_t *)g_input;
  }

//---------------------------------------------------------------------------
// Main program
//---------------------------------------------------------------------------

void setup() {
  // Initialise the serial port
  Serial.begin(57600);
  sendMessage("Initialising network");
  // Initialise SPI
  SPI.begin();
  }

void loop() {
  uint8_t *packet;
  // Allow the NRF to update it's internal state
  g_nrf.update();
  // Check for any serial input
  while(Serial.available()) {
    char ch = Serial.read();
    if(g_inputIndex==MAX_INPUT_LINE) {
      // Too much data, drop it and start a new line
      g_inputIndex = 0;
      }
    g_input[g_inputIndex++] = ch;
    // Do we have an EOL?
    if(ch=='\n') {
      if(g_input[0]=='>') {
        packet = readPacket();
        if(packet!=NULL)
          g_nrf.send(packet);
        }
      else if(g_input[0]=='!') {
        // Initialise the NRF module
        g_nrf.init();
        }
      else if(g_input[0]=='+') {
        // Enable the adapter
        g_nrf.enable(true);
        }
      else if(g_input[0]=='-') {
        // Disable the adapter
        g_nrf.enable(false);
        }
      // Wait for the next line
      g_inputIndex = 0;
      }
    }
  // Check for any received packets
  packet = g_nrf.receive();
  if(packet!=NULL)
    sendPacket(packet);
  }
