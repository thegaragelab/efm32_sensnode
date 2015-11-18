#include "nrfconst.h"

/*--------------------------------------------------------------------------*
* Simple NRF24L01 to Serial adapter.
*--------------------------------------------------------------------------*/

// Pin assignments
#define NRF24L01_CE   2
#define NRF24L01_CSN  3
#define PIN_MISO 4
#define PIN_MOSI 5
#define PIN_SCK  6

// Network configuration
#define TX_ADDRESS "hub  "
#define RX_ADDRESS "node "

// Maximum payload size
#define MAX_PAYLOAD 32

// Serial input (allow for 64 hex digits + command char + EOL)
#define MAX_INPUT_LINE 70

// SPI configuration
#define SPI_SPEED 1000000
#define SPI_MODE  SPI_MODE0
#define SPI_ORDER MSBFIRST

// Forward definitions so we can report messages from the NRF24L01 class
void sendMessage(const char *cszMessage);
void writeHex(const uint8_t *data, int length);

// SPI configuration
static bool g_spiPolarity = false;
static bool g_spiPhase = false;
static bool g_spiMSBFirst = false;

/** Initialise the SPI subsystem
 */
void initSPI() {
  // Set up the pins
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
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
  delayMicroseconds(10);
  while(mask) {
    digitalWrite(PIN_MOSI, data & mask);
    // Change to active state
    delayMicroseconds(10);
    digitalWrite(PIN_SCK, g_spiPolarity?0:1);
    if(!g_spiPhase)
      input |= (digitalRead(PIN_MISO)?mask:0);
    // Change to idle state
    delayMicroseconds(10);
    digitalWrite(PIN_SCK, g_spiPolarity?1:0);
    if(g_spiPhase)
      input |= (digitalRead(PIN_MISO)?mask:0);
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
    if(g_spiPhase)
      digitalWrite(PIN_MOSI, data & mask);
    // Change to active state
    digitalWrite(PIN_SCK, g_spiPolarity?0:1);
    if(!g_spiPhase) {
      input |= (digitalRead(PIN_MISO)?mask:0);
      digitalWrite(PIN_MOSI, data & mask);
      }
    // Change to idle state
    digitalWrite(PIN_SCK, g_spiPolarity?1:0);
    if(g_spiPhase)
      input |= (digitalRead(PIN_MISO)?mask:0);
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
  digitalWrite(PIN_SCK, g_spiPolarity?1:0);
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
    int     m_clkPin;
    int     m_mosiPin;
    int     m_misoPin;
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
      spiConfig(false, true, true);
      digitalWrite(m_csnPin, 0);
      reg = NRF_W_REGISTER | (reg & NRF_W_REGISTER_DATA);
      spiWrite(&reg, 1);
      spiWrite(data, length);
      digitalWrite(m_csnPin, 1);
      }

    /** Read a value from the specified register
     */
    void readRegister(uint8_t reg, uint8_t *data, int length) {
      delay(1);
      spiConfig(false, false, true);
      digitalWrite(m_csnPin, 0);
      reg = NRF_R_REGISTER | (reg & NRF_R_REGISTER_DATA);
      spiWrite(&reg, 1);
      spiRead(data, length);
      digitalWrite(m_csnPin, 1);
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
      // We can't set, or change from, Startup
      if((mode==Startup)||(m_mode==Startup)) {
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
      m_mode = Startup;
      }

    /** Initialise the module
     *
     * This does the basic initialisation (setting addresses, packet sizes,
     * etc).
     */
    void init() {
      if(m_mode==Startup) {
        // Set up the pins
        pinMode(m_cePin, OUTPUT);
        digitalWrite(m_cePin, 0);
        pinMode(m_csnPin, OUTPUT);
        digitalWrite(m_csnPin, 1);
        }
      else if(m_mode!=Idle) {
        // Make sure we are in idle mode
        setMode(Idle);
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
      //m_mode = Idle;
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
      if(enabled&&(m_mode==Idle))
        setMode(Receive);
      if((!enabled)&&(m_mode!=Idle)) {
        if(m_mode==Transmit)
          m_next = Idle;
        else
          setMode(Idle);
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
      if(m_mode!=Receive)
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
      if((m_mode!=Transmit)&&(m_mode!=Receive))
        return; // No transmission in Idle or Startup modes
      if(m_mode==Receive) {
        m_next = m_mode; // Remember to come back to receive mode
        }
      }

    /** Update the module state
     *
     * This method must be called in the main application loop. It will check
     * for incoming packets, handle transmission and update the module state.
     */
    void update() {
      if(m_mode==Receive) {
        // TODO: Look for new packets
        }
      else if(m_mode==Transmit) {
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
  for(int i=0; i<length; i++) {
    if(data[i]<=0x0f)
      Serial.write('0');
    Serial.print(data[i], HEX);
    }
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
  initSPI();
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
