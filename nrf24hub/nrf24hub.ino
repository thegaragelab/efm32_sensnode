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

// Forward definition so we can report messages from the NRF24L01 class
void sendMessage(const char *cszMessage);

/** Wrapper class for the NRF24L01
 */
class NRF24L01 {
  private:
    int m_cePin; //!< Chip enable pin
    int m_csnPin; //!< Chip select pin

  public:
    /** Constructor
     */
    NRF24L01(int cePin, int csnPin) {
      m_cePin = cePin;
      m_csnPin = csnPin;
      }

    /** Initialise the module
     *
     * This does the basic initialisation (setting addresses, packet sizes,
     * etc).
     */
    void init() {
      // Set up the pins
      pinMode(m_cePin, OUTPUT);
      digitalWrite(m_cePin, 0);
      pinMode(m_csnPin, OUTPUT);
      digitalWrite(m_csnPin, 1);
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
      }

    /** Update the module state
     *
     * This method must be called in the main application loop. It will check
     * for incoming packets, handle transmission and update the module state.
     */
    void update() {
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

/** Send a packet on the serial link
 */
void sendPacket(const uint8_t *packet) {
  Serial.write('<');
  for(int i=0; i<MAX_PAYLOAD; i++)
    Serial.print(packet[i], HEX);
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
