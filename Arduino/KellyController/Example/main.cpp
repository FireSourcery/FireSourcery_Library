/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules
 Pick one up today in the adafruit shop!
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include "Arduino\KellyController\KellyController.h"
#include <bluefruit.h>

BLEDfu bledfu; // OTA DFU service
BLEUart bleuart; // Uart over BLE service

KellyController kellyController;

static void startAdv(void);
// Function prototypes for packetparser.cpp
uint8_t readPacket (BLEUart *ble_uart, uint16_t timeout);
float   parsefloat (uint8_t *buffer);
void    printHex   (const uint8_t * data, const uint32_t numBytes);

// Packet buffer
extern uint8_t packetbuffer[];

void setup(void)
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println(F("Adafruit Bluefruit52 Controller App Example"));
  Serial.println(F("-------------------------------------------"));

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  kellyController.begin();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void printTx(void)
{
  Serial.print("Tx: ");
  Serial.printBuffer(kellyController.getPtrTxPacket(), kellyController.getTxLength());
  Serial.print("\n\r");
}

void printRx(void)
{
  Serial.printBuffer(kellyController.getPtrRxPacket(), kellyController.getRxLength());
  Serial.print("\n\r");
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  Protocol_RxCode_T status = kellyController.procRxReqTxResp();

  switch (status)
  {
    case PROTOCOL_RX_CODE_PACKET_TIMEOUT:
      Serial.print("Rx Timeout: ");
      printRx();
      break; // timeout during rx, to timeout tx idle
    case PROTOCOL_RX_CODE_PACKET_COMPLETE:
      Serial.print("Rx Success: ");
      printRx();
      break;
    case PROTOCOL_RX_CODE_PACKET_ERROR:
      Serial.print("Rx Error: ");
      printRx();
      break;
    default:
      break;
  }

  // Wait for new data to arrive
  uint8_t len = readPacket(&bleuart, 500);
  if (len == 0) return;

  // Got a packet!
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if(packetbuffer[1] == 'B')
  {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print("Button "); Serial.print(buttnum);
    if(pressed) { Serial.println(" pressed"); }
    else { Serial.println(" released"); }

    switch(buttnum)
    {
      case 1U:
        if(pressed) { kellyController.ping(); printTx();}
        break;
      case 2U:
        if(pressed) { kellyController.writeStopAll(); printTx();}
        break;
      case 3U:
        if(pressed) { kellyController.readSpeed(); printTx();}
        break;
      case 5U:
        if(pressed) { kellyController.writeThrottle(65535); printTx();}
        else { kellyController.writeRelease(); printTx();}
        break;
      case 6U:
        if(pressed) { kellyController.writeBrake(65535); printTx();}
        else { kellyController.writeRelease(); printTx();} 
        break;
      default: break;
    } 
  }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}
