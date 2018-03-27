/*

 SPBTLE_BeaconDemo

 This sketch provides a default example how to use BLE with:
  - Discovery L475VG IoT board

 For the Beacon Service, two modes are supported:
 - UID mode, you can choose the Namespace and the ID. This data are sent
   to the associated device (for example your smartphone).
   Or
 - URL mode, you can choose the webURL sended.

 You can choose the bluetooth MAC address of the device by configuring SERVER_BDADDR.

 You can test this application by connecting it with your smartphone.
 On Android, donwload any Beacon Scanner Apps (e.g. iBeacon & Eddystone Scanner
 by flurp laboratories https://play.google.com/store/apps/details?id=de.flurp.beaconscanner.app).
 Then start the app, enable the bluetooth on your smartphone, start scanning and
 you will see the device.
 If you use UID mode, you will the see the Namespace and the instance.
 If you use URL mode, you will see the URL, you can click on it and you will
 send to the web page.

*/

#include <SPI.h>
#include <SPBTLE_RF.h>
#include <beacon_service.h>

#define PIN_BLE_SPI_MOSI   (PC12)
#define PIN_BLE_SPI_MISO   (PC11)
#define PIN_BLE_SPI_SCK    (PC10)

#define PIN_BLE_SPI_nCS    (PD13)
#define PIN_BLE_SPI_RESET  (PA8)
#define PIN_BLE_SPI_IRQ    (PE6)

#define PIN_BLE_LED    (LED4)

// Configure BTLE_SPI
SPIClass BTLE_SPI(PIN_BLE_SPI_MOSI, PIN_BLE_SPI_MISO, PIN_BLE_SPI_SCK);

// Configure BTLE pins
SPBTLERFClass BTLE(&BTLE_SPI, PIN_BLE_SPI_nCS, PIN_BLE_SPI_IRQ, PIN_BLE_SPI_RESET, PIN_BLE_LED);

// Mac address
uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03};

//Comment this line to use URL mode
#define USE_UID_MODE

#ifdef USE_UID_MODE
// Beacon ID, the 6 last bytes are used for NameSpace
uint8_t NameSpace[] = "ST BTLE";
uint8_t beaconID[] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6};
#else
char url[] = "www.st.com";
#endif

void setup() {
  Serial.begin(9600);

  if(BTLE.begin())
  {
    Serial.println("Bluetooth module configuration error!");
    while(1);
  }

#ifdef USE_UID_MODE
  // Enable the beacon service in UID mode
  if(BeaconService.begin(SERVER_BDADDR, beaconID, NameSpace))
  {
    Serial.println("Beacon service configuration error!");
    while(1);
  }
  else
  {
    Serial.println("Beacon service started!");
  }
#else
  //Enable the beacon service in URL mode
  if(BeaconService.begin(SERVER_BDADDR, url))
  {
    Serial.println("Beacon service configuration error!");
    while(1);
  }
  else
  {
    Serial.println("Beacon service started!");
  }
#endif
}

void loop() {
  // Update the BLE module state
  BTLE.update();
}
