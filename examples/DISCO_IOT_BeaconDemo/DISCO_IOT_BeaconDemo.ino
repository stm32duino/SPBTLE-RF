/*

 DISCO_IOT_BeaconDemo

 This sketch provides a default example how to use the BLE module of the
 Discovery L475VG IoT board.

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

/* Configure SPI3
  MOSI: PC12
  MISO: PC11
  SCLK: PC10
  */
SPIClass SPI_3(PC12, PC11, PC10);

// Configure BTLE pins
SPBTLERFClass BTLE(&SPI_3, PD13, PE6, PA8, LED4);

// Mac address
uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03};

//Comment this line to use URL mode
#define USE_UID_MODE

#ifdef USE_UID_MODE
// Beacon ID, the 6 last bytes are used for NameSpace
uint8_t NameSpace[] = "DISCO_IOT";
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
