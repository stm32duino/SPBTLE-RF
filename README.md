# SPBTLE-RF
Arduino library to support the Bluetooth (V4.1 compliant) SPBTLE-RF module

## API

The library provides a basic BLE class to configure and enable the Bluetooth module.
Each profile provides its own class. See Beacon and sensorDemo profiles.

## Examples

The library includes two sketches. They are very similar, one sketch provides a Beacon Service, an other a Sensor Service.

For the Beacon Service sketch, we can see on the monitor window all the initialization phase, and the message
"Beacon service start!" when the bluetooth module is started and ready.
Two mode are supported, UID mode and URL mode. On both mode, user can choose the bluetooth MAC address of the device by
configuring SERVER_BDADDR in the Arduino sketch.
On UID mode, user can choose the Namespace and the Instance. This data are sent to the associated device (for example your smartphone).
On URL mode, user can choose the webURL sended.

You can test this application by connecting it with your smartphone.
On Android, donwload the Beacon Scanner Apps (iBeacon & Eddystone Scanner by flurp laboratories). The Apps can
also be found [here](https://play.google.com/store/apps/details?id=de.flurp.beaconscanner.app).
Then start the app, it will ask you to enable the bluetooth on your smartphone. Start scanning and you will see the the device.
If you use it on UID mode, you will the see the Namespace and the instance.
If you use it on URL mode, you will see the URL, you can click on it and you will send the web page.


For the Sensor Service sketch, we can see on the monitor window all the initialization phase, and a message for each service started.
Three services are started : Acc, Environnemental and Time.
For testing the sketch, you can download on the playstore the "BLueNRG" application provided by STMicroelectronics.
Launch the application and enable Bluetooth on your smartphone. Connect it to the BLueNRG device. You will see all the services,
you can click on each one and read the data.
Pay attention that the device name can't be more than 7 characters long. If the string passed to the begin function is longer, it is 
automatically trimmed to the first 7 characters.
The BlueNRG app expects "BlueNRG" as device name, using anything else will make the device not connectable.


The SPBTLE-RF uses SPI. You need to configure the pin used for spi link.
  SPIClass SPI_3(MOSI, MISO, CLK);

Choose the SPI used by the SPBTLE-RF, and the pinout of the device. A cheep select pin, spiIRQ pin, reset PIN and a led (optional) are required.
  SPBTLERFClass BTLE(SPI_X, CS pin, IRQ pin, reset pin);
  SPBTLERFClass BTLE(SPI_X, CS pin, IRQ pin, reset pin, LED pin);

Start the bluetooth module.
  BTLE.begin();

Start the service. For example the BeaconService in UID_TYPE.
  BeaconService.begin(SERVER_BDADDR, beaconID, NameSpace);

## BLE stack

Version: 3.0.0  
The Bluetooth stack comes from [STM32CubeExpansion_BLE1_V3.0.0](http://www.st.com/content/st_com/en/products/embedded-software/mcus-embedded-software/stm32-embedded-software/stm32cube-embedded-software-expansion/x-cube-ble1.html).  

The BlueNRG stack is composed of some specific parts:  

* **HCI** files provide API for HCI (Host Controller Interface) layer  
* **GAP** files provide API for GAP (Generic Access Profile) layer  
* **GATT** files provide API for GATT (Generic Attribute Profile) layer  
* **L2CAP** files provide API for L2CAP (Logical Link Control and Adaptation Protocol) layer  

More information about the different layers:  
* https://www.bluetooth.com/specifications/bluetooth-core-specification
* https://www.bluetooth.com/specifications/gatt  

## Note

At the compilation time a warning is raised about an IFR configuration not valid.
This is normal because the library proposes an API to update the firmware of the
BLE module but the configuration flag isn't declared. See STM32CubeExpansion_BLE1_V3.0.0
documentation for more information about the IFR updater.

## Documentation

You can find the source files at  
https://github.com/stm32duino/SPBTLE-RF

The SPBTLE-RF module datasheet is available at  
http://www.st.com/content/st_com/en/products/wireless-connectivity/bluetooth-bluetooth-low-energy/spbtle-rf.html
