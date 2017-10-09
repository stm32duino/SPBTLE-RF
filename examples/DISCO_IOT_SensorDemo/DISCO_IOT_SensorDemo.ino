/*

 DISCO_IOT_SensorDemo

 This sketch provides a default example how to use the BLE module of the
 Discovery L475VG IoT board.

 For the Sensor Service sketch, 3 services are started : Acc, Environnemental and Time.
 For testing the sketch, you can download on the playstore the "BlueNRG"
 application provided by STMICROELECTRONICS.
 Launch the application and enable Bluetooth on your smartphone. Connect it to
 the BLueNRG device. You will see all the services, you can click on each one.

 You can choose the bluetooth MAC address of the device by configuring SERVER_BDADDR.

 Accelerometer values are updated on user action (press user button).
 Environnemental values (Temperature, humidity and pressure) are updated each seconds.
 Each minute a notification is sent to the user and seconds can be read.

 */

#include <SPI.h>
#include <SPBTLE_RF.h>
#include <sensor_service.h>

/* Configure SPI3
  MOSI: PC12
  MISO: PC11
  SCLK: PC10
  */
SPIClass SPI_3(PC12, PC11, PC10);

// Configure BTLE pins
SPBTLERFClass BTLE(&SPI_3, PD13, PE6, PA8, LED4);

const char *name = "BlueNRG";
uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03};

AxesRaw_t axes_data;
uint32_t previousSecond = 0;

void setup() {
  int ret;

  Serial.begin(9600);

  if(BTLE.begin() == SPBTLERF_ERROR)
  {
    Serial.println("Bluetooth module configuration error!");
    while(1);
  }

  if(SensorService.begin(name, SERVER_BDADDR))
  {
    Serial.println("Sensor service configuration error!");
    while(1);
  }

  /* Configure the User Button in GPIO Mode */
  pinMode(USER_BTN, INPUT);

  ret = SensorService.Add_Acc_Service();

  if(ret == BLE_STATUS_SUCCESS)
    Serial.println("Acc service added successfully.");
  else
    Serial.println("Error while adding Acc service.");

  ret = SensorService.Add_Environmental_Sensor_Service();

  if(ret == BLE_STATUS_SUCCESS)
    Serial.println("Environmental Sensor service added successfully.");
  else
    Serial.println("Error while adding Environmental Sensor service.");

  randomSeed(analogRead(A0));

  /* Instantiate Timer Service with two characteristics:
   * - seconds characteristic (Readable only)
   * - minutes characteristics (Readable and Notifiable )
   */
  ret = SensorService.Add_Time_Service();

  if(ret == BLE_STATUS_SUCCESS)
    Serial.println("Time service added successfully.");
  else
    Serial.println("Error while adding Time service.");
}

void loop() {
  BTLE.update();

  if(SensorService.isConnected() == TRUE)
  {
    //Update accelerometer values
    User_Process(&axes_data);

    //Update time
    SensorService.Update_Time_Characteristics();

    if((millis() - previousSecond) >= 1000)
    {
      //Update environnemental data
      //Data are set with random values but can be replace with data from sensors.
      previousSecond = millis();
      SensorService.Temp_Update(random(-100,400));
      SensorService.Press_Update(random(95000,105000));
      SensorService.Humidity_Update(random(0,100));
    }
  }
  else
  {
    //Keep the Bluetooth module in discoverable mode
    SensorService.setConnectable();
  }
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send the updated acceleration data to the remote client.
 *
 * @param  AxesRaw_t* p_axes
 * @retval None
 */
void User_Process(AxesRaw_t* p_axes)
{
  /* Check if the user has pushed the button */
  if(digitalRead(USER_BTN) == RESET)
  {
    while (digitalRead(USER_BTN) == RESET);

    /* Update acceleration data */
    p_axes->AXIS_X += 100;
    p_axes->AXIS_Y += 100;
    p_axes->AXIS_Z += 100;
    SensorService.Acc_Update(p_axes);
  }
}
