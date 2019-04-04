/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   Add a sample service using a vendor specific profile.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "sensor_service.h"
#include "bluenrg_hal_aci.h"
#include "hci.h"
#include "hci_le.h"
#include "bluenrg_utils.h"
#include "stm32_bluenrg_ble.h"
#include "osal.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "gp_timer.h"
#include "Arduino.h"

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup SensorDemo
 *  @{
 */

/** @defgroup SENSOR_SERVICE
 * @{
 */

/** @defgroup SENSOR_SERVICE_Private_Variables
 * @{
 */
/* Public variables ----------------------------------------------------------*/
extern uint8_t bnrg_expansion_board;
SensorServiceClass SensorService;

/**
 * @}
 */

/** @defgroup SENSOR_SERVICE_Private_Macros
 * @{
 */
/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

#define COPY_ACC_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x02,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_FREE_FALL_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0xe2,0x3e,0x78,0xa0, 0xcf,0x4a, 0x11,0xe1, 0x8f,0xfc, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x34,0x0a,0x1b,0x80, 0xcf,0x4b, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define COPY_ENV_SENS_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x42,0x82,0x1a,0x40, 0xe4,0x77, 0x11,0xe2, 0x82,0xd0, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TEMP_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0xa3,0x2e,0x55,0x20, 0xe4,0x77, 0x11,0xe2, 0xa9,0xe3, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_PRESS_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0xcd,0x20,0xc4,0x80, 0xe4,0x8b, 0x11,0xe2, 0x84,0x0b, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_HUMIDITY_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x01,0xc5,0x0b,0x60, 0xe4,0x8c, 0x11,0xe2, 0xa0,0x73, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

// Time service: uuid = 0x08, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b
//      straight uuid = 0x08366e80cf3a11e19ab40002a5d5c51b
#define COPY_TIME_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x08,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TIME_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x09,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_MINUTE_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x0a,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

// LED service
#define COPY_LED_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x0b,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_LED_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x0c,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)


/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/* Private Prototypes --------------------------------------------------------*/
void Sensor_HCI_Event_CB(void *pckt);




/**
 * @}
 */

/** @defgroup SENSOR_SERVICE_Exported_Functions
 * @{
 */

tBleStatus SensorServiceClass::begin(const char *name, uint8_t addr[BDADDR_SIZE])
{
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

  uint8_t  hwVersion;
  uint16_t fwVersion;
  
  memset(dev_name, 0, sizeof(dev_name));
  
  dev_name[0] =AD_TYPE_COMPLETE_LOCAL_NAME;
  strncpy(&dev_name[1], name, (strlen(name)<7) ? strlen(name) : 7);

  int ret;

  if((name == NULL) || (addr == NULL)) {
    return BLE_STATUS_NULL_PARAM;
  }

  attach_HCI_CB(Sensor_HCI_Event_CB);

  /* get the BlueNRG HW and FW versions */
  ret = getBlueNRGVersion(&hwVersion, &fwVersion);
  if(ret) {
    PRINTF("Reading Version failed.\n");
    return ret;
  }

  /*
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  BlueNRG_RST();

  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1;
    /*
     * Change the MAC address to avoid issues with Android cache:
     * if different boards have the same MAC address, Android
     * applications unless you restart Bluetooth on tablet/phone
     */
    addr[5] = 0x02;
  }

  /* The Nucleo board must be configured as SERVER */
  Osal_MemCpy(bdaddr, addr, BDADDR_SIZE);

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret){
    PRINTF("Setting BD_ADDR failed.\n");
    return ret;
  }

  ret = aci_gatt_init();
  if(ret){
    PRINTF("GATT_Init failed.\n");
    return ret;
  }

  if (bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(ret){
    PRINTF("GAP_Init failed.\n");
    return ret;
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   (strlen(name)<7) ? strlen(name) : 7, (uint8_t *)name);

  if(ret){
    PRINTF("aci_gatt_update_char_value failed.\n");
    return ret;
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret) {
    PRINTF("BLE Stack Initialization failed.\n");
    return ret;
  }

  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

  if (ret) {
    PRINTF("Setting Tx Power Level failed.\n");
  }

  return ret;
}



/**
 * @brief  Add an accelerometer service using a vendor specific profile.
 *
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus SensorServiceClass::Add_Acc_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_ACC_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7,
                          &accServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  COPY_FREE_FALL_UUID(uuid);
  ret =  aci_gatt_add_char(accServHandle, UUID_TYPE_128, uuid, 1,
                           CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 0, &freeFallCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  COPY_ACC_UUID(uuid);
  ret =  aci_gatt_add_char(accServHandle, UUID_TYPE_128, uuid, 6,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &accCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  PRINTF("Service ACC added. Handle 0x%04X, Free fall Charac handle: 0x%04X, Acc Charac handle: 0x%04X\n",accServHandle, freeFallCharHandle, accCharHandle);
  return BLE_STATUS_SUCCESS;

fail:
  PRINTF("Error while adding ACC service.\n");
  return BLE_STATUS_ERROR ;

}

/**
 * @brief  Send a notification for a Free Fall detection.
 *
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus SensorServiceClass::Free_Fall_Notify(void)
{
  uint8_t val;
  tBleStatus ret;

  val = 0x01;
  ret = aci_gatt_update_char_value(accServHandle, freeFallCharHandle, 0, 1,
                                   &val);

  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating FFall characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update acceleration characteristic value.
 *
 * @param  Structure containing acceleration value in mg
 * @retval Status
 */
tBleStatus SensorServiceClass::Acc_Update(AxesRaw_t *data)
{
  tBleStatus ret;
  uint8_t buff[6];

  axes_data.AXIS_X = data->AXIS_X;
  axes_data.AXIS_Y = data->AXIS_Y;
  axes_data.AXIS_Z = data->AXIS_Z;

  STORE_LE_16(buff,data->AXIS_X);
  STORE_LE_16(buff+2,data->AXIS_Y);
  STORE_LE_16(buff+4,data->AXIS_Z);

  ret = aci_gatt_update_char_value(accServHandle, accCharHandle, 0, 6, buff);

  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating ACC characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the Environmental Sensor service.
 *
 * @param  None
 * @retval Status
 */
tBleStatus SensorServiceClass::Add_Environmental_Sensor_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  uint16_t uuid16;
  charactFormat charFormat;
  uint16_t descHandle;

  COPY_ENV_SENS_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 10,
                          &envSensServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  /* Temperature Characteristic */
  COPY_TEMP_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, uuid, 2,
                           CHAR_PROP_READ, ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &tempCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  charFormat.format = FORMAT_SINT16;
  charFormat.exp = -1;
  charFormat.unit = UNIT_TEMP_CELSIUS;
  charFormat.name_space = 0;
  charFormat.desc = 0;

  uuid16 = CHAR_FORMAT_DESC_UUID;

  ret = aci_gatt_add_char_desc(envSensServHandle,
                               tempCharHandle,
                               UUID_TYPE_16,
                               (uint8_t *)&uuid16,
                               7,
                               7,
                               (void *)&charFormat,
                               ATTR_PERMISSION_NONE,
                               ATTR_ACCESS_READ_ONLY,
                               0,
                               16,
                               FALSE,
                               &descHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  /* Pressure Characteristic */
  COPY_PRESS_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, uuid, 3,
                           CHAR_PROP_READ, ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &pressCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  charFormat.format = FORMAT_UINT32;
  charFormat.exp = -1;
  charFormat.unit = UNIT_PRESSURE_PASCAL;
  charFormat.name_space = 0;
  charFormat.desc = 0;

  uuid16 = CHAR_FORMAT_DESC_UUID;

  ret = aci_gatt_add_char_desc(envSensServHandle,
                               pressCharHandle,
                               UUID_TYPE_16,
                               (uint8_t *)&uuid16,
                               7,
                               7,
                               (void *)&charFormat,
                               ATTR_PERMISSION_NONE,
                               ATTR_ACCESS_READ_ONLY,
                               0,
                               16,
                               FALSE,
                               &descHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  /* Humidity Characteristic */
  COPY_HUMIDITY_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, uuid, 2,
                           CHAR_PROP_READ, ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &humidityCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  charFormat.format = FORMAT_UINT16;
  charFormat.exp = -1;
  charFormat.unit = UNIT_PERCENTAGE;
  charFormat.name_space = 0;
  charFormat.desc = 0;

  uuid16 = CHAR_FORMAT_DESC_UUID;

  ret = aci_gatt_add_char_desc(envSensServHandle,
                               humidityCharHandle,
                               UUID_TYPE_16,
                               (uint8_t *)&uuid16,
                               7,
                               7,
                               (void *)&charFormat,
                               ATTR_PERMISSION_NONE,
                               ATTR_ACCESS_READ_ONLY,
                               0,
                               16,
                               FALSE,
                               &descHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  PRINTF("Service ENV_SENS added. Handle 0x%04X, TEMP Charac handle: 0x%04X, PRESS Charac handle: 0x%04X, HUMID Charac handle: 0x%04X\n",envSensServHandle, tempCharHandle, pressCharHandle, humidityCharHandle);
  return BLE_STATUS_SUCCESS;

fail:
  PRINTF("Error while adding ENV_SENS service.\n");
  return BLE_STATUS_ERROR ;

}

/**
 * @brief  Update temperature characteristic value.
 * @param  Temperature in hundredths of degree celsius
 * @retval Status
 */
tBleStatus SensorServiceClass::Temp_Update(int16_t temp)
{
  tBleStatus ret;

  temp_data = temp;

  ret = aci_gatt_update_char_value(envSensServHandle, tempCharHandle, 0, 2,
                                   (uint8_t*)&temp);

  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating TEMP characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;

}

/**
 * @brief  Update pressure characteristic value.
 * @param  int32_t Pressure in tenths of Pascal
 * @retval tBleStatus Status
 */
tBleStatus SensorServiceClass::Press_Update(uint32_t press)
{
  tBleStatus ret;

  press_data = press;

  ret = aci_gatt_update_char_value(envSensServHandle, pressCharHandle, 0, 3,
                                   (uint8_t*)&press);

  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating TEMP characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;

}

/**
 * @brief  Update humidity characteristic value.
 * @param  uint16_thumidity RH (Relative Humidity) in hundredths of %
 * @retval tBleStatus      Status
 */
tBleStatus SensorServiceClass::Humidity_Update(uint16_t humidity)
{
  tBleStatus ret;

  hum_data = humidity;

  ret = aci_gatt_update_char_value(envSensServHandle, humidityCharHandle, 0, 2,
                                   (uint8_t*)&humidity);

  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating TEMP characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;

}

/**
 * @brief  Puts the device in connectable mode.
 *         If you want to specify a UUID list in the advertising data, those data can
 *         be specified as a parameter in aci_gap_set_discoverable().
 *         For manufacture data, aci_gap_update_adv_data must be called.
 * @param  None
 * @retval None
 */
/* Ex.:
 *
 *  tBleStatus ret;
 *  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};
 *  const uint8_t serviceUUIDList[] = {AD_TYPE_16_BIT_SERV_UUID,0x34,0x12};
 *  const uint8_t manuf_data[] = {4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x05, 0x02, 0x01};
 *
 *  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
 *                                 8, local_name, 3, serviceUUIDList, 0, 0);
 *  ret = aci_gap_update_adv_data(5, manuf_data);
 *
 */
void SensorServiceClass::setConnectable(void)
{
  tBleStatus ret;

  if(set_connectable){
    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);
    PRINTF("General Discoverable Mode.\n");

    ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                   strlen(dev_name), dev_name, 0, NULL, 0, 0);
    if (ret != BLE_STATUS_SUCCESS) {
      PRINTF("Error while setting discoverable mode (%d)\n", ret);
    }
    set_connectable = FALSE;
  }
}

int SensorServiceClass::isConnected(void)
{
  return connected;
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void SensorServiceClass::GAP_ConnectionComplete_CB(uint8_t addr[BDADDR_SIZE], uint16_t handle)
{
  connected = TRUE;
  connection_handle = handle;

#ifdef DEBUG
  PRINTF("Connected to device:");
  for(int i = 5; i > 0; i--){
    PRINTF("%02X-", addr[i]);
  }
  PRINTF("%02X\n", addr[0]);
#else
  UNUSED(addr);
#endif

}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None
 * @retval None
 */
void SensorServiceClass::GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  PRINTF("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;
}

/**
 * @brief  Read request callback.
 * @param  uint16_t Handle of the attribute
 * @retval None
 */
void SensorServiceClass::Read_Request_CB(uint16_t handle)
{
  if(handle == accCharHandle + 1){
    Acc_Update((AxesRaw_t*)&axes_data);
  }
  else if(handle == tempCharHandle + 1){
    Acc_Update((AxesRaw_t*)&axes_data); //FIXME: to overcome issue on Android App
                                        // If the user button is not pressed within
                                        // a short time after the connection,
                                        // a pop-up reports a "No valid characteristics found" error.
    Temp_Update(temp_data);
  }
  else if(handle == pressCharHandle + 1){
    Press_Update(press_data);
  }
  else if(handle == humidityCharHandle + 1){
    Humidity_Update(hum_data);
  }

  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void Sensor_HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;
  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type != HCI_EVENT_PKT)
    return;

  switch(event_pckt->evt){

  case EVT_DISCONN_COMPLETE:
    {
      SensorService.GAP_DisconnectionComplete_CB();
    }
    break;

  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;

      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (evt_le_connection_complete *)evt->data;
          SensorService.GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;

  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;
      switch(blue_evt->ecode){

      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          /* this callback is invoked when a GATT attribute is modified
          extract callback data and pass to suitable handler function */
          if (bnrg_expansion_board == IDB05A1) {
            evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
            SensorService.Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
          }
          else {
            evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
            SensorService.Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
          }
        }
        break;

      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req *)blue_evt->data;
          SensorService.Read_Request_CB(pr->attr_handle);
        }
        break;
      }
    }
    break;
  }
}

/**
 * @brief  Add a time service using a vendor specific profile
 * @param  None
 * @retval Status
 */
tBleStatus SensorServiceClass::Add_Time_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  /* copy "Timer service UUID" defined above to 'uuid' local variable */
  COPY_TIME_SERVICE_UUID(uuid);

  /*
   * now add "Time service" to GATT server, service handle is returned
   * via 'timeServHandle' parameter of aci_gatt_add_serv() API.
   * Please refer to 'BlueNRG Application Command Interface.pdf' for detailed
   * API description
  */
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7,
                          &timeServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  /*
   * now add "Seconds characteristic" to Time service, characteristic handle
   * is returned via 'secondsCharHandle' parameter of aci_gatt_add_char() API.
   * This characteristic is read only, as specified by CHAR_PROP_READ parameter.
   * Please refer to 'BlueNRG Application Command Interface.pdf' for detailed
   * API description
  */
  COPY_TIME_UUID(uuid);
  ret =  aci_gatt_add_char(timeServHandle, UUID_TYPE_128, uuid, 4,
                           CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
                           16, 0, &secondsCharHandle);

  if (ret != BLE_STATUS_SUCCESS) goto fail;

  COPY_MINUTE_UUID(uuid);
  /*
   * Add "Minutes characteristic" to "Time service".
   * This characteristic is readable as well as notifiable only, as specified
   * by CHAR_PROP_NOTIFY|CHAR_PROP_READ parameter below.
   */
  ret = aci_gatt_add_char(timeServHandle, UUID_TYPE_128, uuid, 4,
                          CHAR_PROP_NOTIFY|CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
                          16, 1, &minuteCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  PRINTF("Service TIME added. Handle 0x%04X, TIME Charac handle: 0x%04X\n",timeServHandle, secondsCharHandle);
  return BLE_STATUS_SUCCESS;

  /* return BLE_STATUS_ERROR if we reach this tag */
fail:
  PRINTF("Error while adding Time service.\n");
  return BLE_STATUS_ERROR ;
}

/**
 * @brief  Update seconds characteristic value of Time service.
 * @param  None
 * @retval Status
 */
tBleStatus SensorServiceClass::Seconds_Update(void)
{
  uint32_t val;
  tBleStatus ret;

  /* Obtain system tick value in milliseconds, and convert it to seconds. */
  val = millis();
  val = val/1000;

  /* create a time[] array to pass as last argument of aci_gatt_update_char_value() API*/
  const uint8_t time[4] = {(uint8_t)((val >> 24)&0xFF), (uint8_t)((val >> 16)&0xFF), (uint8_t)((val >> 8)&0xFF), (uint8_t)((val)&0xFF)};

  /*
   * Update value of "Seconds characteristic" using aci_gatt_update_char_value() API
   * Please refer to 'BlueNRG Application Command Interface.pdf' for detailed
   * API description
   */
  ret = aci_gatt_update_char_value(timeServHandle, secondsCharHandle, 0, 4,
                                   time);

  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating TIME characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Send a notification for a minute characteristic of time service.
 * @param  None
 * @retval Status
 */
tBleStatus SensorServiceClass::Minutes_Notify(void)
{
  uint32_t val;
  uint32_t minuteValue;
  tBleStatus ret;

  /* Obtain system tick value in milliseconds */
  val = millis();

  /* update "Minutes characteristic" value iff it has changed w.r.t. previous
   * "minute" value.
   */
  if((minuteValue=val/(60*1000))!=previousMinuteValue) {
    /* memorize this "minute" value for future usage */
    previousMinuteValue = minuteValue;

    /* create a time[] array to pass as last argument of aci_gatt_update_char_value() API*/
    const uint8_t time[4] = {(uint8_t)((minuteValue >> 24)&0xFF), (uint8_t)((minuteValue >> 16)&0xFF), (uint8_t)((minuteValue >> 8)&0xFF), (uint8_t)((minuteValue)&0xFF)};

  /*
   * Update value of "Minutes characteristic" using aci_gatt_update_char_value() API
   * Please refer to 'BlueNRG Application Command Interface.pdf' for detailed
   * API description
   */
    ret = aci_gatt_update_char_value(timeServHandle, minuteCharHandle, 0, 4,
                                     time);
    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error while updating TIME characteristic.\n") ;
      return BLE_STATUS_ERROR ;
    }
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Updates "Seconds and Minutes characteristics" values.
 * @param  None
 * @retval None
 */
void SensorServiceClass::Update_Time_Characteristics(void) {
  /* update "seconds and minutes characteristics" of time service */
  Seconds_Update();
  Minutes_Notify();
}

/*
 * @brief  Add LED button service using a vendor specific profile.
 * @param  None
 * @retval Status
 */
tBleStatus SensorServiceClass::Add_LED_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  /* copy "LED service UUID" defined above to 'uuid' local variable */
  COPY_LED_SERVICE_UUID(uuid);
  /*
   * now add "LED service" to GATT server, service handle is returned
   * via 'ledServHandle' parameter of aci_gatt_add_serv() API.
   * Please refer to 'BlueNRG Application Command Interface.pdf' for detailed
   * API description
  */
  ret = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7,
                          &ledServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  /* copy "LED button characteristic UUID" defined above to 'uuid' local variable */
  COPY_LED_UUID(uuid);
  /*
   * now add "LED button characteristic" to LED service, characteristic handle
   * is returned via 'ledButtonCharHandle' parameter of aci_gatt_add_char() API.
   * This characteristic is writable, as specified by 'CHAR_PROP_WRITE' parameter.
   * Please refer to 'BlueNRG Application Command Interface.pdf' for detailed
   * API description
  */
  ret =  aci_gatt_add_char(ledServHandle, UUID_TYPE_128, uuid, 4,
                           CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &ledButtonCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  PRINTF("Service LED BUTTON added. Handle 0x%04X, LED button Charac handle: 0x%04X\n",ledServHandle, ledButtonCharHandle);
  return BLE_STATUS_SUCCESS;

fail:
  PRINTF("Error while adding LED service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  This function is called attribute value corresponding to
 *         ledButtonCharHandle characteristic gets modified.
 * @param  Handle of the attribute
 * @param  Size of the modified attribute data
 * @param  Pointer to the modified attribute data
 * @retval None
 */
void SensorServiceClass::Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  UNUSED(data_length);
  UNUSED(att_data);

  /* If GATT client has modified 'LED button characteristic' value, toggle LED2 */
  if(handle == ledButtonCharHandle + 1){
      ledState = !ledState;
  }
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

 /**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
