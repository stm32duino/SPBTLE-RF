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
#include "beacon_service.h"
#include "bluenrg_utils.h"
#include "bluenrg_hal_aci.h"
#include "hci.h"
#include "hci_le.h"
#include "bluenrg_utils.h"
#include "stm32_bluenrg_ble.h"
#include "osal.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "SPBTLE_RF.h"

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup Beacon
 *  @{
 */

/** @defgroup SENSOR_SERVICE
 * @{
 */
extern uint8_t bnrg_expansion_board;
BeaconServiceClass BeaconService;

void Beacon_HCI_Event_CB(void *pckt);

/**
 * @brief  Begin a BeaconService
 * @param  addr       : MAC Address
 * @param  beaconType : Type of Beacon, UID or URL
 * @param  webURL     : URL for Beacon Type URL
 * @param  beaconID   : Instace name for UID
 * @param  nameSpace  : Namespace for UID
 * @retval BLE_STATUS_SUCCESS if success
 */
tBleStatus BeaconServiceClass::begin(uint8_t addr[BDADDR_SIZE], uint8_t beaconType,
                                     char* webURL, uint8_t *beaconID, uint8_t *nameSpace)
{
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

  uint8_t  hwVersion;
  uint16_t fwVersion;

  int ret;

  if(addr == NULL) {
    return BLE_STATUS_NULL_PARAM;
  }

  attach_HCI_CB(Beacon_HCI_Event_CB);

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

  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

  if (ret) {
    PRINTF("Setting Tx Power Level failed.\n");
  }

  /* Initialize beacon services */
  if ((beaconType & EDDYSTONE_UID_BEACON_TYPE) &&
      (beaconID != NULL) && (nameSpace != NULL))
  {
    if((beaconID != NULL) && (nameSpace != NULL)) {
      ret = EddystoneUID_Start(beaconID, nameSpace);
    } else {
      ret = BLE_STATUS_NULL_PARAM;
    }
  }
  if ((beaconType & EDDYSTONE_URL_BEACON_TYPE) && (webURL != NULL))
  {
    if(webURL != NULL) {
      ret = EddystoneURL_Start((uint8_t*)webURL);
    } else {
      ret = BLE_STATUS_NULL_PARAM;
    }
  }

  return ret;
}

tBleStatus BeaconServiceClass::begin(uint8_t addr[BDADDR_SIZE], char* webURL)
{
  return begin(addr, URL_TYPE, webURL, NULL, NULL);
}

tBleStatus BeaconServiceClass::begin(uint8_t addr[BDADDR_SIZE],
                                     uint8_t *beaconID, uint8_t *nameSpace)
{
  return begin(addr, UID_TYPE, NULL, beaconID, nameSpace);
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void Beacon_HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;

  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type != HCI_EVENT_PKT)
  {
    return;
  }

  switch(event_pckt->evt)
  {

  case EVT_DISCONN_COMPLETE:
    {
      ;
    }
    break;

  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;

      switch(evt->subevent)
      {
      case EVT_LE_CONN_COMPLETE:
        {
          ;
        }
        break;
      }
    }
    break;

  case EVT_VENDOR:
    {
      ;
    }
    break;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
