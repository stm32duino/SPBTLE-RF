/**
  ******************************************************************************
  * @file    SPBTLE_RF.cpp
  * @author  Wi6Labs
  * @version V1.0.0
  * @date    18-August-2017
  * @brief
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include <SPBTLE_RF.h>
#include "Arduino.h"
#include "hci.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_interface.h"
#include "debug.h"
#include "gp_timer.h"
#include "hal.h"

#define HEADER_SIZE 5
#define MAX_BUFFER_SIZE 255

uint8_t bnrg_expansion_board;

void (*HCI_callback)(void *);

static SPIClass *SPIBTLE;
static uint8_t csPin;
static uint8_t spiIRQPin;
static uint8_t resetPin;
static uint8_t ledPin;

SPBTLERFClass::SPBTLERFClass(SPIClass *SPIx, uint8_t cs, uint8_t spiIRQ,
                              uint8_t reset, uint8_t led)
{
  SPIBTLE = SPIx;
  csPin = cs;
  spiIRQPin = spiIRQ;
  resetPin = reset;
  ledPin = led;
}

SPBTLERF_state_t SPBTLERFClass::begin(void)
{
  /* Initialize the BlueNRG SPI driver */
  // Configure SPI and CS pin
  SPIBTLE->begin();
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  // Enable SPI EXTI interrupt
  attachInterrupt(spiIRQPin, SPI_EXTI_Callback, RISING);

  // Configure Reset pin
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);  // Keep module in reset state

  /* Initialize the BlueNRG HCI */
  HCI_Init();

  /* Reset BlueNRG hardware */
  BlueNRG_RST();

  // If a LED is associated, enable it to indicate the module is ready
  if(ledPin != 0xFF) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
  }

  return SPBTLERF_OK;
}

void SPBTLERFClass::end(void)
{
  digitalWrite(resetPin, LOW);
  detachInterrupt(spiIRQPin);
  SPIBTLE->end();

  if(ledPin != 0xFF) {
    digitalWrite(ledPin, HIGH);
    // Allows to not enable the WiFi LED on board Discovery L475VG IOT
    pinMode(ledPin, INPUT);
  }
}

void SPBTLERFClass::update(void)
{
  HCI_Process();
}

/**
 * @brief  This function is a utility to print the log time
*          in the format HH:MM:SS:MSS (DK GUI time format)
 * @param  None
 * @retval None
 */
#ifdef PRINT_CSV_FORMAT
void print_csv_time(void){
 uint32_t ms = HAL_GetTick();
 PRINT_CSV("%02d:%02d:%02d.%03d", ms/(60*60*1000)%24, ms/(60*1000)%60, (ms/1000)%60, ms%1000);
}
#endif //PRINT_CSV_FORMAT

/**
 * @brief  Writes data to a serial interface.
 * @param  data1   :  1st buffer
 * @param  data2   :  2nd buffer
 * @param  n_bytes1: number of bytes in 1st buffer
 * @param  n_bytes2: number of bytes in 2nd buffer
 * @retval None
 */
void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1,
                       int32_t n_bytes2)
{
  struct timer t;

  Timer_Set(&t, CLOCK_SECOND/5);

  #ifdef PRINT_CSV_FORMAT
  print_csv_time();
  for (int i=0; i<n_bytes1; i++) {
    PRINT_CSV(" %02x", ((uint8_t *)data1)[i]);
  }
  for (int i=0; i<n_bytes2; i++) {
    PRINT_CSV(" %02x", ((uint8_t *)data2)[i]);
  }
  PRINT_CSV("\n");
  #endif

  while(1){
    if(BlueNRG_SPI_Write((uint8_t *)data1,(uint8_t *)data2, n_bytes1, n_bytes2)==0) break;
    if(Timer_Expired(&t)){
      break;
    }
  }
}

/**
 * @brief  Resets the BlueNRG.
 * @param  None
 * @retval None
 */
void BlueNRG_RST(void)
{
  digitalWrite(resetPin, LOW);
  delay(5);
  digitalWrite(resetPin, HIGH);
  delay(5);
}

/**
 * @brief  Reports if the BlueNRG has data for the host micro.
 * @param  None
 * @retval 1 if data are present, 0 otherwise
 */
// FIXME: find a better way to handle this return value (bool type? TRUE and FALSE)
uint8_t BlueNRG_DataPresent(void)
{
  PinName p = digitalPinToPinName(spiIRQPin);
  /* Can't use digitalWrite() because pin is not configured with pinMode()
  function so we must call directly HAL function */
  if (HAL_GPIO_ReadPin(get_GPIO_Port(STM_PORT(p)), STM_GPIO_PIN(p)) == GPIO_PIN_SET)
      return 1;
  else
      return 0;
} /* end BlueNRG_DataPresent() */

/**
 * @brief  Activate internal bootloader using pin.
 * @param  None
 * @retval None
 */
void BlueNRG_HW_Bootloader(void)
{
  Disable_SPI_IRQ();

  pinMode(spiIRQPin, OUTPUT);
  digitalWrite(spiIRQPin, HIGH);

  BlueNRG_RST();
  Enable_SPI_IRQ();
}

/**
 * @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
 * @param  buffer   : Buffer where data from SPI are stored
 * @param  buff_size: Buffer size
 * @retval int32_t  : Number of read bytes
 */
int32_t BlueNRG_SPI_Read_All(uint8_t *buffer,
                            uint8_t buff_size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  uint8_t char_ff = 0xff;
  volatile uint8_t read_char;

  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  /* CS reset */
  digitalWrite(csPin, LOW);

  if(SPIBTLE == NULL)
  return 0;

  /* Read the header */
  SPIBTLE->transfer(header_master, header_slave, HEADER_SIZE);

  if (header_slave[0] == 0x02) {
    /* device is ready */
    byte_count = (header_slave[4]<<8)|header_slave[3];

    if (byte_count > 0) {

      /* avoid to read more data that size of the buffer */
      if (byte_count > buff_size){
        byte_count = buff_size;
      }

      for (len = 0; len < byte_count; len++){
        read_char = SPIBTLE->transfer(char_ff);
        buffer[len] = read_char;
      }

    }
  }
  /* Release CS line */
  digitalWrite(csPin, HIGH);

  // Add a small delay to give time to the BlueNRG to set the IRQ pin low
  // to avoid a useless SPI read at the end of the transaction
  for(volatile int i = 0; i < 2; i++)__NOP();

  #ifdef PRINT_CSV_FORMAT
  if (len > 0) {
    print_csv_time();
    for (int i=0; i<len; i++) {
      PRINT_CSV(" %02x", buffer[i]);
    }
    PRINT_CSV("\n");
  }
  #endif /* PRINT_CSV_FORMAT */

  return len;
}

/**
 * @brief  Writes data from local buffer to SPI.
 * @param  data1    : First data buffer to be written
 * @param  data2    : Second data buffer to be written
 * @param  Nb_bytes1: Size of first data buffer to be written
 * @param  Nb_bytes2: Size of second data buffer to be written
 * @retval Number of read bytes
 */
int32_t BlueNRG_SPI_Write(uint8_t* data1,
                         uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2)
{
  int32_t result = 0;

  unsigned char header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  unsigned char header_slave[HEADER_SIZE]  = {0xaa, 0x00, 0x00, 0x00, 0x00};

  unsigned char read_char_buf[MAX_BUFFER_SIZE];

  Disable_SPI_IRQ();

  /* CS reset */
  digitalWrite(csPin, LOW);

  if(SPIBTLE == NULL)
    return -1;

  /* Exchange header */
  SPIBTLE->transfer(header_master, header_slave, HEADER_SIZE);

  if (header_slave[0] == 0x02) {
    /* SPI is ready */
    if (header_slave[1] >= (Nb_bytes1+Nb_bytes2)) {

      /*  Buffer is big enough */
      if (Nb_bytes1 > 0) {
        SPIBTLE->transfer(data1, read_char_buf, Nb_bytes1);
      }
      if (Nb_bytes2 > 0) {
        SPIBTLE->transfer(data2, read_char_buf, Nb_bytes2);
      }

    } else {
      /* Buffer is too small */
      result = -2;
    }
  } else {
    /* SPI is not ready */
    result = -1;
  }

  /* Release CS line */
  digitalWrite(csPin, HIGH);

  Enable_SPI_IRQ();

  return result;
}

/**
 * @brief  Enable SPI IRQ.
 * @param  None
 * @retval None
 */
void Enable_SPI_IRQ(void)
{
  attachInterrupt(spiIRQPin, SPI_EXTI_Callback, RISING);
}

/**
 * @brief  Disable SPI IRQ.
 * @param  None
 * @retval None
 */
void Disable_SPI_IRQ(void)
{
  detachInterrupt(spiIRQPin);
}

/**
 * @brief  Clear EXTI (External Interrupt) line for SPI IRQ.
 * @Note   This function is kept for compatibility with native source code.
 * @param  None
 * @retval None
 */
void Clear_SPI_EXTI_Flag(void)
{
  //Empty function.
}

/**
 * @brief  This function allows to attach a HCI callback from a profil.
 * @param  callback: function pointer of the callback to attach.
 * @retval None
 */
void attach_HCI_CB(void (*callback)(void *pckt))
{
  HCI_callback = callback;
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  HCI_callback(pckt);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
