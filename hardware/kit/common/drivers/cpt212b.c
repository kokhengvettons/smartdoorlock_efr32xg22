/***************************************************************************//**
 * @file
 * @brief Driver for the cpt212b capacitive touch keypad
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stddef.h>
#include <string.h>
#include "cpt212b.h"
#include "i2cspm.h"
#include "ustimer.h"
#include "cpt212b_a01_gm_init.h"

#include "stddef.h"

/***************************************************************************//**
 * @addtogroup kitdrv
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup cpt212B
 * @brief Silicon Labs CPT212B Capacitive touch keypad I2C driver.
 * @details
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
#define FEATURE_FLASH_NEW_CONFIGURATION_PROFILE		0

static uint8_t event_packet_counter; 

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/** @endcond */

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief
 *  Reset I2C sensor (active low) for CPT212B Capacitive touch keypad
 * @param[in] void
 * @return
 *   Return void.
 *****************************************************************************/
void Cpt212b_SensorReset(void)
{
  // enable (active low) to reset
  GPIO_PinOutClear(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_RESET_PIN);
  USTIMER_Delay(1000);

  // disable reset
  GPIO_PinOutSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_RESET_PIN);
  USTIMER_Delay(1000*15);
}

/**************************************************************************//**
 * @brief
 *  Enable I2C sensor (active low) for CPT212B Capacitive touch keypad
 * @param[in] bEnable
 *  flag to enable I2C sensor
 * @return
 *   Return void.
 *****************************************************************************/
void Cpt212b_SensorEnable(bool bEnable)
{
  if (bEnable)
  {
    GPIO_PinOutClear(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_ENABLE_PIN);
  }
  else
  {
    GPIO_PinOutSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_ENABLE_PIN);
  }
}

/**************************************************************************//**
 * @brief
 *  Initialize CPT212B Capacitive touch keypad 
 * @param[in] void
 * @return
 *   Return results
 *****************************************************************************/
errorcode_t InitCpt212b(void)
{
  errorcode_t err = bg_err_success;

  // Initialization of USTIMER driver
  USTIMER_Init();

  //Initialize I2C sensor
  GPIO_PinModeSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_ENABLE_PIN, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_RESET_PIN, gpioModePushPull, 1);
  Cpt212b_SensorReset();
  Cpt212b_SensorEnable(true);

  // packet counter set to 0 as starting send event packet
  event_packet_counter = 0;

  #if FEATURE_FLASH_NEW_CONFIGURATION_PROFILE
    err = FlashNewConfigurationProfile();
  #endif
  
  if (err == bg_err_success)
  {
    // validate configuration profile
    if ((err = Cpt212b_ConfigurationProfileValidation(I2C0)) == bg_err_success)
    {
      Cpt212b_SensorEnable(false);
      USTIMER_Delay(1000*10);
      Cpt212b_SensorEnable(true);

      // enter sensing mode from configuration loading mode
      err = Cpt212b_EnterSenseMode(I2C0);
      USTIMER_Delay(1000*10);
    }
  }

  return err;
}

/**************************************************************************//**
 * @brief
 *  Event Packet Counter handler 
 * @param[in] bReset
 *   Reset the packet counter if set to true
 * @return void
 *****************************************************************************/
void PacketCounterHandler(bool bReset)
{
  if (bReset == true || event_packet_counter >= 15)
    event_packet_counter = 0;
  else
    event_packet_counter++;
}

/**************************************************************************//**
 * @brief
 *  Flash a new configuration profile into CPT212B Capacitive touch keypad
 * @param[in] void
 * @return
 *   Return results
 *****************************************************************************/
errorcode_t FlashNewConfigurationProfile(void)
{
  errorcode_t err = bg_err_success;

  // step 1: Host sends the configuration loading unlock sequence.
  if ((err = Cpt212b_ConfigurationUnlock(I2C0)) == bg_err_success)
  {
    USTIMER_Delay(1000);

    // step 2: Host sends config erase command, which erases the configuration profile.
    if ((err = Cpt212b_ConfigurationErase(I2C0)) == bg_err_success)
    {
      USTIMER_Delay(1000*50);

      // step 3: Host sends bytes 0-7 of configuration profile in a write config command,
      //         repeats the step until end of profile. 
      //         Note: The host should pad the last [write bytes] command up to a payload 
      //               of 8 bytes, with 0xFF used as padding.
      if ((err = WriteConfigurationProfile()) == bg_err_success)
      {
        USTIMER_Delay(1000*10);

        // step 4: host sends write CRC Command.
        uint8_t crc2 = CPT212B_A01_GM_DEFAULT_CONFIG_CHECKSUM >> 8;
        uint8_t crc1 = CPT212B_A01_GM_DEFAULT_CONFIG_CHECKSUM & 0xFF;       
        err = Cpt212b_ConfigurationWriteCRC(I2C0, crc2, crc1);
        USTIMER_Delay(1000*10);
      }
    }
  }

  if (err != bg_err_success)
    return bg_err_application_write_configuration_profile_failed;
  else
    return err;
}

/**************************************************************************//**
 * @brief
 *  Write configuration profile into non-volatile memory 
 *  loading mode
 * @param[in] void
 * @return
 *   Returns results.
 *****************************************************************************/
errorcode_t WriteConfigurationProfile(void)
{  
  config_profile_t pf = CPT212B_A01_GM_DEFAULT_CONFIG;
  errorcode_t err = bg_err_success;

  // check configuration profile file size
  if ( sizeof(pf) >= CPT212B_MAX_WRITE_CONF_LEN || sizeof(pf) <= 0)
  {
    return bg_err_application_wrong_configuration_profile_length;
  }

  // copy configuration profile into buf
  uint8_t buf[CPT212B_MAX_WRITE_CONF_LEN];
  memcpy(&buf[0], pf.reserved_0, sizeof(pf));

  //prepare to write 8-bytes payloads
  uint8_t bytes[CPT212B_WRITE_CONF_PAYLOADS];
  for (int idx = 0; idx < sizeof(pf); idx +=CPT212B_WRITE_CONF_PAYLOADS)
  {    
    int byte_len = CPT212B_WRITE_CONF_PAYLOADS;
    memset(bytes, 0, sizeof(bytes));

    // if write bytes less than 8-bytes payloads, then use oxff as padding for remaining bytes    
    if (idx + CPT212B_WRITE_CONF_PAYLOADS > sizeof(pf))
    {
      byte_len = sizeof(pf) - idx;
      memset(bytes, 0xff, sizeof(bytes));
    }

    memcpy(bytes, &buf[idx], sizeof(uint8_t)*byte_len);
    err = Cpt212b_ConfigurationWrite(I2C0, bytes, CPT212B_WRITE_CONF_PAYLOADS);
    if (err != bg_err_success)
      return err;

    USTIMER_Delay(1000*5);
  }

  return bg_err_success;
}

/**************************************************************************//**
 * @brief
 *  Enter sensing mode using the mode selection command from configuration 
 *  loading mode
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @return
 *   Returns number of bytes read/write on success. Otherwise returns error codes.
 *****************************************************************************/
errorcode_t Cpt212b_EnterSenseMode(I2C_TypeDef *i2c)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  uint8_t                    i2c_write_data[2];

  i2c_write_data[0] = CPT212B_CONF_MODE_SELECT;
  //i2c_write_data[0] = (event_packet_counter << 4) | 0x08;
  i2c_write_data[1] = 0x01;
  seq.addr          = CPT212B_CONF_MODE_ADDR;
  seq.flags         = I2C_FLAG_WRITE;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;
  
  /* don't require to resend a new start address to read data */
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
    return (int) ret;

  // The event packet counter is reset to 0 upon entrance to sensing mode
  PacketCounterHandler(true);

  return bg_err_success;
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
/**************************************************************************//**
 * @brief
 *  Configuration Profile Validity Check Command.
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @return
 *   Returns number of bytes read/write on success. Otherwise returns error codes 
 *****************************************************************************/
errorcode_t Cpt212b_ConfigurationProfileValidation(I2C_TypeDef *i2c)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];

  seq.buf[0].data = i2c_read_data;
  seq.buf[0].len  = 1;

  seq.addr        = CPT212B_CONF_MODE_ADDR;
  seq.flags       = I2C_FLAG_READ;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone) 
    return((int) ret);  

  if (i2c_read_data[0] == CPT212B_CONF_PROFILE_VALID)
    return bg_err_success;
  
  return bg_err_application_invalid_configuration_profile;
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
/**************************************************************************//**
 * @brief
 *  Read keypad events.
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @param[in] uint8_t data[] 
 *   Return data array
 * @param[in] length
 *   The number of return data array length
 * @return
 *   Returns number of bytes read/write on success. Otherwise returns error codes 
 *****************************************************************************/
errorcode_t Cpt212b_ReadKeypadEvent(I2C_TypeDef *i2c, uint8_t data[], uint16_t length)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[3];

  seq.buf[0].data = i2c_read_data;
  seq.buf[0].len  = 3;

  seq.addr        = CPT212B_SENSE_MODE_ADDR;
  seq.flags       = I2C_FLAG_READ;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
    return((int) ret);

  // copy I2C data into return data array
  memcpy(data, i2c_read_data, sizeof(uint8_t)*length);

  // sync up packet counter
  event_packet_counter = i2c_read_data[0] >> 4;

  return bg_err_success;
}

/**************************************************************************//**
 * @brief
 *  Enter configuration loading unlock sequence  
 *  loading mode
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @return
 *   Returns number of bytes read/write on success. Otherwise returns error codes.
 *****************************************************************************/
errorcode_t Cpt212b_ConfigurationUnlock(I2C_TypeDef *i2c)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[3];
  uint8_t                    i2c_write_data[3];

  i2c_write_data[0] = CPT212B_CONF_UNLOCK;
  //i2c_write_data[0] = (event_packet_counter << 4) | CPT212B_CONF_UNLOCK;
  i2c_write_data[1] = 0xA5;
  i2c_write_data[2] = 0xF1;

  seq.addr          = CPT212B_CONF_MODE_ADDR;
  seq.flags         = I2C_FLAG_WRITE;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 3;
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
    return (int) ret;

  PacketCounterHandler(false);

  return bg_err_success;
}

/**************************************************************************//**
 * @brief
 *  Erase the configure profile
 *  loading mode
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @return
 *   Returns number of bytes read/write on success. Otherwise returns error codes.
 *****************************************************************************/
errorcode_t Cpt212b_ConfigurationErase(I2C_TypeDef *i2c)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[1];

  i2c_write_data[0]  = CPT212B_CONF_ERASE;
  //i2c_write_data[0]  = (event_packet_counter << 4) | CPT212B_CONF_ERASE;

  seq.addr          = CPT212B_CONF_MODE_ADDR;
  seq.flags         = I2C_FLAG_WRITE;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
    return (int) ret;

  PacketCounterHandler(false);

  return bg_err_success;
}
/** @endcond */

/**************************************************************************//**
 * @brief
 *  Write the configure profile
 *  loading mode
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @return
 *   Returns number of bytes read/write on success. Otherwise returns error codes.
 *****************************************************************************/
errorcode_t Cpt212b_ConfigurationWrite(I2C_TypeDef *i2c, uint8_t data[], uint16_t length)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[1 + CPT212B_WRITE_CONF_PAYLOADS];

  i2c_write_data[0] = CPT212B_CONF_WRITE;
  //i2c_write_data[0] = (event_packet_counter << 4) |  CPT212B_CONF_WRITE;
  memcpy(&i2c_write_data[1], &data[0], sizeof(uint8_t) * CPT212B_WRITE_CONF_PAYLOADS);

  seq.addr        = CPT212B_CONF_MODE_ADDR;
  seq.flags       = I2C_FLAG_WRITE;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1 + CPT212B_WRITE_CONF_PAYLOADS;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
    return (int) ret;
  
  PacketCounterHandler(false);

  return bg_err_success;
}

/**************************************************************************//**
 * @brief
 *  Write the configure profile
 *  loading mode
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @param[in] crc_1
 *   LSB byte of CRC
 * @param[in] crc_2
 *   MSB byte of CRC
 *   Returns number of bytes read/write on success. Otherwise returns error codes.
 *****************************************************************************/
errorcode_t Cpt212b_ConfigurationWriteCRC(I2C_TypeDef *i2c, uint8_t crc_1, uint8_t crc_2)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[3];
  uint8_t                    i2c_write_data[3];


  i2c_write_data[0] = CPT212B_CONF_WRITE_CRC;
  //i2c_write_data[0] = (event_packet_counter << 4) | CPT212B_CONF_WRITE_CRC;
  i2c_write_data[1] = crc_1;
  i2c_write_data[2] = crc_2;

  seq.addr          = CPT212B_CONF_MODE_ADDR;
  seq.flags         = I2C_FLAG_WRITE;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 3;
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
    return (int) ret;

  PacketCounterHandler(false);

  return bg_err_success;
}

/** @} (end group cpt212b) */
/** @} (end group kitdrv) */
