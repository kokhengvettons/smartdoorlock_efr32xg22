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
#include "cpt212b.h"
#include "i2cspm.h"
#include "udelay.h"

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
  UDELAY_Delay(50);

  // disable reset
  GPIO_PinOutSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_RESET_PIN);
  UDELAY_Delay(15*1000);
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
  // The delay loop must be calibrated at lease once before use.
  UDELAY_Calibrate();

  //Initialise I2C sensor
  GPIO_PinModeSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_ENABLE_PIN, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_RESET_PIN, gpioModePushPull, 1);
  Cpt212b_SensorReset();
  Cpt212b_SensorEnable(true);
  
  // validate configuration profile
  errorcode_t err = Cpt212b_ConfigurationProfileValidation(I2C0);

  if (err == bg_err_success)
  {
    UDELAY_Delay(10);

    // enter sensing mode from configuration loading mode
    err = Cpt212b_EnterSenseMode(I2C0);
    UDELAY_Delay(10);
  }

  return err;
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

  seq.addr  = CPT212B_CONF_MODE_ADDR;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = 0x08;
  i2c_write_data[1] = 0x01;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone) {
    return (int) ret;
  }

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

  seq.addr  = CPT212B_CONF_MODE_ADDR;
  seq.flags = I2C_FLAG_READ;
  /* Select command to issue */
  seq.buf[0].data = i2c_read_data;
  seq.buf[0].len  = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone) {
    return((int) ret);
  }

  if (i2c_read_data[0] == CPT212B_CONF_PROFILE_VALID)
  {
    return bg_err_success;
  }
  else
  {
    return bg_err_application_invalid_configuration_profile;
  }
}
/** @endcond */

/** @} (end group cpt212b) */
/** @} (end group kitdrv) */
