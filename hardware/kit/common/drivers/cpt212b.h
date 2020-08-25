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

#ifndef __CPT212B_H
#define __CPT212B_H

#include "bg_errorcodes.h"
#include "em_device.h"
#include <stdbool.h>

/***************************************************************************//**
 * @addtogroup kitdrv
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup cpt212b
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** I2C mode selection address for cpt212b */
#define CPT212B_CONF_MODE_ADDR      0xC0
#define CPT212B_SENSE_MODE_ADDR     0xE0

/** I2C sense mode events type for cpt212b */
#define CPT212B_SENSE_EVENT_TOUCH       0x00
#define CPT212B_SENSE_EVENT_RELEASE     0x01
#define CPT212B_SENSE_EVENT_PROXIMITY   0x03

/** I2C configuration loading mode for cpt212b */
#define CPT212B_CONF_MODE_SELECT        0x08
#define CPT212B_CONF_UNLOCK             0x09
#define CPT212B_CONF_ERASE              0x0A
#define CPT212B_CONF_WRITE              0x0B
#define CPT212B_CONF_WRITE_CRC          0x0C

/** I2C touch events for cpt212b */
#define CPT212B_TOUCH_EVENT_CS0         0x00
#define CPT212B_TOUCH_EVENT_CS1         0x01
#define CPT212B_TOUCH_EVENT_CS2         0x02
#define CPT212B_TOUCH_EVENT_CS3         0x03
#define CPT212B_TOUCH_EVENT_CS4         0x04
#define CPT212B_TOUCH_EVENT_CS5         0x05
#define CPT212B_TOUCH_EVENT_CS6         0x06
#define CPT212B_TOUCH_EVENT_CS7         0x07
#define CPT212B_TOUCH_EVENT_CS8         0x08
#define CPT212B_TOUCH_EVENT_CS9         0x09
#define CPT212B_TOUCH_EVENT_CS10        0x0A
#define CPT212B_TOUCH_EVENT_CS11        0x0B

/** I2C configuration profile validity check results for cpt212b */
#define CPT212B_CONF_PROFILE_VALID      0x80
#define CPT212B_CONF_PROFILE_INVALID    0x01

/** I2C Hardware for cpt212b **/
#define CPT212B_I2CSENSOR_ENABLE_PIN    0x01
#define CPT212B_I2CSENSOR_RESET_PIN     0x03
#define CPT212B_I2CSENSOR_CONTROL_PORT  gpioPortC

#define CPT212B_I2CSENSOR_DATA_PORT     gpioPortB
#define CPT212B_I2CSENSOR_SCL           0x02
#define CPT212B_I2CSENSOR_SDA           0x03

/** configuration profile mode*/
#define CPT212B_WRITE_CONF_PAYLOADS     0x08
#define CPT212B_MAX_WRITE_CONF_LEN      512

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/
void initGpioI2c(void);
void cpt212b_SensorReset(void);
void cpt212b_SensorEnable(bool bEnable);
void PacketCounterHandler(bool bReset);
errorcode_t initcpt212b(void);
errorcode_t flashNewConfigurationProfile(void);
errorcode_t writeConfigurationProfile(void);
errorcode_t cpt212b_EnterSenseMode(I2C_TypeDef *i2c);
errorcode_t cpt212b_ConfigurationProfileValidation(I2C_TypeDef *i2c);
errorcode_t cpt212b_ConfigurationUnlock(I2C_TypeDef *i2c);
errorcode_t cpt212b_ConfigurationErase(I2C_TypeDef *i2c);
errorcode_t cpt212b_ConfigurationWrite(I2C_TypeDef *i2c, uint8_t data[], uint16_t length);
errorcode_t cpt212b_ConfigurationWriteCRC(I2C_TypeDef *i2c, uint8_t crc_1, uint8_t crc_2);
errorcode_t cpt212b_ReadKeypadEvent(I2C_TypeDef *i2c, uint8_t data[], uint16_t length);

#ifdef __cplusplus
}
#endif

/** @} (end group cpt212b) */
/** @} (end group kitdrv) */

#endif /* __CPT212B_H */
