/***************************************************************************//**
 * @brief app.h
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H_
#define APP_H_

#include "gecko_configuration.h"

/* DEBUG_LEVEL is used to enable/disable debug prints. Set DEBUG_LEVEL to 1 to enable debug prints */
#define DEBUG_LEVEL 0

/* Set this value to 1 if you want to disable deep sleep completely */
#define DISABLE_SLEEP 0

#if DEBUG_LEVEL
#include "retargetserial.h"
#include <stdio.h>
#endif

#if DEBUG_LEVEL
#define initLog()     RETARGET_SerialInit()
#define flushLog()    RETARGET_SerialFlush()
#define printLog(...) printf(__VA_ARGS__)
#else
#define initLog()
#define flushLog()
#define printLog(...)
#endif

/***************************************************************************************************
 * External signal indication
 **************************************************************************************************/
#define EXT_SIGNAL_I2C_INTERRUP_FLAG     1
#define EXT_SIGNAL_DOOR_SENSOR_FLAG      2
#define EXT_SIGNAL_DOOR_BUTTON_FLAG      3
/***************************************************************************************************
 * Software timer handler
 **************************************************************************************************/
#define SOFT_TIMER_MOTOR_PWM_HANDLER     0
#define SOFT_TIMER_DOOR_SENSOR_HANDLER   1
#define SOFT_TIMER_DOOR_BUTTON_HANDLER   2

#define DOOR_SENSOR_INTERVAL_MS          100     // 100ms
#define DOOR_BUTTON_DEBOUNCE_INTERVAL_MS 200     // 200ms

typedef enum {DOOR_UNLOCK = 0, DOOR_LOCK = 1} door_lock_TypeDef;
typedef enum {DOOR_OPEN = 1, DOOR_CLOSED = 0} door_status_TypeDef;


/* Main application */
void appMain(const gecko_configuration_t *pconfig);
void initMainBoardGPIO(void);

void evt_door_lock(uint8_t data[], uint16_t length);
void evt_door_sensor_send_notification(void);
void evt_door_button_ext_signal(void);

// void evt_set_leds(uint8_t data);
// uint8_t evt_get_leds(void);
#endif
