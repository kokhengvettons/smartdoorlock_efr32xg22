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

#include "battery.h"
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
#define EXT_SIGNAL_I2C_INTERRUP_FLAG        1
#define EXT_SIGNAL_DOOR_SENSOR_FLAG         2
#define EXT_SIGNAL_DOOR_BUTTON_FLAG         3

/***************************************************************************************************
 * Software timer
 **************************************************************************************************/
#define SOFT_TIMER_MOTOR_PWM_HANDLER        0
#define SOFT_TIMER_DOOR_SENSOR_HANDLER      1
#define SOFT_TIMER_DOOR_BUTTON_HANDLER      2
#define SOFT_TIMER_DOOR_ALARM_ON_HANDLER    3
#define SOFT_TIMER_DOOR_ALARM_OFF_HANDLER   4
#define SOFT_TIMER_MOTOR_ADC_MEAS_HANDLER   5

#define DOOR_SENSOR_INTERVAL_MS             100     //  100ms
#define DOOR_BUTTON_DEBOUNCE_INTERVAL_MS    500     //  500ms
#define DOOR_ALARM_DEFAULT_INTERVAL_MS      10000   //  10s
#define DOOR_ALARM_OFF_INTERVAL_MS          200     //  200ms
#define MOTOR_ADC_MEAS_INTERVAL_MS          10       // 10ms

/***************************************************************************************************
 * Interrupt source
 **************************************************************************************************/
#define INT_SOURCE_KEYPAD_EVENT             1
#define INT_SOURCE_DOOR_SENSOR              2
#define INT_SOURCE_DOOR_OPEN_BUTTON         3

#define PS_KEY_BASE                         0x4000

typedef enum {DOOR_UNLOCK = 0, DOOR_LOCK = 1} door_lock_TypeDef;
typedef enum {DOOR_OPEN = 1, DOOR_CLOSED = 0} door_status_TypeDef;
typedef enum {ALARM_OFF = 0, ALARM_ON = 1} door_alarm_status_TypedDef;

/* Main application */
void appMain(const gecko_configuration_t *pconfig);
void initMainBoardGPIO(void);

void evt_door_lock(uint8_t data[], uint16_t length);
void evt_door_sensor_send_notification(void);
void evt_door_alarm_send_notification(door_alarm_status_TypedDef alarm_status);
void evt_door_button_ext_signal(void);
void evt_motor_battery_measurement(battery_measure_TypeDef measure_type);

// void evt_set_leds(uint8_t data);
// uint8_t evt_get_leds(void);
#endif
