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
#define EXT_SIGNAL_I2C_INTERRUP_FLAG        1
#define EXT_SIGNAL_DOOR_SENSOR_FLAG         2
#define EXT_SIGNAL_DOOR_BUTTON_FLAG         3

/***************************************************************************************************
 * Software timer
 **************************************************************************************************/
#define SOFT_TIMER_MOTOR_PWM_HANDLER        1
#define SOFT_TIMER_DOOR_SENSOR_HANDLER      2
#define SOFT_TIMER_DOOR_BUTTON_HANDLER      3
#define SOFT_TIMER_DOOR_AUTO_LOCK_HANDLER   4
#define SOFT_TIMER_SENSOR_ALARM_HANDLER     5
#define SOFT_TIMER_MOTOR_ADC_MEAS_HANDLER   6
#define SOFT_TIMER_BATTERY_MEAS_HANDLER     7
#define SOFT_TIMER_FAC_RESET_HANDLER        8
#define SOFT_TIMER_SEND_NOTIF_HANDLER       9

#define DOOR_SENSOR_INTERVAL_MS             100     //  100ms
#define DOOR_BUTTON_DEBOUNCE_INTERVAL_MS    500     //  500ms
#define MOTOR_ADC_MEAS_INTERVAL_MS          10      //  10ms
#define BATTERY_ADC_MEAS_INTERVAL_MS        60000   //  60s
#define BATTERY_SEND_NOTIF_INTERVAL_MS      1000    //  1s
#define FACTORY_RESET_INTERVAL_MS           2000    //  2s

#define MAX_DOOR_AUTO_LOCK_S                300     //  300s
#define MIN_DOOR_AUTO_LOCK_S                60      //  60s
#define MAX_DOOR_SENSOR_ALARM_S             300     //  300s
#define MIN_DOOR_SENSOR_ALARM_S             30      //  30s
/***************************************************************************************************
 * Interrupt source
 **************************************************************************************************/
#define INT_SOURCE_KEYPAD_EVENT             1
#define INT_SOURCE_DOOR_SENSOR              2
#define INT_SOURCE_DOOR_OPEN_BUTTON         3

#define PS_KEY_BASE                         0x4000

/***************************************************************************************************
 * Door status
 **************************************************************************************************/
typedef enum {DOOR_UNLOCK = 0, DOOR_LOCK = 1} door_lock_TypeDef;
typedef enum {DOOR_OPEN = 1, DOOR_CLOSED = 0} door_status_TypeDef;
typedef enum {DISABLE_AUTO_LOCK = 0x00, ENABLE_AUTO_LOCK = 0x01} enable_auto_lock_TypeDef;

/***************************************************************************************************
 * Special command
 **************************************************************************************************/
enum special_cmd_error_code
{
	special_cmd_success = 0x00,
	special_cmd_err_write_profile = 0xF0,
    special_cmd_err_hardware_keypad = 0xF1,
    special_cmd_err_hardware_battery = 0xF2,
    special_cmd_err_hardware_door_motor = 0xF3,
	special_cmd_unsupported_cmd = 0xFE,
	special_cmd_unknown_err = 0xFF,
};
/***************************************************************************************************
 * IADC Battery measurement
 **************************************************************************************************/
#define NUM_ADC_SAMPLE                      200
#define NUM_ADC_INPUT                       2
uint16_t batterySteps[NUM_ADC_INPUT];
uint16_t motorBatterySteps[NUM_ADC_SAMPLE];

/* Main application */
void appMain(const gecko_configuration_t *pconfig);
void initMainBoardGPIO(void);

void evt_door_lock(uint8_t data[], uint16_t length);
void evt_door_sensor_send_notification(void);
void evt_door_button_ext_signal(void);
void evt_motor_battery_measurement(void);
void evt_update_battery_measurement(void);
void evt_send_notification_battery_level(void);
void flash_keypad_configuration_profile(void);
void factory_reset(void);
void special_command_default_handler(void);
void hardware_self_test_for_keypad(void);
void hardware_self_test_for_battery(void);
void hardware_self_test_dc_motor(void);
uint8_t battery_measurement_test(void);

#endif
