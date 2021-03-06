/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
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

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "app.h"
#include "motor.h"
#include "battery.h"
#include "cpt212b.h"
#include "bg_errorcodes.h"
#include "ustimer.h"
#include <stdbool.h>

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu            = 0;

/* Flag for indicating Door status */
const uint8_t doorLock[4]                = {"LOCK"};
const uint8_t doorUnlock[6]              = {"UNLOCK"};
const uint8_t door_open[4]               = {"OPEN"};
const uint8_t door_closed[6]             = {"CLOSED"};
const uint8_t fac_rst_device_name[15]    = {"VettonsDoorLock"};
const uint8_t fac_rst_manufact_name[13]  = {"SmartDoorLock"};
const uint8_t fac_rst_auto_lock_time[2]  = {0x3C, 0x00};
const uint8_t fac_rst_door_alarm_time[2] = {0x1E, 0x00};
const uint8_t fac_rst_sn_string[36]      = {"00000000-0000-0000-0000-000000000000"};
const uint8_t fac_rst_door_auto_lock     = DISABLE_AUTO_LOCK; 

static uint8_t door_lock_status           = DOOR_UNLOCK;
static uint8_t door_status                = DOOR_OPEN;
static uint8_t enable_auto_door_lock      = DISABLE_AUTO_LOCK;
static uint32_t door_auto_lock_time_in_s  = MIN_DOOR_AUTO_LOCK_S;
static uint32_t door_alarm_time_in_s      = MIN_DOOR_SENSOR_ALARM_S;
static uint8_t battery_level[NUM_ADC_INPUT];
static uint8_t door_sensor_alarm_status   = DOOR_SENSOR_ALARM_OFF;

/* static function */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);
static void evt_door_sensor_send_read_response(struct gecko_msg_gatt_server_user_read_request_evt_t* readReqevt);
static void evt_door_lock_send_read_response(struct gecko_msg_gatt_server_user_read_request_evt_t* readReqevt);
static void evt_write_attribute(uint16_t attribute_id, struct gecko_msg_gatt_server_attribute_value_evt_t* attr_val);
static void evt_write_attribute_from_flash(uint16_t attribute_id);
static void evt_special_command_handler(struct gecko_msg_gatt_server_attribute_value_evt_t* attr_val);

/* Main application */
void appMain(const gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();

  /* Initialize stack */
  gecko_init(pconfig);

  /* measure battery level */
  triggerBatteryMeasurement(ALL_TYPE_BATTERY);

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* if there are no events pending then the next call to gecko_wait_event() may cause
     * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
    if (!gecko_event_pending()) {
      flushLog();
    }

    /* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
        bootMessage(&(evt->data.evt_system_boot));
        printLog("boot event - starting advertising\r\n");

        /* set transmit power as 0 dBm*/
        gecko_cmd_system_set_tx_power(0);

        /* Set adv on 37, 38 channels */
        gecko_cmd_le_gap_set_advertise_channel_map(0,7);

        /* Set advertising parameters. 300ms advertisement interval.*/
        gecko_cmd_le_gap_set_advertise_timing(0, 480, 480, 0, 0);

        // retrieve the attributes value from flash before start advertising
        evt_write_attribute_from_flash(gattdb_device_name);
        evt_write_attribute_from_flash(gattdb_manufacturer_name_string);
        evt_write_attribute_from_flash(gattdb_serial_number_string);
        evt_write_attribute_from_flash(gattdb_door_auto_lock_time);
        evt_write_attribute_from_flash(gattdb_enable_auto_door_lock);
        evt_write_attribute_from_flash(gattdb_door_sensor_alarm_time);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        break;

      case gecko_evt_le_connection_opened_id:

        printLog("connection opened\r\n");

	/* max interval: 200ms, latency: 2, supervision timeout: 6s */
        gecko_cmd_le_connection_set_timing_parameters(evt->data.evt_le_connection_opened.connection, 120,160, 2, 600, 0, 0xffff);
        break;

      case gecko_evt_le_connection_closed_id:

        printLog("connection closed, reason: 0x%2.2x\r\n", evt->data.evt_le_connection_closed.reason);

        /* Check if need to boot to OTA DFU mode */
        if (boot_to_dfu) {
          /* Enter to OTA DFU mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Stop timer in case client disconnected before indications were turned off */
          gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        }
        break;

      case gecko_evt_gatt_server_user_write_request_id:
        switch(evt->data.evt_gatt_server_user_write_request.characteristic)
        {
          case gattdb_ota_control:
            /* Events related to OTA upgrading
            ----------------------------------------------------------------------------- */
            /* Check if the user-type OTA Control Characteristic was written.
            * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */

            /* Set flag to enter to OTA mode */
            boot_to_dfu = 1;

            /* Send response to Write Request */
            gecko_cmd_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection, 
                                                            gattdb_ota_control, bg_err_success);

            /* Close connection to enter to DFU OTA mode */
            gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
            break;

          case gattdb_door_lock:
            evt_door_lock(evt->data.evt_gatt_server_attribute_value.value.data, 
                          evt->data.evt_gatt_server_attribute_value.value.len);
            gecko_cmd_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection, 
                                                            gattdb_door_lock, bg_err_success);
            break;
          default:
            break;
        }
        break;

      case gecko_evt_gatt_server_user_read_request_id:
        switch(evt->data.evt_gatt_server_user_read_request.characteristic)
        {
          case gattdb_door_lock:
            evt_door_lock_send_read_response(&(evt->data.evt_gatt_server_user_read_request));
            break;
          case gattdb_door_status:
            evt_door_sensor_send_read_response(&(evt->data.evt_gatt_server_user_read_request));
            break;
          default:
            break;
        }
        break;

      case gecko_evt_gatt_server_attribute_value_id:
        switch(evt->data.evt_gatt_server_attribute_value.attribute)
        {
          case gattdb_device_name:
            evt_write_attribute(gattdb_device_name, &(evt->data.evt_gatt_server_attribute_value));
            break;
          case gattdb_manufacturer_name_string:
            evt_write_attribute(gattdb_manufacturer_name_string, &(evt->data.evt_gatt_server_attribute_value));
            break;
          case gattdb_serial_number_string:
            evt_write_attribute(gattdb_serial_number_string, &(evt->data.evt_gatt_server_attribute_value));
            break;
          case gattdb_door_auto_lock_time:
            evt_write_attribute(gattdb_door_auto_lock_time, &(evt->data.evt_gatt_server_attribute_value));
            break;
          case gattdb_enable_auto_door_lock:
            evt_write_attribute(gattdb_enable_auto_door_lock, &(evt->data.evt_gatt_server_attribute_value));
            break;
          case gattdb_door_sensor_alarm_time:
            evt_write_attribute(gattdb_door_sensor_alarm_time, &(evt->data.evt_gatt_server_attribute_value));
            break;
          case gattdb_special_command:
            evt_special_command_handler(&(evt->data.evt_gatt_server_attribute_value));
            break;
          default:
            break;
        }
        break;

      /* External signal indication (comes from the interrupt handler) */
      case gecko_evt_system_external_signal_id:
        switch (evt->data.evt_system_external_signal.extsignals)
        {
          case EXT_SIGNAL_DOOR_BUTTON_FLAG:
            evt_door_button_ext_signal();
            break;
          case EXT_SIGNAL_DOOR_SENSOR_FLAG:
            gecko_cmd_hardware_set_soft_timer(32768 * DOOR_SENSOR_INTERVAL_MS / 1000, SOFT_TIMER_DOOR_SENSOR_HANDLER, true);
            break;
          case EXT_SIGNAL_I2C_INTERRUP_FLAG:
            break;
          
          default:
            break;
        }
        break;

      case gecko_evt_hardware_soft_timer_id:
        switch(evt->data.evt_hardware_soft_timer.handle)
        {
          case SOFT_TIMER_MOTOR_PWM_HANDLER:
            endDoorLock();
            break;
          case SOFT_TIMER_DOOR_SENSOR_HANDLER:
            evt_door_sensor_send_notification();
            break;
          case SOFT_TIMER_DOOR_BUTTON_HANDLER:
            GPIO_IntClear(1 << INT_SOURCE_DOOR_OPEN_BUTTON);
            GPIO_IntEnable(1 << INT_SOURCE_DOOR_OPEN_BUTTON);
            break;
          case SOFT_TIMER_MOTOR_ADC_MEAS_HANDLER:
            evt_motor_battery_measurement();
            break;
          case SOFT_TIMER_BATTERY_MEAS_HANDLER:
            evt_update_battery_measurement();
            break;
          case SOFT_TIMER_SEND_NOTIF_HANDLER:
            evt_send_notification_battery_level();
            break;
          case SOFT_TIMER_FAC_RESET_HANDLER:
            gecko_cmd_system_reset(0);
            break;
          case SOFT_TIMER_DOOR_AUTO_LOCK_HANDLER:
            if (door_lock_status == DOOR_UNLOCK)
            {
              if ((door_status = GPIO_PinInGet(gpioPortC, 2)) == DOOR_CLOSED)
              {
                triggerDoorLock(true);
                door_lock_status = DOOR_LOCK;
                gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_lock, sizeof(doorLock), doorLock);
              }
            }
            break;
          case SOFT_TIMER_DOOR_ALARM_ON_HANDLER:
            evt_send_notification_door_sensor_alarm(true);
            break;
          case SOFT_TIMER_DOOR_ALARM_OFF_HANDLER:
            evt_send_notification_door_sensor_alarm(false);
            break;
          default:
            break;
        }
        break;

      default:
        break;
    }
  }
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++) {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}

/* lock the door if incoming message show "LOCK", otherwise unlock it */
void evt_door_lock(uint8_t data[], uint16_t length)
{
  if (length == sizeof(doorLock))
  {
    if (memcmp(data, doorLock, length) == 0)
    {
      triggerDoorLock(true);
      door_lock_status = DOOR_LOCK;
    }
  }
  else if (length == sizeof(doorUnlock))
  {
    if (memcmp(data, doorUnlock, length) == 0)
    {
      triggerDoorLock(false);
      door_lock_status = DOOR_UNLOCK;

      // trigger auto door lock timer
      if (enable_auto_door_lock == ENABLE_AUTO_LOCK)
        gecko_cmd_hardware_set_soft_timer(32768 * door_auto_lock_time_in_s, SOFT_TIMER_DOOR_AUTO_LOCK_HANDLER, true);
    }
  }
}

/* press door button to lock/unloc the door */
void evt_door_button_ext_signal(void)
{
  if (door_lock_status == DOOR_UNLOCK)
  {
    triggerDoorLock(true);
    door_lock_status = DOOR_LOCK;
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_lock, sizeof(doorLock), doorLock);
  }
  else
  {
    triggerDoorLock(false);
    door_lock_status = DOOR_UNLOCK;
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF,gattdb_door_lock, sizeof(doorUnlock), doorUnlock);

    // trigger auto door lock timer
    if (enable_auto_door_lock == ENABLE_AUTO_LOCK)
      gecko_cmd_hardware_set_soft_timer(32768 * door_auto_lock_time_in_s, SOFT_TIMER_DOOR_AUTO_LOCK_HANDLER, true);
  }
}

/* Send an notification/indications to remote GATT client for door status*/
void evt_door_sensor_send_notification(void)
{
  door_status = GPIO_PinInGet(gpioPortC, 2);

  if (door_status == DOOR_OPEN)
  {
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_status, sizeof(door_open), door_open);

    // trigger door sensor alarm timer when door open
    gecko_cmd_hardware_set_soft_timer(32768 * door_alarm_time_in_s, SOFT_TIMER_DOOR_ALARM_ON_HANDLER, true);
  }
  else
  {
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_status, sizeof(door_closed), door_closed);

    // added some delay to prevent two notifications send at same time
    gecko_cmd_hardware_set_soft_timer(32768, SOFT_TIMER_DOOR_ALARM_OFF_HANDLER, true);

    // trigger auto door lock timer when door close
    if (enable_auto_door_lock == ENABLE_AUTO_LOCK)
      gecko_cmd_hardware_set_soft_timer(32768 * door_auto_lock_time_in_s, SOFT_TIMER_DOOR_AUTO_LOCK_HANDLER, true);
  }
}

/* send notification for battery level when lower than battery threshold */
void evt_send_notification_battery_level()
{
  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_battery_level_motor, 1, &battery_level[1]);
}

/* send notification for door sensor alarm when door open exceed time limit */
void evt_send_notification_door_sensor_alarm(bool bEnableAlarm)
{
  if (bEnableAlarm == true)
  {
    if ((door_status = GPIO_PinInGet(gpioPortC, 2)) == DOOR_OPEN)
    {
      //TODO: turn on buzzer

      door_sensor_alarm_status = DOOR_SENSOR_ALARM_ON;
      gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_sensor_alarm_status, 
                                                            1, &door_sensor_alarm_status);
    }
  }
  else
  {
    // TODO: turn off buzzer
    
    door_sensor_alarm_status = DOOR_SENSOR_ALARM_OFF;
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_sensor_alarm_status, 
                                                      1, &door_sensor_alarm_status);
  }
  
  
}

/* Send an read response to remote GATT client for door status*/
void evt_door_sensor_send_read_response(struct gecko_msg_gatt_server_user_read_request_evt_t* readReqevt)
{
  door_status = GPIO_PinInGet(gpioPortC, 2);

  if (door_status == DOOR_OPEN)
  {
    gecko_cmd_gatt_server_send_user_read_response(readReqevt->connection, gattdb_door_status, bg_err_success,
						  sizeof(door_open), door_open);
  }
  else
  {
    gecko_cmd_gatt_server_send_user_read_response(readReqevt->connection, gattdb_door_status, bg_err_success,
						  sizeof(door_closed), door_closed);
  }
}

/* Send an read response to remote GATT client for door lock status*/
void evt_door_lock_send_read_response(struct gecko_msg_gatt_server_user_read_request_evt_t* readReqevt)
{
  if (door_lock_status == DOOR_LOCK)
  {
    gecko_cmd_gatt_server_send_user_read_response(readReqevt->connection, gattdb_door_lock, bg_err_success, 
                                                  sizeof(doorLock), doorLock);
  }
  else
  {
    gecko_cmd_gatt_server_send_user_read_response(readReqevt->connection, gattdb_door_lock, bg_err_success, 
                                                  sizeof(doorUnlock), doorUnlock);           
  }
}

/* Store attribute value into flash by specified PS key */
void evt_write_attribute(uint16_t attribute_id, struct gecko_msg_gatt_server_attribute_value_evt_t* attr_val)
{
  gecko_cmd_flash_ps_save(PS_KEY_BASE + attribute_id, attr_val->value.len, attr_val->value.data);
  gecko_cmd_gatt_server_write_attribute_value(attribute_id, 0, attr_val->value.len, attr_val->value.data);

  if (attribute_id == gattdb_door_auto_lock_time || attribute_id == gattdb_door_sensor_alarm_time)
  {
      // in case user enter in less than 2 bytes value
      uint16_t time_in_s = 0;
      if (attr_val->value.len == 2)
        time_in_s = (attr_val->value.data[1] << 8) + attr_val->value.data[0];
      else
        time_in_s = attr_val->value.data[0];

      if (attribute_id == gattdb_door_auto_lock_time)
      {
        if (time_in_s <= MIN_DOOR_AUTO_LOCK_S || time_in_s >= MAX_DOOR_AUTO_LOCK_S)
          door_auto_lock_time_in_s = MIN_DOOR_AUTO_LOCK_S;
        else
          door_auto_lock_time_in_s  = time_in_s;
      }

      if (attribute_id == gattdb_door_sensor_alarm_time)
      {
        if (time_in_s <= MIN_DOOR_SENSOR_ALARM_S || time_in_s >= MAX_DOOR_SENSOR_ALARM_S)
          door_alarm_time_in_s = MIN_DOOR_SENSOR_ALARM_S;
        else
          door_alarm_time_in_s  = time_in_s;
      }
  }

  if (attribute_id == gattdb_enable_auto_door_lock)
    enable_auto_door_lock = attr_val->value.data[0];
}

/* Read attribute value from flash then write the attribute */
void evt_write_attribute_from_flash(uint16_t attribute_id)
{
  struct gecko_msg_flash_ps_load_rsp_t *pResp;
  pResp = gecko_cmd_flash_ps_load(PS_KEY_BASE + attribute_id);
  if (pResp->result == 0)
  {
    gecko_cmd_gatt_server_write_attribute_value(attribute_id, 0, pResp->value.len, pResp->value.data);

    if (attribute_id == gattdb_door_auto_lock_time)
    {
      // in case user enter in less than 2 bytes value
      if (pResp->value.len == 2)
        door_auto_lock_time_in_s = (pResp->value.data[1] << 8) + pResp->value.data[0];
      else
        door_auto_lock_time_in_s = pResp->value.data[0];
    }

    if (attribute_id == gattdb_door_sensor_alarm_time)
    {
      // in case user enter in less than 2 bytes value
      if (pResp->value.len == 2)
        door_alarm_time_in_s = (pResp->value.data[1] << 8) + pResp->value.data[0];
      else
        door_alarm_time_in_s = pResp->value.data[0];
    }

    if (attribute_id == gattdb_enable_auto_door_lock)
      enable_auto_door_lock = pResp->value.data[0];
  }
}

/* special command handler */
void evt_special_command_handler(struct gecko_msg_gatt_server_attribute_value_evt_t* attr_val)
{
  switch (attr_val->value.data[0])
  {
    case 0x01: // factory reset
      factory_reset();
      break;
    case 0x02: // flash keypad configuration profile
      flash_keypad_configuration_profile();
      break;
    case 0x03: // hardware self test for keypad
      hardware_self_test_for_keypad();
      break;
    case 0x04: // hardware self test for battery
      hardware_self_test_for_battery();
      break;
    case 0x05: // hardware self test for dc motor
      hardware_self_test_dc_motor();
      break;
    default:
      special_command_default_handler();
      break;
  }
}

/* motor battery measurement */
void evt_motor_battery_measurement(void)
{
  if (indexAdcSample >= NUM_ADC_SAMPLE)
    terminateMotorBatteryMeasurement();
  else              
    triggerADCScanAgain();
}

/* update the battery measurement value to batter level attribute */
void evt_update_battery_measurement(void)
{
  float voltage;
  float batteryPercentage;

  // for cell coil battery measurement
  voltage = (batterySteps[0] * 3.3 / 0xFFF) * 1;
  batteryPercentage = (voltage - COIL_CELL_BATTERY_MIN)/(COIL_CELL_BATTERY_MAX - COIL_CELL_BATTERY_MIN) * 100;
  batteryPercentage = batteryPercentage > 100 ? 100 : batteryPercentage;
  battery_level[0] = (uint8_t) batteryPercentage;


  // for dc motor battery measurement
  voltage = (batterySteps[1] * 3.3 / 0xFFF) * 1;
  batteryPercentage = (voltage - MOTOR_BATTERY_MIN)/(MOTOR_BATTERY_MAX - MOTOR_BATTERY_MIN) * 100;
  batteryPercentage = batteryPercentage > 100 ? 100 : batteryPercentage;
  battery_level[1] = (uint8_t) batteryPercentage;

  gecko_cmd_gatt_server_write_attribute_value(gattdb_battery_level_cell, 0, 1, &battery_level[0]);
  gecko_cmd_gatt_server_write_attribute_value(gattdb_battery_level_motor, 0, 1, &battery_level[1]);

  if (battery_level[0] < BATTERY_LEVEL_LOW)
  {
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_battery_level_cell, 1, &battery_level[0]);
  }

  // schedule 1 seconds delay for dc motor battery notification due to two notification can't send at same time
  if (battery_level[1] < BATTERY_LEVEL_LOW)
  {
    gecko_cmd_hardware_set_soft_timer(32768 * BATTERY_SEND_NOTIF_INTERVAL_MS / 1000, SOFT_TIMER_SEND_NOTIF_HANDLER, true);
  }

  triggerADCScanAgain();
}

/* perform factory reset */
void factory_reset(void)
{
  // Perform a factory reset by erasing PS storage.
  gecko_cmd_flash_ps_erase_all();

  // write default device name and alarm trigger time into PS storage.
  gecko_cmd_flash_ps_save(PS_KEY_BASE + gattdb_device_name, sizeof(fac_rst_device_name), fac_rst_device_name);
  gecko_cmd_flash_ps_save(PS_KEY_BASE + gattdb_manufacturer_name_string, sizeof(fac_rst_manufact_name), fac_rst_manufact_name);
  gecko_cmd_flash_ps_save(PS_KEY_BASE + gattdb_door_auto_lock_time, sizeof(fac_rst_auto_lock_time), fac_rst_auto_lock_time);
  gecko_cmd_flash_ps_save(PS_KEY_BASE + gattdb_door_sensor_alarm_time, sizeof(fac_rst_door_alarm_time), fac_rst_door_alarm_time);
  gecko_cmd_flash_ps_save(PS_KEY_BASE + gattdb_enable_auto_door_lock, sizeof(fac_rst_door_auto_lock), &fac_rst_door_auto_lock);
  gecko_cmd_flash_ps_save(PS_KEY_BASE + gattdb_serial_number_string, sizeof(fac_rst_sn_string), fac_rst_sn_string);

  uint8_t error_code = special_cmd_success;
  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_special_command, 1, &error_code);

  // add some delay to reset the device 
  gecko_cmd_hardware_set_soft_timer(32768 * FACTORY_RESET_INTERVAL_MS / 1000, SOFT_TIMER_FAC_RESET_HANDLER, true);
}

/* overwrite keypad configuration profile */
void flash_keypad_configuration_profile(void)
{
  uint8_t err_spec_cmd = special_cmd_success;

  errorcode_t error_code = initcpt212b(true);  
  if (error_code != bg_err_success)
  {
    err_spec_cmd = special_cmd_err_write_profile;
  }

  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_special_command, 1, &err_spec_cmd);
}

/* send unsupported command return code when received invalid special command  */
void special_command_default_handler(void)
{
  uint8_t err_spec_cmd = special_cmd_unsupported_cmd;
  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_special_command, 1, &err_spec_cmd);
}

/* perform hardware self test for keypad */
void hardware_self_test_for_keypad(void)
{
  errorcode_t bg_err = bg_err_success;
  uint8_t err_code = special_cmd_success;

  if ((bg_err =initcpt212b(false)) != bg_err_success)
  {
    err_code = special_cmd_err_hardware_keypad;    
  }

  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_special_command, 1, &err_code);
}

/* perform hardware self test for battery */
void hardware_self_test_for_battery()
{
  uint8_t err_code = battery_measurement_test();
  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_special_command, 1, &err_code);
}

/* perform hardware self test for dc motor */
void hardware_self_test_dc_motor()
{
  uint8_t err_code = special_cmd_success;
  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_special_command, 1, &err_code);
}

/* hardware test for measure battery level */
uint8_t battery_measurement_test(void)
{
  //terminate all the battery measurement 
  terminateBatteryMeasurement();
  terminateMotorBatteryMeasurement();
  
  triggerADCScanAgain();
  USTIMER_Delay(2000);

  // convert ADC steps into voltage
  float coinCellBatInVolt = (batterySteps[0] * 3.3 / 0xFFF) * 1;
  float motorBatInVolt = (batterySteps[1] * 3.3 / 0xFFF) * 1;

  // resume the battery measurement
  triggerBatteryMeasurement(ALL_TYPE_BATTERY);

  if ((coinCellBatInVolt <= 0 && coinCellBatInVolt >= 3.6) ||
      (motorBatInVolt <= 0 && motorBatInVolt >= 6.5))
  {
    return special_cmd_err_hardware_battery;
  }

  return special_cmd_success;
}
