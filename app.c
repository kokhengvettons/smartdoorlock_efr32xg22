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

#include <stdbool.h>

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu            = 0;

/* Flag for indicating Door status */
static uint8_t door_lock_status       = DOOR_UNLOCK;
static uint8_t door_status            = DOOR_OPEN;
static uint8_t door_alarm_status      = ALARM_OFF;
const uint8_t doorLock[4]             = {"LOCK"};
const uint8_t doorUnlock[6]           = {"UNLOCK"};
const uint8_t door_open[4]            = {"OPEN"};
const uint8_t door_closed[6]          = {"CLOSED"};
const uint8_t door_alarm_on[2]        = {"ON"};
const uint8_t door_alarm_off[3]       = {"OFF"};

static uint32_t door_alarm_time_in_s  = DOOR_ALARM_DEFAULT_INTERVAL_MS / 1000;

/* static function */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);
static void evt_door_sensor_send_read_response(struct gecko_msg_gatt_server_user_read_request_evt_t* readReqevt);
static void evt_door_lock_send_read_response(struct gecko_msg_gatt_server_user_read_request_evt_t* readReqevt);
static void evt_door_alarm_status_send_read_response(struct gecko_msg_gatt_server_user_read_request_evt_t* readReqevt);
static void evt_write_attribute(uint16_t attribute_id, struct gecko_msg_gatt_server_attribute_value_evt_t* attr_val);
static void evt_write_attribute_from_flash(uint16_t attribute_id);

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

        /* Set advertising parameters. 100ms advertisement interval.
         * The first parameter is advertising set handle
         * The next two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6).
         * The last two parameters are duration and maxevents left as default. */
        gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

        // retrieve the attributes value from flash before start advertising
        evt_write_attribute_from_flash(gattdb_device_name);
        evt_write_attribute_from_flash(gattdb_serial_number_string);
        evt_write_attribute_from_flash(gattdb_door_alarm_trigger_time);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        break;

      case gecko_evt_le_connection_opened_id:

        printLog("connection opened\r\n");
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
          case gattdb_battery_level:
            break;
          case gattdb_door_password:
            break;
          case gattdb_door_lock:
            evt_door_lock_send_read_response(&(evt->data.evt_gatt_server_user_read_request));
            break;
          case gattdb_door_status:
            evt_door_sensor_send_read_response(&(evt->data.evt_gatt_server_user_read_request));
            break;
          case gattdb_door_alarm_status:
            evt_door_alarm_status_send_read_response(&(evt->data.evt_gatt_server_user_read_request));
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
          case gattdb_serial_number_string:
            evt_write_attribute(gattdb_serial_number_string, &(evt->data.evt_gatt_server_attribute_value));
            break;
          case gattdb_door_alarm_trigger_time:
            evt_write_attribute(gattdb_door_alarm_trigger_time, &(evt->data.evt_gatt_server_attribute_value));
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
            GPIO_IntEnable(1 << INT_SOURCE_DOOR_OPEN_BUTTON);
            break;
          case SOFT_TIMER_DOOR_ALARM_ON_HANDLER:
            evt_door_alarm_send_notification(ALARM_ON);
            break;
          case SOFT_TIMER_DOOR_ALARM_OFF_HANDLER:
            evt_door_alarm_send_notification(ALARM_OFF);
            break;
          case SOFT_TIMER_MOTOR_ADC_MEAS_HANDLER:
            evt_motor_battery_measurement(MOTOR_BAT);
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
  }
}

/* Send an notification/indications to remote GATT client for door status*/
void evt_door_sensor_send_notification(void)
{
  door_status = GPIO_PinInGet(gpioPortC, 2);

  if (door_status == DOOR_OPEN)
  {
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_status, sizeof(door_open), door_open);

    // when door open then trigger the door alarm timer 
    gecko_cmd_hardware_set_soft_timer(32768 * door_alarm_time_in_s, SOFT_TIMER_DOOR_ALARM_ON_HANDLER, true);
  }
  else
  {
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_status, sizeof(door_closed), door_closed);

    // when detected close door event happened, send an notification to outside world
    gecko_cmd_hardware_set_soft_timer(32768 * DOOR_ALARM_OFF_INTERVAL_MS / 1000, SOFT_TIMER_DOOR_ALARM_OFF_HANDLER, true);
  }
}

/* Send an notification/indications to remote GATT client when triggered door alarm*/
void evt_door_alarm_send_notification(door_alarm_status_TypedDef alarm_status)
{
  door_alarm_status = alarm_status;

  if (alarm_status == ALARM_ON)
  {
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_alarm_status, sizeof(door_alarm_on), door_alarm_on);
    //TODO: Turn on Buzzer
  }
  else
  {
    gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_alarm_status, sizeof(door_alarm_off), door_alarm_off);
    //TODO: Turn off Buzzer
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

/* Send an read response to remote GATT client for door alarm status*/
void evt_door_alarm_status_send_read_response(struct gecko_msg_gatt_server_user_read_request_evt_t* readReqevt)
{
  if (door_alarm_status == ALARM_ON)
  {
    gecko_cmd_gatt_server_send_user_read_response(readReqevt->connection, gattdb_door_lock, bg_err_success, 
                                                  sizeof(door_alarm_on), door_alarm_on);
  }
  else
  {
    gecko_cmd_gatt_server_send_user_read_response(readReqevt->connection, gattdb_door_lock, bg_err_success, 
                                                  sizeof(door_alarm_off), door_alarm_off);    
  }  
}

/* Store attribute value into flash by specified PS key */
void evt_write_attribute(uint16_t attribute_id, struct gecko_msg_gatt_server_attribute_value_evt_t* attr_val)
{
  gecko_cmd_flash_ps_save(PS_KEY_BASE + attribute_id, attr_val->value.len, attr_val->value.data);
  gecko_cmd_gatt_server_write_attribute_value(attribute_id, 0, attr_val->value.len, attr_val->value.data);

  if (attribute_id == gattdb_door_alarm_trigger_time)
  {
    if (attr_val->value.len == 2)
    {
      door_alarm_time_in_s  = (attr_val->value.data[1] << 8) + attr_val->value.data[0];
    }
    else
    {
      door_alarm_time_in_s = DOOR_ALARM_DEFAULT_INTERVAL_MS / 1000;
    }      
  }
}

/* Read attribute value from flash then write the attribute */
void evt_write_attribute_from_flash(uint16_t attribute_id)
{
  struct gecko_msg_flash_ps_load_rsp_t *pResp;
  pResp = gecko_cmd_flash_ps_load(PS_KEY_BASE + attribute_id);
  if (pResp->result == 0)
  {
    gecko_cmd_gatt_server_write_attribute_value(attribute_id, 0, pResp->value.len, pResp->value.data);

    if (attribute_id == gattdb_door_alarm_trigger_time)
    {
      if (pResp->value.len == 2)
      {
        door_alarm_time_in_s  = (pResp->value.data[1] << 8) + pResp->value.data[0];
      }
      else
      {
        door_alarm_time_in_s = DOOR_ALARM_DEFAULT_INTERVAL_MS / 1000;
      }      
    }
  }
}

/* motor battery measurement */
void evt_motor_battery_measurement(battery_measure_TypeDef measure_type)
{
  if (indexAdcSample >= NUM_ADC_SAMPLE)
    terminateBatteryMeasurement(measure_type);
  else              
    setBatteryADCCommand(iadcCmdStartScan);
}