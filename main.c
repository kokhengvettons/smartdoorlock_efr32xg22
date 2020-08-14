/***************************************************************************//**
 * @file
 * @brief Silicon Labs Thermometer Example Application
 * This Thermometer and OTA example allows the user to measure temperature
 * using the temperature sensor on the WSTK. The values can be read with the
 * Health Thermometer reader on the Blue Gecko smartphone app.
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

/* Board Headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "infrastructure.h"

/* GATT database */
#include "gatt_db.h"

/* EM library (EMlib) */
#include "em_system.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#include"gpiointerrupt.h"

#ifdef FEATURE_BOARD_DETECTED
#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#else
#error This sample app only works with a Silicon Labs Board
#endif

#ifdef FEATURE_I2C_SENSOR
#include "i2cspm.h"
#include "si7013.h"
#include "cpt212b.h"
#include "tempsens.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/* Application declaration*/
#define DOOR_KEY_PW_SIZE  6

static uint8_t keypadPassword[DOOR_KEY_PW_SIZE];
static uint8_t doorAccessKey[DOOR_KEY_PW_SIZE];
static uint8_t passwordCounter = 0;
static bool doorAccessPass = false;

/* Gecko configuration parameters (see gecko_configuration.h) */
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

static const gecko_configuration_t config = {
  .config_flags = 0,
#if defined(FEATURE_LFXO) || defined(PLFRCO_PRESENT)
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
#else
  .sleep.flags = 0,
#endif
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
#if defined(FEATURE_LFXO)
  .bluetooth.sleep_clock_accuracy = 100, // ppm
#elif defined(PLFRCO_PRESENT)
  .bluetooth.sleep_clock_accuracy = 500, // ppm
#endif
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
  .rf.flags = GECKO_RF_CONFIG_ANTENNA,                 /* Enable antenna configuration. */
  .rf.antenna = GECKO_RF_ANTENNA,                      /* Select antenna path! */
};

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

/**
 * @brief Function for taking a single temperature measurement with the WSTK Relative Humidity and Temperature (RHT) sensor.
 */
void temperatureMeasure()
{
  uint8_t htmTempBuffer[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
  uint8_t flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
  int32_t tempData;     /* Stores the Temperature data read from the RHT sensor. */
  uint32_t rhData = 0;    /* Dummy needed for storing Relative Humidity data. */
  uint32_t temperature;   /* Stores the temperature data read from the sensor in the correct format */
  uint8_t *p = htmTempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */
  static int32_t DummyValue = 0l; /* This dummy value can substitute the temperature sensor value if the sensor is N/A. */

  /* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
  UINT8_TO_BITSTREAM(p, flags);

#ifdef FEATURE_I2C_SENSOR
  /* Sensor relative humidity and temperature measurement returns 0 on success, nonzero otherwise */
  if (Si7013_MeasureRHAndTemp(I2C0, SI7021_ADDR, &rhData, &tempData) != 0)
#endif
  {
    /* Use the dummy value and go between 20 and 40 if the sensor read failed.
     * The ramp-up value will be seen in the characteristic in the receiving end. */
    tempData = DummyValue + 20000l;
    DummyValue = (DummyValue + 1000l) % 21000l;
  }
  /* Convert sensor data to correct temperature format */
  temperature = FLT_TO_UINT32(tempData, -3);
  /* Convert temperature to bitstream and place it in the HTM temperature data buffer (htmTempBuffer) */
  UINT32_TO_BITSTREAM(p, temperature);

  /* Send indication of the temperature in htmTempBuffer to all "listening" clients.
   * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
   *  0xFF as connection ID will send indications to all connections. */
  gecko_cmd_gatt_server_send_characteristic_notification(
    0xFF, gattdb_temperature_measurement, 5, htmTempBuffer);
}

/**
 * @brief  retrieve the door access key from flash memory
 */
void retrieveDoorKeyAccess(void)
{
  // TODO: [LAI] retrieve the access key from flash memory.
  doorAccessKey[0] = CPT212B_TOUCH_EVENT_CS1;
  doorAccessKey[1] = CPT212B_TOUCH_EVENT_CS2;
  doorAccessKey[2] = CPT212B_TOUCH_EVENT_CS3;
  doorAccessKey[3] = CPT212B_TOUCH_EVENT_CS4;
  doorAccessKey[4] = CPT212B_TOUCH_EVENT_CS5;
  doorAccessKey[5] = CPT212B_TOUCH_EVENT_CS6;
}

/**
 * @brief  Reset the door key from the user
 */
void resetDoorKeyValue(void)
{
  memset(keypadPassword, 0, sizeof(uint8_t)*DOOR_KEY_PW_SIZE);
  passwordCounter = 0;
}

/**
 * @brief  Initialize the door access validation process
 */
void initDoorAccess(void)
{
  retrieveDoorKeyAccess();
  resetDoorKeyValue();
}

/**
 * @brief  Open the door after success the door key validation
 */
void openDoor(void)
{
  // open the door

  // reset the door key from the user
  resetDoorKeyValue();
}

/**
 * @brief  keypad touch event callback handler
 */
void keypadTouchEventHandler_cb(uint8_t pinNum)
{
  uint8_t eventData[3];
  Cpt212b_ReadKeypadEvent(I2C0, eventData, 3);

  int eventTypeValue = eventData[0] & 0x0F;
  if (eventTypeValue == CPT212B_SENSE_EVENT_TOUCH)
  {
    // reset the door key value when user press reset button
    if (eventData[1] == CPT212B_TOUCH_EVENT_CS10)
    {
      resetDoorKeyValue();
    }
    else
    {
      keypadPassword[passwordCounter] = eventData[1];
      if (keypadPassword[passwordCounter] == doorAccessKey[passwordCounter])
        doorAccessPass = true;
      else
        doorAccessPass = false;
      
      // at here, increase the counter
      passwordCounter++;

      if (passwordCounter == DOOR_KEY_PW_SIZE && doorAccessPass == true)
      {
        openDoor();
      }
    }
  }
  else if (eventTypeValue == CPT212B_SENSE_EVENT_PROXIMITY)
  {

  }

  // clear the interrupt flags
  GPIO_IntClear(GPIO_IntGet());
}

/**
 * @brief  configure GPIO Interrupt
 */
void configureGPIO_Interrupt(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PC1 as input enabled for keypad
  GPIO_PinModeSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_ENABLE_PIN, gpioModeInputPull, 1);
 
  // configure PC1 as falling edge trigger on GPIO interrupt source 1
  GPIO_ExtIntConfig(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_ENABLE_PIN, 1, false, true, true);

  // Initialize GPIOINT and register a callback
  GPIOINT_Init();
  GPIOINT_CallbackRegister(1, keypadTouchEventHandler_cb);
}

/**
 * @brief  Main function
 */
int main(void)
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();
  initVcomEnable();
  // Initialize stack
  gecko_init(&config);

  #ifdef FEATURE_I2C_SENSOR
  // Initialize the Temperature Sensor
  //Si7013_Detect(I2C0, SI7021_ADDR, NULL);

  // Initialize the capacitive touch keypad
  InitCpt212b();

  #endif 

  // configure interrupts
  configureGPIO_Interrupt();

  // initialize door access
  initDoorAccess();

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
        /* Set advertising parameters. 100ms advertisement interval.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). */
        gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        break;

      /* This event is generated when a connected client has either
       * 1) changed a Characteristic Client Configuration, meaning that they have enabled
       * or disabled Notifications or Indications, or
       * 2) sent a confirmation upon a successful reception of the indication. */
      case gecko_evt_gatt_server_characteristic_status_id:
        /* Check that the characteristic in question is temperature - its ID is defined
         * in gatt.xml as "temperature_measurement". Also check that status_flags = 1, meaning that
         * the characteristic client configuration was changed (notifications or indications
         * enabled or disabled). */
        if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
            && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)) {
          if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02) {
            /* Indications have been turned ON - start the repeating timer. The 1st parameter '32768'
             * tells the timer to run for 1 second (32.768 kHz oscillator), the 2nd parameter is
             * the timer handle and the 3rd parameter '0' tells the timer to repeat continuously until
             * stopped manually.*/
            gecko_cmd_hardware_set_soft_timer(32768, 0, 0);
          } else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x00) {
            /* Indications have been turned OFF - stop the timer. */
            gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          }
        }
        break;

      /* This event is generated when the software timer has ticked. In this example the temperature
       * is read after every 1 second and then the indication of that is sent to the listening client. */
      case gecko_evt_hardware_soft_timer_id:
        /* Measure the temperature as defined in the function temperatureMeasure() */
        temperatureMeasure();
        break;

      case gecko_evt_le_connection_closed_id:
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Stop timer in case client disconnected before indications were turned off */
          gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        }
        break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Checks if the user-type OTA Control Characteristic was written.
       * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      default:
        break;
    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
