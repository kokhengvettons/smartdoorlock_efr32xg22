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

#include "battery.h"
#include "motor.h"
#include "app.h"
#include "em_iadc.h"
#include "bsp.h"
#include "ustimer.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/* Application declaration*/
#define DOOR_KEY_ACCESS_LEN  6
static uint8_t keypadValue[DOOR_KEY_ACCESS_LEN * 2];
static uint8_t keypadNum = 0;

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
//  gecko_cmd_gatt_server_send_characteristic_notification(
//    0xFF, gattdb_temperature_measurement, 5, htmTempBuffer);
}

/**
 * @brief  Reset the door key from the user
 */
void resetDoorKeyValue(void)
{
  memset(keypadValue, 0, sizeof(uint8_t)*DOOR_KEY_ACCESS_LEN);
  keypadNum = 0;
}

/**
 * @brief  Initialize the door access validation process
 */
void initDoorAccess(void)
{
  resetDoorKeyValue();
}

void keypadEventInterruptHandler(uint8_t pinNum)
{
  uint8_t eventData[3];
  cpt212b_ReadKeypadEvent(I2C0, eventData, 3);

  int eventTypeValue = eventData[0] & 0x0F;
  if (eventTypeValue == CPT212B_SENSE_EVENT_TOUCH)
  {
    // when pressed # button, send the notification with door passs key to client
    if (eventData[1] == CPT212B_TOUCH_EVENT_CS10)
    {     
      uint8_t key[DOOR_KEY_ACCESS_LEN]; 
      if (keypadNum >= DOOR_KEY_ACCESS_LEN)
      {
        memcpy(key, &keypadValue[keypadNum - DOOR_KEY_ACCESS_LEN], sizeof(uint8_t) * DOOR_KEY_ACCESS_LEN);
        gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_password, sizeof(key), key);
      }
      resetDoorKeyValue();
    }
    else
    {
      keypadValue[keypadNum++] = eventData[1];
      if (keypadNum == DOOR_KEY_ACCESS_LEN * 2)
        keypadNum = 0;
    }
  }
  else if (eventTypeValue == CPT212B_SENSE_EVENT_PROXIMITY)
  {
    // TODO: activate the keypad
  }

  // clear the interrupt flags
  GPIO_IntClear(GPIO_IntGet());
}

/**
 * @brief  door open sensor Interrupt callback handler
 */
void doorSensorInterruptHandler(uint8_t pinNum)
{
  gecko_external_signal(EXT_SIGNAL_DOOR_SENSOR_FLAG);

  // clear the interrupt flags
  GPIO_IntClear(GPIO_IntGet());
}

/**
 * @brief  door push button Interrupt callback handler
 */
void doorOpenButtonInterruptHandler(uint8_t pinNum)
{
  gecko_external_signal(EXT_SIGNAL_DOOR_BUTTON_FLAG);

  // clear the interrupt flags
  GPIO_IntClear(GPIO_IntGet());
  
  // disable the interrupt to prevent button debouncing issue
  GPIO_IntDisable(1 << INT_SOURCE_DOOR_OPEN_BUTTON);
}

/**
 * @brief  initialize GPIO module
 */
void initGpio(void)
{
  // configure PD02 as LED0 and PD03 as LED1
  //GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, 0);
  //GPIO_PinModeSet(BSP_LED1_PORT, BSP_LED1_PIN, gpioModePushPull, 0);

  // configure PC02 as door sensor and PB00 as door open button
  GPIO_PinModeSet(gpioPortC, 2, gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(gpioPortB, 0, gpioModeInputPull, 1);

  // configure PC02 as rising & falling edge trigger on GPIO interrupt source 2 for door sensor
  GPIO_ExtIntConfig(gpioPortC, 2, INT_SOURCE_DOOR_SENSOR, true, true, true);

  // configure PB00 as falling edge trigger on GPIO interrupt source 3 for door sensor
  GPIO_ExtIntConfig(gpioPortB, 0, INT_SOURCE_DOOR_OPEN_BUTTON, false, true, true);

  // Initialize GPIOINT and register a callback
  GPIOINT_Init();
  GPIOINT_CallbackRegister(INT_SOURCE_KEYPAD_EVENT, keypadEventInterruptHandler);
  GPIOINT_CallbackRegister(INT_SOURCE_DOOR_SENSOR, doorSensorInterruptHandler);
  GPIOINT_CallbackRegister(INT_SOURCE_DOOR_OPEN_BUTTON, doorOpenButtonInterruptHandler);
}

/**
 * @brief  initialize user app settings before run the appMain function
 */
void initUserApp()
{
  // Initialize of USTIMER driver
  USTIMER_Init();

  // Initialize the capacitive touch keypad
  initcpt212b();

  // Initialize motor PWM module
  initMotorPwm();

  // Initialize measure battery module
  initBatteryADCMeasurement();

  // initialize door access
  initDoorAccess();

  // Initialize GPIO module
  initGpio();
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

  #ifdef FEATURE_I2C_SENSOR
  // Initialize the Temperature Sensor
  //Si7013_Detect(I2C0, SI7021_ADDR, NULL);
  #endif

  // Initialize user application
  initUserApp();

  // main application 
  appMain(&config);
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
