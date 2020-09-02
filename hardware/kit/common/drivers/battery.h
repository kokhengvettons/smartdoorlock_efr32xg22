/***************************************************************************//**
 * @file
 * @brief battery.h
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

#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "em_iadc.h"
#include <stdbool.h>

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

// Set HFRCODLL clock to 80MHz
#define HFRCODPLL_FREQ            cmuHFRCODPLLFreq_80M0Hz

// Set CLK_ADC to 40MHz
#define CLK_SRC_ADC_FREQ          40000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ              10000000 // CLK_ADC - 10MHz max in normal mode

// When changing GPIO port/pins below, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_0_BUS          CDBUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0
#define IADC_INPUT_1_BUS          CDBUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_CDBUSALLOC_CDODD0_ADC0

#define COIL_CELL_BATTERY_MIN     1.8
#define COIL_CELL_BATTERY_MAX     3.3
#define MOTOR_BATTERY_MIN         4.0
#define MOTOR_BATTERY_MAX         6.0
#define COIL_VOLT_DIV_SCALE_FACT  COIL_CELL_BATTERY_MAX / 1.2
#define MOTOR_VOL_DIV_SCALE_FACT  MOTOR_BATTERY_MAX / 1.2

typedef enum {ALL_TYPE_BATTER = 0, MOTOR_BATTERY_ONLY = 1} battery_measure_TypeDef;

uint16_t indexAdcSample;

void initGpioIAdc(void);
void initIAdc(void);
void initBatteryADCMeasurement(void);
void triggerADCScanAgain(void);
void terminateBatteryMeasurement(void);
void terminateMotorBatteryMeasurement(void);
void triggerBatteryMeasurement(battery_measure_TypeDef measure_type);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_H
