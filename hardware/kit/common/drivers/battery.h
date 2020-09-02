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

// Number of scan channels
#define NUM_INPUTS                2

#define NUM_ADC_SAMPLE            200

// When changing GPIO port/pins below, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_0_BUS          CDBUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0
#define IADC_INPUT_1_BUS          CDBUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_CDBUSALLOC_CDODD0_ADC0

typedef enum {COIL_BAT = 0, MOTOR_BAT = 1} battery_measure_TypeDef;

uint16_t indexAdcSample;

void initGpioIAdc(void);
void initIAdc(void);
void initBatteryADCMeasurement(void);
void setBatteryADCCommand(IADC_Cmd_t command);
void triggerBatteryMeasurement(battery_measure_TypeDef measure_type);
void terminateBatteryMeasurement(battery_measure_TypeDef measure_type);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_H
