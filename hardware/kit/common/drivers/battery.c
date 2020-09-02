/***************************************************************************//**
 * @file
 * @brief battery.c
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
#include "battery.h"
#include "em_iadc.h"
#include "em_cmu.h"
#include "app.h"
#include "native_gecko.h"

static battery_measure_TypeDef batteryMeasureType; 

/**************************************************************************//**
 * @brief  GPIO initialization for battery measurement
 *****************************************************************************/
void initGpioIAdc(void)
{
  GPIO_PinModeSet(gpioPortC, 0x00, gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(gpioPortC, 0x06, gpioModeInputPullFilter, 1);
}

/**************************************************************************//**
 * @brief  IADC Initializer
 *****************************************************************************/
void initIAdc (void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  IADC_ScanTable_t initScanTable = IADC_SCANTABLE_DEFAULT;  // Scan Table

  // Enable IADC0 clock branch
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified by
  // other code
  IADC_reset(IADC0);

  // Set HFRCODPLL band and the tuning value based on the value in the calibration table made during production.
  CMU_HFRCODPLLBandSet(HFRCODPLL_FREQ);

  // Select HFRCODPLL as the EM01GRPA clock
  CMU_ClockSelectSet(cmuClock_EM01GRPACLK, cmuSelect_HFRCODPLL);

  // Select clock for IADC
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_EM01GRPACLK);

  // Modify init structs and initialize
  init.warmup = iadcWarmupNormal;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // 25ns per cycle, 40000 cycles make 1ms timer event
  init.timerCycles = 20000; // 0.5ms

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
  
  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                             CLK_ADC_FREQ,
                                             0,
                                             iadcCfgModeNormal,
                                             init.srcClkPrescale);

  // Scan initialization
  initScan.triggerSelect = iadcTriggerSelTimer;
  initScan.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID2;

  // Tag FIFO entry with scan table entry id.
  initScan.showId = true;

  // Configure entries in scan table, CH0 is single ended from input 0, CH1 is
  // single ended from input 1
  initScanTable.entries[0].posInput = iadcPosInputPortCPin0; // PC00 -> P01 on BRD4001 J102
  initScanTable.entries[0].negInput = iadcNegInputGnd;
  initScanTable.entries[0].includeInScan = true;

  initScanTable.entries[1].posInput = iadcPosInputPortCPin6; // PC06 -> P29 on BRD4001 J102
  initScanTable.entries[1].negInput = iadcNegInputGnd;
  initScanTable.entries[1].includeInScan = true;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initScan(IADC0, &initScan, &initScanTable);

  // Enable the IADC timer - can only be done after the IADC has been enabled
  IADC_command(IADC0, iadcCmdEnableTimer);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;
  
  // Enable Scan interrupts
  IADC_enableInt(IADC0, IADC_IEN_SCANFIFODVL);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

/**************************************************************************//**
 * @brief Enable the IADC module for battery measurement 
 *****************************************************************************/
void initBatteryADCMeasurement(void)
{
  // Initialize the GPIO
  initGpioIAdc();

  // Initialize the IADC
  initIAdc();

  // reset the data buffer
  memset(batterySteps, 0, sizeof(batterySteps));
}

/**************************************************************************//**
 * @brief trigger IADC for battery measurement 
 *****************************************************************************/
void triggerBatteryMeasurement(battery_measure_TypeDef measure_type)
{
  batteryMeasureType = measure_type;

  if (measure_type == MOTOR_BATTERY_ONLY)
  {
    indexAdcSample = 0;
    memset(motorBatterySteps, 0, sizeof(motorBatterySteps));
    gecko_cmd_hardware_set_soft_timer(32768 * MOTOR_ADC_MEAS_INTERVAL_MS / 1000, 
                                      SOFT_TIMER_MOTOR_ADC_MEAS_HANDLER, false);

    //GPIO_PinOutSet(BSP_LED1_PORT, BSP_LED1_PIN);  
  }
  else
  {    
    gecko_cmd_hardware_set_soft_timer(32768 * BATTERY_ADC_MEAS_INTERVAL_MS / 1000, 
                                      SOFT_TIMER_BATTERY_MEAS_HANDLER, false);
  }  

  IADC_command(IADC0, iadcCmdStartScan);  
}

/**************************************************************************//**
 * @brief terminate the IADC for battery measurement 
 *****************************************************************************/
void terminateBatteryMeasurement(void)
{
  // removes the scheduled timer if set value to 0
  gecko_cmd_hardware_set_soft_timer(0, SOFT_TIMER_BATTERY_MEAS_HANDLER, false);

  IADC_command(IADC0, iadcCmdStopScan);
}

/**************************************************************************//**
 * @brief terminate the IADC for motor battery measurement 
 *****************************************************************************/
void terminateMotorBatteryMeasurement(void)
{
  // removes the scheduled timer if set value to 0
  gecko_cmd_hardware_set_soft_timer(0, SOFT_TIMER_MOTOR_ADC_MEAS_HANDLER, false);
  //GPIO_PinOutClear(BSP_LED1_PORT, BSP_LED1_PIN);

  IADC_command(IADC0, iadcCmdStopScan);
}

/**************************************************************************//**
 * @brief trigger IADC Scan again  
 *****************************************************************************/
void triggerADCScanAgain(void)
{
  IADC_command(IADC0, iadcCmdStartScan);
}

/**************************************************************************//**
 * @brief  IADC Handler
 *****************************************************************************/
void IADC_IRQHandler(void)
{
  IADC_Result_t sample;

  // Get ADC results
  while(IADC_getScanFifoCnt(IADC0))
  {
    // Read data from the scan FIFO
    sample = IADC_pullScanFifoResult(IADC0);

    // collect door lock/unlock motor battery voltage profile
    if (batteryMeasureType == MOTOR_BATTERY_ONLY)
    {
      if (sample.id == 1)
      {
        motorBatterySteps[indexAdcSample++] = sample.data;
      }
    }

    // Calculate input voltage:
    //  For single-ended the result range is 0 to +Vref, i.e.,
    //  for Vref = AVDD = 3.30V, 12 bits represents 3.30V full scale IADC range.
    //scanResults[sample.id] = sample.data * 3.3 / 0xFFF;
    batterySteps[sample.id] = sample.data;
  }

  // Start next IADC conversion
  IADC_clearInt(IADC0, IADC_IF_SCANFIFODVL); // flags are sticky; must be cleared in software
}
