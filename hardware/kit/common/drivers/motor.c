/***************************************************************************//**
 * @file
 * @brief motor.c
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
#include "motor.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "ustimer.h"
#include "native_gecko.h"

#include <stdbool.h>

static bool InitializedPwm = false;

/**************************************************************************//**
 * @brief GPIO initialization for output PWM
 *****************************************************************************/
void initGpioPwm(void)
{
  GPIO_PinModeSet(gpioPortA, 0x07, gpioModePushPull, 1);  
}

/**************************************************************************//**
 * @brief Clock initialization for Low Energy timer
 *****************************************************************************/
void initClockLetimer(void)
{
  // Enable clock to LETIMER0
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  CMU_LFXOInit_TypeDef lfxoInit = CMU_LFXOINIT_DEFAULT;

  // Select LFXO for the LETIMER
  CMU_LFXOInit(&lfxoInit);
  CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFXO);
}

/**************************************************************************//**
 * @brief LETIMER initialization
 *****************************************************************************/
void initLetimer(unsigned int dutyCycle)
{

  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

  // Calculate the top value (frequency) based on clock source
  uint32_t topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / OUT_FREQ;

  // Reload top on underflow, PWM output, and run in free mode
  letimerInit.comp0Top = true;
  letimerInit.topValue = topValue;
  letimerInit.ufoa0 = letimerUFOAPwm;
  letimerInit.repMode = letimerRepeatFree;


  // Enable LETIMER0 output0 on PA6
  GPIO->LETIMERROUTE[0].ROUTEEN = GPIO_LETIMER_ROUTEEN_OUT0PEN;
  GPIO->LETIMERROUTE[0].OUT0ROUTE = \
      (gpioPortA << _GPIO_LETIMER_OUT0ROUTE_PORT_SHIFT) \
      | (7 << _GPIO_LETIMER_OUT0ROUTE_PIN_SHIFT);

  // Set COMP0 to control duty cycle
  LETIMER_CompareSet(LETIMER0, 0, topValue * dutyCycle / 100);

  // Initialize LETIMER
  LETIMER_Init(LETIMER0, &letimerInit);
}

/**************************************************************************//**
 * @brief enable the Motor PWM module 
 *****************************************************************************/
void enableMotorPwm(void)
{
  initGpioPwm();

  initClockLetimer();

  InitializedPwm = true;
}

/**************************************************************************//**
 * @brief trigger the Motor to lock/unlock the door 
 *****************************************************************************/
void triggerDoorLock(bool bLock)
{
  if (InitializedPwm != true)
    enableMotorPwm();

  if (bLock)
  {
    initLetimer(DUTY_CYCLE_LOCK);
    gecko_cmd_hardware_set_soft_timer(11468, 1, true);
  }
  else
  {
    initLetimer(DUTY_CYCLE_UNLOCK);
    gecko_cmd_hardware_set_soft_timer(11468, 1, true);
  }
 };

/**************************************************************************//**
 * @brief end the door lock process 
 *****************************************************************************/
void endDoorLock(void)
{
  LETIMER_Enable(LETIMER0, false);
}
