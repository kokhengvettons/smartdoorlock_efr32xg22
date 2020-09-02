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
#include "battery.h"
#include "app.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "ustimer.h"
#include "native_gecko.h"

#include <stdbool.h>


/**************************************************************************//**
 * @brief GPIO initialization for output PWM
 *****************************************************************************/
void initGpioPwm(void)
{
  GPIO_PinModeSet(gpioPortA, 0x07, gpioModePushPull, 1);
  GPIO_PinOutClear(gpioPortA, 0x07);
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
 * @brief Initialize the Motor PWM module 
 *****************************************************************************/
void initMotorPwm(void)
{
  initGpioPwm();

  initClockLetimer();
}

/**************************************************************************//**
 * @brief trigger the Motor to lock/unlock the door 
 *****************************************************************************/
void triggerDoorLock(bool bLock)
{
  // prevent door lock trigger by door open button
  GPIO_IntDisable(1 << INT_SOURCE_DOOR_OPEN_BUTTON);

  if (bLock)
  {
    gecko_cmd_hardware_set_soft_timer(32768 * MOTOR_PWM_INTERVAL_MS / 1000, 
                                      SOFT_TIMER_MOTOR_PWM_HANDLER, true);

    initLetimer(DUTY_CYCLE_LOCK);
  }
  else
  {
    gecko_cmd_hardware_set_soft_timer(32768 * MOTOR_PWM_INTERVAL_MS / 1000, 
                                      SOFT_TIMER_MOTOR_PWM_HANDLER, true);

    initLetimer(DUTY_CYCLE_UNLOCK);
  }

  // trigger motor battery measurement
  triggerBatteryMeasurement(MOTOR_BAT);
 };

/**************************************************************************//**
 * @brief end the door lock process 
 *****************************************************************************/
void endDoorLock(void)
{
  LETIMER_Enable(LETIMER0, false);

  // door open button interrupt will trigger by software timer instead
  gecko_cmd_hardware_set_soft_timer(32768 * DOOR_BUTTON_DEBOUNCE_INTERVAL_MS / 1000, SOFT_TIMER_DOOR_BUTTON_HANDLER, true);
}
