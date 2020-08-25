/***************************************************************************//**
 * @file
 * @brief motor.h
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

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>


/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

// Desired PWM frequency in Hz
#define OUT_FREQ                50

// Duty cycle percentage
#define DUTY_CYCLE_LOCK         4       // 0.8ms
#define DUTY_CYCLE_UNLOCK       15      // 3ms

#define MOTOR_PWM_INTERVAL_MS   350     // 350ms

void initLetimer(unsigned int dutyCycle);
void initClockLetimer(void);
void initGpioPwm(void);
void initMotorPwm(void);
void triggerDoorLock(bool bLock);
void endDoorLock(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H
