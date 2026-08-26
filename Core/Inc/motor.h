/*
 * motor.h
 *
 * Motor + encoder driver for WHEELTEC C30D V2.1 (STM32F407VET6)
 *
 * Topology: PWM / PWM  (AT8236 dual-input H-bridge, NO enable pin)
 *   Forward : PWM on IN1, IN2 held low
 *   Reverse : IN1 held low, PWM on IN2
 *   Stop    : both low (coast)
 *
 * Verified against the C30D schematic:
 *   U8 -> AOUT1/AOUT2 (Motor A) : IN pins PB8 / PB9
 *   U9 -> BOUT1/BOUT2 (Motor B) : IN pins PE5 / PE6
 *
 *   Motor A drive   : TIM4_CH3 (PB8), TIM4_CH4 (PB9)
 *   Motor B drive   : TIM9_CH1 (PE5), TIM9_CH2 (PE6)
 *   Motor A encoder : TIM2 (PA15/PB3)
 *   Motor B encoder : TIM3 (PB4/PB5)
 *
 * PC6/PC7/PA2/PA3/PA4/PA5 are NOT connected to any motor driver on this
 * board revision. The Lab_4 pin map was for a different board.
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

/* ARR = 7199 @ 72 MHz timer clock -> 10 kHz PWM */
#define PWM_MAX  7199
#define PWM_MIN  1000   /* below this the motor stalls */

#define TICKS_PER_REV  260

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;

/* Start all PWM channels + encoders, force both motors to coast.
 * Call once from USER CODE BEGIN 2, after the MX_*_Init() calls. */
void motors_init(void);

/* Signed PWM. Positive = one direction, negative = the other, 0 = coast.
 * Magnitude clamped to PWM_MAX. If a wheel turns the wrong way, swap the
 * two channel constants for that motor in motor.c. */
void motorA(int16_t pwm);
void motorB(int16_t pwm);

void motors_stop(void);

uint16_t encA_count(void);
uint16_t encB_count(void);
int16_t  encA_delta(void);
int16_t  encB_delta(void);

#endif /* INC_MOTOR_H_ */
