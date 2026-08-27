/*
 * motor.h
 *
 * Motor + encoder driver for WHEELTEC C30D V2.1 (STM32F407VET6)
 *
 * Topology: AT8236 dual-input H-bridge, PWM/PWM, fast decay.
 *   Forward : PWM on one input, other held at 0
 *   Reverse : swap which input is PWM'd
 *   Stop    : both 0 (coast)
 *
 * Pin map verified against C30D V2.1 schematic sheet 3/3:
 *   U8 (Motor A) : IN1 = PB9 (TIM4_CH4), IN2 = PB8 (TIM4_CH3)
 *   U9 (Motor B) : IN1 = PE5 (TIM9_CH1), IN2 = PE6 (TIM9_CH2)
 *   Encoder A    : TIM2  PA15 / PB3      (32-bit, ARR forced to 65535)
 *   Encoder B    : TIM3  PB4  / PB5      (16-bit)
 *
 * Clock: HSE 8 MHz -> PLL(M=4,N=72,P=2) -> 72 MHz SYSCLK.
 * APB1 /2 and APB2 /1 both yield a 72 MHz timer clock, so TIM4 and TIM9
 * share an identical timebase. ARR 7199 -> 10 kHz PWM on both motors.
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

#define PWM_MAX        7199   /* = ARR. 100% duty */
#define PWM_DEADBAND   1000   /* below this the motor stalls, don't bother */

/* PROVISIONAL - must be measured. Reference repo uses 330 PPR x4 = 1320.
 * Mark a wheel, rotate exactly 10 turns by hand, read encA_count()/10. */
#define TICKS_PER_REV  1320
#define WHEEL_DIAM_CM  5.7f

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;

void motors_init(void);
void motorA(int16_t pwm);      /* signed; 0 = coast, sign = direction */
void motorB(int16_t pwm);
void motors_stop(void);

uint16_t encA_count(void);
uint16_t encB_count(void);
int16_t  encA_delta(void);
int16_t  encB_delta(void);

#endif /* INC_MOTOR_H_ */
