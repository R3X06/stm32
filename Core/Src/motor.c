/*
 * motor.c
 *
 * Motor + encoder driver for WHEELTEC C30D V2.1
 * AT8236 PWM/PWM topology.
 */

#include "motor.h"

/* Motor A -> TIM4 : CH3 = PB8, CH4 = PB9 */
#define A_TIM       (&htim4)
#define A_CH_FWD    TIM_CHANNEL_3
#define A_CH_REV    TIM_CHANNEL_4

/* Motor B -> TIM9 : CH1 = PE5, CH2 = PE6 */
#define B_TIM       (&htim9)
#define B_CH_FWD    TIM_CHANNEL_1
#define B_CH_REV    TIM_CHANNEL_2

static uint16_t encA_prev = 0;
static uint16_t encB_prev = 0;

void motors_init(void)
{
    HAL_TIM_PWM_Start(A_TIM, A_CH_FWD);
    HAL_TIM_PWM_Start(A_TIM, A_CH_REV);
    HAL_TIM_PWM_Start(B_TIM, B_CH_FWD);
    HAL_TIM_PWM_Start(B_TIM, B_CH_REV);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    encA_prev = 0;
    encB_prev = 0;

    motors_stop();
}

void motorA(int16_t pwm)
{
    if (pwm > PWM_MAX)  pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm >= 0) {
        __HAL_TIM_SET_COMPARE(A_TIM, A_CH_REV, 0);
        __HAL_TIM_SET_COMPARE(A_TIM, A_CH_FWD, (uint16_t)pwm);
    } else {
        __HAL_TIM_SET_COMPARE(A_TIM, A_CH_FWD, 0);
        __HAL_TIM_SET_COMPARE(A_TIM, A_CH_REV, (uint16_t)(-pwm));
    }
}

void motorB(int16_t pwm)
{
    if (pwm > PWM_MAX)  pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm >= 0) {
        __HAL_TIM_SET_COMPARE(B_TIM, B_CH_REV, 0);
        __HAL_TIM_SET_COMPARE(B_TIM, B_CH_FWD, (uint16_t)pwm);
    } else {
        __HAL_TIM_SET_COMPARE(B_TIM, B_CH_FWD, 0);
        __HAL_TIM_SET_COMPARE(B_TIM, B_CH_REV, (uint16_t)(-pwm));
    }
}

void motors_stop(void)
{
    __HAL_TIM_SET_COMPARE(A_TIM, A_CH_FWD, 0);
    __HAL_TIM_SET_COMPARE(A_TIM, A_CH_REV, 0);
    __HAL_TIM_SET_COMPARE(B_TIM, B_CH_FWD, 0);
    __HAL_TIM_SET_COMPARE(B_TIM, B_CH_REV, 0);
}

uint16_t encA_count(void) { return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2); }
uint16_t encB_count(void) { return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3); }

int16_t encA_delta(void)
{
    uint16_t now = encA_count();
    int16_t  d   = (int16_t)(now - encA_prev);
    encA_prev = now;
    return d;
}

int16_t encB_delta(void)
{
    uint16_t now = encB_count();
    int16_t  d   = (int16_t)(now - encB_prev);
    encB_prev = now;
    return d;
}
