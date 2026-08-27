/*
 * motor.c  -  WHEELTEC C30D V2.1, AT8236 PWM/PWM fast decay.
 */

#include "motor.h"

/* Motor A -> TIM4 : CH4 = PB9 = IN1, CH3 = PB8 = IN2 */
#define A_TIM       (&htim4)
#define A_CH_FWD    TIM_CHANNEL_4      /* PB9 / IN1 */
#define A_CH_REV    TIM_CHANNEL_3      /* PB8 / IN2 */

/* Motor B -> TIM9 : CH1 = PE5 = IN1, CH2 = PE6 = IN2 */
#define B_TIM       (&htim9)
#define B_CH_FWD    TIM_CHANNEL_1      /* PE5 / IN1 */
#define B_CH_REV    TIM_CHANNEL_2      /* PE6 / IN2 */

/* Set to 1 if that wheel turns the wrong way. Measure, don't guess. */
#define A_INVERT    0
#define B_INVERT    0

static uint16_t encA_prev, encB_prev;

static void drive(TIM_HandleTypeDef *tim, uint32_t fwd, uint32_t rev,
                  int16_t pwm, uint8_t invert)
{
    if (invert) pwm = -pwm;

    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    /* Below deadband there is no useful torque - coast instead of buzzing. */
    if (pwm > -PWM_DEADBAND && pwm < PWM_DEADBAND) {
        __HAL_TIM_SET_COMPARE(tim, fwd, 0);
        __HAL_TIM_SET_COMPARE(tim, rev, 0);
        return;
    }

    if (pwm > 0) {
        __HAL_TIM_SET_COMPARE(tim, rev, 0);
        __HAL_TIM_SET_COMPARE(tim, fwd, (uint32_t)pwm);
    } else {
        __HAL_TIM_SET_COMPARE(tim, fwd, 0);
        __HAL_TIM_SET_COMPARE(tim, rev, (uint32_t)(-pwm));
    }
}

void motors_init(void)
{
    /* CCRs to zero BEFORE the outputs go live, so nothing twitches at boot. */
    __HAL_TIM_SET_COMPARE(A_TIM, A_CH_FWD, 0);
    __HAL_TIM_SET_COMPARE(A_TIM, A_CH_REV, 0);
    __HAL_TIM_SET_COMPARE(B_TIM, B_CH_FWD, 0);
    __HAL_TIM_SET_COMPARE(B_TIM, B_CH_REV, 0);

    HAL_TIM_PWM_Start(A_TIM, A_CH_FWD);
    HAL_TIM_PWM_Start(A_TIM, A_CH_REV);
    HAL_TIM_PWM_Start(B_TIM, B_CH_FWD);
    HAL_TIM_PWM_Start(B_TIM, B_CH_REV);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    encA_prev = encB_prev = 0;
}

void motorA(int16_t pwm) { drive(A_TIM, A_CH_FWD, A_CH_REV, pwm, A_INVERT); }
void motorB(int16_t pwm) { drive(B_TIM, B_CH_FWD, B_CH_REV, pwm, B_INVERT); }

void motors_stop(void)
{
    __HAL_TIM_SET_COMPARE(A_TIM, A_CH_FWD, 0);
    __HAL_TIM_SET_COMPARE(A_TIM, A_CH_REV, 0);
    __HAL_TIM_SET_COMPARE(B_TIM, B_CH_FWD, 0);
    __HAL_TIM_SET_COMPARE(B_TIM, B_CH_REV, 0);
}

uint16_t encA_count(void) { return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2); }
uint16_t encB_count(void) { return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3); }

/* uint16 subtraction then reinterpret as signed: handles wrap in both
 * directions with no branch. Valid because TIM2's ARR is forced to 65535. */
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
