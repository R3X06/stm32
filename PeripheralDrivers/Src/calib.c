#include "calib.h"
#include "motors.h"
#include "encoders.h"
#include "pid.h"
#include "odom.h"
#include "oled.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim12;

#define BTN_PORT    GPIOE
#define BTN_PIN     GPIO_PIN_0      /* user button, active low, ext 10k pull-up */

static uint16_t s_us;

/* ------------------------------------------------------------------ */
/* Button                                                              */
/* ------------------------------------------------------------------ */

/* Blocks while held. Returns 0 = nothing, 1 = short press, 2 = long press. */
static uint8_t Btn_Poll(void)
{
    uint32_t t0, held;

    if (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) != GPIO_PIN_RESET)
    {
        return 0;
    }

    t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_RESET)
    {
        if (HAL_GetTick() - t0 > 4000U) break;
    }
    held = HAL_GetTick() - t0;

    if (held < 40U)  return 0;          /* contact bounce */
    return (held > 800U) ? 2 : 1;
}

/* ------------------------------------------------------------------ */
/* Servo, unclamped                                                    */
/* ------------------------------------------------------------------ */

/* Writes CCR directly. Bypasses the SERVO_MIN_US/SERVO_MAX_US clamp on
 * purpose - endstop mode has to be able to reach past the current guess. */
static void Calib_SetRaw(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
}

static void Calib_Show(const char *tag, int32_t value)
{
    char line[24];
    snprintf(line, sizeof(line), "%s", tag);
    OLED_ShowString(0, 0, (const uint8_t *)line);
    snprintf(line, sizeof(line), "US %4u", (unsigned)s_us);
    OLED_ShowString(0, 16, (const uint8_t *)line);
    snprintf(line, sizeof(line), "D %5ld mm", (long)value);
    OLED_ShowString(0, 32, (const uint8_t *)line);
    OLED_Refresh_Gram();
}

/* ------------------------------------------------------------------ */
/* Straight run - no heading hold                                      */
/* ------------------------------------------------------------------ */

/* Both wheels at the same RPM under speed PID, steering held at s_us.
 * Heading hold stays OFF: the whole point is to see what the steering
 * alone does. If the trim were active it would mask the error you are
 * trying to measure. */
static void Calib_Run2m(void)
{
    uint32_t t0;

    Odom_Stop();                /* also clears heading hold */
    Calib_SetRaw(s_us);
    HAL_Delay(400);             /* let the steering settle before moving */

    Encoders_Reset();
    Odom_Reset();

    PID_Enable(1);
    PID_SetTargets(CALIB_RUN_RPM, CALIB_RUN_RPM);

    t0 = HAL_GetTick();
    while (Odom_GetDistance() < CALIB_RUN_MM)
    {
        if (HAL_GetTick() - t0 > CALIB_RUN_TIMEOUT_MS) break;
        Calib_Show("RUNNING", (int32_t)Odom_GetDistance());
        HAL_Delay(50);
    }

    PID_SetTargets(0, 0);
    Motors_Brake();
    HAL_Delay(400);
    PID_Enable(0);
    Motors_Coast();

    /* Final reading stays on screen so you can note it with the tape. */
    Calib_Show("DONE", (int32_t)Odom_GetDistance());
}

/* ------------------------------------------------------------------ */

void Calib_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    g.Pin   = BTN_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_NOPULL;          /* external 10k pull-up on the board */
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BTN_PORT, &g);

    s_us = SERVO_CENTER_US;
    Calib_SetRaw(s_us);
}

void Calib_Run(void)
{
    PID_Enable(0);
    Motors_Coast();

#if (CALIB_MODE == CALIB_MODE_ENDSTOP)
    Calib_Show("ENDSTOP", 0);
#else
    Calib_Show("CENTRE", 0);
#endif

    for (;;)
    {
        uint8_t ev = Btn_Poll();

        if (ev == 1)                        /* short press: step up, wrap */
        {
            s_us += CALIB_STEP_US;
            if (s_us > CALIB_SPAN_MAX_US) s_us = CALIB_SPAN_MIN_US;
            Calib_SetRaw(s_us);
#if (CALIB_MODE == CALIB_MODE_ENDSTOP)
            Calib_Show("ENDSTOP", 0);
#else
            Calib_Show("CENTRE", 0);
#endif
        }
        else if (ev == 2)                   /* long press */
        {
#if (CALIB_MODE == CALIB_MODE_ENDSTOP)
            /* Nothing to drive. Jump back to centre as a sanity check. */
            s_us = SERVO_CENTER_US;
            Calib_SetRaw(s_us);
            Calib_Show("RECENTRED", 0);
#else
            Calib_Run2m();
#endif
        }

        HAL_Delay(20);
    }
}
