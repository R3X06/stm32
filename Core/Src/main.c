/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MOTION + DISTANCE CALIBRATION BUILD  (C30D V2.1 / F407VET6)
  *
  *  Self-contained. Depends only on main.h, oled.h and the CubeMX-generated
  *  stm32f4xx_hal_msp.c / stm32f4xx_it.c already in the repo. No motors.c,
  *  encoders.c, odom.c, pid.c or calib.c required.
  *
  *  Four modes, cycled with a LONG press on the user button (PE0).
  *  SHORT press performs that mode's action. Any press while running aborts.
  *
  *    M1 ENC   motors coast. Raw signed encoder totals. Push the robot or
  *             spin a wheel by hand to find ENC_*_INVERT and counts/rev.
  *             SHORT = zero the totals.
  *    M2 DUTY  open loop, both wheels at OPEN_LOOP_DUTY_PCT. Confirms motor
  *             direction and gives a feel for deadband.  SHORT = start/stop.
  *    M3 RPM   closed-loop speed hold at RPM_SET. Confirms the PID.
  *             SHORT = start/stop.
  *    M4 DIST  drives DIST_TARGET_CM closed loop with an end taper, then
  *             reports counts over USART3.  SHORT = start/abort.
  *
  *  Telemetry on USART3 (PD8/PD9) 115200 8N1 — open it in PuTTY and the
  *  whole calibration session is logged for you.
  *
  *  SAFETY: M2/M3 spin the wheels the moment you short-press. Put the car on
  *  a stand for those two. Only M4 is a floor test.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "oled.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

/* ==========================================================================
 *  CALIBRATION CONSTANTS   <<<<<< THIS IS THE BLOCK YOU EDIT >>>>>>
 * ========================================================================== */

/* Encoder counts for one full output-shaft revolution.
   1320 = JGB37-520: 11 PPR x 30:1 gearbox x 4 edges. PROVISIONAL.
   Measure it in M1 before you trust anything downstream.                    */
#define TICKS_PER_REV        1320.0f

/* Encoder counts per centimetre of ground travel. PROVISIONAL, assumes a
   65 mm wheel: 1320 / (pi * 6.5) = 64.6. This is the number the whole
   calibration exists to determine. Update it after every M4 run:

       NEW = OLD * (commanded_cm / tape_measured_cm)                         */
#define COUNTS_PER_CM        64.6f

/* Distance M4 drives, in centimetres. Checklist A.3 wants 80-120 cm.        */
#define DIST_TARGET_CM       100.0f

/* Set to 1 if that wheel counts DOWN when the robot is pushed forward.
   Determine in M1: push the car forward by hand, both totals must rise.     */
#define ENC_A_INVERT         0
#define ENC_B_INVERT         1

/* Set to 1 if that wheel spins BACKWARD on a positive duty.
   Determine in M2: on a stand, both wheels must turn forward.               */
#define MOTOR_A_INVERT       0
#define MOTOR_B_INVERT       0

/* Steering servo, TIM12_CH2 @ 1 us/count. Centre from your servo calib.     */
#define SERVO_CENTER_US      1500

/* Speed setpoints */
#define RPM_SET              60.0f    /* cruise target, output shaft RPM     */
#define RPM_MIN              20.0f    /* floor during the end taper          */
#define TAPER_CM             15.0f    /* start slowing this far from target  */
#define RPM_SLEW_PER_TICK    3.0f     /* target ramp, RPM per 10 ms tick     */

/* Open-loop duty for M2, percent */
#define OPEN_LOOP_DUTY_PCT   50

/* Speed PID, operating in counts-per-tick.
   At 60 RPM: 1320 * 60/60 / 100 ticks-per-sec = 13.2 counts/tick.           */
#define KP                   80.0f
#define KI                   400.0f
#define KD                   0.0f
#define KFF                  55.0f    /* feedforward duty per count/tick     */

/* Abort a move that has not finished in this many ms */
#define MOVE_TIMEOUT_MS      15000u

/* ==========================================================================
 *  Derived / fixed
 * ========================================================================== */

#define PWM_MAX              4199     /* TIM4 and TIM9 ARR, both = 4199      */
#define TICK_MS              10
#define DT_S                 0.01f
#define TICKS_PER_SEC        100.0f
#define BTN_LONG_TICKS       60       /* 600 ms */
#define BRAKE_TICKS          40       /* 400 ms settle after stopping        */

/* counts/tick  ->  output-shaft RPM */
#define CPT_TO_RPM           ((TICKS_PER_SEC * 60.0f) / TICKS_PER_REV)

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;    /* encoder A  PA15 / PB3   */
TIM_HandleTypeDef htim3;    /* encoder B  PB4  / PB5   */
TIM_HandleTypeDef htim4;    /* motor A    PB9=CH4 PB8=CH3 */
TIM_HandleTypeDef htim6;    /* 10 ms control tick      */
TIM_HandleTypeDef htim9;    /* motor B    PE5=CH1 PE6=CH2 */
TIM_HandleTypeDef htim12;   /* servo      PB15         */
UART_HandleTypeDef huart3;  /* PD8 / PD9               */

/* Never initialised. They exist only so stm32f4xx_it.c links. The TIM8 and
   DMA2_Stream0 interrupts are never enabled, so their handlers never run. */
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_adc1;

typedef enum { M_ENC = 0, M_DUTY, M_RPM, M_DIST, M_COUNT } calmode_t;
typedef enum { R_IDLE = 0, R_RUN, R_BRAKE } runstate_t;

typedef struct { float integ; float prev_err; } pidctl_t;

static volatile calmode_t     g_mode  = M_ENC;
static volatile runstate_t g_state = R_IDLE;

static volatile int32_t g_totA = 0, g_totB = 0;   /* signed, sign-corrected  */
static volatile int16_t g_dA = 0,   g_dB = 0;     /* counts this tick        */
static uint16_t s_lastA = 0, s_lastB = 0;
static uint8_t  s_encPrimed = 0;

static volatile float g_rpmA = 0.0f, g_rpmB = 0.0f;   /* filtered, display   */
static float  s_rpmCmd = 0.0f;                        /* slewed setpoint     */
static pidctl_t  s_pidA, s_pidB;

static int32_t  s_targetCounts = 0;
static uint32_t s_runTicks = 0;
static uint16_t s_brakeTicks = 0;

/* Result of the last M4 run, latched for the main loop to print */
static volatile uint8_t  g_reportReady = 0;
static volatile int32_t  g_repA = 0, g_repB = 0, g_repAvg = 0, g_repTarget = 0;
static volatile uint32_t g_repMs = 0;
static volatile uint8_t  g_repTimeout = 0;

/* Button events, produced in the tick, consumed in the tick */
static volatile uint8_t g_evtShort = 0, g_evtLong = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);

static void MotorA_Set(int32_t duty);
static void MotorB_Set(int32_t duty);
static void Motors_Coast(void);
static void Motors_Brake(void);
static void Servo_Set(uint16_t us);
static void Enc_Zero(void);
static void Enc_Update(void);
static float Pid_Step(pidctl_t *p, float target_cpt, float meas_cpt);
static void Control_Tick(void);
static void Btn_Poll(void);
static void Display(void);
static void U3(const char *fmt, ...);
static void PrintFixed2(const char *label, int32_t val_x100, const char *unit);

/* ==========================================================================
 *  Motors — AT8236, PWM/PWM fast decay. Both inputs low = coast.
 *  Motor A: PB9 (Ain1) = TIM4_CH4 forward,  PB8 (Ain2) = TIM4_CH3 reverse
 *  Motor B: PE5 (Bin1) = TIM9_CH1 forward,  PE6 (Bin2) = TIM9_CH2 reverse
 * ========================================================================== */

static void MotorA_Set(int32_t duty)
{
#if MOTOR_A_INVERT
    duty = -duty;
#endif
    if (duty >  PWM_MAX) duty =  PWM_MAX;
    if (duty < -PWM_MAX) duty = -PWM_MAX;

    if (duty >= 0) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint32_t)duty);
    } else {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)(-duty));
    }
}

static void MotorB_Set(int32_t duty)
{
#if MOTOR_B_INVERT
    duty = -duty;
#endif
    if (duty >  PWM_MAX) duty =  PWM_MAX;
    if (duty < -PWM_MAX) duty = -PWM_MAX;

    if (duty >= 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (uint32_t)duty);
    } else {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, (uint32_t)(-duty));
    }
}

static void Motors_Coast(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
}

/* Both inputs high = brake. Stops the coast-on that would otherwise smear
   the distance measurement. */
static void Motors_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_MAX + 1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_MAX + 1);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_MAX + 1);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_MAX + 1);
}

static void Servo_Set(uint16_t us)
{
    if (us < 1000u) us = 1000u;
    if (us > 2000u) us = 2000u;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
}

/* ==========================================================================
 *  Encoders — wrap-safe signed deltas. TIM2 ARR is forced to 65535 so both
 *  timers can be treated as 16-bit and the same (int16_t) cast works.
 * ========================================================================== */

static void Enc_Zero(void)
{
    __disable_irq();
    s_lastA = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_lastB = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_totA = 0; g_totB = 0;
    g_dA = 0;   g_dB = 0;
    g_rpmA = 0.0f; g_rpmB = 0.0f;
    s_encPrimed = 1;
    __enable_irq();
}

static void Enc_Update(void)
{
    uint16_t nA = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t nB = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t  dA, dB;

    if (!s_encPrimed) { s_lastA = nA; s_lastB = nB; s_encPrimed = 1; return; }

    dA = (int16_t)(nA - s_lastA);
    dB = (int16_t)(nB - s_lastB);
    s_lastA = nA;
    s_lastB = nB;

#if ENC_A_INVERT
    dA = (int16_t)(-dA);
#endif
#if ENC_B_INVERT
    dB = (int16_t)(-dB);
#endif

    g_dA = dA;
    g_dB = dB;
    g_totA += dA;
    g_totB += dB;

    /* light EMA so the display is readable */
    g_rpmA += 0.25f * (((float)dA * CPT_TO_RPM) - g_rpmA);
    g_rpmB += 0.25f * (((float)dB * CPT_TO_RPM) - g_rpmB);
}

/* ==========================================================================
 *  Speed PID, in counts-per-tick
 * ========================================================================== */

static float Pid_Step(pidctl_t *p, float target_cpt, float meas_cpt)
{
    float err = target_cpt - meas_cpt;
    float out, d;

    p->integ += err * DT_S;
    if (KI > 0.0f) {                       /* clamp the I contribution */
        float lim = (float)PWM_MAX / KI;
        if (p->integ >  lim) p->integ =  lim;
        if (p->integ < -lim) p->integ = -lim;
    }

    d = (err - p->prev_err) / DT_S;
    p->prev_err = err;

    out = (KFF * target_cpt) + (KP * err) + (KI * p->integ) + (KD * d);

    if (out >  (float)PWM_MAX) out =  (float)PWM_MAX;
    if (out < -(float)PWM_MAX) out = -(float)PWM_MAX;
    return out;
}

static void Pid_Reset(void)
{
    s_pidA.integ = 0.0f; s_pidA.prev_err = 0.0f;
    s_pidB.integ = 0.0f; s_pidB.prev_err = 0.0f;
    s_rpmCmd = 0.0f;
}

/* ==========================================================================
 *  Button, PE0, active low, external 10k pull-up
 * ========================================================================== */

static void Btn_Poll(void)
{
    static uint8_t  stable = 1, cnt = 0;
    static uint16_t held = 0;
    static uint8_t  longFired = 0;

    uint8_t raw = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_SET) ? 1u : 0u;

    if (raw != stable) {
        if (++cnt >= 3) { stable = raw; cnt = 0; }
    } else {
        cnt = 0;
    }

    if (stable == 0u) {                    /* pressed */
        if (held < 60000u) held++;
        if (!longFired && held >= BTN_LONG_TICKS) { g_evtLong = 1; longFired = 1; }
    } else {                               /* released */
        if (held > 0u && !longFired) g_evtShort = 1;
        held = 0;
        longFired = 0;
    }
}

/* ==========================================================================
 *  10 ms control tick — called from TIM6
 * ========================================================================== */

static void Start_Run(void)
{
    Pid_Reset();
    Enc_Zero();
    Servo_Set(SERVO_CENTER_US);
    s_runTicks = 0;
    s_targetCounts = (int32_t)(DIST_TARGET_CM * COUNTS_PER_CM);
    g_state = R_RUN;
}

static void Stop_Run(uint8_t timedOut)
{
    Motors_Brake();
    g_repA       = g_totA;
    g_repB       = g_totB;
    g_repTarget  = s_targetCounts;
    g_repMs      = s_runTicks * TICK_MS;
    g_repTimeout = timedOut;
    s_brakeTicks = 0;
    g_state = R_BRAKE;
}

static void Control_Tick(void)
{
    Enc_Update();
    Btn_Poll();

    /* ---- button events ---- */
    if (g_evtLong) {
        g_evtLong = 0;
        if (g_state != R_IDLE) {
            Stop_Run(0);
        } else {
            g_mode = (calmode_t)((g_mode + 1) % M_COUNT);
            Motors_Coast();
            Enc_Zero();
        }
    }
    if (g_evtShort) {
        g_evtShort = 0;
        if (g_state != R_IDLE) {
            Stop_Run(0);
        } else {
            switch (g_mode) {
            case M_ENC:
                Enc_Zero();
                break;
            case M_DUTY:
            case M_RPM:
                Pid_Reset();
                Enc_Zero();
                Servo_Set(SERVO_CENTER_US);
                s_runTicks = 0;
                s_targetCounts = 0;
                g_state = R_RUN;
                break;
            case M_DIST:
                Start_Run();
                break;
            default:
                break;
            }
        }
    }

    /* ---- brake settle ---- */
    if (g_state == R_BRAKE) {
        if (++s_brakeTicks >= BRAKE_TICKS) {
            Motors_Coast();
            /* counts AFTER the wheels have actually stopped — this is the
               figure that matches the tape measure */
            g_repAvg = (g_totA + g_totB) / 2;
            g_repA   = g_totA;
            g_repB   = g_totB;
            g_reportReady = 1;
            g_state = R_IDLE;
        }
        return;
    }

    if (g_state == R_IDLE) { Motors_Coast(); return; }

    /* ---- running ---- */
    s_runTicks++;
    if ((s_runTicks * TICK_MS) > MOVE_TIMEOUT_MS) { Stop_Run(1); return; }

    if (g_mode == M_DUTY) {
        int32_t duty = (PWM_MAX * OPEN_LOOP_DUTY_PCT) / 100;
        MotorA_Set(duty);
        MotorB_Set(duty);
        return;
    }

    /* M_RPM and M_DIST both run the speed PID */
    {
        float rpmWanted = RPM_SET;
        float tgt_cpt, outA, outB;

        if (g_mode == M_DIST) {
            int32_t travelled = (g_totA + g_totB) / 2;
            int32_t remaining = s_targetCounts - travelled;
            int32_t taper     = (int32_t)(TAPER_CM * COUNTS_PER_CM);

            if (remaining <= 0) { Stop_Run(0); return; }

            if (taper > 0 && remaining < taper) {
                rpmWanted = RPM_MIN +
                    (RPM_SET - RPM_MIN) * ((float)remaining / (float)taper);
                if (rpmWanted < RPM_MIN) rpmWanted = RPM_MIN;
            }
        }

        /* slew the setpoint so we do not break traction on launch */
        if (s_rpmCmd < rpmWanted) {
            s_rpmCmd += RPM_SLEW_PER_TICK;
            if (s_rpmCmd > rpmWanted) s_rpmCmd = rpmWanted;
        } else if (s_rpmCmd > rpmWanted) {
            s_rpmCmd -= RPM_SLEW_PER_TICK;
            if (s_rpmCmd < rpmWanted) s_rpmCmd = rpmWanted;
        }

        tgt_cpt = s_rpmCmd / CPT_TO_RPM;

        outA = Pid_Step(&s_pidA, tgt_cpt, (float)g_dA);
        outB = Pid_Step(&s_pidB, tgt_cpt, (float)g_dB);

        MotorA_Set((int32_t)outA);
        MotorB_Set((int32_t)outB);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        Control_Tick();
    }
}

/* ==========================================================================
 *  USART3 helpers.  No %f anywhere — newlib-nano will not print floats
 *  unless you link the float printf, so everything is scaled integers.
 * ========================================================================== */

static void U3(const char *fmt, ...)
{
    char buf[96];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 200);
    }
}

static void PrintFixed2(const char *label, int32_t val_x100, const char *unit)
{
    int32_t whole = val_x100 / 100;
    int32_t frac  = val_x100 % 100;
    if (frac < 0) frac = -frac;
    U3("%s%ld.%02ld%s\r\n", label, (long)whole, (long)frac, unit);
}

/* ==========================================================================
 *  OLED — 16 chars per line, 4 lines. Lines are padded so old text clears.
 * ========================================================================== */

static void ShowLine(uint8_t y, const char *s)
{
    char pad[17];
    int i = 0;
    while (s[i] != '\0' && i < 16) { pad[i] = s[i]; i++; }
    while (i < 16) { pad[i++] = ' '; }
    pad[16] = '\0';
    OLED_ShowString(0, y, (const uint8_t *)pad);
}

static void Display(void)
{
    static const char *names[M_COUNT] = { "ENC ", "DUTY", "RPM ", "DIST" };
    char l[24];
    int32_t tA = g_totA, tB = g_totB;

    snprintf(l, sizeof(l), "M%d %s %s", (int)g_mode + 1, names[g_mode],
             (g_state == R_IDLE) ? "---" : "RUN");
    ShowLine(0, l);

    snprintf(l, sizeof(l), "A%+7ld %3d", (long)tA, (int)g_rpmA);
    ShowLine(16, l);

    snprintf(l, sizeof(l), "B%+7ld %3d", (long)tB, (int)g_rpmB);
    ShowLine(32, l);

    switch (g_mode) {
    case M_ENC: {
        /* revolutions x10, from each wheel */
        int32_t rA = (int32_t)((float)tA * 10.0f / TICKS_PER_REV);
        int32_t rB = (int32_t)((float)tB * 10.0f / TICKS_PER_REV);
        snprintf(l, sizeof(l), "rev A%3ld B%3ld", (long)rA, (long)rB);
        break;
    }
    case M_DUTY:
        snprintf(l, sizeof(l), "OPEN LOOP %2d%%", OPEN_LOOP_DUTY_PCT);
        break;
    case M_RPM:
        snprintf(l, sizeof(l), "TGT %3d RPM", (int)s_rpmCmd);
        break;
    case M_DIST: {
        int32_t mmNow = (int32_t)(((float)((tA + tB) / 2) / COUNTS_PER_CM) * 10.0f);
        int32_t mmTgt = (int32_t)(DIST_TARGET_CM * 10.0f);
        snprintf(l, sizeof(l), "%5ldmm/%4ld", (long)mmNow, (long)mmTgt);
        break;
    }
    default:
        l[0] = '\0';
        break;
    }
    ShowLine(48, l);

    OLED_Refresh_Gram();
}

/* ==========================================================================
 *  main
 * ========================================================================== */

int main(void)
{
    uint32_t tDisp = 0, tLed = 0, tTele = 0;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();

    /* Motor PWM up and at zero BEFORE anything can spin. Floating AT8236
       inputs are a confirmed runaway mode on this board. */
    MX_TIM4_Init();
    MX_TIM9_Init();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    MX_TIM12_Init();
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    Servo_Set(SERVO_CENTER_US);

    MX_TIM2_Init();
    MX_TIM3_Init();
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    MX_USART3_UART_Init();
    OLED_Init();
    OLED_Clear();

    Enc_Zero();
    Pid_Reset();

    U3("\r\n=== C30D CALIBRATION BUILD ===\r\n");
    U3("TICKS_PER_REV %d\r\n", (int)TICKS_PER_REV);
    PrintFixed2("COUNTS_PER_CM ", (int32_t)(COUNTS_PER_CM * 100.0f), "");
    PrintFixed2("DIST_TARGET   ", (int32_t)(DIST_TARGET_CM * 100.0f), " cm");
    U3("ENC_INV A%d B%d  MOT_INV A%d B%d\r\n",
       ENC_A_INVERT, ENC_B_INVERT, MOTOR_A_INVERT, MOTOR_B_INVERT);
    U3("LONG press = mode, SHORT = action\r\n\r\n");

    /* Tick LAST — nothing fires against an uninitialised module. */
    MX_TIM6_Init();
    HAL_TIM_Base_Start_IT(&htim6);

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (now - tDisp >= 150u) { tDisp = now; Display(); }

        /* LED3 (PE8, active low): slow blink idle, fast blink running */
        if (now - tLed >= ((g_state == R_IDLE) ? 500u : 100u)) {
            tLed = now;
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
        }

        /* live telemetry while running */
        if (g_state == R_RUN && (now - tTele >= 250u)) {
            tTele = now;
            U3("A %ld (%d rpm)  B %ld (%d rpm)\r\n",
               (long)g_totA, (int)g_rpmA, (long)g_totB, (int)g_rpmB);
        }

        /* end-of-run report */
        if (g_reportReady) {
            int32_t avg, est_x100, tgt;
            g_reportReady = 0;

            avg = g_repAvg;
            tgt = g_repTarget;
            est_x100 = (int32_t)(((float)avg / COUNTS_PER_CM) * 100.0f);

            U3("\r\n--- RUN %s ---\r\n", g_repTimeout ? "TIMEOUT" : "DONE");
            U3("counts  A %ld  B %ld  avg %ld\r\n",
               (long)g_repA, (long)g_repB, (long)avg);
            U3("target counts %ld   time %lu ms\r\n",
               (long)tgt, (unsigned long)g_repMs);
            PrintFixed2("commanded  ", (int32_t)(DIST_TARGET_CM * 100.0f), " cm");
            PrintFixed2("est travel ", est_x100, " cm");
            U3("Now tape-measure it, then set\r\n");
            U3("COUNTS_PER_CM = %ld.%02ld * %d / measured_cm\r\n\r\n",
               (long)((int32_t)(COUNTS_PER_CM * 100.0f) / 100),
               (long)((int32_t)(COUNTS_PER_CM * 100.0f) % 100),
               (int)DIST_TARGET_CM);
        }
    }
}

/* ==========================================================================
 *  Peripheral init. GPIO/AF/NVIC for every peripheral below is already
 *  handled by the generated stm32f4xx_hal_msp.c, so these only set registers.
 * ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8;
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;    /* PCLK1 42 MHz, timers 84 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;    /* PCLK2 84 MHz, timers 168 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* TIM2 — encoder A, PA15 / PB3. 32-bit part, ARR forced to 65535 so the
   (int16_t) delta cast is valid. */
static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef enc = {0};
    TIM_MasterConfigTypeDef mst = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 65535;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 10;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 10;
    if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &mst) != HAL_OK) Error_Handler();
}

/* TIM3 — encoder B, PB4 / PB5 */
static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef enc = {0};
    TIM_MasterConfigTypeDef mst = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 10;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 10;
    if (HAL_TIM_Encoder_Init(&htim3, &enc) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &mst) != HAL_OK) Error_Handler();
}

/* TIM4 — motor A PWM. APB1 timer clock 84 MHz, ARR 4199 -> 20 kHz. */
static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef mst = {0};
    TIM_OC_InitTypeDef      oc  = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = PWM_MAX;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &mst) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
}

/* TIM6 — 10 ms control tick. 84 MHz / 8400 = 10 kHz, / 100 = 100 Hz. */
static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef mst = {0};

    htim6.Instance           = TIM6;
    htim6.Init.Prescaler     = 8399;
    htim6.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim6.Init.Period        = 99;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

    mst.MasterOutputTrigger = TIM_TRGO_RESET;
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &mst) != HAL_OK) Error_Handler();
}

/* TIM9 — motor B PWM. APB2 timer clock 168 MHz, PSC 1 -> 84 MHz,
   ARR 4199 -> 20 kHz, identical to TIM4. */
static void MX_TIM9_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    htim9.Instance               = TIM9;
    htim9.Init.Prescaler         = 1;
    htim9.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim9.Init.Period            = PWM_MAX;
    htim9.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim9);
}

/* TIM12 — servo. APB1 timer clock 84 MHz, PSC 83 -> 1 MHz (1 us/count),
   ARR 19999 -> 50 Hz. NOTE: PSC is 83, not 167. TIM12 is on APB1. */
static void MX_TIM12_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    htim12.Instance               = TIM12;
    htim12.Init.Prescaler         = 83;
    htim12.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim12.Init.Period            = 19999;
    htim12.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = SERVO_CENTER_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim12);
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

/* Only the pins the MSP does not own: OLED bit-bang, LED3, user button,
   and the ultrasonic trigger held low so it cannot chirp. */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* OLED: PD11 DC, PD12 RES, PD13 SDA, PD14 SCL */
    g.Pin   = OLED_DC_Pin | OLED_RES_Pin | OLED_SDA_Pin | OLED_SCL_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &g);
    HAL_GPIO_WritePin(GPIOD, g.Pin, GPIO_PIN_RESET);

    /* LED3 PE8, active low -> start off */
    g.Pin   = LED3_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);
    HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);

    /* User button PE0, active low, external 10k pull-up already fitted */
    g.Pin  = GPIO_PIN_0;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &g);

    /* Ultrasonic trigger PB14 parked low */
    g.Pin   = US_Trig_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, US_Trig_Pin, GPIO_PIN_RESET);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
