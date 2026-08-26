/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MDP-G15 robot firmware
  *                   Steps 3-7: drive layer, encoder read, speed PID,
  *                   drive_straight/rotate, command protocol scaffold.
  *
  *   HARDWARE STATUS: Motor B driver channel on the board is faulty (stuck).
  *   Everything below is written for BOTH wheels but is bench-testable on
  *   Motor A + both encoders + servo NOW. When the replacement board arrives,
  *   no code changes are needed - flash, run factory self-test, then validate.
  *
  *   BUILD MODES (see USER CODE BEGIN PD):
  *     TEST_MODE 1 = single-wheel bench test on Motor A (use while board faulty)
  *     TEST_MODE 0 = normal two-wheel operation (use when board replaced)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "commands.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* command protocol types (scmd_type_t, script_item_t) now live in commands.h */
typedef struct { float integral; float prev_err; } pid_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- Build mode ---- */
#define TEST_MODE          1        /* 1 = single-wheel bench (Motor A), 0 = two-wheel */

/* ---- PWM (TIM8 ARR = 7199) ---- */
#define PWM_MAX            7199
#define PWM_MIN_MOVE       1000     /* min duty to overcome static friction */

/* ---- Encoder / distance ---- */
#define ENC_PPR            330      /* CONFIRM against your motor datasheet */
#define ENC_X4             4
#define ENC_CPR            (ENC_PPR * ENC_X4)
#define WHEEL_DIAM_CM      6.0f     /* CONFIRM by measuring your wheel */
#define WHEEL_CIRC_CM      (3.14159f * WHEEL_DIAM_CM)
#define COUNTS_PER_CM      ((float)ENC_CPR / WHEEL_CIRC_CM)

/* ---- Servo (TIM12, ARR=19999 -> 1 count = 1us) ---- */
/* PLACEHOLDERS - run servo_sweep(), read the wheel angle, replace these */
#define SERVO_CENTER_CCR   1500
#define SERVO_LEFT_CCR     1900
#define SERVO_RIGHT_CCR    1100

/* ---- Speed PID (per wheel) ---- */
#define PID_DT_MS          20       /* control loop period */
#define KP_SPEED           8.0f
#define KI_SPEED           0.5f
#define KD_SPEED           0.0f
#define TARGET_CPS         2000.0f  /* target counts/sec during a straight move */

/* ---- Command queue ---- */
#define CMDQ_CAP           32
#define INTER_CMD_MS       200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char buf[48];
volatile uint8_t run_test = 0;               /* set by user button (rising edge) */
volatile uint8_t test_sel = 0;               /* which bench test is selected */
#define N_TESTS 3

/* encoder deltas (counts per PID tick), signed, wrap-safe */
volatile int16_t deltaA = 0, deltaB = 0;

/* per-wheel speed PID state */
static pid_state_t pidA = {0}, pidB = {0};

/* command queue (ring buffer) */
static volatile script_item_t cmdq[CMDQ_CAP];
static volatile uint8_t q_head = 0, q_tail = 0;

/* UART RX (Person B wires the actual receive; buffer + parser live here) */
volatile uint8_t rx_byte;
char cmd_buf[128];
volatile uint16_t cmd_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void motorA_drive(int16_t duty);
void motorB_drive(int16_t duty);
void motors_brake(void);
void servo_set(uint16_t ccr);
void update_encoders(void);
float pid_step(pid_state_t *s, float target_cps, float meas_cps);
void drive_straight(float distance_cm);
void rotate(float degrees, uint8_t left);
void servo_sweep(void);
static int cmdq_push(script_item_t it);
static int cmdq_pop(script_item_t *out);
void mdp_enqueue_line(char *line);
void run_command(script_item_t it);
void uart_puts(const char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart_puts(const char *s){
  HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* -------- Drive layer (PWM+DIR; positive duty = forward) -------- */
void motorA_drive(int16_t duty){        /* Motor A / LEFT : TIM8 CH1, AIN1/AIN2 */
    if (duty >= 0){
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
        duty = -duty;
    }
    if (duty > PWM_MAX) duty = PWM_MAX;
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, duty);
}

void motorB_drive(int16_t duty){        /* Motor B / RIGHT : TIM8 CH2, BIN1/BIN2 */
    if (duty >= 0){
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
        duty = -duty;
    }
    if (duty > PWM_MAX) duty = PWM_MAX;
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, duty);
}

void motors_brake(void){
    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
}

void servo_set(uint16_t ccr){ __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, ccr); }

/* -------- Encoder read: signed wrap-safe delta since last call -------- */
void update_encoders(void){
    static uint16_t lastA = 0, lastB = 0;
    static uint8_t init = 0;
    uint16_t nowA = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t nowB = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    if (!init){ lastA = nowA; lastB = nowB; init = 1; deltaA = deltaB = 0; return; }
    deltaA = (int16_t)(nowA - lastA);
    deltaB = (int16_t)(nowB - lastB);
    /* deltaB = -deltaB; // UNCOMMENT if right wheel counts backward vs left (set on hardware) */
    lastA = nowA; lastB = nowB;
}

/* -------- Speed PID: returns duty for a target counts/sec -------- */
float pid_step(pid_state_t *s, float target_cps, float meas_cps){
    float err = target_cps - meas_cps;
    s->integral += err * (PID_DT_MS / 1000.0f);
    float i_max = PWM_MAX / (KI_SPEED > 0 ? KI_SPEED : 1);
    if (s->integral >  i_max) s->integral =  i_max;
    if (s->integral < -i_max) s->integral = -i_max;
    float deriv = (err - s->prev_err) / (PID_DT_MS / 1000.0f);
    s->prev_err = err;
    float out = KP_SPEED*err + KI_SPEED*s->integral + KD_SPEED*deriv;
    if (out >  PWM_MAX) out =  PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;
    return out;
}

/* -------- drive_straight: closed-loop, distance in cm (+fwd / -rev) -------- */
void drive_straight(float distance_cm){
    int32_t target_counts = (int32_t)(fabsf(distance_cm) * COUNTS_PER_CM);
    int8_t  dir = (distance_cm >= 0) ? 1 : -1;
    int32_t travelledA = 0, travelledB = 0;

    pidA.integral = pidA.prev_err = 0;
    pidB.integral = pidB.prev_err = 0;
    servo_set(SERVO_CENTER_CCR);
    update_encoders();  /* prime */

    uint32_t t_last = HAL_GetTick();
    while (1){
        if ((HAL_GetTick() - t_last) < PID_DT_MS) continue;
        t_last += PID_DT_MS;

        update_encoders();
        travelledA += (dir > 0 ? deltaA : -deltaA);
        travelledB += (dir > 0 ? deltaB : -deltaB);

#if TEST_MODE
        int32_t progress = travelledA;
#else
        int32_t progress = (travelledA + travelledB) / 2;
#endif
        if (progress >= target_counts) break;

        float cpsA = (float)deltaA * (1000.0f / PID_DT_MS);
        float cpsB = (float)deltaB * (1000.0f / PID_DT_MS);
        float tgt  = dir * TARGET_CPS;

        float dutyA = pid_step(&pidA, tgt, cpsA);
        float dutyB = pid_step(&pidB, tgt, cpsB);

        if (fabsf(dutyA) > 0 && fabsf(dutyA) < PWM_MIN_MOVE)
            dutyA = (dutyA > 0 ? PWM_MIN_MOVE : -PWM_MIN_MOVE);
        if (fabsf(dutyB) > 0 && fabsf(dutyB) < PWM_MIN_MOVE)
            dutyB = (dutyB > 0 ? PWM_MIN_MOVE : -PWM_MIN_MOVE);

        motorA_drive((int16_t)dutyA);
#if !TEST_MODE
        motorB_drive((int16_t)dutyB);
#endif
    }
    motors_brake();
}

/* -------- rotate: arc turn (steered), degrees, left/right -------- *
 * Accurate arc rotation (checklist A.4) needs IMU heading from Person B for a
 * proper feedback loop. This is a servo-steer + drive skeleton to fill in once
 * gyro data is available. Structure here so command dispatch compiles & the
 * servo/steer path is testable.                                              */
void rotate(float degrees, uint8_t left){
    servo_set(left ? SERVO_LEFT_CCR : SERVO_RIGHT_CCR);
    HAL_Delay(150);                       /* let servo reach lock */
    drive_straight( degrees * 0.10f );    /* placeholder scale - calibrate w/ IMU */
    servo_set(SERVO_CENTER_CCR);
}

/* -------- command queue -------- */
static int cmdq_push(script_item_t it){
    uint8_t next = (uint8_t)((q_tail + 1) % CMDQ_CAP);
    if (next == q_head) return 0;
    cmdq[q_tail] = it; q_tail = next; return 1;
}
static int cmdq_pop(script_item_t *out){
    if (q_head == q_tail) return 0;
    *out = cmdq[q_head]; q_head = (uint8_t)((q_head + 1) % CMDQ_CAP); return 1;
}

/* -------- parse comma/space-separated line into queue items -------- *
 * Protocol tokens (see commands.h): fwd{n} rvs{n} fwdR{n} fwdL{n}
 *                                   rvsR{n} rvsL{n} stp RST
 * Case-insensitive. IMPORTANT: match 4-char arc tokens BEFORE 3-char fwd/rvs. */
void mdp_enqueue_line(char *line){
    for (char *p = line; *p; ++p) if (*p==',') *p=' ';   /* commas -> spaces */
    char *tok = strtok(line, " \t\r\n");
    while (tok){
        for (char *c = tok; *c; ++c) if (*c>='A'&&*c<='Z') *c += 32;  /* lowercase */
        script_item_t it = { SCMD_NONE, 0 };
        if      (!strncmp(tok,"fwdr",4)){ it.type=SCMD_ARC_FR; it.value=atoi(tok+4); }
        else if (!strncmp(tok,"fwdl",4)){ it.type=SCMD_ARC_FL; it.value=atoi(tok+4); }
        else if (!strncmp(tok,"rvsr",4)){ it.type=SCMD_ARC_RR; it.value=atoi(tok+4); }
        else if (!strncmp(tok,"rvsl",4)){ it.type=SCMD_ARC_RL; it.value=atoi(tok+4); }
        else if (!strncmp(tok,"fwd",3)) { it.type=SCMD_FWD_CM; it.value=atoi(tok+3); }
        else if (!strncmp(tok,"rvs",3)) { it.type=SCMD_REV_CM; it.value=atoi(tok+3); }
        else if (!strncmp(tok,"stp",3)) { it.type=SCMD_STOP;   it.value=0; }
        else if (!strncmp(tok,"rst",3)) { it.type=SCMD_STOP;   it.value=0; } /* TODO: real abort (flush queue) */
        if (it.type != SCMD_NONE) cmdq_push(it);
        tok = strtok(NULL, " \t\r\n");
    }
    script_item_t eos = { SCMD_EOS, 0 };
    cmdq_push(eos);
}

/* -------- execute one queue item -------- */
void run_command(script_item_t it){
    switch (it.type){
        case SCMD_FWD_CM: drive_straight( (float)it.value); break;
        case SCMD_REV_CM: drive_straight(-(float)it.value); break;
        case SCMD_ARC_FR: rotate((float)it.value, 0); break;
        case SCMD_ARC_FL: rotate((float)it.value, 1); break;
        case SCMD_ARC_RR: rotate((float)it.value, 0); break;
        case SCMD_ARC_RL: rotate((float)it.value, 1); break;
        case SCMD_STOP:   motors_brake(); servo_set(SERVO_CENTER_CCR); break;
        default: break;
    }
    HAL_Delay(INTER_CMD_MS);
}

/* -------- servo calibration sweep -------- *
 * Steps CCR across the range with delays so you can read the wheel angle and
 * record true center/left/right CCRs, then set the #defines above.
 * Call from the button branch during bench testing.                          */
void servo_sweep(void){
    for (uint16_t ccr = 1000; ccr <= 2000; ccr += 100){
        servo_set(ccr);
        sprintf(buf, "CCR=%u", ccr);
        OLED_ShowString(0, 40, (uint8_t*)buf); OLED_Refresh_Gram();
        HAL_Delay(1500);
    }
    servo_set(SERVO_CENTER_CCR);
}

/* -------- Person B hook: UART RX interrupt callback --------
 * Person B enables HAL_UART_Receive_IT(&huart3, &rx_byte, 1) after init and
 * owns wire-level RX. On a full line they call mdp_enqueue_line(cmd_buf).
 * Contract shown here; not active until they enable it:
 *
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
 *   if (huart->Instance == USART3){
 *     char c = rx_byte;
 *     if (c=='\n'){ cmd_buf[cmd_len]=0; mdp_enqueue_line(cmd_buf); cmd_len=0; }
 *     else if (cmd_len < sizeof(cmd_buf)-1){ cmd_buf[cmd_len++]=c; }
 *     HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_byte, 1);
 *   }
 * }
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if (GPIO_Pin == USER_PB_Pin) run_test = 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM12_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);        /* Motor A PWM */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);        /* Motor B PWM */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  /* Motor A enc */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  /* Motor B enc */
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);       /* Servo */

  servo_set(SERVO_CENTER_CCR);
  motors_brake();

  OLED_Init();
  OLED_ShowString(10, 5, (uint8_t*)"MDP-G15");
#if TEST_MODE
  OLED_ShowString(0, 30, (uint8_t*)"TEST: Motor A");
#else
  OLED_ShowString(0, 30, (uint8_t*)"2-wheel mode");
#endif
  OLED_Refresh_Gram();
  uart_puts("MDP-G15 ready\r\n");
  HAL_Delay(1500);
  OLED_Clear();

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
      update_encoders();
      int16_t a = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
      int16_t b = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
      sprintf(buf, "encA:%6d", a); OLED_ShowString(0, 0,  (uint8_t*)buf);
      sprintf(buf, "encB:%6d", b); OLED_ShowString(0, 15, (uint8_t*)buf);
      sprintf(buf, "press->T%d", test_sel + 1); OLED_ShowString(0, 45, (uint8_t*)buf);
      OLED_Refresh_Gram();

      if (run_test){
          run_test = 0;
          HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);   /* LED blinks on each press */

          switch (test_sel){
            case 0:  /* --- TEST 1: servo CCR sweep (record center/L/R) --- */
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t*)"T1 servo sweep");
                OLED_Refresh_Gram();
                servo_sweep();
                break;

            case 1:  /* --- TEST 2: live encoder watch (turn wheels by hand) --- */
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t*)"T2 encoders");
                OLED_Refresh_Gram();
                for (int k = 0; k < 200; k++){         /* ~10 s window */
                    int16_t ea = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
                    int16_t eb = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
                    sprintf(buf, "A:%6d", ea); OLED_ShowString(0, 20, (uint8_t*)buf);
                    sprintf(buf, "B:%6d", eb); OLED_ShowString(0, 35, (uint8_t*)buf);
                    OLED_Refresh_Gram();
                    sprintf(buf, "A:%d B:%d\r\n", ea, eb);
                    uart_puts(buf);
                    HAL_Delay(50);
                }
                break;

            case 2:  /* --- TEST 3: Motor A open-loop spin (dir + enc sign) --- */
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t*)"T3 MotorA spin");
                OLED_Refresh_Gram();
                {
                    int16_t startA = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
                    motorA_drive(2000);                 /* forward, low duty */
                    HAL_Delay(800);
                    motorA_drive(0);
                    int16_t endA = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
                    int16_t d = endA - startA;
                    /* forward should make count INCREASE: report sign */
                    sprintf(buf, "dA:%d", d); OLED_ShowString(0, 20, (uint8_t*)buf);
                    OLED_ShowString(0, 35, (uint8_t*)(d >= 0 ? "sign +ve OK" : "sign -ve FLIP"));
                    OLED_Refresh_Gram();
                    sprintf(buf, "MotorA dA=%d\r\n", d);
                    uart_puts(buf);
                }
                break;
          }

          /* advance to next test for the next press */
          test_sel = (uint8_t)((test_sel + 1) % N_TESTS);
          HAL_Delay(400);   /* crude debounce after running */
          OLED_Clear();
      }

      script_item_t it;
      while (cmdq_pop(&it)){
          if (it.type == SCMD_EOS){ uart_puts("OK\r\n"); break; }
          run_command(it);
      }

      HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  */
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  */
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM8 Initialization Function
  */
static void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim8);
}

/**
  * @brief TIM12 Initialization Function
  */
static void MX_TIM12_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 71;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim12);
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, OLED1_Pin|OLED2_Pin|OLED3_Pin|OLED4_Pin
                          |LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|DIN1_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = OLED1_Pin|OLED2_Pin|OLED3_Pin|OLED4_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Buzzer_Pin|DIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USER_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_PB_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* OLED pins on GPIOD: SCL=PD14, SDA=PD13, RST=PD12, DC=PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
