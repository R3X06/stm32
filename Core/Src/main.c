/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : HC-SR04 ISOLATION TEST BUILD
  *
  *  Brings up ONLY: clocks, OLED, PB14 (trig), PC7 (echo, TIM8_CH2).
  *  Motors are held in coast (both H-bridge inputs low, no PWM).
  *  Servo, encoders, PID, odometry, ADC/IR and the TIM6 control tick
  *  are all left out, so the 5V rail carries almost nothing.
  *
  *  If the ultrasonic works here but not in the full build, the problem
  *  is power. If it fails here too, it is the sensor or its wiring.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "oled.h"

/* Private define ------------------------------------------------------------*/
#define US_TRIG_PORT   GPIOB
#define US_TRIG_PIN    GPIO_PIN_14
#define US_ECHO_PORT   GPIOC
#define US_ECHO_PIN    GPIO_PIN_7

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim8;

/* Unused here, but stm32f4xx_it.c references them. Never initialised,
   so the peripherals stay off and their interrupts never fire.        */
TIM_HandleTypeDef  htim6;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef  hdma_adc1;

volatile uint16_t us_t1        = 0;
volatile uint16_t us_echo_us   = 0;    /* width from capture path  */
volatile uint16_t us_poll_us   = 0;    /* width from polled path   */
volatile float    us_distance_cm = -1.0f;
volatile uint8_t  us_waiting   = 0;
volatile uint8_t  us_edge      = 0;
volatile uint8_t  us_pin_high  = 0;
volatile uint16_t us_poll_ok   = 0;
volatile uint16_t us_isr_n     = 0;
volatile uint16_t loop_n       = 0;

volatile uint8_t  pin_pu = 9, pin_pd = 9;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM8_Init(void);
static void Pins_Init(void);
static void Motors_Safe(void);
static uint8_t PC7_Read(uint32_t pull);
static void HCSR04_Trigger(void);
static void HCSR04_Poll(void);
static void Show(void);

/* ---------------------------------------------------------------------------*/

/* Both H-bridge inputs low on each motor = coast. No PWM anywhere.
   Servo signal pin held low so the servo receives no pulses.        */
static void Motors_Safe(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_15;   /* Ain2, Ain1, servo */
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_15, GPIO_PIN_RESET);

    g.Pin = GPIO_PIN_5 | GPIO_PIN_6;                 /* Bin1, Bin2 */
    HAL_GPIO_Init(GPIOE, &g);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
}

/* Read PC7 as a plain input with the requested internal pull. */
static uint8_t PC7_Read(uint32_t pull)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    g.Pin   = US_ECHO_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = pull;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(US_ECHO_PORT, &g);
    HAL_Delay(5);
    return (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

/* PB14 -> output (trig).  PC7 -> AF3 / TIM8_CH2 with pull-down (echo). */
static void Pins_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    g.Pin   = US_TRIG_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(US_TRIG_PORT, &g);
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);

    g.Pin       = US_ECHO_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLDOWN;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(US_ECHO_PORT, &g);
}

static void HCSR04_Trigger(void)
{
    volatile uint32_t i;

    us_edge     = 0;
    us_waiting  = 1;
    us_pin_high = 0;
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC2);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC2OF);

    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
    for (i = 0; i < 150u;  i++) { __NOP(); }    /* ~5 us  settle */
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET);
    for (i = 0; i < 1000u; i++) { __NOP(); }    /* ~30 us pulse  */
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
}

/* Straight read of the pin. Does not use the capture unit or the NVIC. */
static void HCSR04_Poll(void)
{
    uint32_t t0;
    uint16_t a, b, w;

    t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) > 30u) { us_waiting = 0; return; }
    }
    us_pin_high = 1;
    a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

    t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - t0) > 40u) { us_waiting = 0; return; }
    }
    b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

    w = (uint16_t)(b - a);
    us_poll_us = w;
    if (w >= 100u && w <= 25000u) {
        us_poll_ok++;
        us_distance_cm = (float)w / 58.0f;
    }
    us_waiting = 0;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint16_t cap, w;

    if (htim->Instance != TIM8) return;
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) return;
    us_isr_n++;

    cap = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    if (!us_waiting) return;

    if (us_edge == 0) {
        us_t1   = cap;
        us_edge = 1;
    } else {
        w       = (uint16_t)(cap - us_t1);
        us_edge = 0;
        if (w >= 100u && w <= 25000u) us_echo_us = w;
    }
}

static void Show(void)
{
    char line[24];

    snprintf(line, sizeof(line), "L%3d PU%d PD%d",
             (int)(loop_n % 1000), (int)pin_pu, (int)pin_pd);
    OLED_ShowString(0, 0, (const uint8_t *)line);

    snprintf(line, sizeof(line), "H%d N%3d",
             (int)us_pin_high, (int)(us_poll_ok % 1000));
    OLED_ShowString(0, 16, (const uint8_t *)line);

    snprintf(line, sizeof(line), "P%5d D%4d",
             (int)us_poll_us, (int)us_distance_cm);
    OLED_ShowString(0, 32, (const uint8_t *)line);

    snprintf(line, sizeof(line), "U%5d C%3d",
             (int)us_echo_us, (int)(us_isr_n % 1000));
    OLED_ShowString(0, 48, (const uint8_t *)line);

    OLED_Refresh_Gram();
}

/* ---------------------------------------------------------------------------*/

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    Motors_Safe();          /* hold the H-bridges in coast first */

    {
        GPIO_InitTypeDef g = {0};
        __HAL_RCC_GPIOD_CLK_ENABLE();
        g.Pin   = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
        g.Mode  = GPIO_MODE_OUTPUT_PP;
        g.Pull  = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOD, &g);
        HAL_GPIO_WritePin(GPIOD, g.Pin, GPIO_PIN_RESET);
    }
    OLED_Init();
    MX_TIM8_Init();

    pin_pu = PC7_Read(GPIO_PULLUP);
    pin_pd = PC7_Read(GPIO_PULLDOWN);

    Pins_Init();
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);

    while (1)
    {
        static uint32_t t_ping = 0, t_disp = 0;

        if (HAL_GetTick() - t_ping >= 100u) {
            t_ping = HAL_GetTick();
            loop_n++;
            HCSR04_Trigger();
            HCSR04_Poll();
        }

        if (HAL_GetTick() - t_disp >= 200u) {
            t_disp = HAL_GetTick();
            Show();
        }
    }
}

/* ---------------------------------------------------------------------------*/

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 168-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

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
