#ifndef __MOTORS_H
#define __MOTORS_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Board: WHEELTEC STM32F407VET6 C30D-V2 (schematic rev 23.0)
 *
 * Four onboard AT8236 H-bridges. This driver uses the first two.
 *
 *   Motor A  = U8,  header MOTORA
 *       drive   PB8, PB9   -> TIM4_CH3, TIM4_CH4   (AF2)
 *   Motor B  = U9,  header MOTORB
 *       drive   PE5, PE6   -> TIM9_CH1, TIM9_CH2   (AF3)
 *
 *   Servos   = headers J1/J2/J4/J5
 *       PC6, PC7, PC8, PC9 -> TIM8_CH1..CH4        (AF3)
 *       Powered from the 5V5 rail (U7), separate from the Pi 5V supply.
 *
 * The AT8236 has no direction pin. Direction comes from which of the two
 * inputs carries the PWM:
 *      IN1=PWM IN2=0   one way
 *      IN1=0   IN2=PWM the other way
 *      IN1=0   IN2=0   coast
 *      IN1=1   IN2=1   brake
 *
 * Which physical direction is "forward" depends on how the motor leads are
 * crimped, so check on the bench and flip the INVERT flags below if needed.
 *
 * ---------------------------------------------------------------------------
 * NOT USED HERE, but noted so you do not trip over it later:
 *
 * Motor A's drive sits on TIM4_CH3/CH4, and Motor C's ENCODER sits on
 * PB6/PB7 = TIM4_CH1/CH2. Encoder mode claims the whole timer, so you cannot
 * have both. If you add Motor C, move Motor A's drive to TIM10_CH1 (PB8) and
 * TIM11_CH1 (PB9), both AF3, which frees TIM4 for the encoder. Splitting the
 * two inputs across separate timers is harmless - only one is ever driven at
 * a time.
 *
 * ---------------------------------------------------------------------------
 * CLOCK ASSUMPTION
 *
 * Values below assume HSE 8 MHz crystal -> PLL -> 168 MHz SYSCLK,
 * APB1 prescaler 4 (TIM4 clock 84 MHz), APB2 prescaler 2 (TIM8/TIM9 168 MHz).
 *
 *   TIM4: PSC = 0,   ARR = 4199   -> 84 MHz / 4200 = 20 kHz
 *   TIM9: PSC = 1,   ARR = 4199   -> 84 MHz / 4200 = 20 kHz  (same scale)
 *   TIM8: PSC = 167, ARR = 19999  ->  1 MHz / 20000 = 50 Hz, 1 us resolution
 *
 * If you stay on the default HSI 16 MHz with no PLL, use instead:
 *   TIM4 PSC 0 ARR 799, TIM9 PSC 0 ARR 799, TIM8 PSC 15 ARR 19999,
 * and change MOTOR_TIM_ARR below to 799.
 * ------------------------------------------------------------------------- */

/* Must match the Counter Period set for BOTH TIM4 and TIM9 in CubeMX. */
#define MOTOR_TIM_ARR       4199U

/* Public speed scale. Motor_x_Set() takes -1000 .. +1000. */
#define MOTOR_SPEED_MAX     1000

/* Measured on the bench, 12 V, wheels free.
 * Deadband: lowest duty where both wheels start from rest (575 observed,
 * rounded up for margin). Max RPM: free-running speed at duty 1000. */
#define MOTOR_DEADBAND   600
#define MOTOR_A_MAX_RPM  378
#define MOTOR_B_MAX_RPM  362

/* Set to 1 if a motor spins the wrong way for a positive speed. */
#define MOTOR_A_INVERT      1
#define MOTOR_B_INVERT      1

/* Servo pulse limits in microseconds. 500-2500 suits MG996R / SG90 class
 * servos. Narrow to 1000/2000 if yours buzzes at the extremes - a servo held
 * past its mechanical stop draws stall current until it burns out. */
#define SERVO_MIN_US        500U
#define SERVO_MAX_US        2500U
#define SERVO_CENTER_US     1500U

/* Which TIM8 channel each servo header pin maps to. */
#define SERVO_PC6           TIM_CHANNEL_1
#define SERVO_PC7           TIM_CHANNEL_2
#define SERVO_PC8           TIM_CHANNEL_3
#define SERVO_PC9           TIM_CHANNEL_4

void Motors_Init(void);

/* speed: -1000 (full reverse) .. 0 (coast) .. +1000 (full forward) */
void Motor_A_Set(int16_t speed);
void Motor_B_Set(int16_t speed);

/* Both inputs high - shorts the motor terminals, stops hard. */
void Motor_A_Brake(void);
void Motor_B_Brake(void);
void Motors_Brake(void);

/* Both inputs low - outputs float, motor freewheels. */
void Motors_Coast(void);

/* Starts all four TIM8 servo channels at centre. */
void Servos_Init(void);

/* channel: one of the SERVO_PCx constants above. */
void Servo_SetMicroseconds(uint32_t channel, uint16_t us);
void Servo_SetAngle(uint32_t channel, uint8_t degrees);

/* Blocking bring-up sequence. Chassis on blocks before calling. */
void Motors_TestSequence(void);

#endif /* __MOTORS_H */
