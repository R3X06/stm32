#ifndef __MOTORS_H
#define __MOTORS_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Board: WHEELTEC STM32F407VET6 C30D-V2 (schematic rev 23.0)
 *
 * Ackermann chassis: the two rear wheels drive, the front axle is steered by
 * a single servo. The robot CANNOT turn on the spot - every turn is an arc.
 *
 * Four onboard AT8236 H-bridges. This driver uses the first two.
 *
 *   Motor A  = U8,  header MOTORA   (left rear)
 *       drive   PB8, PB9   -> TIM4_CH3, TIM4_CH4   (AF2)
 *   Motor B  = U9,  header MOTORB   (right rear)
 *       drive   PE5, PE6   -> TIM9_CH1, TIM9_CH2   (AF3)
 *
 *   Steering servo, header per Robot car test V30D:
 *       PB15       -> TIM12_CH2                    (AF9)
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
 * PINS DELIBERATELY LEFT FREE - do not use:
 *
 *   PB14       HC-SR04 trigger, plain GPIO out, header J6
 *   PC7        HC-SR04 echo, TIM8_CH2 capture, header J2
 *   PB10, PB11 I2C2, ICM20948 IMU                       - Person B
 *
 * Note PB10/PB11 are also USART3_TX/RX on AF7. USART3 must stay on PD8/PD9
 * or it takes the IMU bus, and the failure looks nothing like a pin clash.
 *
 * PB14 IS NOT AVAILABLE FOR A SECOND SERVO. It is TIM12_CH1 electrically, and
 * J6 is a 3-pin servo-style header with 5V5 and GND on it, which is exactly
 * why it suits the HC-SR04 - the module gets its 5 V from the same connector.
 * But the trigger owns that pin. There is ONE servo on this robot, on PB15.
 *
 * PC6, PC8 and PC9 are also free servo-style headers (J1, J4, J5) if a second
 * servo is ever needed. PC7 is not - the echo capture has it.
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
 * APB1 prescaler 4 (TIM4/TIM12 clock 84 MHz), APB2 prescaler 2 (TIM9 168 MHz).
 *
 *   TIM4:  PSC = 0,   ARR = 4199   -> 84 MHz / 4200  = 20 kHz
 *   TIM9:  PSC = 1,   ARR = 4199   -> 84 MHz / 4200  = 20 kHz  (same scale)
 *   TIM12: PSC = 83,  ARR = 19999  ->  1 MHz / 20000 = 50 Hz, 1 us resolution
 *
 * TIM12 is a general-purpose timer, 16-bit, two channels, no complementary
 * outputs - so unlike TIM8 there is no MOE to enable before PWM appears.
 *
 * If you stay on the default HSI 16 MHz with no PLL, use instead:
 *   TIM4 PSC 0 ARR 799, TIM9 PSC 0 ARR 799, TIM12 PSC 15 ARR 19999,
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

/* Set to 1 if a motor spins the wrong way for a positive speed.
 *
 * B CHANGED FROM 1 TO 0 - VERIFY THIS ON THE STAND BEFORE A FLOOR RUN.
 *
 * These did not agree with the calibration build that was in main.c. Working
 * it through channel by channel:
 *
 *   Motor A: motors.c passes CH3 as in1 and CH4 as in2, so INVERT 1 puts the
 *            PWM on CH4 (PB9) for a positive speed. The calibration build put
 *            it on CH4 too. Same behaviour - A stays at 1.
 *
 *   Motor B: motors.c passes CH1 as in1 and CH2 as in2, so INVERT 1 put the
 *            PWM on CH2 (PE6) for a positive speed. The calibration build put
 *            it on CH1 (PE5). Opposite - so B becomes 0 to match.
 *
 * The calibration build is the more recent of the two and is the one that was
 * driven on the bench, so it wins. If Motor B turns out to run backwards,
 * put this back to 1 and re-check A at the same time. */
#define MOTOR_A_INVERT      1
#define MOTOR_B_INVERT      1

/* Servo pulse limits, microseconds.
 * The linkage - not the servo - sets the usable range. Per the datasheet
 * handout the working span is roughly 65-85 ticks at 20 us, i.e. 1300-1700.
 * Commanding 500 or 2500 drives the steering into its mechanical stop and
 * stalls the servo. All three values are PROVISIONAL - measured in Phase 2. */
#define SERVO_MIN_US        1250U
#define SERVO_MAX_US        1750U
#define SERVO_CENTER_US     1500U


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

/* Starts the steering servo on TIM12_CH2, held at centre. */
void Servos_Init(void);

/* Pulse width in microseconds, clamped to SERVO_MIN_US..SERVO_MAX_US. */
void Servo_SetMicroseconds(uint16_t us);

/* 0..180 mapped linearly across the clamped range. 90 is NOT the straight-
 * ahead position unless SERVO_CENTER_US happens to sit mid-range - use
 * Servo_SetMicroseconds(SERVO_CENTER_US) to centre the steering. */
void Servo_SetAngle(uint8_t degrees);

/* Blocking bring-up sequence. Chassis on blocks before calling. */
void Motors_TestSequence(void);

#endif /* __MOTORS_H */
