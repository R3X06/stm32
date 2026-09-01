#ifndef __ENCODERS_H
#define __ENCODERS_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Board: WHEELTEC STM32F407VET6 C30D-V2 (schematic rev 23.0)
 *
 *   Encoder A (header MOTORA) -> PA15 TIM2_CH1, PB3 TIM2_CH2  (AF1)
 *   Encoder B (header MOTORB) -> PB4  TIM3_CH1, PB5 TIM3_CH2  (AF2)
 *
 * Encoder supply on the motor headers is 3V3, and R56-R59 are 100R series
 * resistors on the signal lines, so no level shifting is needed.
 *
 * *** PA15 and PB3 are JTDI and JTDO. ***
 * Encoder A will not work unless CubeMX has SYS -> Debug -> Serial Wire.
 * The board's debug header is SWD (PA13/PA14), so nothing is lost, but
 * leaving Debug set to JTAG or No Debug produces an encoder that silently
 * never counts.
 *
 * CubeMX for TIM2 and TIM3, identically:
 *   Combined Channels = Encoder Mode
 *   Encoder Mode = TI1 and TI2
 *   Prescaler 0, Counter Period 65535
 *   Input Filter 10 on both channels
 *
 * "TI1 and TI2" counts every edge on both channels, so one encoder pulse
 * gives 4 counts. That is already folded into COUNTS_PER_REV below.
 *
 * The timers count in hardware, so a slow main loop costs speed resolution
 * but never loses position.
 * ------------------------------------------------------------------------- */

/* ---- CALIBRATE THESE FOR YOUR MOTORS --------------------------------------
 * ENCODER_PPR is pulses per revolution of the MOTOR shaft on ONE channel,
 * before the gearbox and before the x4 quadrature multiplication.
 * The defaults suit a JGB37-520 style gearmotor (11 PPR, 30:1). Check your
 * motor - a wrong value scales every distance and speed reading by a
 * constant factor.
 *
 * Easiest measurement: mark the wheel, call Encoders_Reset(), turn the wheel
 * exactly 10 revolutions by hand, read Encoder_A_GetCount(), divide by 10.
 * That result is COUNTS_PER_REV directly.
 * ------------------------------------------------------------------------- */
#define ENCODER_PPR             13U
#define ENCODER_GEAR_RATIO      30U
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * 4U * ENCODER_GEAR_RATIO)

/* If a wheel counts down while its motor is commanded forward, set the
 * matching flag to 1. */
#define ENCODER_A_INVERT        0
#define ENCODER_B_INVERT        1

/* Minimum gap between samples. Shorter is more responsive but noisier. */
#define ENCODER_SAMPLE_MS       20U

void Encoders_Init(void);

/* Call from the main loop as often as you like. Rate-limits itself to
 * ENCODER_SAMPLE_MS and returns immediately in between. */
void Encoders_Update(void);

/* Zeroes accumulated position. Does not disturb speed readings. */
void Encoders_Reset(void);

int32_t Encoder_A_GetCount(void);
int32_t Encoder_B_GetCount(void);

int16_t Encoder_A_GetDelta(void);
int16_t Encoder_B_GetDelta(void);

/* Output-shaft speed in whole RPM, signed. Integer maths throughout, so it
 * is safe with a non-float-enabled printf. */
int32_t Encoder_A_GetRPM(void);
int32_t Encoder_B_GetRPM(void);

#endif /* __ENCODERS_H */
