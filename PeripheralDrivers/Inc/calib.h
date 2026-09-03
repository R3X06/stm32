#ifndef __CALIB_H
#define __CALIB_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Phase 2 steering calibration harness. TEMPORARY - delete after Phase 2.
 *
 * Finds three numbers that go into motors.h:
 *      SERVO_MIN_US    left mechanical limit, minus margin
 *      SERVO_MAX_US    right mechanical limit, minus margin
 *      SERVO_CENTER_US the value that actually drives straight
 *
 * Everything is driven from the user button on PE0 so you never reflash
 * between measurements. Calib_Init() configures PE0 itself, so this needs
 * no .ioc change.
 * ------------------------------------------------------------------------- */

/* Pick one. Rebuild and reflash to switch. */
#define CALIB_MODE_ENDSTOP  0   /* linkage DETACHED - find the limits    */
#define CALIB_MODE_CENTRE   1   /* linkage ATTACHED - find straight      */

#define CALIB_MODE          CALIB_MODE_CENTRE

/* Search span. Deliberately wider than SERVO_MIN_US/SERVO_MAX_US because the
 * whole point of endstop mode is to find where those should be. The harness
 * writes the compare register directly and bypasses the clamp in
 * Servo_SetMicroseconds(), which is ONLY safe with the linkage detached.
 *
 * Once you have the real limits, narrow these to match before running
 * centre mode with the linkage on. */
#define CALIB_SPAN_MIN_US   1400U
#define CALIB_SPAN_MAX_US   1600U
#define CALIB_STEP_US       10U

/* Run parameters for centre mode. */
#define CALIB_RUN_RPM       150
#define CALIB_RUN_MM        2000.0f
#define CALIB_RUN_TIMEOUT_MS 12000U

/* Configures PE0 and parks the servo at centre. Call after Servos_Init(). */
void Calib_Init(void);

/* Never returns. Call at the end of USER CODE 2 instead of the drive test. */
void Calib_Run(void);

#endif /* __CALIB_H */
