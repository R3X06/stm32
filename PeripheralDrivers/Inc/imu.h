#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * ICM-20948 gyroscope, for heading.
 *
 *   U18 on the C30D schematic. I2C2 on PB10 (SCL) and PB11 (SDA), AF4.
 *
 * THREE THINGS ABOUT THIS PART ON THIS BOARD
 *
 * 1. It runs at 1.8 V, not 3.3 V. U20 (SSP6206-18NR) makes the 1.8 V rail and
 *    U19 (RS0102YH8) level-shifts SCL and SDA. So the pull-ups you can see
 *    next to the chip are on the far side of the shifter and are not yours.
 *    From PB10/PB11 it looks like an ordinary 3.3 V I2C bus.
 *
 * 2. PB12 must be driven HIGH. It is wired into the same block, and on the
 *    ICM-20948 pin 22 is nCS: low selects SPI, high selects I2C. Left low or
 *    floating the part simply never acknowledges, and it looks exactly like a
 *    dead sensor or a wrong address. IMU_Init() drives it.
 *
 * 3. The 7-bit address is 0x68 or 0x69 depending on the AD0 pin. IMU_Init()
 *    probes both rather than assuming, and IMU_GetAddress() reports which one
 *    answered.
 *
 * ---------------------------------------------------------------------------
 * WHY ONLY THE GYRO
 *
 * Yaw rate is the one quantity we need. The accelerometer cannot see yaw at
 * all when the robot is level, and the magnetometer needs hard-iron
 * calibration and is useless next to two motors pulling amps.
 *
 * The usual objection to a bare gyro is drift, and it does not really apply
 * here. An A.3 run lasts about three seconds. Bias-corrected, residual drift
 * over three seconds is a fraction of a degree. Gyros are poor at holding a
 * heading for minutes and excellent for seconds - and seconds is all we need,
 * because Motion_DriveDistance() zeroes the heading at the start of every
 * move anyway.
 *
 * ---------------------------------------------------------------------------
 * SPLIT BETWEEN MAIN LOOP AND TICK
 *
 * IMU_Poll()  main loop. Does the I2C transaction.
 * IMU_Tick()  control tick. Integrates the most recent sample at fixed dt.
 *
 * The read is deliberately NOT in the tick. A two-byte transfer at 400 kHz is
 * about 90 us, which is tolerable, but a bus glitch turns it into a HAL
 * timeout that stalls the control loop for milliseconds. Same reasoning as
 * the ultrasonic trigger.
 *
 * The gyro's output data rate is set to ~102 Hz, near enough to the 100 Hz
 * tick that integrating the latest sample each tick loses nothing.
 * ------------------------------------------------------------------------- */

/* Full scale. 250 dps is the most sensitive setting and gives 131 LSB per
 * degree per second. A robot correcting its heading turns at a few tens of
 * dps at most, so the wider ranges only throw away resolution. */
#define IMU_GYRO_FS_DPS         250.0f
#define IMU_GYRO_LSB_PER_DPS    131.0f

/* Integration period. MUST equal the TIM6 tick. */
#define IMU_DT_S                0.01f

/* Mounting sign. Odometry counts heading counter-clockwise positive. If the
 * board is mounted with the chip's Z axis pointing UP, the gyro agrees and
 * this is +1. Mounted upside down, use -1.
 *
 * Check it: with the robot flat, rotate it counter-clockwise (to the left)
 * seen from above. IMU_GetHeading() must INCREASE. */
#define IMU_Z_SIGN              (+1)

/* Samples averaged for the bias measurement, at ~10 ms each. 200 is two
 * seconds. The robot MUST be still and on the ground for this. */
#define IMU_BIAS_SAMPLES        200U

/* Bias larger than this in raw counts means the robot was moving during
 * calibration, or the part is faulty. 131 counts is 1 dps, which is already
 * a lot for a stationary sensor. */
#define IMU_BIAS_SANITY_LSB     2000

/* Bring-up attempts before giving up, 100 ms apart. Covers a slow 1.8 V rail
 * and a warm MCU reset that left the sensor mid-reset. */
#define IMU_INIT_RETRIES        5U

/* Start-up. Call after MX_I2C2_Init(), BEFORE the TIM6 tick is started -
 * it uses HAL_Delay() and takes about 2.5 seconds, most of it bias
 * calibration. Returns 1 on success. */
uint8_t IMU_Init(I2C_HandleTypeDef *hi2c);

/* Re-measure the zero-rate bias. Robot must be stationary. Blocking, ~2 s. */
uint8_t IMU_CalibrateBias(void);

/* Main loop: one I2C read of the Z gyro. Call as often as you like. */
void IMU_Poll(void);

/* Control tick: integrate the last sample. Call once per tick, from TIM6. */
void IMU_Tick(void);

/* Zero the integrated heading. Call whenever a move starts. */
void IMU_ResetHeading(void);

/* Integrated heading in degrees, counter-clockwise positive. Free-running,
 * does NOT wrap - wrap it at the point of use if you need +-180. */
float IMU_GetHeading(void);

/* Current yaw rate, degrees per second, bias removed. */
float IMU_GetRateDps(void);

/* 1 once the part has answered and been configured. */
uint8_t IMU_IsReady(void);

/* Diagnostics for the OLED. */
uint8_t  IMU_GetAddress(void);      /* 0x68, 0x69, or 0 if nothing answered */
uint8_t  IMU_GetWhoAmI(void);       /* should be 0xEA                       */
int16_t  IMU_GetBias(void);         /* raw counts subtracted from every read */
int16_t  IMU_GetRawZ(void);         /* last raw reading, before bias         */
uint32_t IMU_GetErrorCount(void);   /* failed I2C transactions since boot    */

#endif /* __IMU_H */
