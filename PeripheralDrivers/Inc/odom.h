#ifndef __ODOM_H
#define __ODOM_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Differential-drive odometry for the WHEELTEC C30D-V2.
 *
 * Integrates the two wheel encoders into a pose (x, y, heading) and provides
 * a heading-hold trim that corrects drift by measuring it, rather than by
 * assuming a fixed offset between the wheels.
 *
 * Odom_Update() runs from the TIM6 interrupt, after Encoders_Update() and
 * before PID_Update(), so the trim it computes is applied the same tick.
 * ------------------------------------------------------------------------- */

/* Geometry. Measured on the bench: 2.5" wheels, 5" between wheel centres.
 * Both are starting values -- refine WHEEL_BASE_MM with the spin test
 * (Odom_CalibrateSpin) once the final chassis weight is on. Track width is
 * the number that sets heading accuracy, so it is worth getting right. */
#define WHEEL_DIAMETER_MM   63.5f
#define WHEEL_BASE_MM       127.0f

/* From the ten-revolution calibration. */
#define ODOM_COUNTS_PER_REV 1560.0f

/* 63.5 * pi / 1560 -- about 0.128 mm of travel per count. */
#define MM_PER_COUNT  ((WHEEL_DIAMETER_MM * 3.14159265f) / ODOM_COUNTS_PER_REV)

/* Heading-hold gain, in RPM of wheel trim per degree of heading error.
 * 0.5 means a 10 degree error trims the wheels by 5 RPM each way. Start here
 * and raise it until the robot corrects briskly without weaving. */
#define HEADING_KP          0.5f

/* Ceiling on the trim, in RPM. Stops a large heading error from commanding a
 * spin when you wanted a gentle correction, and keeps both wheels inside
 * their controllable range. */
#define HEADING_TRIM_MAX    40.0f

typedef struct
{
    float x_mm;         /* forward from the start pose   */
    float y_mm;         /* left of the start pose        */
    float heading_deg;  /* CCW positive, wraps to +-180  */
    float distance_mm;  /* total path length travelled   */
} Odom_Pose_t;

/* Zeroes the pose and syncs to the current encoder counts. Call at startup
 * after Encoders_Init(). */
void Odom_Init(void);

/* Clears the pose to the origin without disturbing the encoders. Use this to
 * set a new reference point mid-run. */
void Odom_Reset(void);

/* One integration step. Call from the TIM6 ISR, after Encoders_Update()
 * and before PID_Update(). */
void Odom_Update(void);

/* Current pose. Safe to call from the main loop. */
void Odom_GetPose(Odom_Pose_t *out);

float Odom_GetHeading(void);
float Odom_GetDistance(void);

/* ------------------------------------------------------------------ */
/* Heading hold                                                        */
/* ------------------------------------------------------------------ */

/* Drive straight at the given speed, holding whatever heading the robot has
 * right now. This is the drift fix: instead of trimming one wheel by a fixed
 * amount, it measures how far off course the robot has actually gone and
 * corrects continuously. Works at any speed and survives a change in weight
 * distribution, which a fixed offset does not. */
void Odom_DriveStraight(int16_t rpm);

/* Drive straight while holding a specific heading in degrees. */
void Odom_DriveHeading(int16_t rpm, float heading_deg);

/* Stops the robot and disables heading hold. */
void Odom_Stop(void);

/* Called by Odom_Update() while heading hold is active. Exposed so you can
 * see what the correction is doing while tuning. */
float Odom_GetTrim(void);

/* ------------------------------------------------------------------ */
/* Calibration helper                                                  */
/* ------------------------------------------------------------------ */

/* Spin-test support for refining WHEEL_BASE_MM.
 *
 * Call Odom_CalibrateSpinStart(), turn the robot on the spot through exactly
 * one full revolution (by hand or under power), then read the value this
 * returns. It is the wheel base implied by the counts actually seen, which
 * accounts for tyre scrub and load in a way a ruler cannot. Put the result
 * in WHEEL_BASE_MM. */
void  Odom_CalibrateSpinStart(void);
float Odom_CalibrateSpinResult(void);

#endif /* __ODOM_H */
