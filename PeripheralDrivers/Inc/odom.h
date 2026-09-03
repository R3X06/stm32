#ifndef __ODOM_H
#define __ODOM_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Odometry and heading hold for the WHEELTEC C30D-V2.
 *
 * PHASE 3 CHANGE: heading is now held by the STEERING SERVO, not by trimming
 * one rear wheel against the other.
 *
 * This chassis is Ackermann. The front wheels are locked at whatever angle
 * the servo holds, so a wheel-speed difference cannot change the heading -
 * it only yaws the body against the front tyres and scrubs them sideways.
 * The old trim approach worked, but only by dragging the tyres, and it
 * needed a permanent offset to hold a straight line.
 *
 * The per-wheel speed PID stays. Its job is now purely to make Motor A and
 * Motor B run at the SAME speed. Steering is the servo's job alone.
 *
 * Odom_Update() runs from the TIM6 interrupt, after Encoders_Update() and
 * before PID_Update().
 * ------------------------------------------------------------------------- */

/* Geometry. WHEEL_BASE_MM here is the TRACK width (distance between the two
 * rear wheel centres) and is used only to infer heading from the encoder
 * difference. On an Ackermann chassis that inference is weak - the rear
 * wheels barely differ on a gentle curve - so treat encoder heading as a
 * rough estimate until the IMU is available. */
#define WHEEL_DIAMETER_MM   63.5f
#define WHEEL_BASE_MM       127.0f

/* From the ten-revolution calibration. */
#define ODOM_COUNTS_PER_REV 1560.0f

/* 63.5 * pi / 1560 -- about 0.128 mm of travel per count. */
#define MM_PER_COUNT  ((WHEEL_DIAMETER_MM * 3.14159265f) / ODOM_COUNTS_PER_REV)

/* ---------------------------------------------------------------------------
 * Heading hold tuning
 * ------------------------------------------------------------------------- */

/* Microseconds of servo deflection per degree of heading error.
 *
 * You measured roughly 19 us per degree of wheel steer. A gain of 8 means a
 * 5 degree heading error commands about 2 degrees of corrective steer, which
 * is a gentle correction. Raise until it corrects briskly; back off if it
 * weaves. */
#define HEADING_KP_US       8.0f

/* Ceiling on the correction, in microseconds either side of centre. Keeps a
 * large error from slamming the steering to full lock mid-run. 150 us is
 * about 8 degrees of steer. */
#define HEADING_MAX_US      150.0f

/* Below this the correction is suppressed entirely. Without it the servo
 * hunts around centre and buzzes. */
#define HEADING_DEADBAND_DEG  0.3f

/* Backlash compensation, microseconds.
 *
 * You measured roughly 50 us of slack between commanding a direction change
 * and the wheels responding. A plain proportional term produces corrections
 * smaller than that whenever the error is under ~2.5 degrees, so nothing
 * happens, the error grows, and then it over-corrects - the robot weaves.
 *
 * Once the correction is non-zero we add half the measured slack in the same
 * direction, which pushes through it. Set to 0 to disable and see the
 * difference. */
#define SERVO_BACKLASH_US   25.0f

/* Sign convention. MUST BE VERIFIED ON THE ROBOT - see the procedure in the
 * comment above Odom_DriveHeading() in odom.c. Getting this backwards makes
 * the robot steer INTO the error and leave the line immediately.
 *
 * +1 or -1 only. */
#define HEADING_SIGN        (+1)

typedef struct
{
    float x_mm;         /* forward from the start pose   */
    float y_mm;         /* left of the start pose        */
    float heading_deg;  /* CCW positive, wraps to +-180  */
    float distance_mm;  /* total path length travelled   */
} Odom_Pose_t;

/* Zeroes the pose and syncs to the current encoder counts. Call at startup
 * after Encoders_Init() and Servos_Init(). */
void Odom_Init(void);

/* Clears the pose to the origin without disturbing the encoders. */
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

/* Drive at the given speed holding the current heading. Negative rpm
 * reverses; the correction sign is flipped automatically. */
void Odom_DriveStraight(int16_t rpm);

/* Drive at the given speed holding a specific heading in degrees. */
void Odom_DriveHeading(int16_t rpm, float heading_deg);

/* Stops the robot, recentres the steering, disables heading hold. */
void Odom_Stop(void);

/* The servo pulse width the heading loop is currently commanding, in
 * microseconds. Equals SERVO_CENTER_US when the robot is on course.
 * Exposed for tuning - put it on the OLED. */
uint16_t Odom_GetServoUs(void);

/* Heading error the loop is currently acting on, degrees. Also for tuning. */
float Odom_GetHeadingError(void);

/* ------------------------------------------------------------------ */
/* Calibration helper                                                  */
/* ------------------------------------------------------------------ */

/* Spin-test support for refining WHEEL_BASE_MM. Note this is of limited use
 * on an Ackermann chassis, which cannot turn on the spot - it is kept for
 * the hand-pushed bench check only. */
void  Odom_CalibrateSpinStart(void);
float Odom_CalibrateSpinResult(void);

#endif /* __ODOM_H */
