#ifndef __MOTION_H
#define __MOTION_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Motion primitives. Non-blocking state machine, ticked from TIM6 at 100 Hz.
 *
 * Checklist A.3: traverse a straight line, stop at a distance between 80 and
 * 120 cm specified by the supervisor, within +/-6% of target, with no visible
 * deviation from the line.
 *
 * Nothing here blocks. Motion_DriveDistance() returns immediately; the run
 * happens in the tick. Motion_IsBusy() is the boundary the command layer
 * polls to decide when to emit "OK".
 *
 * CALL ORDER in the TIM6 ISR:
 *      Encoders_Update();
 *      Motion_Tick();      <-- sets the speed for this tick
 *      Odom_Update();      <-- applies it, steers
 *      PID_Update();
 * ------------------------------------------------------------------------- */

/* Cruise speed, RPM. Same value Phase 3 was tuned at. */
#define MOTION_CRUISE_RPM       100

/* Approach speed for the last stretch. Slowing before the target cuts the
 * spread in stopping distance, which is what +/-6% actually depends on -
 * a fast stop is not inaccurate, it is inconsistent.
 *
 * This was equal to MOTION_CRUISE_RPM, which disabled the taper entirely:
 * want never differed from s_lastRpm, so the speed change was never issued
 * and every run braked from full cruise. 40 RPM is a starting point; raise
 * it if the approach crawls for too long. */
#define MOTION_APPROACH_RPM     40
#define MOTION_APPROACH_MM      150.0f

/* Coast after braking, mm. MEASURE THIS - see the procedure in motion.c.
 * Subtracted from the target so the robot ends up on the mark rather than
 * past it. Starting guess only. */
#define MOTION_BRAKE_MM         0.0f

/* Brake settling time before declaring the move finished, in 10 ms ticks. */
#define MOTION_BRAKE_TICKS      40U

/* Watchdog. A stalled wheel must not hang the primitive forever, or it hangs
 * the RPi link with no reply and nothing to explain it. */
#define MOTION_TIMEOUT_TICKS    1500U   /* 15 s */

/* ---------------------------------------------------------------------------
 * ARCS - BOTH OF THESE MUST BE MEASURED
 *
 * This chassis is Ackermann and cannot turn on the spot, so FR/FL/RR/RL are
 * arcs. An arc is driven by parking the steering at a fixed deflection and
 * running a known arc length, because the encoder-derived heading on this
 * chassis is far too noisy to terminate a turn on.
 *
 *     arc length at the rear-axle centre = radius * angle_in_radians
 *
 * MOTION_ARC_STEER_US is the deflection from SERVO_CENTER_US used for every
 * arc. It must stay inside SERVO_MIN_US..SERVO_MAX_US or the clamp in
 * Servo_SetMicroseconds() will quietly shorten it and every arc will come
 * out wide.
 *
 * MOTION_ARC_RADIUS_MM is the turn radius that deflection actually produces.
 * MEASURE IT: park the steering at centre + MOTION_ARC_STEER_US, drive a full
 * circle at MOTION_CRUISE_RPM, mark where the robot returns to its start, and
 * measure the circle's diameter across the tyre tracks. radius = diameter / 2.
 * Do it in both directions - if left and right differ by more than a few
 * percent the linkage is not symmetric about centre and SERVO_CENTER_US is
 * the thing that is wrong, not the radius.
 * ------------------------------------------------------------------------- */
#define MOTION_ARC_STEER_US     200.0f
#define MOTION_ARC_RADIUS_MM    600.0f

/* Settling time for the steering before the wheels are allowed to turn, in
 * 10 ms ticks.
 *
 * Without this the motors start in the SAME TICK the servo is commanded to
 * its starting angle, so the robot begins moving while the steering is still
 * swinging. A hobby servo takes roughly 20-30 ms to cross the deflection a
 * heading correction can leave behind, and the linkage backlash adds more on
 * top. At cruise the robot covers about 7 mm in that time - with the front
 * wheels pointing somewhere they were never asked to point, while the heading
 * for the whole run is being latched.
 *
 * 200 ms is generous, unnoticeable, and paid only when the steering actually
 * has to move (see MOTION_ALIGN_SKIP_US). */
#define MOTION_ALIGN_TICKS      20U

/* If the servo is already within this many microseconds of where the move
 * wants it, skip the settle entirely. Back-to-back straight commands from the
 * RPi would otherwise each pay 200 ms for a servo that is not going to move. */
#define MOTION_ALIGN_SKIP_US    20U

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_ALIGN,
    MOTION_RUN,
    MOTION_BRAKE,
    MOTION_DONE,
    MOTION_TIMEOUT
} MotionState_t;

/* Call once at startup, after Odom_Init(). */
void Motion_Init(void);

/* Call from the TIM6 ISR, between Encoders_Update() and Odom_Update(). */
void Motion_Tick(void);

/* Drive a straight line. Negative mm reverses. Holds the heading the robot
 * has at the moment of the call. Returns immediately. */
void Motion_DriveDistance(int32_t mm);

/* Drive an arc through the given angle. Returns immediately.
 *   degrees  > 0, the swept angle
 *   forward  1 to travel forwards, 0 to reverse
 *   right    1 to curve right, 0 to curve left
 *
 * Note reversing does NOT flip which way the body turns for a given steering
 * angle - RR is the mirror of FR in path, not in steering. The steering side
 * is set by 'right' alone. */
void Motion_DriveArc(int16_t degrees, uint8_t forward, uint8_t right);

/* Abort now: brake, recentre steering, go to IDLE. */
void Motion_Stop(void);

/* 1 while a primitive is running. Poll this to know when to reply OK. */
uint8_t Motion_IsBusy(void);

MotionState_t Motion_GetState(void);

/* Clears DONE/TIMEOUT back to IDLE once the caller has read the result. */
void Motion_ClearState(void);

/* Distance still to go, mm. For the OLED while tuning. */
int32_t Motion_GetRemaining(void);

/* Distance actually covered by the current or last primitive, mm. */
int32_t Motion_GetTravelled(void);

#endif /* __MOTION_H */
