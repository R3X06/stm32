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
 * happens in the tick. Motion_IsBusy() is the boundary the UART command layer
 * will poll in Phase 7 to decide when to emit "OK".
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
 * a fast stop is not inaccurate, it is inconsistent. */
#define MOTION_APPROACH_RPM     100
#define MOTION_APPROACH_MM      150.0f

/* Coast after braking, mm. MEASURE THIS - see the procedure in motion.c.
 * Subtracted from the target so the robot ends up on the mark rather than
 * past it. Starting guess only. */
#define MOTION_BRAKE_MM         0.0f

/* Brake settling time before declaring the move finished, in 10 ms ticks. */
#define MOTION_BRAKE_TICKS      40U

/* Watchdog. A stalled wheel must not hang the primitive forever, or in
 * Phase 7 it hangs the RPi link with no reply and nothing to explain it. */
#define MOTION_TIMEOUT_TICKS    1500U   /* 15 s */

typedef enum
{
    MOTION_IDLE = 0,
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

/* Abort now: brake, recentre steering, go to IDLE. */
void Motion_Stop(void);

/* 1 while a primitive is running. Poll this to know when to reply OK. */
uint8_t Motion_IsBusy(void);

MotionState_t Motion_GetState(void);

/* Distance still to go, mm. For the OLED while tuning. */
int32_t Motion_GetRemaining(void);

#endif /* __MOTION_H */
