#include "motion.h"
#include "odom.h"
#include "pid.h"
#include "motors.h"

static MotionState_t s_state;
static float    s_targetMm;      /* absolute, brake compensation applied */
static float    s_holdHeading;   /* heading latched at the start of the move */
static int8_t   s_dir;           /* +1 forward, -1 reverse                   */
static int16_t  s_lastRpm;       /* last speed issued to Odom_DriveHeading   */
static uint32_t s_ticks;
static uint32_t s_brakeTicks;

/* ---------------------------------------------------------------------------
 * MEASURING MOTION_BRAKE_MM
 *
 * Set MOTION_BRAKE_MM to 0, command 1000 mm, and measure where the robot
 * actually stops with a tape. If it lands at 1015, the coast is 15 mm - put
 * that in MOTION_BRAKE_MM and it will land on 1000.
 *
 * Do it five times and average. Coast varies with battery charge and floor
 * surface, so re-check if you demo on a different floor to the one you
 * calibrated on.
 *
 * Measure at MOTION_APPROACH_RPM, not cruise - the last stretch before the
 * stop is always at approach speed, so that is the speed that sets the coast.
 * ------------------------------------------------------------------------- */

void Motion_Init(void)
{
    s_state       = MOTION_IDLE;
    s_targetMm    = 0.0f;
    s_holdHeading = 0.0f;
    s_dir         = 1;
    s_lastRpm     = 0;
    s_ticks       = 0U;
    s_brakeTicks  = 0U;
}

void Motion_DriveDistance(int32_t mm)
{
    float target;

    if (mm == 0) { return; }

    s_dir = (mm < 0) ? -1 : 1;

    target = (float)((mm < 0) ? -mm : mm) - MOTION_BRAKE_MM;
    if (target < 0.0f) { target = 0.0f; }
    s_targetMm = target;

    /* Odom_GetDistance() is a path length that only ever grows, so zero it
     * and measure this move from scratch. */
    Odom_Reset();

    s_holdHeading = Odom_GetHeading();   /* zero, immediately after the reset */
    s_ticks       = 0U;
    s_brakeTicks  = 0U;
    s_state       = MOTION_RUN;

    s_lastRpm = (int16_t)(s_dir * MOTION_CRUISE_RPM);

    PID_Enable(1);
    Odom_DriveHeading(s_lastRpm, s_holdHeading);
}

void Motion_Stop(void)
{
    Odom_Stop();          /* recentres steering, stops the PID */
    Motors_Brake();
    s_lastRpm = 0;
    s_state   = MOTION_IDLE;
}

void Motion_Tick(void)
{
    float   travelled;
    float   remaining;
    int16_t want;

    if ((s_state != MOTION_RUN) && (s_state != MOTION_BRAKE))
    {
        return;
    }

    s_ticks++;
    if (s_ticks > MOTION_TIMEOUT_TICKS)
    {
        /* Wheel stalled, or the encoder stopped counting. Give up cleanly
         * rather than sit here forever. */
        Odom_Stop();
        Motors_Brake();
        s_lastRpm = 0;
        s_state   = MOTION_TIMEOUT;
        return;
    }

    travelled = Odom_GetDistance();     /* always positive */
    remaining = s_targetMm - travelled;

    if (s_state == MOTION_RUN)
    {
        if (remaining <= 0.0f)
        {
            /* Target reached. Brake and let it settle before declaring done -
             * reading the distance mid-coast would give a short answer. */
            Odom_Stop();
            Motors_Brake();
            s_lastRpm    = 0;
            s_brakeTicks = 0U;
            s_state      = MOTION_BRAKE;
            return;
        }

        want = (remaining <= MOTION_APPROACH_MM)
             ? (int16_t)(s_dir * MOTION_APPROACH_RPM)
             : (int16_t)(s_dir * MOTION_CRUISE_RPM);

        /* Only re-issue when the speed actually changes.
         *
         * Odom_DriveHeading() zeroes the heading error and slams the servo
         * back to centre. Calling it every tick means the heading loop is
         * reset 100 times a second and never keeps any state - the reported
         * error is always stale and the servo is fighting itself.
         *
         * Note the heading target passed is s_holdHeading, NOT the current
         * heading. Odom_DriveStraight() would re-latch onto wherever the
         * robot is pointing now and quietly forget the line it set out on. */
        if (want != s_lastRpm)
        {
            Odom_DriveHeading(want, s_holdHeading);
            s_lastRpm = want;
        }
    }
    else /* MOTION_BRAKE */
    {
        s_brakeTicks++;
        if (s_brakeTicks >= MOTION_BRAKE_TICKS)
        {
            Motors_Coast();
            s_state = MOTION_DONE;
        }
    }
}

uint8_t Motion_IsBusy(void)
{
    return ((s_state == MOTION_RUN) || (s_state == MOTION_BRAKE)) ? 1U : 0U;
}

MotionState_t Motion_GetState(void) { return s_state; }

int32_t Motion_GetRemaining(void)
{
    return (int32_t)(s_targetMm - Odom_GetDistance());
}
