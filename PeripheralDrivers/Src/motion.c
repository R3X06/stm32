#include "motion.h"
#include "odom.h"
#include "pid.h"
#include "motors.h"

typedef enum { MOVE_STRAIGHT = 0, MOVE_ARC } MoveKind_t;

static MotionState_t s_state;
static MoveKind_t s_kind;
static float    s_targetMm;      /* absolute, brake compensation applied */
static float    s_holdHeading;   /* heading latched at the start of a straight */
static int8_t   s_dir;           /* +1 forward, -1 reverse                   */
static uint8_t  s_arcRight;      /* which way the arc curves                 */
static uint16_t s_arcServoUs;    /* servo pulse held for the whole arc       */
static int16_t  s_lastRpm;       /* last speed issued to the odom layer      */
static uint32_t s_ticks;
static uint32_t s_brakeTicks;
static uint32_t s_alignTicks;
static uint16_t s_startServoUs;

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

/* ---------------------------------------------------------------------------
 * Starting and stopping a primitive is done from the MAIN LOOP, while the
 * control tick is running in an interrupt at 100 Hz. Every one of these entry
 * points rewrites state the tick reads on the very next pass - the target,
 * the direction, the odometry pose - and none of those writes is atomic.
 *
 * Without a critical section a command arriving from the RPi at the wrong
 * microsecond can leave the tick reading a new target against an old pose,
 * which shows up as a move that stops instantly or overshoots by a metre and
 * never reproduces on the bench. Cheap to prevent, miserable to debug.
 *
 * PRIMASK is saved and restored rather than blindly re-enabled, so these are
 * safe to nest and safe to call from an interrupt if that is ever needed.
 * ------------------------------------------------------------------------- */
static inline uint32_t crit_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void crit_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* ---------------------------------------------------------------------------
 * Stopping the wheels.
 *
 * PID_Enable(0) MUST come before Motors_Brake(). Braking is done by driving
 * both H-bridge inputs high; the PID drives them from the same tick, so if it
 * is left enabled the very next PID_Update() writes a zero duty over the top
 * and the brake silently becomes a coast. That was costing the whole of the
 * stopping-distance consistency the +/-6% requirement depends on.
 * ------------------------------------------------------------------------- */
static void motion_halt(void)
{
    Odom_Stop();        /* recentres steering, drops the odom drive mode */
    PID_Enable(0);      /* stop the loop writing duty - MUST precede brake */
    Motors_Brake();
    s_lastRpm = 0;
}


/* Park the steering where the move needs it and hold everything still while
 * it gets there. The odometry is NOT zeroed here - that happens when the
 * settle finishes, so any twitch while the servo swings is discarded rather
 * than counted as travel. */
static void motion_begin_align(uint16_t servo_us)
{
    uint16_t now = Odom_GetServoUs();
    uint16_t diff = (now > servo_us) ? (uint16_t)(now - servo_us)
                                     : (uint16_t)(servo_us - now);

    s_startServoUs = servo_us;

    PID_Enable(0);
    Motors_Coast();
    Servo_SetMicroseconds(servo_us);

    /* Already there - no point waiting for a servo that will not move.
     *
     * The MOTION_ALIGN_SKIP_US > 0 test is not redundant. Without it, setting
     * the threshold to zero to force the settle does nothing, because a servo
     * sitting exactly on target gives diff == 0 and 0 <= 0 still skips. And
     * exactly on target is the normal case: Odom_Stop() recentres the
     * steering at the end of every move, so the next straight run starts with
     * diff of precisely zero. Zero now means never skip. */
    s_alignTicks = ((MOTION_ALIGN_SKIP_US > 0U) && (diff <= MOTION_ALIGN_SKIP_US))
                 ? MOTION_ALIGN_TICKS
                 : 0U;

    s_ticks = 0U;
    s_state = MOTION_ALIGN;
}

/* Settle finished: zero the odometry now, then start driving. */
static void motion_launch(void)
{
    Odom_Reset();

    s_ticks      = 0U;
    s_brakeTicks = 0U;
    s_lastRpm    = (int16_t)(s_dir * MOTION_CRUISE_RPM);
    s_state      = MOTION_RUN;

    PID_Enable(1);

    if (s_kind == MOVE_ARC)
    {
        Odom_DriveArc(s_lastRpm, s_arcServoUs, MOTION_ARC_RADIUS_MM, s_arcRight);
    }
    else
    {
        s_holdHeading = Odom_GetHeading();   /* zero, just after the reset */
        Odom_DriveHeading(s_lastRpm, s_holdHeading);
    }
}

void Motion_Init(void)
{
    s_state       = MOTION_IDLE;
    s_kind        = MOVE_STRAIGHT;
    s_targetMm    = 0.0f;
    s_holdHeading = 0.0f;
    s_dir         = 1;
    s_arcRight    = 0U;
    s_arcServoUs  = SERVO_CENTER_US;
    s_lastRpm     = 0;
    s_ticks       = 0U;
    s_brakeTicks  = 0U;
    s_alignTicks  = 0U;
    s_startServoUs = SERVO_CENTER_US;
}

void Motion_DriveDistance(int32_t mm)
{
    float    target;
    uint32_t pm;

    if (mm == 0) { return; }

    pm = crit_enter();

    s_kind = MOVE_STRAIGHT;
    s_dir  = (mm < 0) ? -1 : 1;

    target = (float)((mm < 0) ? -mm : mm) - MOTION_BRAKE_MM;
    if (target < 0.0f) { target = 0.0f; }
    s_targetMm = target;

    motion_begin_align(SERVO_CENTER_US);

    crit_exit(pm);
}

void Motion_DriveArc(int16_t degrees, uint8_t forward, uint8_t right)
{
    float    rad;
    float    arc_mm;
    uint32_t pm;

    if (degrees <= 0) { return; }

    pm = crit_enter();

    s_kind     = MOVE_ARC;
    s_dir      = forward ? 1 : -1;
    s_arcRight = right ? 1U : 0U;

    s_arcServoUs = right
                 ? (uint16_t)((float)SERVO_CENTER_US + MOTION_ARC_STEER_US)
                 : (uint16_t)((float)SERVO_CENTER_US - MOTION_ARC_STEER_US);

    rad    = (float)degrees * (3.14159265f / 180.0f);
    arc_mm = MOTION_ARC_RADIUS_MM * rad;

    arc_mm -= MOTION_BRAKE_MM;
    if (arc_mm < 0.0f) { arc_mm = 0.0f; }
    s_targetMm = arc_mm;

    /* An arc needs a much bigger steering movement than a straight line, so
     * the settle matters more here, not less. */
    motion_begin_align(s_arcServoUs);

    crit_exit(pm);
}

void Motion_Stop(void)
{
    uint32_t pm = crit_enter();

    motion_halt();
    s_state = MOTION_IDLE;

    crit_exit(pm);
}

/* Change speed mid-move.
 *
 * Odom_SetSpeed(), NOT Odom_DriveHeading()/Odom_DriveArc(). Those two start a
 * move: they latch a heading, clear the accumulated error and recentre the
 * servo. Using them for the approach taper would discard the steering
 * correction the loop had settled on and snap the wheels straight 150 mm
 * before the stop. */
static void motion_issue(int16_t rpm)
{
    Odom_SetSpeed(rpm);
}

void Motion_Tick(void)
{
    float   travelled;
    float   remaining;
    int16_t want;

    if (s_state == MOTION_ALIGN)
    {
        s_alignTicks++;
        if (s_alignTicks >= MOTION_ALIGN_TICKS)
        {
            motion_launch();
        }
        return;
    }

    if ((s_state != MOTION_RUN) && (s_state != MOTION_BRAKE))
    {
        return;
    }

    s_ticks++;
    if (s_ticks > MOTION_TIMEOUT_TICKS)
    {
        /* Wheel stalled, or the encoder stopped counting. Give up cleanly
         * rather than sit here forever. */
        motion_halt();
        s_state = MOTION_TIMEOUT;
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
            motion_halt();
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
         * error is always stale and the servo is fighting itself. */
        if (want != s_lastRpm)
        {
            motion_issue(want);
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
    return ((s_state == MOTION_ALIGN) ||
            (s_state == MOTION_RUN)   ||
            (s_state == MOTION_BRAKE)) ? 1U : 0U;
}

MotionState_t Motion_GetState(void) { return s_state; }

void Motion_ClearState(void)
{
    uint32_t pm = crit_enter();

    if ((s_state == MOTION_DONE) || (s_state == MOTION_TIMEOUT))
    {
        s_state = MOTION_IDLE;
    }

    crit_exit(pm);
}

int32_t Motion_GetRemaining(void)
{
    return (int32_t)(s_targetMm - Odom_GetDistance());
}

int32_t Motion_GetTravelled(void)
{
    return (int32_t)Odom_GetDistance();
}
