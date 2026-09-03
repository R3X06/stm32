#include "odom.h"
#include "encoders.h"
#include "pid.h"
#include "motors.h"
#include <math.h>

static Odom_Pose_t s_pose;

static int32_t s_lastCountA;
static int32_t s_lastCountB;

/* Heading hold state */
static uint8_t  s_holdActive;
static int16_t  s_holdRpm;
static float    s_holdHeading;
static float    s_error;
static uint16_t s_servoUs;

/* Spin calibration state */
static int32_t s_spinStartA;
static int32_t s_spinStartB;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Keeps an angle in -180..+180. Without this, heading errors across the
 * +-180 boundary come out as ~360 degrees and the correction slams the
 * wrong way. */
static float wrap180(float deg)
{
    while (deg >  180.0f) { deg -= 360.0f; }
    while (deg < -180.0f) { deg += 360.0f; }
    return deg;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* ------------------------------------------------------------------ */
/* Pose integration                                                    */
/* ------------------------------------------------------------------ */

void Odom_Init(void)
{
    s_lastCountA = Encoder_A_GetCount();
    s_lastCountB = Encoder_B_GetCount();

    s_holdActive = 0U;
    s_holdRpm    = 0;
    s_error      = 0.0f;
    s_servoUs    = SERVO_CENTER_US;

    Odom_Reset();
}

void Odom_Reset(void)
{
    s_pose.x_mm        = 0.0f;
    s_pose.y_mm        = 0.0f;
    s_pose.heading_deg = 0.0f;
    s_pose.distance_mm = 0.0f;
}

void Odom_Update(void)
{
    int32_t countA;
    int32_t countB;
    float   dA_mm;
    float   dB_mm;
    float   d_centre;
    float   d_theta_deg;
    float   heading_rad;
    float   correction;

    countA = Encoder_A_GetCount();
    countB = Encoder_B_GetCount();

    /* Distance each wheel rolled since the last tick. */
    dA_mm = (float)(countA - s_lastCountA) * MM_PER_COUNT;
    dB_mm = (float)(countB - s_lastCountB) * MM_PER_COUNT;

    s_lastCountA = countA;
    s_lastCountB = countB;

    /* Centre of the axle moves the average of the two wheels; the robot
     * rotates by their difference over the track width. At 10 ms and 250 RPM
     * a step is under 4 mm, so treating the arc as a straight line is fine.
     *
     * On an Ackermann chassis the two rear wheels differ only slightly on a
     * gentle curve, so this heading estimate is noisy. It is good enough for
     * A.3 and A.4; the IMU will replace it later. */
    d_centre    = (dA_mm + dB_mm) * 0.5f;
    d_theta_deg = ((dA_mm - dB_mm) / WHEEL_BASE_MM) * (180.0f / 3.14159265f);

    /* Integrate at the midpoint heading rather than the start heading. */
    heading_rad = (s_pose.heading_deg + d_theta_deg * 0.5f) * (3.14159265f / 180.0f);

    s_pose.x_mm += d_centre * cosf(heading_rad);
    s_pose.y_mm += d_centre * sinf(heading_rad);

    s_pose.heading_deg = wrap180(s_pose.heading_deg + d_theta_deg);

    /* Path length, always positive, so reversing does not subtract from it. */
    s_pose.distance_mm += (d_centre < 0.0f) ? -d_centre : d_centre;

    /* ---- heading hold, via the steering servo ---- */

    if (s_holdActive)
    {
        s_error = wrap180(s_holdHeading - s_pose.heading_deg);

        /* Deadband. Below this the servo would only hunt and buzz. */
        if ((s_error < HEADING_DEADBAND_DEG) && (s_error > -HEADING_DEADBAND_DEG))
        {
            correction = 0.0f;
        }
        else
        {
            correction = HEADING_KP_US * s_error * (float)HEADING_SIGN;

            /* Reversing: the same steering angle bends the path the other
             * way relative to travel, so flip the correction. */
            if (s_holdRpm < 0) { correction = -correction; }

            /* Push through the linkage slack. Without this, corrections
             * smaller than the backlash do nothing at all, the error grows
             * until they exceed it, and the robot weaves. */
            if (correction > 0.0f)      { correction += SERVO_BACKLASH_US; }
            else if (correction < 0.0f) { correction -= SERVO_BACKLASH_US; }

            correction = clampf(correction, -HEADING_MAX_US, HEADING_MAX_US);
        }

        s_servoUs = (uint16_t)((float)SERVO_CENTER_US + correction);
        Servo_SetMicroseconds(s_servoUs);   /* clamps to MIN/MAX internally */

        /* Both rear wheels at the same speed. The PID's only job now is to
         * make A and B match each other - steering is the servo's. */
        PID_SetTargets(s_holdRpm, s_holdRpm);
    }
}

void Odom_GetPose(Odom_Pose_t *out)
{
    if (out != 0)
    {
        *out = s_pose;
    }
}

float    Odom_GetHeading(void)      { return s_pose.heading_deg; }
float    Odom_GetDistance(void)     { return s_pose.distance_mm; }
uint16_t Odom_GetServoUs(void)      { return s_servoUs; }
float    Odom_GetHeadingError(void) { return s_error; }

/* ------------------------------------------------------------------ */
/* Heading hold                                                        */
/* ------------------------------------------------------------------ */

/* SETTING HEADING_SIGN
 *
 * Two conventions have to agree and neither has been verified on this robot:
 * which way Odom heading counts, and which way rising microseconds steer.
 * HEADING_SIGN absorbs both. Find it like this, on the floor:
 *
 *   1. Flash with HEADING_SIGN as +1.
 *   2. Start a straight run and let it settle.
 *   3. Nudge the robot's nose to one side by hand, a few degrees.
 *
 *   Correct: the front wheels turn to steer BACK toward the original
 *   heading and the robot recovers.
 *
 *   Wrong: the front wheels turn the SAME way you nudged, the error grows,
 *   and the robot leaves the line immediately. Flip HEADING_SIGN to -1.
 *
 * The wrong sign is unmistakable - it diverges within a metre. Do this test
 * before spending any time on HEADING_KP_US. */

void Odom_DriveStraight(int16_t rpm)
{
    Odom_DriveHeading(rpm, s_pose.heading_deg);
}

void Odom_DriveHeading(int16_t rpm, float heading_deg)
{
    s_holdHeading = wrap180(heading_deg);
    s_holdRpm     = rpm;
    s_error       = 0.0f;
    s_servoUs     = SERVO_CENTER_US;

    Servo_SetMicroseconds(SERVO_CENTER_US);
    s_holdActive  = 1U;
}

void Odom_Stop(void)
{
    s_holdActive = 0U;
    s_error      = 0.0f;
    s_servoUs    = SERVO_CENTER_US;

    Servo_SetMicroseconds(SERVO_CENTER_US);
    PID_Stop();
}

/* ------------------------------------------------------------------ */
/* Spin calibration                                                    */
/* ------------------------------------------------------------------ */

void Odom_CalibrateSpinStart(void)
{
    s_spinStartA = Encoder_A_GetCount();
    s_spinStartB = Encoder_B_GetCount();
}

float Odom_CalibrateSpinResult(void)
{
    float dA_mm;
    float dB_mm;
    float diff;

    dA_mm = (float)(Encoder_A_GetCount() - s_spinStartA) * MM_PER_COUNT;
    dB_mm = (float)(Encoder_B_GetCount() - s_spinStartB) * MM_PER_COUNT;

    diff = dA_mm - dB_mm;
    if (diff < 0.0f) { diff = -diff; }

    return diff / 3.14159265f;
}
