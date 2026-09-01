#include "odom.h"
#include "encoders.h"
#include "pid.h"
#include <math.h>

static Odom_Pose_t s_pose;

static int32_t s_lastCountA;
static int32_t s_lastCountB;

/* Heading hold state */
static uint8_t s_holdActive;
static int16_t s_holdRpm;
static float   s_holdHeading;
static float   s_trim;

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
    s_trim       = 0.0f;

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
    float   error;

    countA = Encoder_A_GetCount();
    countB = Encoder_B_GetCount();

    /* Distance each wheel rolled since the last tick. */
    dA_mm = (float)(countA - s_lastCountA) * MM_PER_COUNT;
    dB_mm = (float)(countB - s_lastCountB) * MM_PER_COUNT;

    s_lastCountA = countA;
    s_lastCountB = countB;

    /* Centre of the axle moves the average of the two wheels; the robot
     * rotates by their difference over the track width. This is the standard
     * differential-drive model, and it is exact in the limit of small steps.
     * At 10 ms and 250 RPM a step is under 4 mm, so the error from treating
     * the arc as a straight line is negligible. */
    d_centre    = (dA_mm + dB_mm) * 0.5f;
    d_theta_deg = ((dA_mm - dB_mm) / WHEEL_BASE_MM) * (180.0f / 3.14159265f);

    /* Integrate at the midpoint heading rather than the start heading. Costs
     * nothing and roughly halves the drift on curved paths. */
    heading_rad = (s_pose.heading_deg + d_theta_deg * 0.5f) * (3.14159265f / 180.0f);

    s_pose.x_mm += d_centre * cosf(heading_rad);
    s_pose.y_mm += d_centre * sinf(heading_rad);

    s_pose.heading_deg = wrap180(s_pose.heading_deg + d_theta_deg);

    /* Path length, always positive, so reversing does not subtract from it. */
    s_pose.distance_mm += (d_centre < 0.0f) ? -d_centre : d_centre;

    /* ---- heading hold ---- */

    if (s_holdActive)
    {
        /* Positive error means the robot has drifted clockwise of where it
         * should be, so wheel A needs to slow and B to speed up. */
        error  = wrap180(s_holdHeading - s_pose.heading_deg);
        s_trim = clampf(HEADING_KP * error, -HEADING_TRIM_MAX, HEADING_TRIM_MAX);

        PID_SetTargets((int16_t)((float)s_holdRpm + s_trim),
                       (int16_t)((float)s_holdRpm - s_trim));
    }
}

void Odom_GetPose(Odom_Pose_t *out)
{
    if (out != 0)
    {
        *out = s_pose;
    }
}

float Odom_GetHeading(void)  { return s_pose.heading_deg; }
float Odom_GetDistance(void) { return s_pose.distance_mm; }
float Odom_GetTrim(void)     { return s_trim; }

/* ------------------------------------------------------------------ */
/* Heading hold                                                        */
/* ------------------------------------------------------------------ */

void Odom_DriveStraight(int16_t rpm)
{
    Odom_DriveHeading(rpm, s_pose.heading_deg);
}

void Odom_DriveHeading(int16_t rpm, float heading_deg)
{
    s_holdHeading = wrap180(heading_deg);
    s_holdRpm     = rpm;
    s_trim        = 0.0f;
    s_holdActive  = 1U;
}

void Odom_Stop(void)
{
    s_holdActive = 0U;
    s_trim       = 0.0f;
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

    /* One full turn on the spot means the wheel-difference equals the
     * circumference of a circle whose diameter is the track width. So
     * base = (dA - dB) / pi. */
    diff = dA_mm - dB_mm;
    if (diff < 0.0f) { diff = -diff; }

    return diff / 3.14159265f;
}
