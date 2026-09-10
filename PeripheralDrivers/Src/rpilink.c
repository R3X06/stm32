#include "rpilink.h"
#include "motion.h"
#include "motors.h"
#include "pid.h"
#include "odom.h"
#include <string.h>

static UART_HandleTypeDef *s_uart;

/* Receive path ----------------------------------------------------- */
static uint8_t  s_rxByte;
static char     s_rxLine[CMD_LINE_MAX];    /* filled by the ISR          */
static uint16_t s_rxLen;
static char     s_pending[CMD_LINE_MAX];   /* handed to the main loop    */
static volatile uint8_t s_lineReady;
static volatile uint8_t s_overrun;

/* Executor --------------------------------------------------------- */
static uint8_t     s_lineActive;   /* a line is in flight, owes a reply */
static uint8_t     s_f0Active;     /* F0 running, poll the ultrasound   */
static CmdOpcode_t s_lastOp = CMD_NONE;

/* ===================================================================
 * Weak sensor stubs.
 *
 * These let the whole firmware link and run the checklist before any
 * sensor driver exists. Person B's strong definitions override them
 * automatically at link time - no #ifdef, no coordination.
 *
 * They are deliberately pessimistic: no reading, no obstacle, no valid
 * heading. Nothing downstream should ever mistake a stub for data.
 * =================================================================== */

__attribute__((weak)) uint16_t Sensors_FrontDistanceCm(void)
{
    return SENSOR_NO_READING;
}

__attribute__((weak)) uint8_t Sensors_ObstacleAhead(uint16_t stop_cm)
{
    (void)stop_cm;
    return 0U;
}

__attribute__((weak)) float Sensors_HeadingDeg(void)
{
    return 0.0f;
}

__attribute__((weak)) uint8_t Sensors_HeadingValid(void)
{
    return 0U;
}

/* ------------------------------------------------------------------ */
/* Transmit                                                            */
/* ------------------------------------------------------------------ */

void RpiLink_Send(const char *s)
{
    if ((s_uart == 0) || (s == 0)) { return; }

    (void)HAL_UART_Transmit(s_uart, (uint8_t *)s, (uint16_t)strlen(s), 100U);
}

/* ------------------------------------------------------------------ */
/* Receive                                                             */
/* ------------------------------------------------------------------ */

void RpiLink_Init(UART_HandleTypeDef *huart)
{
    s_uart       = huart;
    s_rxLen      = 0U;
    s_lineReady  = 0U;
    s_overrun    = 0U;
    s_lineActive = 0U;
    s_f0Active   = 0U;
    s_lastOp     = CMD_NONE;

    Cmd_Init();

    (void)HAL_UART_Receive_IT(s_uart, &s_rxByte, 1U);
}

void RpiLink_RxCallback(void)
{
    char c = (char)s_rxByte;

    if (c == CMD_TERMINATOR)
    {
        s_rxLine[s_rxLen] = '\0';

        /* If the main loop has not consumed the previous line yet, drop this
         * one and remember to say RESEND. Overwriting s_pending mid-parse
         * would corrupt a line that has already been accepted. */
        if (s_lineReady)
        {
            s_overrun = 1U;
        }
        else
        {
            (void)memcpy(s_pending, s_rxLine, (size_t)s_rxLen + 1U);
            s_lineReady = 1U;
        }

        s_rxLen = 0U;
    }
    else if (c == '\r')
    {
        /* Tolerated and discarded, per the wire format. */
    }
    else
    {
        if (s_rxLen < (CMD_LINE_MAX - 1U))
        {
            s_rxLine[s_rxLen] = c;
            s_rxLen++;
        }
        else
        {
            /* Line too long. Keep swallowing until the terminator so the
             * next line starts clean; the parse will fail and reply RESEND. */
            s_rxLine[CMD_LINE_MAX - 2U] = '?';
        }
    }

    (void)HAL_UART_Receive_IT(s_uart, &s_rxByte, 1U);
}

/* ------------------------------------------------------------------ */
/* Executor                                                            */
/* ------------------------------------------------------------------ */

static void abort_everything(void)
{
    Cmd_QueueFlush();
    Motion_Stop();
    Motion_ClearState();
    Motors_Coast();
    s_lineActive = 0U;
    s_f0Active   = 0U;
}

/* Start one primitive. Returns 1 if it set the motion layer running. */
static uint8_t dispatch(Command_t c)
{
    s_lastOp   = c.op;
    s_f0Active = 0U;

    switch (c.op)
    {
    case CMD_FORWARD:
        if (c.arg == 0)
        {
            /* F0: run forward until an obstacle stops us. Implemented as a
             * very long move that RpiLink_Poll() cuts short, rather than a
             * special motion mode, so the motion layer stays unaware of
             * sensors and the watchdog still covers it.
             *
             * With the weak stub in place this will run to the motion
             * timeout. That is the correct behaviour for "no sensor yet" -
             * it stops, and the line still gets its reply. */
            Motion_DriveDistance(RPILINK_F0_MAX_MM);
            s_f0Active = 1U;
        }
        else
        {
            Motion_DriveDistance((int32_t)c.arg * 10);   /* cm -> mm */
        }
        return 1U;

    case CMD_REVERSE:
        Motion_DriveDistance(-((int32_t)c.arg * 10));
        return 1U;

    case CMD_ARC_FWD_RIGHT: Motion_DriveArc(c.arg, 1U, 1U); return 1U;
    case CMD_ARC_FWD_LEFT:  Motion_DriveArc(c.arg, 1U, 0U); return 1U;
    case CMD_ARC_REV_RIGHT: Motion_DriveArc(c.arg, 0U, 1U); return 1U;
    case CMD_ARC_REV_LEFT:  Motion_DriveArc(c.arg, 0U, 0U); return 1U;

    case CMD_STOP:
        Motion_Stop();
        Motion_ClearState();
        Motors_Coast();
        return 0U;      /* completes instantly */

    case CMD_RESET:
        /* Should have been intercepted before it ever reached the queue,
         * but handle it defensively rather than fall through to default. */
        abort_everything();
        return 0U;

    default:
        return 0U;
    }
}

void RpiLink_Poll(void)
{
    char      line[CMD_LINE_MAX];
    Command_t next;
    uint16_t  i;

    /* ---- 1. a dropped line still owes the sender an answer ---- */
    if (s_overrun)
    {
        s_overrun = 0U;
        RpiLink_Send(CMD_REPLY_RESEND);
    }

    /* ---- 2. new line in ---- */
    if (s_lineReady)
    {
        (void)memcpy(line, s_pending, sizeof(line));
        s_lineReady = 0U;

        /* Lowercase on receive, so sender casing is irrelevant everywhere
         * downstream. */
        for (i = 0U; i < CMD_LINE_MAX; i++)
        {
            if (line[i] == '\0') { break; }
            if ((line[i] >= 'A') && (line[i] <= 'Z'))
            {
                line[i] = (char)(line[i] - 'A' + 'a');
            }
        }

        if (strcmp(line, "rst") == 0)
        {
            /* Emergency abort. Drops everything queued, brakes now, and
             * replies NOTHING. The silence is part of the frozen protocol -
             * do not "helpfully" add an OK here. */
            abort_everything();
        }
        else if (Cmd_ParseLine(line))
        {
            s_lineActive = 1U;
        }
        else
        {
            /* All-or-nothing: nothing was queued, so nothing to undo. */
            RpiLink_Send(CMD_REPLY_RESEND);
        }
    }

    /* ---- 3. F0 obstacle check ---- */
    if (s_f0Active && Motion_IsBusy())
    {
        if (Sensors_ObstacleAhead(RPILINK_F0_STOP_CM))
        {
            Motion_Stop();
            s_f0Active = 0U;
        }
    }

    /* ---- 4. advance the line ---- */
    if (!s_lineActive) { return; }

    if (Motion_IsBusy()) { return; }

    /* A primitive that finished or timed out: clear it either way. The line
     * replies OK even on a timeout - a stalled wheel must not leave the RPi
     * blocked forever waiting on a reply that never comes. */
    Motion_ClearState();

    next = Cmd_QueuePop();

    if (next.op == CMD_NONE)
    {
        /* Whole line executed. One reply, now. */
        s_lineActive = 0U;
        s_f0Active   = 0U;
        RpiLink_Send(CMD_REPLY_OK);
        return;
    }

    if (!dispatch(next))
    {
        /* Instant primitive such as S. Nothing to wait for - the next Poll()
         * pass picks up whatever follows it on the line. */
    }
}

uint8_t RpiLink_IsBusy(void) { return s_lineActive; }

CmdOpcode_t RpiLink_LastOpcode(void) { return s_lastOp; }
