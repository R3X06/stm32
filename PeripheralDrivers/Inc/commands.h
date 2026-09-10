/**
  ******************************************************************************
  * @file    commands.h
  * @brief   RPi <-> STM32 command protocol.  FROZEN INTERFACE.
  *
  *          MDP Group 15 - WHEELTEC C30D V2.1 (STM32F407VET6)
  *
  *          This header is the ONLY coupling point between the motion/firmware
  *          workstream (Person A) and the sensors/comms workstream (Person B).
  *          Wire format is inherited verbatim from the previous year's proven
  *          implementation so that the RPi-side driver needs no changes.
  *
  *          DO NOT change token spellings, argument units, or reply strings
  *          without agreement from both owners AND the RPi team.
  ******************************************************************************
  * OWNERSHIP
  *
  *   Person A (Mahan)  : USART3 driver, line parser, command queue,
  *                       motion primitives, OK/RESEND emission.
  *   Person B          : sensor drivers behind the Sensors_* hooks at the
  *                       bottom of this file.  Nothing else in here.
  ******************************************************************************
  */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * 1. WIRE FORMAT
 * ===========================================================================
 *
 *   RPi -> STM :   <tok1>,<tok2>,...,<tokN>\n
 *   STM -> RPi :   OK\n          after the ENTIRE line has executed
 *                  RESEND\n      on parse failure, nothing executed
 *
 *   - Tokens separated by comma OR space. Both accepted.
 *   - Line terminated by '\n'. A '\r' before it is tolerated and discarded.
 *   - Input is lowercased on receive, so sender casing is irrelevant.
 *   - ONE reply per LINE, never per token.
 *   - RST is the sole exception: it aborts immediately and replies nothing.
 *
 *   Example:  "FR90,F20,S\n"  ->  arc right 90 deg, forward 20 cm, stop, "OK\n"
 */

#define CMD_BAUD_RATE           115200u   /* USART3, 8N1, no flow control     */
#define CMD_LINE_MAX            128u      /* max bytes in one line inc. '\n'  */
#define CMD_QUEUE_DEPTH         16u       /* max tokens buffered per line     */
#define CMD_TERMINATOR          '\n'
#define CMD_DELIM_PRIMARY       ','
#define CMD_DELIM_SECONDARY     ' '

#define CMD_REPLY_OK            "OK\n"
#define CMD_REPLY_RESEND        "RESEND\n"

/* Per-primitive watchdog. If a primitive has not completed within this many
 * milliseconds the motion layer aborts it, brakes, and the line still replies
 * so the RPi is never left waiting forever on a stalled wheel.
 *
 * *** NOT CURRENTLY HONOURED - THE FIRMWARE USES 15 s. ***
 *
 * motion.c enforces MOTION_TIMEOUT_TICKS, which is 1500 ticks at 10 ms. This
 * constant is left here rather than deleted because the gap is a protocol
 * question, not dead code: if the RPi gives up at 8 s while the STM is still
 * working through a 15 s watchdog, the two desynchronise and the next reply
 * lands against the wrong command. Reconcile the two numbers with the RPi
 * owner before the first wire test, and make whichever survives the one both
 * sides read. */
#define CMD_PRIMITIVE_TIMEOUT_MS   8000u


/* ===========================================================================
 * 2. OPCODES
 * ===========================================================================
 *
 *  Token    Arg unit   Meaning
 *  -------  ---------  ------------------------------------------------------
 *  F{n}     cm         Forward n cm.
 *  F0       -          Forward indefinitely until an obstacle stops it.
 *                      Requires Sensors_ObstacleAhead(). Person B dependency.
 *  R{n}     cm         Reverse n cm.
 *  FR{n}    degrees    Arc forward-right through n degrees.
 *  FL{n}    degrees    Arc forward-left  through n degrees.
 *  RR{n}    degrees    Arc reverse-right through n degrees.
 *  RL{n}    degrees    Arc reverse-left  through n degrees.
 *  S        -          Stop: brake motors, recentre servo. Replies OK.
 *  RST      -          Emergency abort: drop queue, brake NOW. Replies nothing.
 *
 *  NOTE ON ARCS: this chassis is Ackermann-steered and CANNOT turn on the
 *  spot. Every turn is an arc with forward or reverse travel. The RPi path
 *  planner must account for the swept area.
 */
typedef enum
{
    CMD_NONE = 0,       /* empty slot / parser sentinel            */
    CMD_FORWARD,        /* F{n}   arg = cm      (arg 0 => until obstacle) */
    CMD_REVERSE,        /* R{n}   arg = cm                         */
    CMD_ARC_FWD_RIGHT,  /* FR{n}  arg = degrees                    */
    CMD_ARC_FWD_LEFT,   /* FL{n}  arg = degrees                    */
    CMD_ARC_REV_RIGHT,  /* RR{n}  arg = degrees                    */
    CMD_ARC_REV_LEFT,   /* RL{n}  arg = degrees                    */
    CMD_STOP,           /* S      arg unused                       */
    CMD_RESET,          /* RST    arg unused                       */
    CMD_INVALID         /* unrecognised token -> whole line RESEND */
} CmdOpcode_t;

typedef struct
{
    CmdOpcode_t op;
    int16_t     arg;    /* cm or degrees, always positive; direction is in op */
} Command_t;


/* ===========================================================================
 * 3. RESERVED - TASK 2 ONLY. Not implemented for the checklist.
 * ===========================================================================
 * Listed so the enum above is never renumbered when these are added later.
 * Parser must return CMD_INVALID for these until Task 2 begins.
 *
 *   FU{n}   Forward until ultrasound reads <= n cm
 *   FIR     Forward until right IR sees wall disappear
 *   FIL     Forward until left  IR sees wall disappear
 *   FIRO    Forward until right IR sees wall appear
 *   FILO    Forward until left  IR sees wall appear
 *   SR / SL Diagonal slide right / left
 *
 * Task 2 replies, also reserved:
 *   ir{d}\n          IR distance, cm
 *   us{d1},{d2}\n    ultrasound: travelled dist, stop dist, cm
 */


/* ===========================================================================
 * 4. PARSER + QUEUE API   -- implemented by Person A in commands.c
 * ===========================================================================
 */

/** Reset parser and queue to empty. Call on init and on RST. */
void Cmd_Init(void);

/**
 * Parse one complete received line into the queue.
 * The line must be NUL-terminated with the '\n' already stripped.
 * Parsing is all-or-nothing: on any invalid token the queue is left
 * untouched and the caller must reply RESEND.
 *
 * @return 1 if the whole line parsed and was queued, 0 on failure.
 */
uint8_t Cmd_ParseLine(const char *line);

/** Parse a single token. Returns op = CMD_INVALID if unrecognised. */
Command_t Cmd_ParseToken(const char *token);

/** Pop the next queued primitive. Returns op = CMD_NONE when empty. */
Command_t Cmd_QueuePop(void);

/** Number of primitives still queued. */
uint8_t Cmd_QueueCount(void);

/** Drop everything queued. Used by RST. */
void Cmd_QueueFlush(void);


/* ===========================================================================
 * 5. SENSOR HOOKS   -- implemented by Person B
 * ===========================================================================
 * These are the ONLY symbols Person B must provide to the motion layer.
 * Person A ships weak default stubs so the firmware links and runs the
 * checklist before any sensor exists. Person B's strong definitions override
 * them automatically at link time - no #ifdef, no coordination needed.
 *
 * Keep these non-blocking. They are polled from the 10 ms TIM6 control tick
 * and MUST return promptly. Do not busy-wait for an ultrasound echo inside
 * them; capture in your own interrupt and return the last cached value.
 */

/** Most recent front distance in cm. Return SENSOR_NO_READING if unknown. */
#define SENSOR_NO_READING   0xFFFFu
uint16_t Sensors_FrontDistanceCm(void);

/** 1 if an obstacle is within stop_cm ahead, else 0. Backs the F0 command. */
uint8_t  Sensors_ObstacleAhead(uint16_t stop_cm);

/** IMU yaw in degrees, positive counter-clockwise, wrapped to (-180, 180]. */
float    Sensors_HeadingDeg(void);

/** 1 once the IMU has been initialised and its heading is trustworthy. */
uint8_t  Sensors_HeadingValid(void);


#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
