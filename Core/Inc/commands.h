/**
  ******************************************************************************
  * @file    commands.h
  * @brief   MDP-G15 shared command protocol - the interface contract between
  *          Person A (motion/firmware core) and Person B (UART/sensors/RPi).
  *
  *  Both sides #include this file. It is the SINGLE source of truth for the
  *  RPi <-> STM32 command protocol. Do not redefine these tokens or types
  *  anywhere else - change them here and both sides stay in sync.
  *
  *  OWNERSHIP
  *    Person B owns: USART3 RX. Assemble incoming bytes into one line, then
  *                   call  mdp_enqueue_line(line)  when a '\n' arrives.
  *                   That is the ONLY function Person B needs from Person A.
  *    Person A owns: parsing the line, the command queue, motion execution,
  *                   and emitting the reply (MDP_REPLY_OK) when a line finishes.
  ******************************************************************************
  */
#ifndef __COMMANDS_H
#define __COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 *  WIRE FORMAT  (RPi -> STM32, over USART3 @ 115200 8N1)
 *
 *    <cmd1>,<cmd2>,...,<cmdN>\n
 *
 *  - Commands in one line are comma- OR space-separated (both accepted).
 *  - The line is newline-terminated ('\n').
 *  - Case-insensitive (parser lowercases).
 *  - ONE reply is sent per LINE (not per command), after the whole line runs.
 *
 *  Example:   FR90,F20,S\n     -> arc right 90 deg, forward 20 cm, stop
 * ============================================================================
 */

/* ---- Command tokens -------------------------------------------------------
 *
 *   TOKEN       MEANING
 *   fwd{n}      Forward n cm            (e.g. fwd20)
 *   rvs{n}      Reverse n cm            (e.g. rvs15)
 *   fwdR{n}     Arc forward-right n deg (e.g. fwdR90)
 *   fwdL{n}     Arc forward-left  n deg
 *   rvsR{n}     Arc reverse-right n deg
 *   rvsL{n}     Arc reverse-left  n deg
 *   stp         Stop (brake + servo center)
 *   RST         Emergency abort: drop queue and brake immediately
 *
 *  Tokens are case-insensitive (parser lowercases input), so the casing shown
 *  above is only for readability - fwdR, FWDR, fwdr all parse the same.
 *  NOTE for the parser: the 4-char arc tokens (fwdR/fwdL/rvsR/rvsL) MUST be
 *  matched BEFORE the 3-char fwd/rvs, or "fwdR90" wrongly matches "fwd" val 0.
 *
 *  If you (Person B) add sensor-driven tokens later (FU{n}, FIR, etc. from the
 *  reference task2 set), add them to the enum + parser here so both sides agree.
 * --------------------------------------------------------------------------- */

/* ---- Command types (parsed form of a token) ---- */
typedef enum {
  SCMD_NONE = 0,
  SCMD_FWD_CM,      /* value = cm  (+)  */
  SCMD_REV_CM,      /* value = cm  (+)  */
  SCMD_ARC_FR,      /* value = deg      */
  SCMD_ARC_FL,
  SCMD_ARC_RR,
  SCMD_ARC_RL,
  SCMD_STOP,
  SCMD_EOS          /* end-of-line marker: triggers the single OK reply */
} scmd_type_t;

typedef struct {
  scmd_type_t type;
  int16_t     value;
} script_item_t;

/* ---- Replies (STM32 -> RPi) ---- */
#define MDP_REPLY_OK      "OK\n"       /* whole line executed              */
#define MDP_REPLY_RESEND  "RESEND\n"   /* parse/format error - RPi resends */

/* ============================================================================
 *  INTERFACE FUNCTION  (the contract)
 *
 *  Person B calls this from the USART3 RX handler once a complete, newline-
 *  terminated line has been assembled. Person A implements it (in main.c):
 *  it parses the line into queue items and appends an SCMD_EOS marker.
 *  The line buffer may be modified in place by the parser.
 * ============================================================================
 */
void mdp_enqueue_line(char *line);

#ifdef __cplusplus
}
#endif

#endif /* __COMMANDS_H */
