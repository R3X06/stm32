/**
  ******************************************************************************
  * @file    commands.c
  * @brief   Parser and queue for the RPi <-> STM32 protocol frozen in
  *          commands.h. Person A side. No hardware here - this file is pure
  *          string handling and can be unit-tested off-target.
  *
  *          Parsing is all-or-nothing per LINE. Tokens are validated into a
  *          scratch array first and only committed to the queue once every
  *          one of them has passed, so a line ending in garbage never leaves
  *          half a manoeuvre queued behind a RESEND.
  ******************************************************************************
  */

#include "commands.h"
#include <string.h>

/* Circular queue. Head is the next slot to pop, count is what is in it. */
static Command_t s_queue[CMD_QUEUE_DEPTH];
static uint8_t   s_head;
static uint8_t   s_count;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static char lower(char c)
{
    return ((c >= 'A') && (c <= 'Z')) ? (char)(c - 'A' + 'a') : c;
}

/* Case-insensitive compare of a whole token against a literal. */
static uint8_t token_is(const char *tok, const char *lit)
{
    uint8_t i = 0U;

    while ((lit[i] != '\0') && (tok[i] != '\0'))
    {
        if (lower(tok[i]) != lit[i]) { return 0U; }
        i++;
    }

    return ((lit[i] == '\0') && (tok[i] == '\0')) ? 1U : 0U;
}

/* Case-insensitive prefix match. Returns the prefix length, or 0. */
static uint8_t token_starts(const char *tok, const char *pfx)
{
    uint8_t i = 0U;

    while (pfx[i] != '\0')
    {
        if (lower(tok[i]) != pfx[i]) { return 0U; }
        i++;
    }

    return i;
}

/* Strict unsigned decimal. The whole remainder must be digits and there must
 * be at least one, so "F", "F1x" and "F-5" are all rejected rather than
 * silently becoming F0.
 *
 * Returns 1 on success. Anything above INT16_MAX is a parse failure, not a
 * clamp - a distance the sender did not mean is worse than a RESEND. */
static uint8_t parse_uint(const char *s, int16_t *out)
{
    uint32_t v = 0U;
    uint8_t  n = 0U;

    while (s[n] != '\0')
    {
        if ((s[n] < '0') || (s[n] > '9')) { return 0U; }

        v = (v * 10U) + (uint32_t)(s[n] - '0');
        if (v > 32767U) { return 0U; }

        n++;
    }

    if (n == 0U) { return 0U; }

    *out = (int16_t)v;
    return 1U;
}

/* ------------------------------------------------------------------ */
/* Token parsing                                                       */
/* ------------------------------------------------------------------ */

Command_t Cmd_ParseToken(const char *token)
{
    Command_t   cmd;
    uint8_t     n;
    int16_t     arg = 0;
    CmdOpcode_t op  = CMD_INVALID;

    cmd.op  = CMD_INVALID;
    cmd.arg = 0;

    if ((token == 0) || (token[0] == '\0')) { return cmd; }

    /* Exact matches first. "RST" has to be tested before the R{n} prefix or
     * it parses as a reverse of "st" and fails for the wrong reason. */
    if (token_is(token, "rst")) { cmd.op = CMD_RESET; return cmd; }
    if (token_is(token, "s"))   { cmd.op = CMD_STOP;  return cmd; }

    /* Two-letter arc prefixes before the one-letter drive prefixes, so "fr"
     * is not eaten by "f". */
    if      ((n = token_starts(token, "fr")) != 0U) { op = CMD_ARC_FWD_RIGHT; }
    else if ((n = token_starts(token, "fl")) != 0U) { op = CMD_ARC_FWD_LEFT;  }
    else if ((n = token_starts(token, "rr")) != 0U) { op = CMD_ARC_REV_RIGHT; }
    else if ((n = token_starts(token, "rl")) != 0U) { op = CMD_ARC_REV_LEFT;  }
    else if ((n = token_starts(token, "f"))  != 0U) { op = CMD_FORWARD;       }
    else if ((n = token_starts(token, "r"))  != 0U) { op = CMD_REVERSE;       }
    else                                            { return cmd; }

    if (!parse_uint(&token[n], &arg)) { return cmd; }

    /* A zero-degree arc is not a manoeuvre. F0 IS meaningful - it is
     * "forward until an obstacle stops you" - and R0 is not, so only F is
     * allowed to carry a zero. */
    if (arg == 0)
    {
        if (op != CMD_FORWARD) { return cmd; }
    }

    cmd.op  = op;
    cmd.arg = arg;
    return cmd;
}

/* ------------------------------------------------------------------ */
/* Line parsing                                                        */
/* ------------------------------------------------------------------ */

uint8_t Cmd_ParseLine(const char *line)
{
    Command_t staged[CMD_QUEUE_DEPTH];
    char      tok[16];
    uint8_t   nstaged = 0U;
    uint8_t   tlen    = 0U;
    uint16_t  i       = 0U;
    uint8_t   j;
    char      c;

    if (line == 0) { return 0U; }

    /* Walk the line one character at a time, cutting a token at every comma,
     * space or end of string. Empty runs of delimiters are skipped so
     * "F10,,F10" and "F10  F10" both behave. */
    for (;;)
    {
        c = line[i];

        if ((c == CMD_DELIM_PRIMARY) || (c == CMD_DELIM_SECONDARY) ||
            (c == '\t') || (c == '\r') || (c == '\0'))
        {
            if (tlen > 0U)
            {
                tok[tlen] = '\0';

                if (nstaged >= CMD_QUEUE_DEPTH) { return 0U; }

                staged[nstaged] = Cmd_ParseToken(tok);
                if (staged[nstaged].op == CMD_INVALID) { return 0U; }

                nstaged++;
                tlen = 0U;
            }

            if (c == '\0') { break; }
        }
        else
        {
            /* A token longer than the buffer cannot be valid, and letting it
             * run would overflow tok[]. */
            if (tlen >= (sizeof(tok) - 1U)) { return 0U; }

            tok[tlen] = c;
            tlen++;
        }

        i++;
        if (i >= CMD_LINE_MAX) { return 0U; }
    }

    /* An empty line is not a command. Replying OK to it would let a stray
     * newline look like a completed manoeuvre. */
    if (nstaged == 0U) { return 0U; }

    /* Room check before committing anything. */
    if ((uint16_t)s_count + (uint16_t)nstaged > (uint16_t)CMD_QUEUE_DEPTH)
    {
        return 0U;
    }

    for (j = 0U; j < nstaged; j++)
    {
        s_queue[(uint8_t)((s_head + s_count) % CMD_QUEUE_DEPTH)] = staged[j];
        s_count++;
    }

    return 1U;
}

/* ------------------------------------------------------------------ */
/* Queue                                                               */
/* ------------------------------------------------------------------ */

void Cmd_Init(void)
{
    s_head  = 0U;
    s_count = 0U;
}

Command_t Cmd_QueuePop(void)
{
    Command_t out;

    if (s_count == 0U)
    {
        out.op  = CMD_NONE;
        out.arg = 0;
        return out;
    }

    out    = s_queue[s_head];
    s_head = (uint8_t)((s_head + 1U) % CMD_QUEUE_DEPTH);
    s_count--;

    return out;
}

uint8_t Cmd_QueueCount(void) { return s_count; }

void Cmd_QueueFlush(void)
{
    s_head  = 0U;
    s_count = 0U;
}
