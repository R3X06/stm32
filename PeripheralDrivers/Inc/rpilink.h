#ifndef __RPILINK_H
#define __RPILINK_H

#include "stm32f4xx_hal.h"
#include "commands.h"

/* ---------------------------------------------------------------------------
 * USART3 link to the Raspberry Pi, and the executor that turns queued
 * commands into motion primitives.
 *
 *   PD8 = TX, PD9 = RX, 115200 8N1, no flow control.
 *
 * PD8/PD9 and NOT PB10/PB11. USART3 is available on both, but PB10/PB11 are
 * the I2C2 bus the IMU sits on. Moving USART3 there takes the IMU off the
 * board and the failure looks like a dead sensor, not a pin clash.
 *
 * RECEIVE PATH
 *   One byte at a time under interrupt. Bytes accumulate into a line buffer
 *   in the ISR; on '\n' the line is copied into a second buffer and a flag is
 *   raised. Nothing is parsed in the ISR - RpiLink_Poll() does that from the
 *   main loop, where a blocking transmit is harmless.
 *
 *   Double-buffering matters: without it a line arriving while the previous
 *   one is still being parsed would overwrite it mid-parse.
 *
 * REPLY RULE
 *   Exactly one reply per LINE, never per token. "OK" goes out only once the
 *   last primitive on the line has finished moving. RST replies nothing at
 *   all - that is deliberate and the RPi side already expects it.
 *
 * TRANSMIT PATH
 *   Blocking, from the main loop. "OK\n" is three bytes, 260 us at 115200.
 *   Doing it under interrupt would buy nothing and cost a state machine.
 * ------------------------------------------------------------------------- */

/* Distance at which F0 gives up and stops, cm. Only has any effect once a
 * real Sensors_ObstacleAhead() replaces the weak stub. */
#define RPILINK_F0_STOP_CM      15U

/* How far F0 will run before the motion watchdog calls it a day, mm. */
#define RPILINK_F0_MAX_MM       20000

void RpiLink_Init(UART_HandleTypeDef *huart);

/* Call from the main loop as often as possible. Never blocks for long. */
void RpiLink_Poll(void);

/* Call from HAL_UART_RxCpltCallback() when the instance is USART3. */
void RpiLink_RxCallback(void);

/* ---------------------------------------------------------------------------
 * Plain text out on the same port, for telemetry and bring-up logging.
 * Main loop only.
 *
 * THIS GOES SILENT ONCE THE RPi HAS SPOKEN, and that is deliberate.
 *
 * USART3 is both the bench console and the command link - there is only one
 * port and the protocol owns it. While no host is connected the reports are
 * the most useful diagnostic on the robot, so they run freely. The moment a
 * valid command line parses off the wire, a real host is driving and anything
 * else on this port lands in the middle of the OK/RESEND stream it is parsing.
 * So the first parsed line latches the console off for good.
 *
 * Consequence worth knowing on the bench: connect the RPi and your terminal
 * output stops. That is not a fault. Reset the board to get it back, with the
 * host quiet.
 *
 * Protocol replies do NOT go through here - they use an internal path that is
 * never gated, because a suppressed OK hangs the host forever. */
void RpiLink_Log(const char *s);

/* 1 once the console has latched off. */
uint8_t RpiLink_IsQuiet(void);

/* ---------------------------------------------------------------------------
 * Photo handshake, used by the NAV mode in main.c. The STM is the one asking
 * here, which is the reverse of every other exchange on this link:
 *
 *   STM -> RPi :  SNAP,<n>\n       please take photo n
 *   RPi -> STM :  !SNAPOK<n>\n     photo n is taken  (STM replies OK)
 *
 * RpiLink_Send() is ungated - unlike RpiLink_Log() it still transmits after
 * the console has latched quiet. Main loop only. */
void     RpiLink_Send(const char *s);

/* Id from the last !SNAPOK<n>, 0 if none has arrived since reset. */
uint16_t RpiLink_GetSnapAckId(void);

/* Times reception had to be re-armed after the HAL tore it down on a line
 * error. Should be 0. Anything else means the link is glitching and being
 * silently recovered - go and look at the wiring before it bites in a run. */
uint32_t RpiLink_GetRearmCount(void);

/* 1 while a line is being executed. For the OLED. */
uint8_t RpiLink_IsBusy(void);

/* Last command the executor started. For the OLED. */
CmdOpcode_t RpiLink_LastOpcode(void);

#endif /* __RPILINK_H */
