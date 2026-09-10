#ifndef __IR_H
#define __IR_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Sharp GP2Y0A21YK analog IR distance sensors, one each side.
 *
 *   IR_L  PC0 -> ADC1_IN10
 *   IR_R  PC1 -> ADC1_IN11
 *
 * ADC1 runs free: scan mode over both channels, continuous conversion, DMA in
 * circular mode into a two-word buffer. Nothing has to start a conversion or
 * wait for one - the buffer always holds the latest pair and reading it is
 * just a memory access. That matters because these are polled from the 10 ms
 * control tick.
 *
 * Sampling time is 480 cycles at 10.5 MHz ADC clock, about 47 us per channel.
 * Long, deliberately: the Sharp's output is a lightly filtered analog line and
 * the long aperture is free noise rejection.
 *
 * ---------------------------------------------------------------------------
 * THE RESPONSE CURVE IS NOT LINEAR, AND IT IS NOT MONOTONIC
 *
 * Output voltage rises as an object gets closer, but only down to about 10 cm.
 * Closer than that the voltage FALLS again, so a reading of 1.4 V means
 * "either 25 cm or 4 cm" and there is no way to tell which from one sample.
 *
 * This is the single most common way to get burned by these sensors: the
 * robot creeps up on a wall, passes 10 cm, the reported distance starts
 * INCREASING, and the control loop drives it straight into the wall while
 * believing it has room. IR_MIN_VALID_CM exists to make that region report
 * SENSOR_NO_READING instead of a plausible lie. Mount the sensors so nothing
 * can get inside 10 cm of them, and treat a dropout as "too close", never as
 * "clear".
 *
 * Above ~80 cm the output flattens into the noise floor and distance cannot be
 * resolved at all.
 *
 * The default fit below is the datasheet-typical power law. Every unit is
 * different by a few percent - see the calibration note in ir.c.
 * ------------------------------------------------------------------------- */

/* Usable span, cm. Outside this the reading is reported as no reading. */
#define IR_MIN_VALID_CM         10U
#define IR_MAX_VALID_CM         80U

/* Median filter depth, in samples taken one per control tick. 5 ticks is
 * 50 ms of lag, which is nothing next to how much these sensors twitch. */
#define IR_MEDIAN_N             5U

/* ADC reference and full scale. VREF+ is tied to VDDA on this board. */
#define IR_ADC_VREF             3.3f
#define IR_ADC_FULL_SCALE       4095.0f

/* Potential divider between the sensor output and the ADC pin, if fitted.
 *
 *      IR_DIVIDER_RATIO = V_at_pin / V_at_sensor
 *
 * 1.0 means the sensor output goes straight to PC0/PC1. The kit's 1k and
 * 2.2k resistors, wired sensor -> 1k -> pin -> 2.2k -> ground, give
 * 2.2/(1.0+2.2) = 0.6875.
 *
 * *** DECIDE WHICH YOU HAVE AND SET THIS BEFORE CALIBRATING. ***
 *
 * It matters more than it looks. The fit below converts pin volts to
 * centimetres through a power law with an exponent of -1.15, so an unnoticed
 * divider does not shift the readings by 31% - it shifts them by
 * 0.6875^-1.15, about 55%, and it does so smoothly across the whole range.
 * The numbers stay plausible the entire time, which is exactly what makes it
 * hard to spot.
 *
 * A note on which is correct, checked against DS8626 Rev 12:
 *
 * Table 7 lists PC0 and PC1 as "I/O  FT (5)", and footnote 5 reads:
 *
 *     "FT = 5 V tolerant except when in analog mode or oscillator mode"
 *
 * So they ARE 5 V tolerant as digital pins - but the tolerance is explicitly
 * switched off in analog mode, which is exactly the mode the ADC uses them
 * in. In this design they must stay inside VREF+ (tied to VDDA = 3.3 V here
 * by R4/R33 on the C30D schematic).
 *
 * The GP2Y0A21YK is a 5 V part whose output peaks near 3.1 V at close range,
 * so it does fit under 3.3 V unaided - with very little margin, and none at
 * all if VDDA sags or the sensor runs a little hot.
 *
 * Table 47 is the stronger argument for fitting the divider: PC0 and PC1 are
 * on the list of pins where the allowed negative injected current is 0 mA,
 * "NA" for positive. A 5 V sensor driving an unpowered MCU - which is what
 * happens whenever the sensor rail comes up first - injects current through
 * the pin protection. The 1 k series leg limits it; a direct wire does not.
 *
 * PC7, where the ultrasonic echo lands, is FT and stays FT because it is used
 * as a digital AF input, not an analog one. It genuinely does not need the
 * divider. This one does.
 *
 * Measure the sensor output with a multimeter at a known distance and compare
 * against the raw counts in IRCAL mode. Do not assume. */
#define IR_DIVIDER_RATIO        1.0f

/* Power-law fit: distance_cm = IR_FIT_A * volts ^ IR_FIT_B
 * Datasheet-typical for the GP2Y0A21YK over 10-80 cm. */
#define IR_FIT_A                27.86f
#define IR_FIT_B                (-1.15f)

void IR_Init(ADC_HandleTypeDef *hadc);

/* One sample into the median filters. Call once per control tick. Cheap -
 * it only reads the DMA buffer, no conversion is started or waited on. */
void IR_Update(void);

/* Filtered distance in cm, or SENSOR_NO_READING when out of range. */
uint16_t IR_LeftCm(void);
uint16_t IR_RightCm(void);

/* Raw ADC counts, for bring-up. If these do not move when you wave your hand
 * at the sensor, the problem is wiring or the ADC, not the curve fit. */
uint16_t IR_LeftRaw(void);
uint16_t IR_RightRaw(void);

#endif /* __IR_H */
