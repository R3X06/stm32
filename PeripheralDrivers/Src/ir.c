#include "ir.h"
#include "commands.h"
#include <math.h>

/* DMA target. Circular, two half-words, rank order from the .ioc:
 *   [0] = ADC_CHANNEL_10 = PC0 = IR_L
 *   [1] = ADC_CHANNEL_11 = PC1 = IR_R
 *
 * volatile because the DMA writes it behind the compiler's back. Without it
 * the reads below can be hoisted out of the tick and the values never change. */
static volatile uint16_t s_dma[2];

static ADC_HandleTypeDef *s_adc;

static uint16_t s_bufL[IR_MEDIAN_N];
static uint16_t s_bufR[IR_MEDIAN_N];
static uint8_t  s_idx;
static uint8_t  s_fill;

static uint16_t s_cmL = SENSOR_NO_READING;
static uint16_t s_cmR = SENSOR_NO_READING;

/* ---------------------------------------------------------------------------
 * CALIBRATING A UNIT
 *
 * The fit in ir.h is the datasheet typical. To do better, tape a target at
 * 10, 15, 20, 30, 40, 50, 60 and 80 cm in turn, note IR_LeftRaw() at each,
 * and fit your own curve. The classic trick is that 1/(d + 0.42) is very
 * nearly linear in output voltage, so plotting raw counts against
 * 1/(d + 0.42) should give a straight line - if it does not, the sensor is
 * seeing something other than your target.
 *
 * Do the calibration with the sensor mounted on the robot, not on the bench.
 * These are sensitive to what is behind and beside the target, and a sensor
 * that is calibrated in free air reads differently once it is 30 mm above a
 * reflective floor.
 * ------------------------------------------------------------------------- */

void IR_Init(ADC_HandleTypeDef *hadc)
{
    uint8_t i;

    s_adc  = hadc;
    s_idx  = 0U;
    s_fill = 0U;
    s_cmL  = SENSOR_NO_READING;
    s_cmR  = SENSOR_NO_READING;

    for (i = 0U; i < IR_MEDIAN_N; i++)
    {
        s_bufL[i] = 0U;
        s_bufR[i] = 0U;
    }

    s_dma[0] = 0U;
    s_dma[1] = 0U;

    /* Cast away volatile for the HAL call only. The buffer genuinely is
     * volatile for our own reads; HAL just wants the address. */
    (void)HAL_ADC_Start_DMA(s_adc, (uint32_t *)(void *)s_dma, 2U);

    /* Mask off the DMA half- and full-transfer interrupts.
     *
     * HAL_ADC_Start_DMA arms them unconditionally, and with a two-entry
     * circular buffer they fire once each per conversion pass. The ADC clock
     * is PCLK2/8 = 10.5 MHz and a 480-cycle sample plus 12 conversion cycles
     * is 492 cycles per channel, so a two-channel pass takes about 94 us:
     *
     *      2 interrupts / 94 us  =  roughly 21000 per second
     *
     * all at NVIC priority 6, which is the SAME priority as the TIM6 control
     * tick. They cannot pre-empt the tick, but the tick cannot pre-empt them
     * either, so every one of them is a chance to delay the control loop. And
     * they achieve nothing: the DMA is circular, s_dma[] is always current,
     * and IR_Update() reads it on its own schedule. Nothing needs telling
     * when a conversion lands.
     *
     * The transfer-error interrupt is deliberately left enabled. */
    __HAL_DMA_DISABLE_IT(s_adc->DMA_Handle, DMA_IT_HT | DMA_IT_TC);
}

/* Insertion sort a copy and take the middle. N is 5, so this is a handful of
 * compares - not worth anything cleverer. */
static uint16_t median_n(const uint16_t *src, uint8_t n)
{
    uint16_t tmp[IR_MEDIAN_N];
    uint8_t  i, j;
    uint16_t key;

    for (i = 0U; i < n; i++) { tmp[i] = src[i]; }

    for (i = 1U; i < n; i++)
    {
        key = tmp[i];
        j   = i;
        while ((j > 0U) && (tmp[j - 1U] > key))
        {
            tmp[j] = tmp[j - 1U];
            j--;
        }
        tmp[j] = key;
    }

    return tmp[n / 2U];
}

/* Raw counts -> cm, or SENSOR_NO_READING outside the trustworthy span. */
static uint16_t raw_to_cm(uint16_t raw)
{
    float volts;
    float cm;

    /* Back out the divider first, so 'volts' is always the voltage the SENSOR
     * produced, which is what the datasheet curve is defined against. */
    volts = ((float)raw / IR_ADC_FULL_SCALE) * IR_ADC_VREF / IR_DIVIDER_RATIO;

    /* Below about 0.3 V the curve is flat and the sensor is telling us
     * nothing except "further than I can see". */
    if (volts < 0.30f) { return SENSOR_NO_READING; }

    cm = IR_FIT_A * powf(volts, IR_FIT_B);

    /* The near-field ambiguity. Anything the fit puts under IR_MIN_VALID_CM
     * could equally be an object much closer than that, on the far side of
     * the response peak. Refuse to guess. */
    if (cm < (float)IR_MIN_VALID_CM) { return SENSOR_NO_READING; }
    if (cm > (float)IR_MAX_VALID_CM) { return SENSOR_NO_READING; }

    return (uint16_t)(cm + 0.5f);
}

void IR_Update(void)
{
    s_bufL[s_idx] = s_dma[0];
    s_bufR[s_idx] = s_dma[1];

    s_idx = (uint8_t)((s_idx + 1U) % IR_MEDIAN_N);
    if (s_fill < IR_MEDIAN_N) { s_fill++; }

    /* Median the RAW counts, then convert once.
     *
     * Order matters. Converting first and medianing the centimetres would
     * throw away every sample the fit rejected, so a burst of out-of-range
     * readings would leave the filter holding an old value with no way to
     * tell it had gone stale. Filtering in the raw domain keeps the rejection
     * decision on the final, filtered number. */
    if (s_fill >= IR_MEDIAN_N)
    {
        s_cmL = raw_to_cm(median_n(s_bufL, IR_MEDIAN_N));
        s_cmR = raw_to_cm(median_n(s_bufR, IR_MEDIAN_N));
    }
}

uint16_t IR_LeftCm(void)   { return s_cmL; }
uint16_t IR_RightCm(void)  { return s_cmR; }
uint16_t IR_LeftRaw(void)  { return s_dma[0]; }
uint16_t IR_RightRaw(void) { return s_dma[1]; }
