#include "arch.h"

/*
 * The flight pack's divider, through this part's ADC.
 *
 * One channel, one conversion at a time, polled - the same shape as the F405
 * port, and the same three things that are easy to get wrong:
 *
 *   - **the clock**. The prescaler is in the common block and divides the AHB
 *     clock, not the APB one, so the value to ask for changes with the core's
 *     speed: 288 MHz divided by eight is the 36 MHz this part is rated for. A
 *     divider that is wrong does not raise a flag, it returns numbers that are
 *     wrong in a way that looks like a pack.
 *   - **the calibration**. This part, like the F4, starts with an uncalibrated
 *     converter; the sequence is reset, then calibrate, and both are waited for
 *     with a bound rather than for ever.
 *   - **the first conversion**, which is not a measurement: the sample capacitor
 *     has not settled, and the number that comes back looks plausible. The F405
 *     port learned that the expensive way - a zero on the first battery reading
 *     of every boot - and this one throws the first conversion away for the same
 *     reason.
 */

#define AK_ADC_GUARD 200000u

void ak_adc_init(uint32_t adc, uint32_t channel)
{
    if (adc != ADC1_BASE) {
        return;
    }

    CRM_APB2EN |= 1u << 8; /* CRM_ADC1_PERIPH_CLOCK = MAKE_VALUE(0x44, 8) */

    /*
     * The clock first, before anything else in the peripheral is touched: the
     * value is divided the part's way, from the AHB clock, and the field is
     * replaced rather than or-ed - the reset value divides by two, which at 288
     * MHz is four times what the converter is rated for.
     */
    AK_ADCCOM_CTRL = (AK_ADCCOM_CTRL &
                      ~(AK_ADCCOM_CTRL_DIV_MASK << AK_ADCCOM_CTRL_DIV_SHIFT)) |
                     ((uint32_t)AK_ADC_HCLK_DIV_8
                      << AK_ADCCOM_CTRL_DIV_SHIFT);

    /* One conversion in the ordinary sequence, on the channel the board
     * measures. Scan mode stays off: one channel is not a sequence to walk. */
    AK_ADC_CTRL1(adc) = 1u << AK_ADC_CTRL1_OCPCNT_SHIFT;
    AK_ADC_OSQ3(adc) = channel & 0x1Fu;

    /* The longest sample time for that channel. 36 MHz of ADC clock into a
     * divider's source impedance wants as much as it can have. */
    if (channel < 10u) {
        AK_ADC_SPT1(adc) = AK_ADC_SPT_LONGEST << AK_ADC_SPT_SHIFT(channel);
    } else {
        AK_ADC_SPT2(adc) = AK_ADC_SPT_LONGEST << AK_ADC_SPT_SHIFT(channel);
    }

    AK_ADC_CTRL2(adc) |= AK_ADC_CTRL2_ADCEN;

    /* Calibration: reset, wait, calibrate, wait. The part clears each bit when
     * it is done, and a converter that never clears one is a converter that is
     * not running - so the waits are bounded and the function carries on to the
     * throwaway conversion either way. */
    AK_ADC_CTRL2(adc) |= AK_ADC_CTRL2_ADCALINIT;
    for (uint32_t guard = AK_ADC_GUARD;
         (AK_ADC_CTRL2(adc) & AK_ADC_CTRL2_ADCALINIT) != 0u && guard-- > 0u;) {
    }
    AK_ADC_CTRL2(adc) |= AK_ADC_CTRL2_ADCAL;
    for (uint32_t guard = AK_ADC_GUARD;
         (AK_ADC_CTRL2(adc) & AK_ADC_CTRL2_ADCAL) != 0u && guard-- > 0u;) {
    }

    uint16_t discarded = 0u;
    (void)ak_adc_read_counts(adc, &discarded);
}

#ifdef AK_HOST_ADC_AT32
#include "host_adc_model.h"
/*
 * The end-of-conversion flag comes from the model (tests/host_adc_model.c) and
 * every other register is the mapped block - see the F405 port, which has the
 * same seam and the same reason: a page of memory cannot finish a conversion.
 * This part clears its flags by writing a *zero*, so the clear below is the
 * complement; what the model needs to know is only that one has been asked for.
 */
#define adc_sts(base)       host_f4adc_sr((base), AK_ADC_STS(base))
#define adc_sts_clear(base) do { AK_ADC_STS(base) = ~AK_ADC_STS_OCCE; \
                                 host_f4adc_start(base); } while (0)
#else
#define adc_sts(base)       AK_ADC_STS(base)
#define adc_sts_clear(base) do { AK_ADC_STS(base) = ~AK_ADC_STS_OCCE; } while (0)
#endif

int ak_adc_read_counts(uint32_t adc, uint16_t *counts)
{
    if (adc != ADC1_BASE || counts == 0) {
        return -1;
    }

    /*
     * The flag is cleared before the conversion starts, and **this part's
     * status bits are cleared by writing a zero to them** - the opposite of the
     * flash controller's flags two files over, which clear by writing a one.
     * Artery's own `adc_flag_clear()` writes the complement of the flag for
     * exactly this reason. Writing a one here would leave a stale end-of-
     * conversion set, and the next read would answer with the previous
     * conversion's value: a battery voltage that is one measurement old, which
     * is the kind of wrong number nothing complains about.
     */
    adc_sts_clear(adc);
    AK_ADC_CTRL2(adc) |= AK_ADC_CTRL2_ADCSWTRG;

    uint32_t guard = AK_ADC_GUARD;

    while ((adc_sts(adc) & AK_ADC_STS_OCCE) == 0u) {
        if (guard-- == 0u) {
            return -1; /* not slow: not happening, and that is worth saying */
        }
    }

    *counts = (uint16_t)(AK_ADC_ODT(adc) & 0xFFFFu);
    return 0;
}
