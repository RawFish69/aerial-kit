#include "arch.h"

/*
 * The ADC, for the one slow thing on this board that is not a sensor: the
 * flight pack, through a divider.
 *
 * One channel, one conversion at a time, polled. That is a deliberate choice
 * rather than a first version: the reading is wanted ten times a second, the
 * hardware has a DMA controller and an interrupt and neither would buy
 * anything except a second path for the same number. Everything here is
 * synchronous, so a caller knows that when it gets a count back, that count is
 * the latest one - which is the property the rest of the firmware wants.
 *
 * Nothing here has been run against silicon yet. The register work is checked
 * against RM0090 section 11 in tests/test_regs.c, where each bit position is
 * asserted on the host, and that is the only part of this file a bench cannot
 * check - a wrong bit in an ADC register does not fail, it converts nothing, or
 * converts the wrong pin, quietly.
 */

void ak_adc_init(uint32_t adc, uint32_t channel)
{
    if (adc == ADC1_BASE) {
        RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
        (void)RCC_APB2ENR;
    } else if (adc == ADC2_BASE) {
        RCC_APB2ENR |= (1u << 9);
        (void)RCC_APB2ENR;
    } else if (adc == ADC3_BASE) {
        RCC_APB2ENR |= (1u << 10);
        (void)RCC_APB2ENR;
    } else {
        return;
    }

    /* The prescaler belongs to all three ADCs and lives in the common block.
     * Without it the ADC has a clock of 84 MHz, which is more than twice what
     * the part is rated for - and the way that fails is not an error flag, it
     * is conversions that come back wrong. */
    ADC_CCR = (ADC_CCR & ~(0x3u << 16)) | ADC_CCR_ADCPRE_DIV4;

    ADC_CR1(adc) = ADC_CR1_RES_12BIT;
    /* End of conversion on each conversion rather than at the end of the
     * sequence: with a sequence of one the two are the same thing today, and
     * they stop being the same the moment somebody adds a channel. */
    ADC_CR2(adc) = ADC_CR2_EOCS;

    /* One conversion in the regular sequence: L is length minus one, and the
     * first (and only) entry is the channel this board measures. */
    ADC_SQR1(adc) = ADC_SQR1_L(1u);
    ADC_SQR3(adc) = ADC_SQR3_SQ1(channel);

    if (channel < 10u) {
        ADC_SMPR2(adc) = ADC_SMPR2_SMP(channel);
    } else {
        ADC_SMPR1(adc) = ADC_SMPR1_SMP(channel);
    }

    ADC_CR2(adc) |= ADC_CR2_ADON;

    /*
     * The first conversion after the ADC is switched on is not a measurement.
     * The part needs a few microseconds to settle, and until it has, a
     * conversion started with SWSTART returns whatever the sample capacitor
     * was holding. So the first one is thrown away here, once, rather than
     * left for the caller to discover as a plausible-looking zero on the first
     * battery reading of every boot.
     */
    uint16_t discarded = 0u;
    (void)ak_adc_read_counts(adc, &discarded);
}

#ifdef AK_HOST_ADC_F4
#include "host_adc_model.h"
/*
 * The end-of-conversion flag comes from the model (tests/host_adc_model.c) and
 * every other register is the mapped block: the model exists because a page of
 * memory cannot finish a conversion, and taking anything else out of the block
 * would cost the checks that read back the port's own configuration.
 */
#define adc_sr(base)         host_f4adc_sr((base), ADC_SR(base))
#define adc_sr_clear(base)   do { ADC_SR(base) = 0u; host_f4adc_start(base); } while (0)
#else
#define adc_sr(base)         ADC_SR(base)
#define adc_sr_clear(base)   do { ADC_SR(base) = 0u; } while (0)
#endif

int ak_adc_read_counts(uint32_t adc, uint16_t *counts)
{
    if (adc != ADC1_BASE && adc != ADC2_BASE && adc != ADC3_BASE) {
        return -1;
    }

    /* Writing one to EOC clears it; the flag is read and cleared this way
     * rather than by a read of DR, because that would hide the case where the
     * conversion is still running. */
    adc_sr_clear(adc);
    ADC_CR2(adc) |= ADC_CR2_SWSTART;

    /* 492 ADC cycles is 23 microseconds at 21 MHz, and the guard is four
     * hundred times that - so a conversion that has not finished by the time
     * the loop gives up is not slow, it is not happening, and saying so is
     * more useful than returning the last value in the data register. */
    uint32_t guard = 200000u;
    while ((adc_sr(adc) & ADC_SR_EOC) == 0u) {
        if (guard-- == 0u) {
            return -1;
        }
    }

    *counts = (uint16_t)(ADC_DR(adc) & 0xFFFFu);
    return 0;
}
