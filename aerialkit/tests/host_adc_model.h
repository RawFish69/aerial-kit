#ifndef AK_HOST_ADC_MODEL_H
#define AK_HOST_ADC_MODEL_H

#include <stdint.h>

/*
 * The end of a conversion, which a mapped register block cannot produce.
 *
 * `ak_adc_read_counts()` does three things: it clears the status register, it
 * starts a conversion, and it polls for the end-of-conversion flag before
 * reading the data register. Against a page of memory the first two work and
 * the third can never happen - a write to a register does not make a
 * conversion finish - so the *success* path of the flight pack's converter had
 * never run anywhere: every host test could only reach the timeout. That is
 * the same shape as the I2C, flash and SPI drivers, and the same answer: a
 * model, behind a seam on that one flag.
 *
 * Deliberately *only* that flag. Every other register stays the mapped block,
 * so the checks that read back what the port configured (the prescaler, the
 * sample time, the sequence) keep reading the real writes; what the model adds
 * is the part a peripheral does on its own, and the counts themselves are put
 * in the data register by the test, the way the hardware would.
 */

void host_f4adc_reset(void);

/* The conversion the port just asked for now has the end flag set - which is
 * what the hardware does and what this exists for. */
void host_f4adc_set_never_finishes(int never);

/* Called by the port when it clears the status register and starts a
 * conversion; `host_f4adc_sr()` is its read of the status register. */
void     host_f4adc_start(uint32_t base);
uint32_t host_f4adc_sr(uint32_t base, uint32_t mapped);

#endif /* AK_HOST_ADC_MODEL_H */
