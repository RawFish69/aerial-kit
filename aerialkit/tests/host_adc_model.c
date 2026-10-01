#include "host_adc_model.h"

/*
 * The end-of-conversion flag, and nothing else: see host_adc_model.h.
 *
 * The model keeps one bit per controller. `host_f4adc_start()` sets it, which is
 * what the hardware would do a few microseconds later, and `host_f4adc_sr()`
 * hands it back on the port's next read of the status register. The bit stays
 * set until the port clears the status register for the next conversion, which
 * is the sequence the driver actually performs - and a test that wants the
 * timeout asks for it.
 */

#define MODEL_ADCS 4u

static uint32_t bases[MODEL_ADCS];
static int      done[MODEL_ADCS];
static int      never_finishes;

static unsigned index_of(uint32_t base)
{
    for (unsigned i = 0; i < MODEL_ADCS; i++) {
        if (bases[i] == base) {
            return i;
        }
        if (bases[i] == 0u) {
            bases[i] = base;
            return i;
        }
    }
    return 0u;
}

void host_f4adc_reset(void)
{
    for (unsigned i = 0; i < MODEL_ADCS; i++) {
        bases[i] = 0u;
        done[i] = 0;
    }
    never_finishes = 0;
}

void host_f4adc_set_never_finishes(int never)
{
    never_finishes = never;
}

void host_f4adc_start(uint32_t base)
{
    unsigned index = index_of(base);

    /* The previous conversion's flag goes with the clear, and this one is
     * finished unless the test has asked for a converter that never is. */
    done[index] = !never_finishes;
}

uint32_t host_f4adc_sr(uint32_t base, uint32_t mapped)
{
    unsigned index = index_of(base);

    /* The mapped value is what the port wrote a moment ago (a zero, or the
     * bits it cleared); the end-of-conversion bit is this model's. */
    return done[index] ? (mapped | 0x2u) : (mapped & ~0x2u);
}
