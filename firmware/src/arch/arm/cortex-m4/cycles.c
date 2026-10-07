#include <stdint.h>

#include "arch.h"
#include "ak_time.h"

/*
 * The cycle counter, for both ARM ports.
 *
 * DWT is a Cortex-M4 *core* peripheral, not a vendor one: CYCCNT is at
 * 0xE0001004 on the STM32F405 and on the AT32F435 alike, which is why this
 * file lives beside `fault.c` in the shared directory rather than in either
 * port. The vendor headers are deliberately not used - the register addresses
 * are architectural, and neither port should need its part's header to compile
 * this.
 *
 * Enabling it takes two writes, and both are needed:
 *
 *   DEMCR.TRCENA    powers the debug/trace block. Without it DWT ignores
 *                   everything, and CYCCNT reads 0 for ever - which looks
 *                   exactly like a working counter on a loop that never
 *                   closes, so it is worth knowing that the failure is silent.
 *   DWT_CTRL.CYCCNTENA
 *                   starts the counter.
 *
 * **This does not require a debugger to be attached.** On the M4 the trace
 * block is clocked from the core, so the counter runs on a board alone on a
 * bench, which is the case that matters here. (It is also why the counter is
 * usable at all in a firmware that ships: nothing about it depends on a host
 * being connected.)
 */

#define DEMCR        AK_REG32(0xE000EDFCu)
#define DWT_CTRL     AK_REG32(0xE0001000u)
#define DWT_CYCCNT   AK_REG32(0xE0001004u)

#define DEMCR_TRCENA     (1u << 24)
#define DWT_CTRL_CYCCNTENA (1u << 0)

#ifdef AK_HOST_CYCLES
#include "host_cycles_model.h"
/*
 * The host has no DWT, and a page of memory cannot count cycles. The model
 * (tests/host_cycles_model.c) is a counter the *test* moves, which is the point:
 * a host test can then drive an exact number of cycles through the real
 * profiling code and check the microseconds it reports, instead of waiting for
 * real time to pass and asserting something loose about it.
 */
#define cycles_read()   host_cycles_now()
#define cycles_write(v) host_cycles_set(v)
#else
#define cycles_read()   (DWT_CYCCNT)
#define cycles_write(v) do { DWT_CYCCNT = (v); } while (0)
#endif

void ak_cycles_init(void)
{
#ifdef AK_HOST_CYCLES
    host_cycles_init();
#else
    DEMCR |= DEMCR_TRCENA;
    cycles_write(0u);
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
#endif
}

uint32_t ak_cycles(void)
{
    return cycles_read();
}

uint32_t ak_cycles_per_us(void)
{
#ifdef AK_HOST_CYCLES
    /* The modelled clock, for the same reason the counter is modelled: the
     * conversion is what ak_perf's arithmetic rests on, and a test has to be
     * able to choose the rate - including zero, which is the port saying it
     * cannot convert at all. */
    return host_cycles_per_us();
#else
    /* The core clock, which is also what DWT counts on both parts. Zero means
     * "this port cannot say", and ak_perf reports nothing rather than a rate it
     * made up. */
    return ak_clk_sysclk_hz() / 1000000u;
#endif
}
