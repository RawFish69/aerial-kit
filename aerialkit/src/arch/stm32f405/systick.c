#include "arch.h"
#include "ak_time.h"

static volatile uint32_t tick_ms;

void SysTick_Handler(void)
{
    tick_ms++;
}

void ak_arch_time_init(void)
{
    SYSTICK_LOAD = (ak_clk_sysclk_hz() / 1000u) - 1u;
    SYSTICK_VAL  = 0;
    SYSTICK_CTRL = SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT |
                   SYSTICK_CTRL_ENABLE;
}

uint32_t ak_arch_time_ms(void)
{
    return tick_ms;
}

#ifdef AK_HOST_TICK
/* The host's clock, moved by the delay that is waiting on it - see the note in
 * src/core/time.c. Nothing else in the host build can advance this, which is
 * why the seam exists at all. */
void ak_host_tick_advance(uint32_t ms)
{
    tick_ms += ms;
}
#endif

/* The portable contract in core/time.h is satisfied here, by the port. */
void ak_time_init(void)
{
    ak_arch_time_init();
}

uint32_t ak_time_ms(void)
{
    return ak_arch_time_ms();
}
