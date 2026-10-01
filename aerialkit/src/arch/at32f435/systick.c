#include "arch.h"

#include "ak_time.h"

/*
 * The tick, which is the one thing on this part that is not a port at all:
 * SysTick is the Cortex-M4's own, so this file is the F405's with this part's
 * clock behind it - and the clock is the interesting half, because the reload
 * is a millisecond of whatever the clock code ended up on. A board that fell
 * back to the internal 8 MHz clock ticks at the right rate too, which is the
 * only reason that fallback is worth having: a board with a dead crystal can
 * still tell you about it, and the console still reads in milliseconds.
 */

static volatile uint32_t tick_ms;

void SysTick_Handler(void)
{
    tick_ms++;
}

void ak_arch_time_init(void)
{
    SYSTICK_LOAD = (ak_clk_sysclk_hz() / 1000u) - 1u;
    SYSTICK_VAL  = 0u;
    SYSTICK_CTRL = AK_SYSTICK_CTRL_CLKSOURCE | AK_SYSTICK_CTRL_TICKINT |
                   AK_SYSTICK_CTRL_ENABLE;
}

uint32_t ak_arch_time_ms(void)
{
    return tick_ms;
}

/* The portable contract in core/time.h is satisfied here, by the port. */
void ak_time_init(void)
{
    ak_arch_time_init();
}

uint32_t ak_time_ms(void)
{
    return ak_arch_time_ms();
}
