/*
 * The I2C timing arithmetic.
 *
 * This is the part of an I2C driver that can be checked without a bus, and it
 * is also the part that goes wrong quietly. A wrong CCR does not produce an
 * error: it produces a bus running at the wrong speed, which the parts answer
 * with corrupted bytes or with nothing, and which reads exactly like a wiring
 * problem. There is no flag for "this bus is at 700 kHz".
 *
 * The formulas are RM0090 27.6.8-27.6.10 and the numbers below were worked out
 * separately from them - by hand, at the figures in the manual - rather than by
 * running this code and writing down what it said. The three clocks are the
 * three this board can be at: 42 MHz APB1 with the crystal, 42 MHz is also
 * what the wing's AT32 will look like, and 16 MHz is the HSI fallback the
 * clock code falls back to when the crystal does not start.
 */

#include <stdint.h>

#include "../src/arch/stm32f405/i2c_timing.h"
#include "tests.h"

#define PCLK_42MHZ 42000000u
#define PCLK_16MHZ 16000000u

void test_i2c_timing(void)
{
    /* FREQ is the APB1 clock in whole MHz, and the part refuses to run below
     * 2 or above 42. */
    expect("42 MHz is 42 in the frequency register",
           ak_i2c_freq_range(PCLK_42MHZ) == 42u);
    expect("16 MHz is 16", ak_i2c_freq_range(PCLK_16MHZ) == 16u);
    expect("and a clock outside the part's range is clamped, not passed on",
           ak_i2c_freq_range(1000000u) == 2u &&
               ak_i2c_freq_range(168000000u) == 42u);

    /*
     * Standard mode: T = 2 * CCR * T_PCLK1, so CCR = fPCLK1 / (2 * fspeed).
     *
     *   42 MHz / (2 * 100 kHz) = 210
     *   16 MHz / (2 * 100 kHz) =  80
     */
    expect("100 kHz at 42 MHz is CCR 210",
           ak_i2c_ccr(PCLK_42MHZ, 100000u, 1) == 210u);
    expect("100 kHz at 16 MHz is CCR 80",
           ak_i2c_ccr(PCLK_16MHZ, 100000u, 1) == 80u);
    expect("and standard mode does not set the fast-mode bit",
           (ak_i2c_ccr(PCLK_42MHZ, 100000u, 1) & I2C_CCR_FS) == 0u);

    /*
     * Fast mode, Tlow = 2 * Thigh: T = 3 * CCR * T_PCLK1.
     *
     *   42 MHz / (3 * 400 kHz) = 35
     *   16 MHz / (3 * 400 kHz) = 13 (13.33, truncated - the reference
     *                                implementations truncate here too, which
     *                                errs on the slow side)
     */
    expect("400 kHz at 42 MHz is CCR 35 plus the fast-mode bit",
           ak_i2c_ccr(PCLK_42MHZ, 400000u, 1) == (35u | I2C_CCR_FS));
    expect("400 kHz at 16 MHz is CCR 13",
           ak_i2c_ccr(PCLK_16MHZ, 400000u, 1) == (13u | I2C_CCR_FS));
    expect("and the duty bit stays clear for the 2:1 split",
           (ak_i2c_ccr(PCLK_42MHZ, 400000u, 1) & I2C_CCR_DUTY) == 0u);

    /* The other duty, 16:9, is 25 clocks a period: 42 MHz / (25 * 400 kHz) is
     * 4.2, which truncates to 4 - and that is why the reference
     * implementations do not use this duty at 400 kHz. */
    expect("the 16:9 duty is a different, shorter period",
           ak_i2c_ccr(PCLK_42MHZ, 400000u, 0) ==
               (4u | I2C_CCR_FS | I2C_CCR_DUTY));

    /* A period of zero is not a bus, so both modes clamp - and the standard
     * one clamps at the four the manual asks for. No clock this board has gets
     * near either floor (100 kHz at the slowest clock it can be at, 16 MHz of
     * HSI, is 80), which is exactly why the clamp needs a test rather than a
     * bench session: nobody would ever see it happen. */
    expect("a standard-mode period is never below the manual's minimum of 4",
           ak_i2c_ccr(200000u, 100000u, 1) == 4u);
    expect("and a fast-mode period is never zero",
           ak_i2c_ccr(1000000u, 1000000u, 1) == (1u | I2C_CCR_FS));
    expect("a speed of zero is treated as the slow one, not a division by it",
           ak_i2c_ccr(PCLK_42MHZ, 0u, 1) == 210u);

    /*
     * TRISE is the maximum rise time in clocks, plus one: 1000 ns for standard
     * mode and 300 ns for fast. At 42 MHz that is 42 + 1 and (42 * 300)/1000 +
     * 1 = 13; at 16 MHz, 17 and (16*300)/1000 + 1 = 5.
     */
    expect("standard-mode rise time at 42 MHz is 43",
           ak_i2c_trise(PCLK_42MHZ, 0) == 43u);
    expect("standard-mode rise time at 16 MHz is 17",
           ak_i2c_trise(PCLK_16MHZ, 0) == 17u);
    expect("fast-mode rise time at 42 MHz is 13",
           ak_i2c_trise(PCLK_42MHZ, 1) == 13u);
    expect("fast-mode rise time at 16 MHz is 5",
           ak_i2c_trise(PCLK_16MHZ, 1) == 5u);
    expect("and fast mode always allows less rise than standard",
           ak_i2c_trise(PCLK_42MHZ, 1) < ak_i2c_trise(PCLK_42MHZ, 0));

    /*
     * The mode bits this file leans on are the ones regs.h defines - a second
     * copy of a bit position is how two copies drift apart, which is why the
     * header includes the port's own definitions instead of repeating them.
     */
    expect("the fast-mode bit is bit 15 and the duty bit is 14",
           I2C_CCR_FS == (1u << 15) && I2C_CCR_DUTY == (1u << 14));
}
