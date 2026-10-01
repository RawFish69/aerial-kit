#ifndef AK_ARCH_STM32F405_I2C_TIMING_H
#define AK_ARCH_STM32F405_I2C_TIMING_H

#include <stdint.h>

#include "regs.h"

/*
 * The three numbers the F4's I2C wants before it can be turned on, as
 * arithmetic rather than as register writes, so a host test can hold them
 * against RM0090 27.6.8-27.6.10.
 *
 * They are worth that on their own: a wrong CCR does not fail, it runs the bus
 * at the wrong speed - too fast and the parts answer with corrupted bytes or
 * not at all, which reads exactly like a wiring problem - and a wrong FREQ
 * moves every timing derived from it. There is no error flag for "this bus is
 * at 700 kHz".
 *
 * The formulas, from RM0090:
 *
 *   FREQ (CR2[5:0])  the APB1 clock in MHz. The part takes 2..42.
 *
 *   CCR  (27.6.9)    the SCL period in APB1 clocks.
 *                    standard mode:   T = 2 * CCR * T_PCLK1
 *                    fast, DUTY = 0:  T = 3 * CCR * T_PCLK1   (Tlow = 2 Thigh)
 *                    fast, DUTY = 1:  T = 25 * CCR * T_PCLK1  (Tlow/Thigh = 16/9)
 *
 *   TRISE (27.6.10)  the maximum SCL rise time, in clocks, plus one: the
 *                    peripheral uses it to know how long the bus is allowed to
 *                    take to come up before it calls it stuck. The manual gives
 *                    1000 ns for standard mode and 300 ns for fast mode.
 *
 * regs.h comes in for the two mode bits, which are the top of the CCR answer
 * and belong next to the rest of the register definitions rather than in a
 * second copy here. Including it costs nothing: it is addresses and bit
 * positions, and the test that uses this header includes it too.
 */

/* The APB1 clock in MHz, clamped to what the peripheral accepts. A clock
 * slower than 2 MHz is not a case this firmware can be in - the slowest thing
 * it ever runs at is 16 MHz of HSI - but a number the part rejects is worse
 * than one it ignores, so it is clamped rather than passed through. */
static inline uint32_t ak_i2c_freq_range(uint32_t pclk1_hz)
{
    uint32_t mhz = pclk1_hz / 1000000u;

    if (mhz < 2u) {
        return 2u;
    }
    if (mhz > 42u) {
        return 42u;
    }
    return mhz;
}

/*
 * The SCL period, with the fast-mode flag and the duty bit already in place -
 * because which of the two modes this is *is* the top bit of the answer, and
 * splitting them would be two numbers that must not be used apart.
 *
 * `duty_two` picks Tlow = 2 * Thigh, which is what the reference
 * implementations use for fast mode: it is the cheaper of the two duties to
 * meet the timings with, and 400 kHz is the slowest thing on this bus.
 */
static inline uint32_t ak_i2c_ccr(uint32_t pclk1_hz, uint32_t speed_hz,
                                 int duty_two)
{
    if (speed_hz == 0u) {
        speed_hz = 100000u;
    }

    if (speed_hz <= 100000u) {
        uint32_t ccr = pclk1_hz / (speed_hz * 2u);

        /* RM0090 27.6.9: four clocks is the shortest period standard mode is
         * allowed to be asked for. */
        return ccr < 4u ? 4u : ccr;
    }

    uint32_t ccr = duty_two ? pclk1_hz / (speed_hz * 3u)
                            : pclk1_hz / (speed_hz * 25u);
    if (ccr < 1u) {
        ccr = 1u;
    }
    ccr |= I2C_CCR_FS;
    if (!duty_two) {
        ccr |= I2C_CCR_DUTY;
    }
    return ccr;
}

static inline uint32_t ak_i2c_trise(uint32_t pclk1_hz, int fast_mode)
{
    uint32_t mhz = ak_i2c_freq_range(pclk1_hz);

    /* 1000 ns in standard mode, 300 ns in fast mode, in APB1 clocks, plus the
     * one the peripheral wants on top. Integer arithmetic on purpose: the
     * rounding here is what the reference implementations do as well. */
    return fast_mode ? (mhz * 300u) / 1000u + 1u : mhz + 1u;
}

#endif /* AK_ARCH_STM32F405_I2C_TIMING_H */
