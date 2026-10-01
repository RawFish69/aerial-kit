#include "arch.h"

/*
 * Clock tree: the board's crystal -> PLL -> 288 MHz SYSCLK, AHB 288 MHz, APB1
 * and APB2 144 MHz.
 *
 *   the PLL's own arithmetic, in Artery's terms: (HEXT / ms) * ns / fr. On the
 *   wing's board, which has an 8 MHz can, that is (8 / 1) * 72 / 2 = 288 MHz -
 *   and those three numbers are now *derived* from the crystal the board
 *   declares (ak_at32_pll_cfg_for below) rather than written down beside it.
 *
 * Three things about it are this part's rather than the F4's, and all three are
 * the kind of difference that makes a port look done and not be:
 *
 *   - the PLL is ms/ns/fr, not M/N/P/Q, and fr is written as an exponent (1
 *     means divide by two);
 *   - the core's regulator has to be at 1.3 V before the PLL runs at 288 MHz,
 *     which is a register Artery's own sequence writes first;
 *   - the flash controller has its own clock divider, and it has to be changed
 *     *before* the core speeds up, not after.
 *
 * **Why the crystal is a declaration here and a measurement on the F405.**
 * The F405 port measures its crystal at boot, because that board's can is
 * 12 MHz where the code assumed 8 and the difference reached USB as nine bus
 * resets and not one decoded frame. The F405 can do that because ST routed
 * HSE/RTCPRE into TIM11's capture input (TIM11_OR's TI1_RMP = 2) while the
 * core runs on the 16 MHz internal RC, so the crystal is timed against a
 * clock that did not come from it.
 *
 * This part has no such route. Every internal timer input the AT32F435 has is
 * in `at32f435_437_tmr.h`'s `tmr_input_remap_type` - TMR2's channel 1 can take
 * TMR8's trigger output, the Ethernet PTP clock, or either OTG's SOF, and
 * TMR5's channel 4 can take LICK, LEXT or the ERTC - and none of them is HEXT.
 * The clock unit's trims are not the crystal's either: `CRM_CTRL` carries
 * `hicktrim` and `hickcal` for the *internal* RC, whose reference is the
 * crystal - the opposite direction, and it cannot tell you what the crystal is.
 * Its only crystal-side status is a failure flag, `CRM_CLOCK_FAILURE_INT`,
 * which says the can stopped and not what it is. And the high-speed clocks a
 * crystal could be counted against all come from the crystal itself. So a
 * boot-time measurement is not a thing this port is missing; it is a thing this
 * part does not have, and what is left is for the *board* to declare its can
 * and for the PLL to be built from that declaration.
 *
 * That is a weaker guarantee than the F405's and it is written down as one: a
 * board that declares 8 MHz and has 12 is still wrong here. What it is not is
 * a silent 1.5x, because the PLL's three numbers follow the declaration
 * instead of being three literals that assumed one. The nearest thing to a
 * measurement this part offers is TMR2 channel 1 on an OTG SOF - a 1 kHz host
 * frame, +/-500 ppm - and it needs USB up, so it can check the crystal the PLL
 * was built on but cannot choose it. The traps file carries the whole reading.
 *
 * If the crystal does not come up, or comes up as one this part cannot turn
 * into the target exactly, the firmware stays on the internal 8 MHz clock and
 * says so, rather than sitting in a wait loop in the dark. A board with a bad
 * crystal should still be able to tell you about it - that is the same rule
 * the F405 port follows, and the preflight prints which clock is running
 * either way.
 */

#define AK_CLOCK_TIMEOUT 1000000u

/* What the core is asked to run at, and the ranges Artery documents for the
 * PLL (at32f435_437_crm.c, the comment above `crm_pll_config`):
 *
 *   1 <= ms <= 15       2 MHz <= HEXT/ms <= 16 MHz
 *   31 <= ns <= 500     500 MHz <= (HEXT/ms)*ns <= 1000 MHz
 *
 * with fr an exponent, 1 to 5, for a divisor of 2, 4, 8, 16 or 32.
 *
 * The target is not a preference. This part's USB clock is the PLL divided by
 * six (regs.h, CRM_MISC1_HICK_TO_USB and CRM_USBDIV_6), so 48 MHz - the only
 * rate a full-speed device works at - is 288 MHz of PLL and nothing else. A
 * crystal this part cannot multiply to exactly 288 is a crystal it cannot run
 * USB from, which is why the search below is for an exact hit and gives up
 * rather than rounding. */
#define AK_CLK_TARGET_HZ 288000000u
#define AK_CLK_VCO_MIN_HZ 500000000u
#define AK_CLK_VCO_MAX_HZ 1000000000u
#define AK_CLK_REF_MIN_HZ 2000000u
#define AK_CLK_REF_MAX_HZ 16000000u
#define AK_CLK_MS_MAX 15u
#define AK_CLK_NS_MIN 31u
#define AK_CLK_NS_MAX 500u
#define AK_CLK_FR_MAX 5u /* the highest exponent the part defines */

static uint32_t sysclk_hz = 8000000u; /* the internal clock, out of reset */
static uint32_t apb1_hz   = 8000000u;
static uint32_t apb2_hz   = 8000000u;
static uint32_t hse_hz; /* the crystal the PLL was built on; 0 if none was */
static int      hse_ok;

/*
 * The PLL's three numbers for a crystal and a target: the smallest output
 * divisor whose VCO lands in range, then the smallest input divider whose
 * reference does, with ns required to come out whole. 0 when this part cannot
 * hit the target exactly from that crystal - which for the wing's 8 MHz is
 * (ms 1, ns 72, fr 1) -> 576 MHz -> 288 MHz, the numbers this file used to
 * carry as literals.
 *
 * Exported so the host can put crystals through it that no board here has: 12
 * and 16 MHz land on 48 and 36, 25 MHz lands on nothing (no divisor of 25 MHz
 * in 2..16 MHz multiplies to 576 exactly), and each of those is a case the
 * arithmetic has to get right before a board with that can is believed.
 */
uint32_t ak_at32_pll_cfg_for(uint32_t hext_hz, uint32_t target_hz,
                            uint32_t *ms_out, uint32_t *ns_out,
                            uint32_t *fr_out)
{
    if (hext_hz == 0u || target_hz == 0u) {
        return 0u;
    }

    for (uint32_t fr = 0u; fr <= AK_CLK_FR_MAX; fr++) {
        uint64_t vco = (uint64_t)target_hz * (uint64_t)(1u << fr);

        if (vco < AK_CLK_VCO_MIN_HZ || vco > AK_CLK_VCO_MAX_HZ) {
            continue;
        }
        for (uint32_t ms = 1u; ms <= AK_CLK_MS_MAX; ms++) {
            uint64_t ns;

            /* The reference range, compared as integers because HEXT/ms is
             * not always whole (25 MHz over 2 is 12.5). */
            if ((uint64_t)hext_hz < (uint64_t)AK_CLK_REF_MIN_HZ * ms ||
                (uint64_t)hext_hz > (uint64_t)AK_CLK_REF_MAX_HZ * ms) {
                continue;
            }
            /* ns = vco * ms / hext, and it has to be whole: the part's field
             * takes an integer and a rounded one is a different frequency. */
            if ((vco * ms) % hext_hz != 0u) {
                continue;
            }
            ns = (vco * ms) / hext_hz;
            if (ns < AK_CLK_NS_MIN || ns > AK_CLK_NS_MAX) {
                continue;
            }
            *ms_out = ms;
            *ns_out = (uint32_t)ns;
            *fr_out = fr;
            return 1u;
        }
    }
    return 0u;
}

uint32_t ak_at32_pll_hz(uint32_t ref_hz, unsigned ms, unsigned ns, unsigned fr)
{
    if (ms == 0u || ref_hz == 0u) {
        return 0u;
    }
    /* fr is the exponent of the post-divider: 0 divides by one, 1 by two. */
    uint32_t divisor = 1u << (fr & 0x7u);
    uint64_t vco = ((uint64_t)ref_hz / ms) * ns;

    return (uint32_t)(vco / divisor);
}

static void wait_for_bits(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t guard = AK_CLOCK_TIMEOUT;

    while ((*reg & mask) != mask && guard-- > 0u) {
    }
}

static void wait_for_field(volatile uint32_t *reg, uint32_t mask,
                           unsigned shift, uint32_t value)
{
    uint32_t guard = AK_CLOCK_TIMEOUT;

    while (((*reg & mask) >> shift) != value && guard-- > 0u) {
    }
}

/* The internal clock, whatever the state the bootloader left the part in: on
 * this part it is 8 MHz out of reset, so the firmware has something to run on
 * while the crystal and the PLL come up, and something to stay on if they do
 * not. */
static void fall_back_to_hick(void)
{
    hse_ok    = 0;
    hse_hz    = 0u;
    sysclk_hz = 8000000u;
    apb1_hz   = 8000000u;
    apb2_hz   = 8000000u;
}

/*
 * `hext_hz` is the board's crystal, and it is an argument rather than a macro
 * this file reaches for because that is how every other board fact arrives
 * here: the console's pins, the USB pair and the receiver's port all come in
 * through the call. A board that declares a crystal this part cannot turn into
 * 288 MHz exactly gets the internal clock and a console line saying so, which is
 * a firmware that can still tell you why - see the header.
 */
void ak_clk_init(uint32_t hext_hz)
{
    /*
     * The regulator first, and this is the step a reader coming from the STM32
     * port will not have seen: at 288 MHz the core needs 1.3 V, and Artery's own
     * sequence raises this before it enables the PLL.
     */
    PWC_LDOOV = (PWC_LDOOV & ~(uint32_t)PWC_LDOOV_MASK) | PWC_LDO_OUTPUT_1V3;

    /*
     * Then the flash controller's divider. Artery's sequence sets it and
     * carries on; this one waits for the controller to agree that the divider
     * is in force, because the next thing that happens is a 36-fold increase in
     * the clock the code is fetching itself from.
     */
    FLASH_DIVR = (FLASH_DIVR & ~(uint32_t)FLASH_DIVR_FDIV_MASK) |
                 FLASH_CLOCK_DIV_3;
    wait_for_field(&FLASH_DIVR, FLASH_DIVR_FDIV_STS,
                   FLASH_DIVR_FDIV_STS_SHIFT, FLASH_CLOCK_DIV_3);

    /* The internal clock, so that there is something running while the crystal
     * starts. Artery's sequence resets the whole clock unit here; this one does
     * not, because the firmware may be running on a clock the ROM bootloader
     * configured and stopping the clock a core is executing on is not something
     * to do twice in a boot. */
    CRM_CTRL |= CRM_CTRL_HICKEN;
    wait_for_bits(&CRM_CTRL, CRM_CTRL_HICKSTBL);

    /* The crystal. */
    CRM_CTRL |= CRM_CTRL_HEXTEN;
    wait_for_bits(&CRM_CTRL, CRM_CTRL_HEXTSTBL);
    if ((CRM_CTRL & CRM_CTRL_HEXTSTBL) == 0u) {
        fall_back_to_hick();
        return;
    }
    hse_ok = 1;

    /*
     * The PLL's three numbers, from the board's crystal. A crystal that
     * cannot reach the target exactly is not run at a nearby frequency: the
     * USB clock is this PLL over six, so "near" is a device that enumerates
     * on nothing. It falls back to the internal clock, where the console can
     * say so.
     */
    uint32_t ms = 0u, ns = 0u, fr = 0u;

    if (ak_at32_pll_cfg_for(hext_hz, AK_CLK_TARGET_HZ, &ms, &ns, &fr) == 0u) {
        fall_back_to_hick();
        return;
    }

    /* The PLL, fed from the crystal. */
    CRM_PLLCFG = ((ms & CRM_PLLCFG_MS_MASK) << CRM_PLLCFG_MS_SHIFT) |
                 ((ns & CRM_PLLCFG_NS_MASK) << CRM_PLLCFG_NS_SHIFT) |
                 ((fr & CRM_PLLCFG_FR_MASK) << CRM_PLLCFG_FR_SHIFT) |
                 CRM_PLLCFG_RCS; /* 1 = the crystal is the reference */
    CRM_CTRL |= CRM_CTRL_PLLEN;
    wait_for_bits(&CRM_CTRL, CRM_CTRL_PLLSTBL);
    if ((CRM_CTRL & CRM_CTRL_PLLSTBL) == 0u) {
        fall_back_to_hick();
        return;
    }

    /*
     * The bus dividers, set before the switch so that the moment the core moves
     * to the PLL the buses are already divided: AHB at the core's own rate, and
     * both APB buses at half of it.
     */
    CRM_CFG = (CRM_CFG & ~((uint32_t)CRM_CFG_AHBDIV_MASK << CRM_CFG_AHBDIV_SHIFT)
                          & ~((uint32_t)CRM_CFG_APB1DIV_MASK << CRM_CFG_APB1DIV_SHIFT)
                          & ~((uint32_t)CRM_CFG_APB2DIV_MASK << CRM_CFG_APB2DIV_SHIFT)) |
              ((uint32_t)CRM_DIV_1 << CRM_CFG_AHBDIV_SHIFT) |
              ((uint32_t)CRM_DIV_2 << CRM_CFG_APB1DIV_SHIFT) |
              ((uint32_t)CRM_DIV_2 << CRM_CFG_APB2DIV_SHIFT);

    /* And the switch itself, with the status field as the answer. */
    CRM_CFG = (CRM_CFG & ~(uint32_t)CRM_CFG_SCLKSEL_MASK) | CRM_CFG_SCLKSEL_PLL;
    wait_for_field(&CRM_CFG, CRM_CFG_SCLKSTS_MASK, CRM_CFG_SCLKSTS_SHIFT,
                   CRM_CFG_SCLKSEL_PLL);

    /*
     * What the clock *is*, decoded from the register rather than repeated from
     * the numbers above: a firmware that reports the frequency it meant to ask
     * for is a firmware that can be wrong about itself, and this one line is
     * where that would happen - the host test deliberately writes a wrong
     * multiplier and checks that both the register and this report notice.
     *
     * The reference is the one number that is *not* read back, because the
     * part has nowhere to read it from: it is the caller's, and `hse_hz`
     * carries it so that the banner's clock line and this report name the same
     * crystal the PLL was built on.
     */
    hse_hz    = hext_hz;
    sysclk_hz = ak_at32_pll_hz(
        hse_hz,
        (unsigned)((CRM_PLLCFG >> CRM_PLLCFG_MS_SHIFT) & CRM_PLLCFG_MS_MASK),
        (unsigned)((CRM_PLLCFG >> CRM_PLLCFG_NS_SHIFT) & CRM_PLLCFG_NS_MASK),
        (unsigned)((CRM_PLLCFG >> CRM_PLLCFG_FR_SHIFT) & CRM_PLLCFG_FR_MASK));
    apb2_hz   = sysclk_hz / 2u;
    apb1_hz   = sysclk_hz / 2u;
}

uint32_t ak_clk_sysclk_hz(void)
{
    return sysclk_hz;
}

uint32_t ak_clk_apb1_hz(void)
{
    return apb1_hz;
}

uint32_t ak_clk_apb2_hz(void)
{
    return apb2_hz;
}

int ak_clk_hse_ok(void)
{
    return hse_ok;
}

/*
 * The crystal the clock was built on, in Hz, or 0 when the firmware is on the
 * internal clock. The name is the F405 port's and the *contract* is the same -
 * the banner's clock line prints it - but what stands behind it is not: on the
 * F405 this is a measurement taken at boot against HSI, and here it is the
 * board's declaration (see the file's header). A caller that needs to know
 * which of the two it is has to ask which arch it is on, and that is on
 * purpose: a number with a measurement behind it and a number with a
 * declaration behind it are not the same evidence, and the console line says
 * which by naming the crystal this firmware asked for.
 */
uint32_t ak_clk_hse_hz(void)
{
    return hse_hz;
}
