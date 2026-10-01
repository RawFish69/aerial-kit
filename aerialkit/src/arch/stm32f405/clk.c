#include "arch.h"

/*
 * Clock tree: HSE -> PLL -> 168 MHz SYSCLK, 168 MHz AHB, 42 MHz APB1,
 * 84 MHz APB2, 48 MHz for USB (PLLQ).
 *
 *   HSE / PLLM = 1 MHz reference, x PLLN 336 = 336 MHz VCO,
 *   / PLLP 2 = 168 MHz, / PLLQ 7 = 48 MHz.
 *
 * **PLLM is the crystal in megahertz, and the crystal is measured, not
 * assumed** (measure_hse below). This tree assumed 8 MHz until 2026-09-28. The
 * WeAct board carries 12 MHz, so every HSE build ran the core at 252 MHz and
 * gave USB 72 MHz: the core saw bus resets and decoded not one frame, and no
 * change to usb.c could have fixed it. The measured build enumerated on its
 * first boot. The paragraphs below were written before that and are kept as
 * the record; where they say the crystal is exonerated, they are wrong.
 *
 * If HSE does not come up - dead crystal, or a board without one - the
 * firmware stays on the 16 MHz HSI and says so, rather than sitting mute in a
 * while() loop. A board with a bad clock should still be able to tell you.
 *
 * -DAK_PLL_FROM_HSI=1 builds the same tree from the other oscillator, and it
 * exists to answer one question with the host as the only instrument.
 *
 * PLLM is the divider that makes the 1 MHz reference out of the source, so it
 * is the one number that has to change with the source: 8 MHz HSE / 8 and
 * 16 MHz HSI / 16 are the same 1 MHz, which leaves the VCO, PLLP and PLLQ
 * untouched. The chip runs at the same 168 MHz with the same 48 MHz for USB.
 * What changes is which oscillator the whole part hangs off.
 *
 * That was argued from this board's **ROM bootloader enumerating** - every
 * dfu-util run is a full enumeration over this same OTG_FS peripheral, on the
 * same pins, cable and host port - together with the belief that the ROM runs
 * USB from HSI. **It does not.** Per AN2606 the F405's ROM measures HSE and
 * runs DFU from it, so the ROM's success was always the *HSE* path working and
 * said nothing at all about HSI. The premise was false and the argument under
 * it does not survive: it is retracted, not merely doubted.
 *
 * **The build was run on 2026-09-27 and did not enumerate**, which is all it
 * ever showed. It was read at the time as settling the question and as
 * exonerating the crystal - see docs/evidence/f405-usb-trace.txt, which now
 * carries the same retraction - and that reading is wrong. This file's own
 * header says what the fault was: `PLLM = 8` against a 12 MHz crystal, which
 * asks the VCO for 504 MHz, past the part's 432 MHz limit.
 *
 * **And the part locked anyway.** That is the second half of the same mistake,
 * and the trace record has always contained the refutation: the 2026-09-27 HSE
 * image reported `sysclk 168000000`, a word written from `ak_clk_sysclk_hz()`,
 * which this file assigns only after `HSERDY`, `PLLRDY` and `SWS` have each
 * said yes. A datasheet maximum is a guarantee inside it and not a cliff at its
 * edge, and nothing in that record measured the VCO. The part ran at 252 MHz
 * and gave USB 72 MHz, which is the failure the record's other words describe:
 * `USBRST` and `ENUMDNE` latched, `FNSOF 0`, not one setup taken, suspended by
 * ten seconds. `usb.c` was not the fault either. Neither of those statements
 * was tested; what retired both was *measuring* the crystal, which no argument
 * about clock sources could have reached - the source was never wrong, the
 * divider was.
 *
 * So the flag is a *switched-off* build rather than an open question, and it is
 * kept because it is the only way to reproduce that result. Its caveat stands,
 * and it is the second reason it could never carry the conclusion it was made
 * to carry: HSI is an RC oscillator good to about ±1%, and USB full speed asks
 * for ±0.25% of the *device's* transmit, so a failure here was the likelier
 * outcome whatever the rest of the firmware did. A build that was going to fail
 * anyway is not evidence about anything else.
 */
#ifndef AK_PLL_FROM_HSI
#define AK_PLL_FROM_HSI 0
#endif

#if AK_PLL_FROM_HSI
#define AK_PLL_M      16u /* 16 MHz / 16 = the same 1 MHz reference */
#define AK_PLL_SRC    0u  /* PLLSRC: HSI */
#else
#define AK_PLL_M      8u
#define AK_PLL_SRC    RCC_PLL_SRC_HSE
#endif

#define AK_HSE_TIMEOUT 1000000u

static uint32_t sysclk_hz = 16000000u;
static uint32_t apb1_hz   = 16000000u;
static uint32_t apb2_hz   = 16000000u;
static int      hse_ok;
static uint32_t hse_hz;

/*
 * Measure the crystal against HSI, before anything is built on it.
 *
 * The tree above assumed an 8 MHz crystal and nothing checked. The 2026-09-28
 * record from the HSE build is what a wrong assumption looks like on USB: the
 * core saw nine bus resets and decoded not one frame marker in ten seconds
 * (FNSOF 0, no SOF, no RX), while the ROM - which measures HSE itself before
 * it starts DFU, per AN2606 - enumerates on the same crystal every time.
 *
 * TIM11's channel 1 can take HSE/RTCPRE in place of a pin. With the part on
 * HSI (16 MHz, APB2 undivided, so the timer counts HSI), capture every eighth
 * edge of HSE/31 and add up 32 of those periods: 7936 crystal cycles, timed in
 * HSI ticks. HSI's +-1% is plenty to name a crystal in whole megahertz.
 * Returns 0 if the capture never fires.
 */
#define AK_HSE_RTCPRE   31u
#define AK_HSE_CAPTURES 32u

static uint32_t measure_hse(void)
{
    uint32_t cfgr = RCC_CFGR;
    uint32_t total = 0u;
    uint16_t last = 0u;
    int ok = 1;

    RCC_CFGR = (cfgr & ~(0x1Fu << 16)) | (AK_HSE_RTCPRE << 16);
    RCC_APB2ENR |= RCC_APB2ENR_TIM11EN;
    (void)RCC_APB2ENR;
    TIM11_CR1 = 0u;
    TIM11_OR = 2u;                     /* TI1 <- HSE_RTC */
    TIM11_PSC = 0u;
    TIM11_ARR = 0xFFFFu;
    TIM11_CCMR1 = 1u | (3u << 2);      /* CC1S = TI1, IC1PSC = /8 */
    TIM11_CCER = 1u;                   /* CC1E, rising edge */
    TIM11_EGR = 1u;
    TIM11_SR = 0u;
    TIM11_CR1 = 1u;

    for (uint32_t n = 0u; n <= AK_HSE_CAPTURES; n++) {
        uint32_t guard = 200000u;
        while ((TIM11_SR & (1u << 1)) == 0u && guard-- > 0u) {
        }
        if ((TIM11_SR & (1u << 1)) == 0u) {
            ok = 0;
            break;
        }
        uint16_t now = (uint16_t)TIM11_CCR1; /* the read clears CC1IF */
        if (n > 0u) {
            total += (uint16_t)(now - last);
        }
        last = now;
    }

    TIM11_CR1 = 0u;
    TIM11_CCER = 0u;
    TIM11_OR = 0u;
    RCC_APB2ENR &= ~RCC_APB2ENR_TIM11EN;
    RCC_CFGR = cfgr;

    if (!ok || total == 0u) {
        return 0u;
    }
    return (uint32_t)(((uint64_t)AK_HSE_CAPTURES * 8u * AK_HSE_RTCPRE *
                       16000000u) / total);
}

/* PLLM for a 1 MHz reference: the crystal in whole megahertz, if the
 * measurement names one this PLL can use; otherwise the old assumption. */
static uint32_t pll_m_for(uint32_t hz)
{
    uint32_t m = (hz + 500000u) / 1000000u;
    uint32_t err;

    if (m < 4u || m > 26u) {
        return AK_PLL_M;
    }
    err = hz > m * 1000000u ? hz - m * 1000000u : m * 1000000u - hz;
    return err <= m * 30000u ? m : AK_PLL_M; /* within 3% */
}

static void wait_for(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t guard = AK_HSE_TIMEOUT;
    while ((*reg & mask) == 0 && guard-- > 0) {
    }
}

static void fall_back_to_hsi(void)
{
    /* 16 MHz is safe at one flash wait state with the default voltage scale. */
    FLASH_ACR = FLASH_ACR_LATENCY(1) | FLASH_ACR_PRFTEN;
    hse_ok    = 0;
    sysclk_hz = 16000000u;
    apb1_hz   = 16000000u;
    apb2_hz   = 16000000u;
}

void ak_clk_init(void)
{
    /* HSI is on after reset; make sure, then bring up HSE. */
    RCC_CR |= RCC_CR_HSION;
    wait_for(&RCC_CR, RCC_CR_HSIRDY);

#if AK_PLL_FROM_HSI
    /* Nothing to bring up: the source is already running and already the one
     * the part came out of reset on. `hse_ok` stays 0, which is the truth for
     * this build - it means the crystal came up, and this build never asked
     * it to. A board that reports 168 MHz and HSE failed is not a
     * contradiction here, and the banner's clock line is where that shows. */
#else
    RCC_CR |= RCC_CR_HSEON;
    wait_for(&RCC_CR, RCC_CR_HSERDY);
    if ((RCC_CR & RCC_CR_HSERDY) == 0) {
        fall_back_to_hsi();
        return;
    }
    hse_ok = 1;
    hse_hz = measure_hse();
#endif

    /* Raise the flash latency before asking for 168 MHz, not after. */
    FLASH_ACR = FLASH_ACR_LATENCY(5) | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |
                FLASH_ACR_DCEN;

#if AK_PLL_FROM_HSI
    uint32_t pll_m = AK_PLL_M;
#else
    uint32_t pll_m = pll_m_for(hse_hz);
#endif
    RCC_PLLCFGR = RCC_PLL_M(pll_m) | RCC_PLL_N(336) | RCC_PLL_P(2) |
                  AK_PLL_SRC | RCC_PLL_Q(7);
    RCC_CR |= RCC_CR_PLLON;
    wait_for(&RCC_CR, RCC_CR_PLLRDY);
    if ((RCC_CR & RCC_CR_PLLRDY) == 0) {
        fall_back_to_hsi();
        return;
    }

    uint32_t cfgr = RCC_CFGR;
    cfgr &= ~(RCC_CFGR_SW_MASK | (0xFu << 4) | (0x7u << 10) | (0x7u << 13));
    cfgr |= RCC_CFGR_SW_PLL | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 |
            RCC_CFGR_PPRE2_DIV2;
    RCC_CFGR = cfgr;

    wait_for(&RCC_CFGR, RCC_CFGR_SWS_MASK);
    if ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) {
        /* The switch did not take. Rather than run at an unknown clock, go back
         * to one we can name, and let the banner report HSE as failed. */
        fall_back_to_hsi();
        return;
    }

    sysclk_hz = 168000000u;
    apb1_hz   = 42000000u;
    apb2_hz   = 84000000u;
}

uint32_t ak_clk_hse_hz(void)
{
    return hse_hz;
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
