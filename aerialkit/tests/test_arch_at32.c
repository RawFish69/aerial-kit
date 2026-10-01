/*
 * The AT32F435 port's clock, run against a mapped register block.
 *
 * This is the same method as tests/test_arch.c and the reason the F405 port was
 * believable before a board existed: the *real* driver runs, the registers it
 * writes land in memory at the addresses the part uses, and what is checked is
 * what the code did to them rather than what it says it did. What it cannot
 * prove is anything electrical - a crystal that starts, a PLL that locks, a
 * flash that keeps up - and that is written down where it belongs.
 *
 * The two arches share one test binary and one mapping, because both parts put
 * their clock unit at 0x40023800. That is not as reassuring as it looks: the
 * registers mean different things, which is what this file is here to pin.
 */

#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "at32f435/arch.h"
#include "at32f435/i2c_timing.h"
#include "ak_dshot_timing.h"
#include "board.h"
#include "host_flash_model_at32.h"
#include "host_i2c_model_at32.h"
#include "host_adc_model.h"
#include "host_spi_model.h"
#include "host_usb_model.h"

/* The port's register loop, under the name the host build keeps for it (the
 * AT32's arch.h renames it, the way it renames everything else this binary
 * links beside the F405's). */
int ak_at32_spi_transfer_loop(uint32_t spi, const uint8_t *tx, uint8_t *rx,
                              unsigned len);
#define ak_spi_transfer_loop ak_at32_spi_transfer_loop
#include "tests.h"

#define PERIPH_BASE 0x40000000UL
#define PERIPH_SIZE 0x40000UL
/* SysTick and the system control block are not in the peripheral windows: they
 * are the Cortex-M4's own, at the same address on this part as on any other. */
#define PRIVATE_BASE 0xE000E000UL
#define PRIVATE_SIZE 0x1000UL
/* The part's own flash: where a page is erased and a word is programmed, and
 * where the modelled controller writes. */
#define FLASH_REGION_BASE 0x08000000UL
#define FLASH_REGION_SIZE 0x00100000UL
/* And the USB controller, which is a window of its own at 0x50000000 - outside
 * both blocks above - with its four FIFO windows 0x1000 apart inside it. */
#define USB_BASE 0x50000000UL
#define USB_SIZE 0x4000UL

static void map_registers(void)
{
    void *apb = mmap((void *)PERIPH_BASE, PERIPH_SIZE, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    void *priv = mmap((void *)PRIVATE_BASE, PRIVATE_SIZE,
                      PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    void *flash = mmap((void *)FLASH_REGION_BASE, FLASH_REGION_SIZE,
                       PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    void *usb = mmap((void *)USB_BASE, USB_SIZE, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

    if (apb == MAP_FAILED || priv == MAP_FAILED || flash == MAP_FAILED ||
        usb == MAP_FAILED) {
        expect("the AT32's peripheral block could be mapped", 0);
    }
}

/* A part that answers its waits, the way a real one does a few hundred
 * microseconds in: the crystal is stable, the PLL is locked, the flash
 * controller has taken its divider, and the system clock is already reporting
 * the source the code is about to ask for (that field is read-only on the part,
 * so the only way to answer it here is to have it saying the right thing). */
/* The same part with a crystal that never starts: the peripheral block as it
 * comes out of reset. Named and exported because the board test needs the same
 * failure, and a second copy of the address range is how two tests come to
 * disagree about it. */
void ak_test_at32_no_crystal(void)
{
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
}

void ak_test_at32_clock_up(void)
{
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    CRM_CTRL = CRM_CTRL_HICKSTBL | CRM_CTRL_HEXTSTBL | CRM_CTRL_PLLSTBL;
    CRM_CFG = (uint32_t)CRM_CFG_SCLKSEL_PLL << CRM_CFG_SCLKSTS_SHIFT;
    FLASH_DIVR = (uint32_t)FLASH_CLOCK_DIV_3 << FLASH_DIVR_FDIV_STS_SHIFT;
}

static void test_the_arithmetic(void)
{
    /* The ROM bootloader's address, which is the `dfu` command's whole payload:
     * Artery's own library returns this number from
     * `systemBootloaderAddress()`. The jump cannot be run on a host - there is
     * nothing at that address in this process - so what is pinned is the number
     * and, in the board test below, that the board offers the hand-over at all. */
    expect("the rom bootloader is where the reference puts it",
           AK_BOOTLOADER_BASE == 0x1FFF0000u);

    /* The PLL is (reference / ms) * ns / fr, and fr is an exponent: the same
     * numbers with fr = 4 are half the frequency, which is how a wrong reading
     * of that field shows up as an aircraft rather than as a compiler error. */
    expect("8 MHz with ms 1, ns 72 and fr 2 is 288 MHz",
           ak_at32_pll_hz(8000000u, 1u, 72u, CRM_PLL_FR_2) == 288000000u);
    expect("the same numbers with fr 4 are 144 MHz",
           ak_at32_pll_hz(8000000u, 1u, 72u, CRM_PLL_FR_4) == 144000000u);
    expect("and a zero input divider is refused rather than divided by",
           ak_at32_pll_hz(8000000u, 0u, 72u, CRM_PLL_FR_2) == 0u);
}

static void test_the_clock_tree(void)
{
    map_registers();
    ak_test_at32_clock_up();
    ak_clk_init(AK_BOARD_HEXT_HZ);

    expect("the crystal is believed", ak_clk_hse_ok() == 1);
    /* And the number the clock reports back is the one it was handed, not a
     * literal of its own: the banner prints this, and a banner that names a
     * crystal the PLL was not built on is the F405's bug in its other form. */
    expect("and the crystal it reports is the one the board declared",
           ak_clk_hse_hz() == AK_BOARD_HEXT_HZ && ak_clk_hse_hz() == 8000000u);

    /* The regulator, which is the step this part has and the F405 does not. */
    expect("the core regulator is at the voltage 288 MHz needs",
           (PWC_LDOOV & PWC_LDOOV_MASK) == PWC_LDO_OUTPUT_1V3);

    /* The flash divider, which has to be in force before the core speeds up. */
    expect("the flash controller is dividing by three",
           (FLASH_DIVR & FLASH_DIVR_FDIV_MASK) == FLASH_CLOCK_DIV_3);

    /* The PLL registers, decoded rather than compared: the arithmetic is the
     * thing that has to be right, and it is read the way the part reads it. */
    uint32_t pll = CRM_PLLCFG;
    unsigned ms = (unsigned)((pll >> CRM_PLLCFG_MS_SHIFT) & CRM_PLLCFG_MS_MASK);
    unsigned ns = (unsigned)((pll >> CRM_PLLCFG_NS_SHIFT) & CRM_PLLCFG_NS_MASK);
    unsigned fr = (unsigned)((pll >> CRM_PLLCFG_FR_SHIFT) & CRM_PLLCFG_FR_MASK);

    expect("the PLL is fed from the crystal", (pll & CRM_PLLCFG_RCS) != 0u);
    /* Against the crystal the clock reports, not against an 8 written here:
     * the register and the report have to agree about one number. */
    expect("and ms, ns and fr multiply that crystal to 288 MHz",
           ak_at32_pll_hz(ak_clk_hse_hz(), ms, ns, fr) == 288000000u);
    expect("which is the three numbers this file used to carry as literals",
           ms == 1u && ns == 72u && fr == CRM_PLL_FR_2);
    /* The reason the search is for an exact hit and not a near one: the USB
     * clock is this PLL over six, and 48 MHz is the only rate a full-speed
     * device works at. */
    expect("and USB is the PLL over six, which is 48 MHz only at 288",
           ak_clk_sysclk_hz() / 6u == 48000000u);

    expect("the system clock is switched to the PLL",
           (CRM_CFG & CRM_CFG_SCLKSEL_MASK) == CRM_CFG_SCLKSEL_PLL);
    expect("AHB runs at the core's own rate",
           ((CRM_CFG >> CRM_CFG_AHBDIV_SHIFT) & CRM_CFG_AHBDIV_MASK) == CRM_DIV_1);
    expect("and both APB buses at half of it",
           ((CRM_CFG >> CRM_CFG_APB1DIV_SHIFT) & CRM_CFG_APB1DIV_MASK) ==
                   CRM_DIV_2 &&
               ((CRM_CFG >> CRM_CFG_APB2DIV_SHIFT) & CRM_CFG_APB2DIV_MASK) ==
                   CRM_DIV_2);
    expect("and the clock the firmware reports is the one it asked for",
           ak_clk_sysclk_hz() == 288000000u && ak_clk_apb1_hz() == 144000000u &&
               ak_clk_apb2_hz() == 144000000u);
}

static void test_the_clock_without_a_crystal(void)
{
    /* A board whose crystal never starts must end up on a clock it can name and
     * must say so, rather than sitting in a wait loop where nothing can tell
     * you why the console is silent. */
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    ak_clk_init(AK_BOARD_HEXT_HZ);

    expect("a crystal that never starts is reported as failed",
           ak_clk_hse_ok() == 0);
    expect("and the clock falls back to the internal one, named",
           ak_clk_sysclk_hz() == 8000000u && ak_clk_apb1_hz() == 8000000u &&
               ak_clk_apb2_hz() == 8000000u);
    expect("with the PLL left off", (CRM_CTRL & CRM_CTRL_PLLEN) == 0u);
    /* And with no crystal to report: the banner would otherwise print the one
     * the board declared beside a clock that is not running on it. */
    expect("and no crystal is claimed", ak_clk_hse_hz() == 0u);
}

/*
 * A crystal that is not the one this board carries, through the real driver.
 *
 * `ak_at32_pll_cfg_for` is arithmetic, and it is asked here twice: once on its
 * own, and once through the code that writes the registers, because the two can
 * disagree - a derived triple that is computed and then not used is exactly the
 * shape of the F405 bug this port is closing. The crystal the board
 * happens to have is 8 MHz and its triple is pinned in the test above; these
 * are the cans a different board could carry, and the arithmetic says 12 and 16
 * land, 24 needs an input divider, and 25 lands on nothing at all.
 */
static void test_the_clock_from_another_crystal(void)
{
    struct crystal {
        uint32_t hz;
        unsigned ms, ns, fr;
    };
    static const struct crystal land[] = {
        {12000000u, 1u, 48u, CRM_PLL_FR_2},
        {16000000u, 1u, 36u, CRM_PLL_FR_2},
        {24000000u, 2u, 48u, CRM_PLL_FR_2},
    };

    /* The arithmetic on its own first, at the boundary the driver calls it
     * through. A triple that comes back here and is not what the registers get
     * is the shape of the F405 bug being closed, so both halves are pinned and
     * neither is taken from the other. */
    uint32_t cms = 0u, cns = 0u, cfr = 0u;

    expect("the arithmetic gives the board's crystal its three numbers",
           ak_at32_pll_cfg_for(8000000u, 288000000u, &cms, &cns, &cfr) == 1u &&
               cms == 1u && cns == 72u && cfr == CRM_PLL_FR_2);
    expect("and refuses one it cannot reach the target from, rather than "
           "rounding",
           ak_at32_pll_cfg_for(25000000u, 288000000u, &cms, &cns, &cfr) == 0u);
    expect("and refuses a zero crystal rather than dividing by it",
           ak_at32_pll_cfg_for(0u, 288000000u, &cms, &cns, &cfr) == 0u);

    for (unsigned i = 0; i < sizeof land / sizeof land[0]; i++) {
        map_registers();
        ak_test_at32_clock_up();
        ak_clk_init(land[i].hz);

        expect("a crystal this part can multiply to 288 exactly is used",
               ak_clk_hse_ok() == 1 && ak_clk_sysclk_hz() == 288000000u);
        expect("and the registers carry the triple the arithmetic chose",
               ((CRM_PLLCFG >> CRM_PLLCFG_MS_SHIFT) & CRM_PLLCFG_MS_MASK) ==
                       land[i].ms &&
                   ((CRM_PLLCFG >> CRM_PLLCFG_NS_SHIFT) & CRM_PLLCFG_NS_MASK) ==
                       land[i].ns &&
                   ((CRM_PLLCFG >> CRM_PLLCFG_FR_SHIFT) & CRM_PLLCFG_FR_MASK) ==
                       land[i].fr);
        expect("and the report names that crystal, not the board's",
               ak_clk_hse_hz() == land[i].hz);
    }

    /* 25 MHz: the reference can be divided into range, but nothing in that
     * range multiplies to 576 MHz in whole ns, so this part cannot run USB from
     * it. It is refused rather than rounded - a PLL at 288.0 MHz from a crystal
     * that only reaches 287.5 is a device that does not enumerate. */
    map_registers();
    ak_test_at32_clock_up();
    ak_clk_init(25000000u);
    expect("a crystal that cannot reach 288 exactly is refused",
           ak_clk_hse_ok() == 0 && ak_clk_sysclk_hz() == 8000000u);
    expect("and the PLL is left off rather than run near the target",
           (CRM_CTRL & CRM_CTRL_PLLEN) == 0u && CRM_PLLCFG == 0u);
    expect("and no crystal is claimed", ak_clk_hse_hz() == 0u);

    /* A board that declares no crystal at all - the value a zeroed board
     * header would give - is the same answer and not a divide. */
    map_registers();
    ak_test_at32_clock_up();
    ak_clk_init(0u);
    expect("a board that declares no crystal falls back rather than dividing",
           ak_clk_hse_ok() == 0 && ak_clk_sysclk_hz() == 8000000u);
}

/*
 * Pins, which is the half of the port where pattern-matching the F405 would be
 * wrong rather than merely untidy: this part has no MODER and no AFR, so every
 * number below is a different number. Each function is checked by the register
 * it is supposed to produce, and - for the mux - by *where* the function number
 * lands, because the two nibble registers are the mistake that would look like
 * a pin that simply does not work.
 */
static void test_the_pins(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);

    /* An output: mode in cfgr, output type in omode, and the port's clock on. */
    ak_pin_output(AK_PIN(GPIOA_BASE, 5), 0, 1);
    expect("an output pin is in output mode",
           ((AK_GPIO_CFGR(GPIOA_BASE) >> 10) & 0x3u) == AK_GPIO_MODE_OUTPUT);
    expect("push-pull, because that is what was asked for",
           (AK_GPIO_OMODE(GPIOA_BASE) & (1u << 5)) == 0u);
    expect("and with the stronger driver when a speed is asked for",
           ((AK_GPIO_ODRVR(GPIOA_BASE) >> 10) & 0x3u) ==
               AK_GPIO_DRIVE_STRONGER);
    expect("and its port's clock is enabled",
           (CRM_AHBEN1 & 1u) != 0u); /* GPIOA is bit 0 of that register */

    /* A mux pin below pin 8: the function number goes in muxl's nibble. */
    ak_pin_af(AK_PIN(GPIOB_BASE, 0), 6, AK_GPIO_PULL_UP);
    expect("a mux pin is in mux mode",
           (AK_GPIO_CFGR(GPIOB_BASE) & 0x3u) == AK_GPIO_MODE_MUX);
    expect("with the pull it was given",
           (AK_GPIO_PULL(GPIOB_BASE) & 0x3u) == AK_GPIO_PULL_UP);
    expect("and the function number in the low mux register",
           (AK_GPIO_MUXL(GPIOB_BASE) & 0xFu) == 6u &&
               AK_GPIO_MUXH(GPIOB_BASE) == 0u);
    expect("and GPIOB's own clock bit, not GPIOA's",
           (CRM_AHBEN1 & 0x3u) == 0x3u);

    /* And one above pin 7, which lands in the other register. */
    ak_pin_af(AK_PIN(GPIOA_BASE, 15), 5, AK_GPIO_PULL_NONE);
    expect("a pin above seven puts its function number in the high register",
           (AK_GPIO_MUXH(GPIOA_BASE) >> 28) == 5u &&
               (AK_GPIO_MUXL(GPIOA_BASE) >> 10 & 0xFu) == 0u);

    /* An analog pin: mode 3, no pull - what an ADC channel wants. */
    ak_pin_analog(AK_PIN(GPIOA_BASE, 1));
    expect("an analog pin is in analog mode with no pull",
           ((AK_GPIO_CFGR(GPIOA_BASE) >> 2) & 0x3u) == AK_GPIO_MODE_ANALOG &&
               ((AK_GPIO_PULL(GPIOA_BASE) >> 2) & 0x3u) == AK_GPIO_PULL_NONE);

    /* Setting, clearing and reading. The set and clear registers are strobes -
     * a one in the pin's bit does the work and nothing else on the port is
     * touched, which is why this part has two of them. */
    AK_GPIO_SCR(GPIOA_BASE) = 0u;
    AK_GPIO_CLR(GPIOA_BASE) = 0u;
    ak_pin_set(AK_PIN(GPIOA_BASE, 5), 1);
    expect("setting a pin writes its bit to the set register",
           AK_GPIO_SCR(GPIOA_BASE) == (1u << 5));
    ak_pin_set(AK_PIN(GPIOA_BASE, 5), 0);
    expect("and clearing it writes the clear register",
           AK_GPIO_CLR(GPIOA_BASE) == (1u << 5));

    /* A toggle reads the output register and inverts it: low goes high, and
     * high goes low. The model is the register file, so the level is set here
     * the way the part would have it. */
    AK_GPIO_ODT(GPIOA_BASE) = 0u;
    AK_GPIO_SCR(GPIOA_BASE) = 0u;
    AK_GPIO_CLR(GPIOA_BASE) = 0u;
    ak_pin_toggle(AK_PIN(GPIOA_BASE, 5));
    expect("toggling a low pin sets it",
           AK_GPIO_SCR(GPIOA_BASE) == (1u << 5));
    AK_GPIO_ODT(GPIOA_BASE) = 1u << 5;
    AK_GPIO_SCR(GPIOA_BASE) = 0u;
    AK_GPIO_CLR(GPIOA_BASE) = 0u;
    ak_pin_toggle(AK_PIN(GPIOA_BASE, 5));
    expect("and toggling a high pin clears it",
           AK_GPIO_CLR(GPIOA_BASE) == (1u << 5));

    /* Reading is the input register, which on the part reflects the pin. */
    AK_GPIO_IDT(GPIOA_BASE) = 1u << 5;
    expect("a pin held high reads high", ak_pin_get(AK_PIN(GPIOA_BASE, 5)) == 1);
    AK_GPIO_IDT(GPIOA_BASE) = 0u;
    expect("and a pin held low reads low", ak_pin_get(AK_PIN(GPIOA_BASE, 5)) == 0);
}

/*
 * The tick. One millisecond, calculated from the clock the clock code ended up
 * on rather than from a number written down twice - so a board that fell back
 * to the internal clock still counts milliseconds, which is the only reason the
 * fallback is worth having.
 */
static void test_the_tick(void)
{
    map_registers();
    memset((void *)PRIVATE_BASE, 0, PRIVATE_SIZE);

    ak_test_at32_clock_up();
    ak_clk_init(AK_BOARD_HEXT_HZ);
    ak_arch_time_init();
    expect("the tick's reload is a millisecond of the clock it is running on",
           SYSTICK_LOAD == (288000000u / 1000u) - 1u);
    expect("and it is enabled, interrupting, on the processor clock",
           (SYSTICK_CTRL & (AK_SYSTICK_CTRL_ENABLE | AK_SYSTICK_CTRL_TICKINT |
                            AK_SYSTICK_CTRL_CLKSOURCE)) ==
               (AK_SYSTICK_CTRL_ENABLE | AK_SYSTICK_CTRL_TICKINT |
                AK_SYSTICK_CTRL_CLKSOURCE));

    /* The handler is the port's, and the counter it feeds is the portable
     * contract's - the same two that the firmware and its tests use. */
    uint32_t before = ak_time_ms();
    SysTick_Handler();
    SysTick_Handler();
    expect("two interrupts are two milliseconds",
           ak_time_ms() == before + 2u);

    /* And on a board whose crystal never started the tick still means a
     * millisecond, because the reload follows the clock that is actually
     * running. */
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    memset((void *)PRIVATE_BASE, 0, PRIVATE_SIZE);
    ak_clk_init(AK_BOARD_HEXT_HZ);
    ak_arch_time_init();
    expect("a board on the internal clock ticks in milliseconds too",
           ak_clk_hse_ok() == 0 && SYSTICK_LOAD == (8000000u / 1000u) - 1u);
}

/*
 * The console's port, which is the second thing a board needs and the second
 * thing that can be checked without one - and where the arithmetic has a known
 * way of being wrong.
 *
 * The port and pins below are this test's own choice rather than a board's:
 * USART1 on PA9/PA10 with function 7 is the pair the wing's INAV target leaves
 * free and brings out as UART1, and this port has no board file yet. What the
 * numbers *mean* is what is being checked, and the board will say which port
 * and pins it actually uses.
 */
#define TEST_CONSOLE_USART USART1_BASE
#define TEST_CONSOLE_TX    AK_PIN(GPIOA_BASE, 9)
#define TEST_CONSOLE_RX    AK_PIN(GPIOA_BASE, 10)
#define TEST_CONSOLE_AF    7u

static void test_the_console(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    memset((void *)PRIVATE_BASE, 0, PRIVATE_SIZE);
    ak_test_at32_clock_up();
    ak_clk_init(AK_BOARD_HEXT_HZ); /* 288 MHz core, 144 MHz on both APB buses */

    ak_uart_init(TEST_CONSOLE_USART, TEST_CONSOLE_TX, TEST_CONSOLE_RX,
                 115200u, TEST_CONSOLE_AF);
    ak_console_attach(TEST_CONSOLE_USART);

    expect("the console is attached to the port it was given",
           ak_console_port() == TEST_CONSOLE_USART);

    /* The divisor: fCK / baud, and not sixteen times it. 144 MHz over 115200 is
     * 1250 exactly, which is why the console's rate is a good one to pin. */
    expect("the divisor is the peripheral clock over the baud rate",
           (AK_USART_BAUDR(TEST_CONSOLE_USART) & 0xFFFFu) ==
               (144000000u / 115200u));
    expect("and the port is enabled with both directions",
           (AK_USART_CTRL1(TEST_CONSOLE_USART) &
            (AK_USART_CTRL1_UEN | AK_USART_CTRL1_TEN | AK_USART_CTRL1_REN)) ==
               (AK_USART_CTRL1_UEN | AK_USART_CTRL1_TEN | AK_USART_CTRL1_REN));
    expect("and its pins are in mux mode with the function it was given",
           ((AK_GPIO_CFGR(GPIOA_BASE) >> 18) & 0x3u) == AK_GPIO_MODE_MUX &&
               ((AK_GPIO_CFGR(GPIOA_BASE) >> 20) & 0x3u) == AK_GPIO_MODE_MUX &&
               (AK_GPIO_MUXH(GPIOA_BASE) >> 4 & 0xFu) == TEST_CONSOLE_AF &&
               (AK_GPIO_MUXH(GPIOA_BASE) >> 8 & 0xFu) == TEST_CONSOLE_AF);
    expect("and USART1's own clock bit is on, in the APB2 register",
           (CRM_APB2EN & (1u << 4)) != 0u && CRM_APB1EN == 0u);

    /*
     * The GPS port, which on this airframe needs the swap bit - the one thing
     * this part's UART can do that the F405's cannot - and the CRSF rate, which
     * is the one that does not divide evenly and so the one where the rounding
     * is the point rather than a detail.
     */
    ak_uart_init_swap(USART3_BASE, AK_PIN(GPIOB_BASE, 11),
                      AK_PIN(GPIOB_BASE, 10), 9600u, 7u);
    expect("the GPS port is at 9600 on the APB1 clock",
           (AK_USART_BAUDR(USART3_BASE) & 0xFFFFu) == (144000000u / 9600u));
    expect("and it has the transmit and receive pins swapped",
           (AK_USART_CTRL2(USART3_BASE) & AK_USART_CTRL2_TRPSWAP) != 0u);
    expect("and USART3's clock bit, not USART1's",
           (CRM_APB1EN & (1u << 18)) != 0u);

    ak_uart_init(USART2_BASE, AK_PIN(GPIOA_BASE, 8), AK_PIN(GPIOB_BASE, 0),
                 420000u, 6u);
    {
        uint32_t div = AK_USART_BAUDR(USART2_BASE) & 0xFFFFu;
        uint32_t actual = 144000000u / div;
        int error_percent_tenths =
            (int)((actual > 420000u ? actual - 420000u : 420000u - actual) *
                  1000u / 420000u);

        expect("the CRSF rate rounds to the nearest divisor",
               div == 343u);
        expect("and lands within a tenth of a per cent of the rate asked for",
               error_percent_tenths <= 1);
    }

    /*
     * The write path, as far as a memory-mapped register can show it: the last
     * byte of what was sent is left in the data register. That the *stream* is
     * right is what the simulator's console capture checks; this is that this
     * port's sink reaches this part's data register at all, and stops when the
     * port says it is busy.
     */
    AK_USART_STS(TEST_CONSOLE_USART) = AK_USART_STS_TDBE;
    ak_console_write_raw("AerialKit\r\n", 11u);
    expect("a line written to the console ends in the data register",
           (AK_USART_DT(TEST_CONSOLE_USART) & 0xFFu) == '\n');

    /* And a port that never accepts anything costs a byte, not the firmware. */
    AK_USART_STS(TEST_CONSOLE_USART) = 0u;
    expect("a port that never says it is empty gives up rather than waiting",
           ak_uart_write_bytes(TEST_CONSOLE_USART, "x", 1u) != 0);

    /* The way back: a byte waiting with its flag, and nothing without one. */
    uint8_t byte = 0;

    AK_USART_STS(TEST_CONSOLE_USART) = 0u;
    expect("an idle port offers no byte",
           ak_uart_poll_rx(TEST_CONSOLE_USART, &byte) == 0);
    AK_USART_STS(TEST_CONSOLE_USART) = AK_USART_STS_RDBF;
    AK_USART_DT(TEST_CONSOLE_USART) = 'Z';
    expect("and a byte in it is one",
           ak_uart_poll_rx(TEST_CONSOLE_USART, &byte) == 1 && byte == 'Z');
}

/*
 * The flash controller, which is the one part of a port that cannot be checked
 * by reading registers back: the part has to *act* on what is written to it, so
 * the driver runs against a modelled controller and the checks read the flash
 * itself. The two traps are the ones the F405's model was written after - a
 * page is a page and not a sector, and the second bank has its own registers -
 * and this part adds the second.
 */
static uint32_t flash_word_at(uint32_t address)
{
    return *(volatile uint32_t *)(uintptr_t)address;
}

static void test_the_flash(void)
{
    const uint32_t page_a = AK_FLASH_BANK1_START;      /* bank 1's first page */
    const uint32_t page_b = AK_FLASH_BANK1_START + AK_FLASH_PAGE_BYTES;
    const uint32_t page_2 = AK_FLASH_BANK2_START;      /* bank 2's first page */
    const uint32_t words[2] = { 0xA5A5A5A5u, 0x5A5A5A5Au };

    map_registers();
    host_at32_flash_model_reset();

    /* A page programs and reads back, and its neighbour does not move: this is
     * the check a model with the wrong page size fails, because the driver
     * never names a size - only an address. */
    host_at32_flash_reg_write(0x14u, page_b); /* a bank-1 address register */
    expect("a page erases and a word lands in it",
           ak_flash_program(page_a, words, sizeof words) == 0 &&
               flash_word_at(page_a) == words[0] &&
               flash_word_at(page_a + 4u) == words[1]);
    expect("and the page after it is untouched",
           flash_word_at(page_b) == 0xFFFFFFFFu);

    /*
     * Programming over a word that is not erased is refused by the part, and the
     * driver has to notice rather than report a save that did not happen. The
     * page at reset is erased, as a real one is, so the "not erased" state has
     * to be made by programming it once and then asking again without an erase
     * between - which is exactly the mistake this check is for.
     */
    expect("a word programs into an erased page",
           ak_flash_program(page_b, words, 4u) == 0 &&
               flash_word_at(page_b) == words[0]);
    expect("programming over a word that is not erased is refused",
           ak_flash_program(page_b, words, 4u) != 0);
    expect("and the word that was there is still there",
           flash_word_at(page_b) == words[0]);

    /* The erase, and what it erases: 2 KB from the address, and not the bank's
     * whole self. */
    expect("erasing a page empties it",
           ak_flash_erase_page(page_a) == 0 &&
               flash_word_at(page_a) == 0xFFFFFFFFu &&
               flash_word_at(page_a + AK_FLASH_PAGE_BYTES - 4u) == 0xFFFFFFFFu);

    /* A misaligned erase address is refused before anything is unlocked. */
    host_at32_flash_model_reset();
    expect("an erase address that is not page aligned is refused",
           ak_flash_erase_page(page_a + 4u) != 0 &&
               host_at32_flash_model_keys() == 0u);
    expect("and a word-aligned program that is not a whole word is refused",
           ak_flash_program(page_a, words, 6u) != 0);

    /*
     * The second bank. This part is two of them on the 1 MB part, each with its
     * own registers - so the same operations have to work on an address in the
     * top half of the part, and a driver that used bank one's registers for
     * everything would erase and program the wrong controller while the first
     * bank's flags said all was well.
     */
    host_at32_flash_model_reset();
    expect("a page in the second bank erases",
           ak_flash_erase_page(page_2) == 0);
    expect("and programs through its own registers",
           ak_flash_program(page_2, words, 4u) == 0 &&
               flash_word_at(page_2) == words[0]);
    expect("and bank one was left alone",
           flash_word_at(page_a) == 0xFFFFFFFFu);

    /* A write that would cross the bank boundary is refused rather than split
     * across two controllers. */
    host_at32_flash_model_reset();
    expect("a program that crosses a bank boundary is refused",
           ak_flash_program(AK_FLASH_BANK1_END - 3u, words, sizeof words) != 0);

    /* The unlock, on the wire: the two keys in order, and a controller that
     * refuses them is reported rather than assumed. */
    host_at32_flash_model_reset();
    host_at32_flash_model_set_unlock_refused(1);
    expect("a controller that will not unlock is reported",
           ak_flash_erase_page(page_a) != 0);
    host_at32_flash_model_set_unlock_refused(0);
    host_at32_flash_model_reset();
    (void)ak_flash_erase_page(page_a);
    expect("the keys that reach the controller are the part's own",
           host_at32_flash_model_keys() == FLASH_UNLOCK_KEY2);
}

/*
 * The controller that never finishes - the other part's version of the F405's
 * check, and the one behaviour neither model used to have.
 *
 * This part's busy bit is `OBF`, and its wait is a loop with a bound in it, so
 * a controller that accepts an operation and never completes it is the state
 * that turns a wing's save into a boot that never reaches the console. What is
 * checked is both halves: the driver gives up (rather than waiting for ever),
 * and it gives up *cleanly* - the page still holds what it held, the erase and
 * program bits are not left armed, and both banks are locked again, which is
 * the state the option bytes and any other caller expect.
 */
static void test_the_flash_that_never_finishes(void)
{
    const uint32_t page_a = AK_FLASH_BANK1_START;
    const uint32_t words[2] = { 0x414B0008u, 0xC0FFEE01u };

    map_registers();
    host_at32_flash_model_reset();

    expect("a page to write into, with a word already in it",
           ak_flash_program(page_a, words, sizeof words) == 0 &&
               flash_word_at(page_a) == words[0]);

    host_at32_flash_model_set_stuck_busy(1);
    expect("a page erase on a controller that never finishes gives up",
           ak_flash_erase_page(page_a) != 0);
    expect("and leaves the controller locked, not open",
           (host_at32_flash_reg_read(0x10u) & FLASH_CTRL_OPLK) != 0u);
    expect("with nothing left armed to erase",
           (host_at32_flash_reg_read(0x10u) & FLASH_CTRL_SECERS) == 0u);
    expect("and the word that was there is still there",
           flash_word_at(page_a) == words[0]);

    expect("a program on the same controller gives up too",
           ak_flash_program(page_a + 4u, words, 4u) != 0);
    expect("with the programming bit cleared rather than left armed",
           (host_at32_flash_reg_read(0x10u) & FLASH_CTRL_FPRGM) == 0u);
    expect("and the second word is the one that was already there",
           flash_word_at(page_a + 4u) == words[1]);

    host_at32_flash_model_set_stuck_busy(0);
    expect("and a controller that answers again erases and programs again",
           ak_flash_erase_page(page_a) == 0 &&
               ak_flash_program(page_a, words, sizeof words) == 0 &&
               flash_word_at(page_a) == words[0] &&
               flash_word_at(page_a + 4u) == words[1]);
}

/*
 * The sensor bus. Mode 3 and eight bits are what the parts on it want, the chip
 * select is the caller's, and the one number that is not the F405's is the
 * divider - this part's SPI1 sits on a 144 MHz bus where the F405's sits on 84,
 * so the same setting would be a faster bus. What is checked here is the
 * configuration and the shape of a transfer; what a *scope* on the pins would
 * show is not checkable anywhere but a bench.
 */
#define TEST_SPI          SPI1_BASE
#define TEST_SPI_SCK      AK_PIN(GPIOA_BASE, 5)
#define TEST_SPI_MISO     AK_PIN(GPIOA_BASE, 6)
#define TEST_SPI_MOSI     AK_PIN(GPIOA_BASE, 7)
#define TEST_SPI_AF       5u

static void test_the_sensor_bus(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);

    /*
     * Somebody else's configuration, left where it was: this part splits its
     * divider between two registers, and the high bit and the divide-by-three
     * are in the second one. A register that resets to zero is not a register
     * that is still zero after a bootloader or a previous flight has been at
     * it, so the init has to clear them rather than assume - and this is the
     * check that says so.
     */
    AK_SPI_CTRL2(TEST_SPI) = AK_SPI_CTRL2_MDIV_H | AK_SPI_CTRL2_MDIV3EN;
    ak_spi_init(TEST_SPI, TEST_SPI_SCK, TEST_SPI_MISO, TEST_SPI_MOSI,
                TEST_SPI_AF);

    expect("the bus is a master in mode 3",
           (AK_SPI_CTRL1(TEST_SPI) & (AK_SPI_CTRL1_MSTEN |
                                      AK_SPI_CTRL1_CLKPOL |
                                      AK_SPI_CTRL1_CLKPHA)) ==
               (AK_SPI_CTRL1_MSTEN | AK_SPI_CTRL1_CLKPOL |
                AK_SPI_CTRL1_CLKPHA));
    expect("with software chip select, idle high",
           (AK_SPI_CTRL1(TEST_SPI) & (AK_SPI_CTRL1_SWCSEN |
                                      AK_SPI_CTRL1_ORA)) ==
               (AK_SPI_CTRL1_SWCSEN | AK_SPI_CTRL1_ORA));
    expect("and a divider that lands near 10 MHz on a 144 MHz bus",
           ((AK_SPI_CTRL1(TEST_SPI) >> AK_SPI_CTRL1_MDIV_SHIFT) &
            AK_SPI_CTRL1_MDIV_MASK) == AK_SPI_MDIV_DIV16 &&
               (AK_SPI_CTRL2(TEST_SPI) & (AK_SPI_CTRL2_MDIV_H |
                                          AK_SPI_CTRL2_MDIV3EN)) == 0u);
    expect("and the port is enabled", (AK_SPI_CTRL1(TEST_SPI) &
                                       AK_SPI_CTRL1_SPIEN) != 0u);
    expect("and its pins are in mux mode with the function it was given",
           ((AK_GPIO_CFGR(GPIOA_BASE) >> 10) & 0x3u) == AK_GPIO_MODE_MUX &&
               ((AK_GPIO_CFGR(GPIOA_BASE) >> 12) & 0x3u) == AK_GPIO_MODE_MUX &&
               ((AK_GPIO_CFGR(GPIOA_BASE) >> 14) & 0x3u) == AK_GPIO_MODE_MUX &&
               (AK_GPIO_MUXL(GPIOA_BASE) >> 20 & 0xFu) == TEST_SPI_AF &&
               (AK_GPIO_MUXL(GPIOA_BASE) >> 24 & 0xFu) == TEST_SPI_AF &&
               (AK_GPIO_MUXL(GPIOA_BASE) >> 28 & 0xFu) == TEST_SPI_AF);
    expect("and SPI1's own clock bit is on in the APB2 register",
           (CRM_APB2EN & (1u << 12)) != 0u);

    /*
     * A transfer. What a register file can show is the *shape* of it: every
     * byte written reaches the data register, and every byte read comes back
     * from it - which is what a wire from MOSI to MISO does, and is the
     * loopback the bring-up checklist asks a person to fit. What no host test
     * can show is a device answering, because that needs a device.
     */
    const uint8_t tx[2] = { 0x80u, 0x00u };
    uint8_t rx[2] = { 0u, 0u };

    AK_SPI_STS(TEST_SPI) = AK_SPI_STS_TDBE;
    AK_SPI_STS(TEST_SPI) = AK_SPI_STS_TDBE | AK_SPI_STS_RDBF;
    expect("a transfer moves every byte through the data register",
           ak_spi_transfer_loop(TEST_SPI, tx, rx, 2u) == 0 && rx[0] == tx[0] &&
               rx[1] == tx[1]);
    expect("and the last byte written is the one left in the register",
           (AK_SPI_DT(TEST_SPI) & 0xFFu) == tx[1]);

    /* A bus that never says its buffer is empty gives up rather than waiting. */
    AK_SPI_STS(TEST_SPI) = 0u;
    expect("a bus that never answers costs a transfer, not the firmware",
           ak_spi_transfer_loop(TEST_SPI, tx, rx, 1u) != 0);
    expect("and a transfer of nothing is not a transfer",
           ak_spi_transfer_loop(TEST_SPI, tx, rx, 0u) == 0);

    /* And the controllers this board does not use, whose clock bits had never
     * been written either: the wing's board is on SPI1, and a port that can
     * only bring up the one its board happens to use fails on the next board. */
    ak_spi_init(SPI2_BASE, TEST_SPI_SCK, TEST_SPI_MISO, TEST_SPI_MOSI,
                TEST_SPI_AF);
    expect("the second controller's clock can be turned on",
           (CRM_APB1EN & (1u << 14)) != 0u);
    ak_spi_init(SPI3_BASE, TEST_SPI_SCK, TEST_SPI_MISO, TEST_SPI_MOSI,
                TEST_SPI_AF);
    expect("and the third's",
           (CRM_APB1EN & (1u << 15)) != 0u);
}

/*
 * The barometer's bus, which is the piece of this port with the most arithmetic
 * in it. This part's I2C is the newer peripheral - one timing register for the
 * whole bus, rather than the F405's clock-control and rise-time pair - so what
 * is checked is that the arithmetic lands on the rate it was asked for, and
 * that the fields it produces can hold the answer.
 *
 * A wrong one does not fail: it runs the bus at the wrong speed, and a bus too
 * fast reads exactly like a barometer that is not answering. That is why the
 * rate is decoded back out of the register rather than compared with a
 * constant.
 */
#define TEST_I2C     I2C2_BASE
#define TEST_I2C_SCL AK_PIN(GPIOH_BASE, 2)
#define TEST_I2C_SDA AK_PIN(GPIOH_BASE, 3)
#define TEST_I2C_AF  4u

static void test_the_barometer_bus(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);

    /* The arithmetic on its own, at the two rates a barometer is read at. */
    uint32_t fast = ak_at32_i2c_clkctrl(144000000u, 400u);
    uint32_t slow = ak_at32_i2c_clkctrl(144000000u, 100u);

    expect("the timing register is found for 400 kHz on a 144 MHz bus",
           fast != 0u);
    expect("and the rate it asks for is the rate it was given",
           ak_at32_i2c_rate_khz(fast, 144000000u) > 380u &&
               ak_at32_i2c_rate_khz(fast, 144000000u) < 420u);
    expect("and for 100 kHz too",
           slow != 0u && ak_at32_i2c_rate_khz(slow, 144000000u) > 95u &&
               ak_at32_i2c_rate_khz(slow, 144000000u) < 110u);

    /*
     * The fields, which is what a register's width costs when it is exceeded:
     * the setup and hold delays must fit in four bits each, and the two halves
     * of the period in eight. At this clock the prescaler has to be searched
     * for - a formula that stopped at one would overflow the setup field, and
     * the part would take the low bits of a number nobody wrote.
     */
    expect("the setup and hold delays fit their four bits",
           ((fast >> AK_I2C_CLKCTRL_SCLD_SHIFT) & AK_I2C_CLKCTRL_FIELD_MASK) <=
                   15u &&
               ((fast >> AK_I2C_CLKCTRL_SDAD_SHIFT) &
                AK_I2C_CLKCTRL_FIELD_MASK) <= 15u);
    expect("and the prescaler that was found is a real one",
           (((fast >> AK_I2C_CLKCTRL_DIVH_SHIFT) & 0xFu) << 4 |
            ((fast >> AK_I2C_CLKCTRL_DIVL_SHIFT) & 0xFu)) >= 1u);
    expect("and a slower bus asks for a longer low period than a faster one",
           ((slow >> AK_I2C_CLKCTRL_SCLL_SHIFT) & 0xFFu) >
               ((fast >> AK_I2C_CLKCTRL_SCLL_SHIFT) & 0xFFu));
    printf("  i2c: 400 kHz at 144 MHz is 0x%08x, which decodes to %u kHz\n",
           fast, ak_at32_i2c_rate_khz(fast, 144000000u));

    /* And the configuration reaches the part: the pins open drain with a
     * pull-up, the clock bit, the timing register, and the peripheral on.
     *
     * The two registers are read back through the *model* rather than out of
     * this process's memory, because this driver's accesses all go through that
     * seam (see src/arch/at32f435/i2c.c): the pins are still checked where they
     * land, in the mapped block, since GPIO has no seam. */
    ak_i2c_init(TEST_I2C, TEST_I2C_SCL, TEST_I2C_SDA, TEST_I2C_AF, 400000u);

    expect("the bus is enabled with its filter set",
           (host_i2c_at32_ctrl1() & AK_I2C_CTRL1_I2CEN) != 0u &&
               ((host_i2c_at32_ctrl1() >> AK_I2C_CTRL1_DFLT_SHIFT) &
                AK_I2C_CTRL1_DFLT_MASK) != 0u);
    expect("and holds the timing register the arithmetic produced",
           host_i2c_at32_clkctrl() ==
               ak_at32_i2c_clkctrl(144000000u, 400u));
    expect("and both lines are open drain with a pull-up",
           (AK_GPIO_OMODE(GPIOH_BASE) & ((1u << 2) | (1u << 3))) ==
               ((1u << 2) | (1u << 3)) &&
               ((AK_GPIO_PULL(GPIOH_BASE) >> 4) & 0x3u) == AK_GPIO_PULL_UP &&
               ((AK_GPIO_PULL(GPIOH_BASE) >> 6) & 0x3u) == AK_GPIO_PULL_UP);
    expect("and I2C2's own clock bit is on in the APB1 register",
           (CRM_APB1EN & (1u << 22)) != 0u);
}

/*
 * And the barometer's *transfers*, against a modelled device.
 *
 * This is the half of the port that has no F405 counterpart, and not because it
 * is the same code with different register names: here the address, the
 * direction and the byte count are one write to ctrl2, the peripheral performs
 * the address phase and the acknowledge itself, and the count is what decides
 * how many bytes move before the stop goes out on its own. A driver written by
 * pattern-matching the F405's file would compile, run, and never move this bus.
 *
 * What the model can say: the order of the register writes, which flag the
 * driver waited for before each byte, that the count it programmed matches the
 * bytes it then moved, that an address nobody answers is noticed and cleared so
 * the next transfer works, and that a bus nobody enabled is a timeout rather
 * than a silent success. What it cannot: whether the silicon agrees, or what any
 * of it looks like on a scope.
 */
#define TEST_BARO_ADDRESS 0x77u /* the strap on this board's DPS310 */

static void test_the_barometers_transfers(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    host_i2c_at32_reset();
    ak_test_at32_clock_up();
    ak_clk_init(AK_BOARD_HEXT_HZ);
    ak_i2c_init(TEST_I2C, TEST_I2C_SCL, TEST_I2C_SDA, TEST_I2C_AF, 400000u);

    static const uint8_t device_registers[8] = {
        0x50u,                /* 0x00: who am i */
        0x01u,
        0x12u, 0x34u,         /* 0x02: a sixteen-bit coefficient */
        0xAAu, 0xBBu, 0xCCu,  /* 0x04: a burst of three */
    };
    int device = host_i2c_at32_attach(TEST_BARO_ADDRESS, device_registers,
                                      sizeof device_registers);
    expect("a device can be attached to the modelled bus", device >= 0);

    /* One byte: the sensor's identity, which is a register number, a repeated
     * start, and one byte with the count doing the acknowledging. */
    uint8_t value = 0u;
    expect("a one-byte read returns the register it named",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &value, 1u) == 0 &&
               value == 0x50u);
    expect("and the device saw its own address",
           host_i2c_at32_last_address() == TEST_BARO_ADDRESS &&
               host_i2c_at32_addressed((unsigned)device));
    expect("and exactly one byte moved",
           host_i2c_at32_last_read() == 1u);

    /* Two: the case the F405 has to handle with a POS bit, because there the
     * acknowledge has to move one byte early. Here it is the same code as any
     * other length, which is the point of the peripheral. */
    uint8_t pair[2] = { 0u, 0u };
    expect("a two-byte read returns both bytes in order",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x02u, pair, 2u) == 0 &&
               pair[0] == 0x12u && pair[1] == 0x34u);

    /* Three: the burst path, where the register pointer walks on its own and
     * the count is what stops it. */
    uint8_t burst[3] = { 0u, 0u, 0u };
    expect("a longer read walks the register pointer",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x04u, burst, 3u) == 0 &&
               burst[0] == 0xAAu && burst[1] == 0xBBu && burst[2] == 0xCCu);
    expect("and the count the part was given is the count that moved",
           host_i2c_at32_last_read() == 3u &&
               host_i2c_at32_last_register() == 0x07u);

    /* A write: the register number and the value are one two-byte transfer,
     * which is what the count is for. */
    expect("a write reaches the device's register",
           ak_i2c_write_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x01u, 0x5Au) == 0 &&
               host_i2c_at32_register((unsigned)device, 0x01u) == 0x5Au);
    expect("and the device was told which register",
           host_i2c_at32_last_written() == 2u);

    /* An address nobody answers: the driver has to notice, clear the flag and
     * leave the bus able to try again - which is the path a board with no
     * barometer fitted takes at every boot. */
    expect("an address nobody answers is an error, not a reading",
           ak_i2c_read_reg(TEST_I2C, 0x60u, 0x00u, &value, 1u) < 0);
    expect("and the bus still works afterwards",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &value, 1u) == 0 &&
               value == 0x50u);

    /* A peripheral that was never enabled does nothing at all, so every wait
     * in the driver runs out. This is the shape of a bring-up that forgot the
     * enable bit: not a wrong reading, no reading. */
    host_i2c_at32_write(TEST_I2C, AK_I2C_CTRL1_OFF, 0u);
    expect("a transfer on a bus nobody enabled is a timeout",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &value, 1u) < 0);
    ak_i2c_init(TEST_I2C, TEST_I2C_SCL, TEST_I2C_SDA, TEST_I2C_AF, 400000u);
    expect("and enabling it again is all it takes",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &value, 1u) == 0 &&
               value == 0x50u);

    /* The two edges of the count, which is eight bits on this part: a read of
     * nothing is nothing to do, and a read longer than the counter can hold is
     * refused rather than truncated - a truncated burst is a barometer whose
     * pressure is right and whose temperature is not, which is worse than an
     * error. */
    expect("a read of nothing is not a transfer",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &value, 0u) == 0);
    expect("and a read longer than the count register can hold is refused",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &value, 256u) < 0);

    /* And the same guard the F405 port grew: a base address that is not one of
     * the three controllers is refused before anything is written anywhere. */
    expect("a controller that is not one of the three is refused",
           ak_i2c_read_reg(ADC1_BASE, TEST_BARO_ADDRESS, 0x00u, &value, 1u) < 0 &&
               ak_i2c_write_reg(ADC1_BASE, TEST_BARO_ADDRESS, 0x00u, 0x5Au) < 0);
}

/*
 * The barometer's bus when it stops answering.
 *
 * This port makes the same promise as the F405's: every wait is bounded, and
 * reaching the bound ends the transfer properly - the stop condition, the
 * flags cleared, ctrl2 back to what it was - rather than leaving a frame
 * half-sent for the next start to be read as a continuation of. Until this
 * existed, every one of those `transfer_abort` paths had been compiled and
 * never run: the modelled bus always answered, and the one failure the tests
 * above reach is an address nobody acknowledges, which this part reports with
 * a flag of its own rather than a wait that runs out.
 *
 * So the model answers a chosen number of status *changes* and then goes quiet,
 * and this walks that number up. Zero is a peripheral that never comes up at
 * all; each step beyond it lets one more wait succeed before the bus dies, so
 * the address phase, each byte's flag and the stop that ends a transfer are
 * asked in turn. Every one of them has to come back as an error rather than a
 * hang, and has to leave a bus that works when it answers again - which is the
 * sensor coming back, or the next boot, or the second half of a flight after a
 * wire was knocked.
 */
static void test_the_barometer_bus_that_stops_answering(void)
{
    static const uint8_t device_registers[8] = {
        0x50u, 0x01u, 0x12u, 0x34u, 0xAAu, 0xBBu, 0xCCu, 0x00u,
    };
    unsigned died = 0u;
    unsigned answered = 0u;

    for (unsigned flags = 0u; flags <= 14u; flags++) {
        char what[120];
        uint8_t value = 0u;
        uint8_t pair[2] = { 0u, 0u };
        uint8_t burst[3] = { 0u, 0u, 0u };

        map_registers();
        memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
        host_i2c_at32_reset();
        ak_test_at32_clock_up();
        ak_clk_init(AK_BOARD_HEXT_HZ);
        host_i2c_at32_stall_after(flags);
        ak_i2c_init(TEST_I2C, TEST_I2C_SCL, TEST_I2C_SDA, TEST_I2C_AF, 400000u);
        (void)host_i2c_at32_attach(TEST_BARO_ADDRESS, device_registers,
                                   sizeof device_registers);

        snprintf(what, sizeof what,
                 "a bus that answers %u flag%s and stops is an error on a "
                 "one-byte read, not a hang",
                 flags, flags == 1u ? "" : "s");
        if (ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &value, 1u) <
            0) {
            died++;
            expect(what, 1);
        } else {
            /* Far enough along the sequence the transfer completes, and that
             * is the control this sweep needs: a read that could never succeed
             * would make every line above pass for the wrong reason. */
            answered++;
            expect("a bus that answers every flag gives the reading instead",
                   value == 0x50u);
        }

        /* The other two shapes of transfer and the write, each of which waits
         * somewhere else in the sequence - and the stall is re-armed before
         * each, because once a transfer has failed at a given budget every
         * *later* transfer on that bus fails at its first wait, so the deeper
         * waits would never be reached without it. */
        host_i2c_at32_stall_after(flags);
        snprintf(what, sizeof what,
                 "and a two-byte read with %u flag%s answered is an error or "
                 "the right pair",
                 flags, flags == 1u ? "" : "s");
        expect(what, ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x02u, pair,
                                     2u) < 0 ||
                         (pair[0] == 0x12u && pair[1] == 0x34u));
        host_i2c_at32_stall_after(flags);
        snprintf(what, sizeof what,
                 "and a burst of three with %u flag%s answered is an error or "
                 "the right bytes",
                 flags, flags == 1u ? "" : "s");
        expect(what, ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x04u, burst,
                                     3u) < 0 ||
                         (burst[0] == 0xAAu && burst[1] == 0xBBu &&
                          burst[2] == 0xCCu));
        host_i2c_at32_stall_after(flags);
        snprintf(what, sizeof what,
                 "and a write with %u flag%s answered is an error or the value "
                 "that arrived",
                 flags, flags == 1u ? "" : "s");
        expect(what, ak_i2c_write_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x01u,
                                      0x5Au) < 0 ||
                         host_i2c_at32_register(0u, 0x01u) == 0x5Au);

        /* And the promise that matters after the error: the transfer was
         * aborted on purpose, so a bus that answers again is usable - this is
         * the sensor coming back without a power cycle. */
        host_i2c_at32_stall_after(HOST_I2C_AT32_ANSWERS_ALL);
        value = 0u;
        snprintf(what, sizeof what, "and the bus works again after %u flag%s",
                 flags, flags == 1u ? "" : "s");
        expect(what, ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u,
                                     &value, 1u) == 0 &&
                         value == 0x50u);
    }

    expect("the sweep reached the sequence dying part-way, not only at once",
           died >= 3u);
    expect("and ended on a bus that answers the whole transfer",
           answered >= 1u);

    /* The other way a transfer does not happen: somebody else is holding the
     * bus. Nothing the driver can do about that either, so what it must do is
     * give up - and be able to start again once the bus is free. */
    host_i2c_at32_bus_busy(1);
    uint8_t stuck = 0u;
    expect("a bus somebody else is holding gives up on a read",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &stuck, 1u) < 0);
    expect("and on a write",
           ak_i2c_write_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x01u, 0x5Au) < 0);
    host_i2c_at32_bus_busy(0);
    expect("and the bus is usable again once it is let go",
           ak_i2c_read_reg(TEST_I2C, TEST_BARO_ADDRESS, 0x00u, &stuck, 1u) ==
                   0 &&
               stuck == 0x50u);

    /* This board's barometer is on I2C2; the other controller the part has is
     * the branch of the clock enable that no test had taken, and a port that
     * can only bring up the bus its own board happens to use is a port that
     * fails on the next board rather than here. */
    ak_i2c_init(I2C1_BASE, TEST_I2C_SCL, TEST_I2C_SDA, TEST_I2C_AF, 400000u);
    expect("the other controller's clock can be turned on",
           (CRM_APB1EN & (1u << 21)) != 0u);
    ak_i2c_init(I2C3_BASE, TEST_I2C_SCL, TEST_I2C_SDA, TEST_I2C_AF, 400000u);
    expect("and a base this part has no clock bit for is left alone",
           (CRM_APB1EN & (1u << 21)) != 0u &&
               (CRM_APB1EN & (1u << 22)) != 0u);
}

/*
 * The interrupt-driven receive, which is the half of a UART the console never
 * needed and the receiver and the GPS do: a byte that arrives between two
 * passes of a loop that is busy flying an aircraft. What is checked is the
 * whole path - the interrupt enable, the vector that points at the handler, the
 * ring the handler fills, and the counts the console reads back - because the
 * failure this catches is not a wrong number, it is a link that works on the
 * bench and loses frames in the air.
 */
static void test_the_receiver_ports(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    memset((void *)PRIVATE_BASE, 0, PRIVATE_SIZE);
    ak_test_at32_clock_up();
    ak_clk_init(AK_BOARD_HEXT_HZ);

    /* The receiver's port: USART2 on the pins the wing's board brings out -
     * through the two-function-number entry point, because on this part the
     * two pins of one port are two different numbers, and on this board they
     * are: PB0 listens with function 6, PA8 talks with function 8. A port
     * configured with one number for both is the bug the board file would have
     * had, and it looks exactly like a receiver that is not bound. */
    ak_uart_rx_init_af(USART2_BASE, AK_PIN(GPIOA_BASE, 8), 8u,
                       AK_PIN(GPIOB_BASE, 0), 6u, 420000u, 0, 0, 0);

    expect("each pin gets its own function number",
           (AK_GPIO_MUXH(GPIOA_BASE) & 0xFu) == 8u &&
               (AK_GPIO_MUXL(GPIOB_BASE) & 0xFu) == 6u);
    expect("and no peripheral pin swap on this port",
           (AK_USART_CTRL2(USART2_BASE) & AK_USART_CTRL2_TRPSWAP) == 0u);

    expect("the port interrupts when a byte arrives",
           (AK_USART_CTRL1(USART2_BASE) & AK_USART_CTRL1_RDBFIEN) != 0u);
    expect("and its interrupt is enabled in the controller",
           (AK_NVIC_ISER(AK_USART2_IRQ / 32u) &
            (1u << (AK_USART2_IRQ % 32u))) != 0u);

    /* A byte arrives: the flag is up and the data register holds it. The
     * handler is the one a real vector table points at, called the way the
     * part would call it. */
    uint8_t byte = 0;

    expect("a quiet port offers nothing",
           ak_uart_rx_pop(USART2_BASE, &byte) == 0);

    AK_USART_STS(USART2_BASE) = AK_USART_STS_RDBF;
    AK_USART_DT(USART2_BASE) = 0xC8u; /* a CRSF frame's first byte */
    USART2_IRQHandler();
    expect("an interrupt moves the byte into the ring",
           ak_uart_rx_pop(USART2_BASE, &byte) == 1 && byte == 0xC8u);

    /* And a second port keeps its own bytes: the GPS on USART3, at its own
     * rate. A single ring shared between them would interleave two protocols
     * into nonsense, which is the bug this check is for. */
    /* The GPS's port: the same entry point with the swap bit set, which is
     * what makes this firmware's transmit land on the pad the module is
     * listening to (see src/boards/AERIALKIT_GHF435/board.h). Both pins are
     * function 7 here - it is the peripheral that is swapped, not the pins. */
    ak_uart_rx_init_af(USART3_BASE, AK_PIN(GPIOB_BASE, 11), 7u,
                       AK_PIN(GPIOB_BASE, 10), 7u, 9600u, 0, 0, 1);

    expect("the gps port has the peripheral's pins swapped",
           (AK_USART_CTRL2(USART3_BASE) & AK_USART_CTRL2_TRPSWAP) != 0u);
    expect("and its two pins still take their own function number",
           ((AK_GPIO_MUXH(GPIOB_BASE) >> 12) & 0xFu) == 7u &&
               ((AK_GPIO_MUXH(GPIOB_BASE) >> 8) & 0xFu) == 7u);
    AK_USART_STS(USART3_BASE) = AK_USART_STS_RDBF;
    AK_USART_DT(USART3_BASE) = 0xB5u; /* a UBX frame's first byte */
    USART3_IRQHandler();

    uint8_t gps = 0;
    uint8_t rx = 0;

    expect("and the two ports keep their own bytes",
           ak_uart_rx_pop(USART3_BASE, &gps) == 1 && gps == 0xB5u &&
               ak_uart_rx_pop(USART2_BASE, &rx) == 0);

    /* An interrupt with nothing behind it must not invent a byte: the handler
     * runs whenever the interrupt fires, and a spurious one is a byte of
     * garbage in a protocol. */
    AK_USART_STS(USART2_BASE) = 0u;
    USART2_IRQHandler();
    expect("an interrupt with no byte behind it adds none",
           ak_uart_rx_pop(USART2_BASE, &rx) == 0);

    /* And what the ring cannot keep is counted rather than lost silently. */
    for (unsigned i = 0; i < 300u; i++) {
        AK_USART_STS(USART2_BASE) = AK_USART_STS_RDBF;
        AK_USART_DT(USART2_BASE) = (uint32_t)i;
        USART2_IRQHandler();
    }
    expect("a port that outruns the loop counts what it dropped",
           ak_uart_rx_dropped(USART2_BASE) > 0u);

    /* And the frame a receiver that is not CRSF speaks: SBUS is 100000 baud,
     * even parity, two stop bits, on the same wire. The wrapper sets the two
     * bits the simple entry point leaves clear, which is the whole of the
     * difference - and the check is here because a receiver on the wrong frame
     * looks exactly like a receiver on the wrong baud rate. */
    ak_uart_rx_init_format(USART2_BASE, AK_PIN(GPIOA_BASE, 8),
                           AK_PIN(GPIOB_BASE, 0), 100000u, 6u, 1, 1);

    expect("sbus is even parity",
           (AK_USART_CTRL1(USART2_BASE) & AK_USART_CTRL1_PEN) != 0u);
    expect("and two stop bits",
           ((AK_USART_CTRL2(USART2_BASE) >> AK_USART_CTRL2_STOP_SHIFT) &
            AK_USART_CTRL2_STOP_MASK) == AK_USART_CTRL2_STOP_2);
}

/*
 * The pack's ADC. Two things are this part's rather than the F4's: the clock
 * prescaler lives in a *common* block and divides the AHB clock (so at 288 MHz
 * the value to ask for is a divide by eight, not the F4's divide by four of a
 * 42 MHz bus), and the sequence and sample-time registers have this part's
 * names. What is checked is the configuration, the calibration sequence, a
 * conversion that answers, and the two ways one does not.
 */
#define TEST_ADC_BASE ADC1_BASE
#define TEST_ADC_CH   1u /* the pack's channel on this board */

static void test_the_pack(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);

    ak_adc_init(TEST_ADC_BASE, TEST_ADC_CH);

    expect("the pack's channel is enabled with its clock",
           (AK_ADC_CTRL2(TEST_ADC_BASE) & AK_ADC_CTRL2_ADCEN) != 0u &&
               (CRM_APB2EN & (1u << 8)) != 0u);
    expect("and the ADC is clocked at a rate it is rated for",
           ((AK_ADCCOM_CTRL >> AK_ADCCOM_CTRL_DIV_SHIFT) &
            AK_ADCCOM_CTRL_DIV_MASK) == AK_ADC_HCLK_DIV_8);
    expect("with one conversion in the sequence, on the channel it was given",
           ((AK_ADC_CTRL1(TEST_ADC_BASE) >> AK_ADC_CTRL1_OCPCNT_SHIFT) &
            AK_ADC_CTRL1_OCPCNT_MASK) == 1u &&
               (AK_ADC_OSQ3(TEST_ADC_BASE) & 0x1Fu) == TEST_ADC_CH);
    expect("and the longest sample time that channel has",
           ((AK_ADC_SPT1(TEST_ADC_BASE) >> AK_ADC_SPT_SHIFT(TEST_ADC_CH)) &
            AK_ADC_SPT_MASK) == AK_ADC_SPT_LONGEST);

    /* The calibration bits are cleared by the part when each step is done, so
     * a mapped register that never clears them is a part that never finished -
     * which the init must survive rather than hang in. */
    expect("the calibration was asked for and did not hang the init",
           (AK_ADC_CTRL2(TEST_ADC_BASE) &
            (AK_ADC_CTRL2_ADCAL | AK_ADC_CTRL2_ADCALINIT)) ==
                   (AK_ADC_CTRL2_ADCAL | AK_ADC_CTRL2_ADCALINIT));

    /*
     * A conversion that finishes - which the model raises the flag for, since a
     * mapped register cannot - and one that never does. The first is the path
     * this board's pack voltage is read on; the second is what a converter that
     * is not running looks like, and it must be a timeout rather than the last
     * value left in the register.
     */
    uint16_t counts = 0u;

    AK_ADC_ODT(TEST_ADC_BASE) = 2000u;
    expect("a finished conversion is read from the data register",
           ak_adc_read_counts(TEST_ADC_BASE, &counts) == 0 && counts == 2000u);
    host_f4adc_set_never_finishes(1);
    expect("a converter that never finishes gives up rather than lying",
           ak_adc_read_counts(TEST_ADC_BASE, &counts) != 0);
    host_f4adc_set_never_finishes(0);

    expect("and an ADC this board does not have is refused",
           ak_adc_read_counts(0u, &counts) != 0);

    /* And a channel on the other side of the split between the two sample-time
     * registers: channel seven is in the same register as the pack's, and the
     * one just above the split is in the other. */
    ak_adc_init(TEST_ADC_BASE, 3u);
    expect("a channel below the split is configured in the first register",
           (AK_ADC_SPT1(TEST_ADC_BASE) >> (AK_ADC_SPT_SHIFT(3u))) != 0u);
    ak_adc_init(TEST_ADC_BASE, 12u);
    expect("and one above it in the second",
           (AK_ADC_SPT2(TEST_ADC_BASE) >> (AK_ADC_SPT_SHIFT(12u))) != 0u);
}

/*
 * Motors and servos - the piece that makes this port an aircraft. What a
 * register file can check is what the timers and the DMA were told to do, and
 * that is more than it sounds: the bit rate is the timer clock over the bit
 * rate the parameter asks for, the compare values come from the portable
 * encoder, the first bit is loaded by hand and the DMA carries the rest, and
 * the request multiplexer has to be told to use its table before the request id
 * means anything. What it cannot check is a waveform: that part is a scope.
 */
static void test_the_outputs(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    memset((void *)PRIVATE_BASE, 0, PRIVATE_SIZE);
    ak_test_at32_clock_up();
    ak_clk_init(AK_BOARD_HEXT_HZ);
    ak_output_init();

    /* --- the servos: 1 us of resolution, a 20 ms period, centred --- */
    expect("the servo timer counts microseconds",
           AK_TMR_DIV(TMR2_BASE) == (288000000u / 1000000u) - 1u &&
               AK_TMR_PR(TMR2_BASE) == 20000u - 1u);
    expect("and both servos start centred",
           AK_TMR_C1DT(TMR2_BASE) == AK_SERVO_CENTER_US &&
               AK_TMR_C2DT(TMR2_BASE) == AK_SERVO_CENTER_US);
    expect("in PWM mode with the preload on, and both channels enabled",
           ((AK_TMR_CCM1(TMR2_BASE) >> 4) & 0x7u) == AK_TMR_CCM_MODE_PWM_A &&
               (AK_TMR_CCM1(TMR2_BASE) & AK_TMR_CCM_PRELOAD(1u)) != 0u &&
               (AK_TMR_CCE(TMR2_BASE) & (AK_TMR_CCE_C1EN | AK_TMR_CCE_C2EN)) ==
                   (AK_TMR_CCE_C1EN | AK_TMR_CCE_C2EN));
    expect("and the timer is running",
           (AK_TMR_CTRL1(TMR2_BASE) & AK_TMR_CTRL1_TMREN) != 0u);
    /* PB8 and PB9 are the *high* mux register's business: the split is at pin
     * eight, which is the same trap the SPI and I2C configuration has. */
    expect("and the servo pins are on TMR2's mux",
           (AK_GPIO_MUXH(GPIOB_BASE) & 0xFu) == AK_GPIO_MUX_1 &&
               ((AK_GPIO_MUXH(GPIOB_BASE) >> 4) & 0xFu) == AK_GPIO_MUX_1);

    /* --- the motors: one timer period per bit at 288 MHz --- */
    ak_output_set_rate(300u);
    expect("DShot300 is 960 ticks of a 288 MHz timer",
           ak_output_dshot_period() == 960u - 1u);
    expect("and the compare values are the encoder's, in order and inside the "
           "period",
           ak_output_ccr_zero() > 0u &&
               ak_output_ccr_zero() < ak_output_ccr_one() &&
               ak_output_ccr_one() < 960u);
    ak_output_set_rate(600u);
    expect("and DShot600 is half of that", ak_output_dshot_period() == 480u - 1u);
    ak_output_set_rate(99u); /* a rate this firmware does not do */
    expect("and a rate nobody supports changes nothing",
           ak_output_dshot_period() == 480u - 1u);
    ak_output_set_rate(300u);

    expect("the motor pins are on TMR4's mux",
           (AK_GPIO_MUXL(GPIOB_BASE) >> 24 & 0xFu) == AK_GPIO_MUX_2 &&
               (AK_GPIO_MUXL(GPIOB_BASE) >> 28 & 0xFu) == AK_GPIO_MUX_2);
    expect("and both compare channels ask the DMA for their own events",
           (AK_TMR_IDEN(TMR4_BASE) & (AK_TMR_IDEN_C1DEN |
                                      (AK_TMR_IDEN_C1DEN << 1u))) ==
               (AK_TMR_IDEN_C1DEN | (AK_TMR_IDEN_C1DEN << 1u)));

    /* --- the DMA channels: one per motor, through the request multiplexer --- */
    expect("the first channel moves halfwords from memory to the timer",
           (AK_DMA_CH_CTRL(DMA1_BASE, 1u) &
            (AK_DMA_CTRL_DTD | AK_DMA_CTRL_MINC | AK_DMA_CTRL_FDTIEN)) ==
               (AK_DMA_CTRL_DTD | AK_DMA_CTRL_MINC | AK_DMA_CTRL_FDTIEN) &&
               ((AK_DMA_CH_CTRL(DMA1_BASE, 1u) >> AK_DMA_CTRL_MWIDTH_SHIFT) &
                0x3u) == AK_DMA_WIDTH_HALFWORD);
    expect("and writes the first motor's compare register",
           AK_DMA_CH_PADDR(DMA1_BASE, 1u) ==
               (uint32_t)(uintptr_t)&AK_TMR_C1DT(TMR4_BASE));
    expect("and carries the frame from the second group on",
           AK_DMA_CH_DTCNT(DMA1_BASE, 1u) == AK_DSHOT_GROUPS - 1u &&
               AK_DMA_CH_MADDR(DMA1_BASE, 1u) != 0u);
    expect("and the multiplexer is in use, with the timer's own request",
           (AK_DMA_MUXSEL(DMA1_BASE) & 1u) != 0u &&
               (AK_DMAMUX_CH(DMA1_BASE, 1u) & AK_DMAMUX_REQSEL_MASK) ==
                   AK_DMAMUX_REQ_TMR4_CH1 &&
               (AK_DMAMUX_CH(DMA1_BASE, 2u) & AK_DMAMUX_REQSEL_MASK) ==
                   AK_DMAMUX_REQ_TMR4_CH2);
    expect("and both channels interrupt when they finish",
           (AK_NVIC_ISER(AK_DMA1_CH1_IRQ / 32u) &
            ((1u << (AK_DMA1_CH1_IRQ % 32u)) |
             (1u << (AK_DMA1_CH2_IRQ % 32u)))) ==
               ((1u << (AK_DMA1_CH1_IRQ % 32u)) |
                (1u << (AK_DMA1_CH2_IRQ % 32u))));

    /* --- a frame on the wire --- */
    ak_output_frame_t frame;

    /* Two frames whose first bits differ, so "the first bit is loaded by hand"
     * is a claim with two answers in it. */
    frame.dshot[0] = 0x8000u; /* the top bit set: a '1' */
    frame.dshot[1] = 0x1234u; /* the top bit clear: a '0' */
    frame.servo_us[0] = 1600u;
    frame.servo_us[1] = 1400u;
    ak_output_write(&frame);

    expect("the servos take the pulse widths the frame asked for",
           AK_TMR_C1DT(TMR2_BASE) == 1600u && AK_TMR_C2DT(TMR2_BASE) == 1400u);
    expect("and the first bit of each motor's frame is loaded by hand",
           AK_TMR_C1DT(TMR4_BASE) == ak_output_ccr_one() &&
               AK_TMR_C2DT(TMR4_BASE) == ak_output_ccr_zero());
    expect("and both DMA channels are running",
           ak_output_busy() &&
               (AK_DMA_CH_CTRL(DMA1_BASE, 1u) & AK_DMA_CTRL_CHEN) != 0u &&
               (AK_DMA_CH_CTRL(DMA1_BASE, 2u) & AK_DMA_CTRL_CHEN) != 0u);

    /* A frame that arrives while the last one is still going out is skipped and
     * counted, rather than corrupting both. */
    ak_output_write(&frame);
    expect("a frame that arrives mid-frame is counted, not sent",
           ak_output_frames_skipped() == 1u && ak_output_frames_sent() == 0u);

    /* And the frame is over when both channels have finished - not when one
     * has, which is the bug that a single interrupt would hide. */
    uint32_t skipped_before = ak_output_frames_skipped();

    DMA1_Channel1_IRQHandler();
    expect("one channel finishing does not end the frame",
           ak_output_busy() && (AK_DMA_CH_CTRL(DMA1_BASE, 1u) &
                                AK_DMA_CTRL_CHEN) == 0u);
    DMA1_Channel2_IRQHandler();
    expect("and both finishing does",
           !ak_output_busy() && ak_output_frames_sent() == 1u &&
               ak_output_frames_skipped() == skipped_before);

    /* The planes the DMA walks are the frame transposed: motor 1's first bit
     * is the encoder's first entry for motor 1, not for motor 2. */
    expect("and the plane a channel carries is that motor's own bits",
           AK_DMA_CH_MADDR(DMA1_BASE, 1u) != AK_DMA_CH_MADDR(DMA1_BASE, 2u));
}

/*
 * The USB controller, which on this part is the same core the F405 has - at the
 * same address, with the same registers and the same bits - and three registers
 * that are not: the core starts in power-down, the phy clock is gated, and the
 * field at GUSBCFG bit 6 is the turn-around time here rather than a transceiver
 * select. Those three are what this file checks that the F405's cannot, and the
 * rest runs the same way for the same reason: a mapped page is a word rather
 * than a queue, but it is the *status entry* that says whose packet it is and
 * how long, so a page is enough to drive the whole driver.
 *
 * What this is not: a bus. There is no timing, no DATA0/DATA1 toggle, no retry
 * when the device NAKs, and the core's own side of the contract - that reading
 * the bytes is what drops the entry, that EPENA clears when a transfer
 * completes - is written here as the test's premise rather than observed.
 */
#define TEST_USB_DM AK_PIN(GPIOA_BASE, 11)
#define TEST_USB_DP AK_PIN(GPIOA_BASE, 12)

/* One entry in the receive status queue: the status word in GRXSTSP, the bytes
 * in FIFO 0, and the flag that says there is something to read. The words the
 * bytes make go into the modelled FIFO first, because that order is part of the
 * protocol (tests/host_usb_model.h). */
static void usb_receive_words(unsigned endpoint, unsigned kind,
                              const uint8_t *bytes, unsigned count)
{
    unsigned words = (count + 3u) / 4u;

    for (unsigned at = 0; at < words; at++) {
        uint32_t word = 0u;

        for (unsigned byte = 0; byte < 4u; byte++) {
            unsigned index = at * 4u + byte;

            if (index < count) {
                word |= (uint32_t)bytes[index] << (8u * byte);
            }
        }
        host_usb_push(word);
    }
    OTG_GRXSTSP = endpoint | (count << 4) | (kind << 17);
    OTG_GINTSTS |= OTG_GINT_RXFLVL;
}

/* Four bytes or fewer, which is one FIFO word. */
static void usb_receive(unsigned endpoint, unsigned kind, uint32_t payload,
                        unsigned count)
{
    host_usb_push(payload);
    OTG_GRXSTSP = endpoint | (count << 4) | (kind << 17);
    OTG_GINTSTS |= OTG_GINT_RXFLVL;
}

/* The host took a packet from an IN endpoint: on the part the transfer-complete
 * interrupt arrives with the endpoint already disabled, and a page of memory
 * has to be told that - and that the register clears, which on the part is what
 * writing the bit back does. */
static void usb_take_in_packet(unsigned endpoint)
{
    OTG_DIEPCTL(endpoint) &= ~OTG_DEPCTL_EPENA;
    OTG_DIEPINT(endpoint) |= OTG_DEPINT_XFRC;
    ak_usb_poll();
    OTG_DIEPINT(endpoint) = 0u;
}

/* On the part, reading the entry is what clears the flag. A page of memory has
 * nothing to drain, so the test says so, once, after each poll that consumed
 * one. */
static void usb_consumed(void)
{
    OTG_GINTSTS &= ~OTG_GINT_RXFLVL;
}

/*
 * A real setup packet: USB 2.0 section 9.3's eight bytes in the order a host
 * puts them on the wire - bmRequestType, bRequest, wValue, wIndex, wLength -
 * queued as the two words the FIFO hands back. The F405's test has the whole
 * story of what this used to be and what it cost; the short version is that a
 * packet with the request in the bmRequestType byte is a packet no host sends,
 * and both ports stalled every real one.
 */
/* Set when a setup packet was answered before the core closed its setup
 * stage - the race the driver used to run. */
static int usb_answered_early;

static void usb_setup_packet(unsigned type, unsigned request, unsigned value,
                             unsigned index, unsigned length)
{
    uint8_t bytes[8];

    bytes[0] = (uint8_t)type;
    bytes[1] = (uint8_t)request;
    bytes[2] = (uint8_t)(value & 0xFFu);
    bytes[3] = (uint8_t)((value >> 8) & 0xFFu);
    bytes[4] = (uint8_t)(index & 0xFFu);
    bytes[5] = (uint8_t)((index >> 8) & 0xFFu);
    bytes[6] = (uint8_t)(length & 0xFFu);
    bytes[7] = (uint8_t)((length >> 8) & 0xFFu);

    /* The two entries a real core queues for one setup packet, and STUP only
     * after the second: the request must not be answered in between. */
    OTG_DIEPCTL0 &= ~OTG_DEPCTL_EPENA;
    usb_receive_words(0u, OTG_GRXSTS_SETUP, bytes, 8u);
    ak_usb_poll();
    usb_consumed();
    if ((OTG_DIEPCTL0 & (OTG_DEPCTL_EPENA | OTG_DEPCTL_STALL)) != 0u) {
        usb_answered_early = 1;
    }
    usb_receive_words(0u, OTG_GRXSTS_SETUP_DONE, bytes, 0u);
    OTG_DOEPINT0 |= OTG_DEPINT_STUP;
    ak_usb_poll();
    usb_consumed();
    OTG_DOEPINT0 &= ~OTG_DEPINT_STUP;
}

static void test_the_usb_console_path(void)
{
    map_registers();
    ak_test_at32_clock_up();
    ak_clk_init(AK_BOARD_HEXT_HZ);
    host_usb_reset();
    ak_usb_init(TEST_USB_DM, TEST_USB_DP);

    /* The clock, which is this part's own arrangement and the first thing that
     * can be wrong: the USB clock comes from the PLL rather than from the
     * crystal, divided by six at 288 MHz to land on the 48 MHz a full-speed
     * device needs. A wrong divider is a device that does not appear at all. */
    expect("the usb clock is taken from the pll, not the internal clock",
           (CRM_MISC1 & CRM_MISC1_HICK_TO_USB) == 0u);
    expect("and divided by six, which is 48 MHz out of 288",
           ((CRM_MISC2 >> CRM_MISC2_USBDIV_SHIFT) & CRM_MISC2_USBDIV_MASK) ==
               (uint32_t)CRM_USBDIV_6);
    expect("the controller's own clock is on in the second ahb enable",
           (CRM_AHBEN2 & (1u << 7)) != 0u);

    /* PA11 and PA12 are above pin 7, so their function number lives in the high
     * mux register: nibble (pin - 8). Function 10 is the OTG FS transceiver;
     * anything else is a device that never appears on the bus. */
    expect("both data pins are alternate function 10",
           ((AK_GPIO_CFGR(GPIOA_BASE) >> (11u * 2u)) & 0x3u) ==
                   AK_GPIO_MODE_MUX &&
               ((AK_GPIO_CFGR(GPIOA_BASE) >> (12u * 2u)) & 0x3u) ==
                   AK_GPIO_MODE_MUX &&
               ((AK_GPIO_MUXH(GPIOA_BASE) >> ((11u - 8u) * 4u)) & 0xFu) ==
                   10u &&
               ((AK_GPIO_MUXH(GPIOA_BASE) >> ((12u - 8u) * 4u)) & 0xFu) ==
                   10u);
    expect("and both are on the stronger driver, which usb wants",
           ((AK_GPIO_ODRVR(GPIOA_BASE) >> (11u * 2u)) & 0x3u) ==
                   AK_GPIO_DRIVE_STRONGER &&
               ((AK_GPIO_ODRVR(GPIOA_BASE) >> (12u * 2u)) & 0x3u) ==
                   AK_GPIO_DRIVE_STRONGER);

    /* The three that are this part's. Each one is a core that does nothing at
     * all: still in power-down, or with its phy clock gated, or with a
     * turn-around time of zero because the F405's PHYSEL bit was written
     * instead. */
    expect("the core is out of power-down", (OTG_GCCFG & OTG_GCCFG_PWRDOWN) != 0u);
    /* And not waiting for a VBUS sense line the board may not wire: the F405's
     * core needed both bits before a host could see it at all (2026-09-17). */
    expect("and is not waiting for VBUS sensing",
           (OTG_GCCFG & OTG_GCCFG_NOVBUSSENS) != 0u);
    expect("and its phy clock is open, not gated",
           (OTG_FS_PCGCCTL & OTG_PCGCCTL_STOPPCLK) == 0u);
    expect("the turn-around time is the reference's five for 48 MHz",
           ((OTG_GUSBCFG >> 10) & 0xFu) == OTG_GUSBCFG_TRDTIM_48MHZ);
    expect("and the core is forced into device mode",
           (OTG_GUSBCFG & OTG_GUSBCFG_FDMOD) != 0u);
    /*
     * The soft disconnect: SDIS high holds D+ down, so the device is not on the
     * bus at all. Init leaves it off and the first poll puts it on, because the
     * poll is the first moment anything drives the controller - see the note at
     * the end of `ak_usb_init()` in this port. The F405's bench session of
     * 2026-09-20 is where that came from; this port is changed to match, and it
     * has never been flashed.
     *
     * This assertion required the opposite until then, and so stated the defect
     * as a requirement.
     */
    expect("the device is held off the bus until something drives it",
           (OTG_DCTL & OTG_DCTL_SDIS) != 0u);
    ak_usb_poll();
    expect("and the first poll is what puts it on the bus",
           (OTG_DCTL & OTG_DCTL_SDIS) == 0u);

    /* The FIFOs, in words: 320 is all this part's controller has too, and going
     * over it is a device that misbehaves in ways that look like anything but a
     * FIFO. */
    unsigned rx_words = OTG_GRXFSIZ;
    unsigned ep0_words = (OTG_DIEPTXF(0u) >> 16) & 0xFFFFu;
    unsigned ep1_words = (OTG_DIEPTXF(1u) >> 16) & 0xFFFFu;
    unsigned ep2_words = (OTG_DIEPTXF(2u) >> 16) & 0xFFFFu;
    expect("the receive fifo is 128 words, as the budget says",
           rx_words == 128u);
    expect("and the four fifos fit the 320 words the controller has",
           rx_words + ep0_words + ep1_words + ep2_words <= 320u);
    expect("each transmit fifo starts where the one below it ends",
           (OTG_DIEPTXF(0u) & 0xFFFFu) == rx_words &&
               (OTG_DIEPTXF(1u) & 0xFFFFu) == rx_words + ep0_words &&
               (OTG_DIEPTXF(2u) & 0xFFFFu) ==
                   rx_words + ep0_words + ep1_words);

    /*
     * And where those registers actually are, checked **at the address** rather
     * than through the symbol - because the three checks above read back through
     * the same macro the driver writes through, so both sides move together when
     * the macro is wrong and the suite stays green. That is exactly what
     * happened: `OTG_DIEPTXF(0u)` used to resolve to 0x100, the *host* periodic
     * fifo register, so endpoint 0's transmit fifo was never configured at all.
     *
     * Artery's own SVD puts endpoint 0's at 0x028 - it names that address
     * `DIEPTXF0`, "IN Endpoint TxFIFO 0 transmit FIFO size register", as well as
     * `GNPTXFSIZ` - and `HPTXFSIZ` at 0x100, with `DIEPTXF1` at 0x104.
     * (`upstream/inav-9.1.0/dev/svd/AT32F437xx_v2.svd`, `USB_OTG1_GLOBAL`.)
     */
    expect("endpoint 0's transmit fifo is written where the core keeps it",
           AK_REG32(USB_BASE + 0x028u) == ((16u << 16) | 128u));
    expect("and the host's periodic fifo register is left alone",
           AK_REG32(USB_BASE + 0x100u) == 0u);

    /* Endpoint 0 armed for setup packets, with STUPCNT written: a device that
     * never sets it answers the first descriptor request and goes deaf, which on
     * a bench looks like a cable. */
    expect("endpoint 0's out side is enabled, 64-byte packets, control",
           (OTG_DOEPCTL0 & (OTG_DEPCTL_EPENA | OTG_DEPCTL_CNAK |
                            OTG_DEPCTL_USBAEP | OTG_DEPCTL_MPS(0x7FFu))) ==
               (OTG_DEPCTL_EPENA | OTG_DEPCTL_CNAK | OTG_DEPCTL_USBAEP |
                OTG_DEPCTL_MPS(64u)));
    expect("and it is armed to hand over three setup packets",
           (OTG_DOEPTSIZ0 & OTG_DOEPTSIZ0_STUPCNT(3u)) != 0u &&
               (OTG_DOEPTSIZ0 & OTG_DOEPTSIZ0_PKTCNT1) != 0u);

    /* The banner, written before any host has configured the device: kept, not
     * sent, and not lost. */
    unsigned queued = ak_usb_write("AerialKit\r\n", 11u);
    expect("a banner written before enumeration is kept whole",
           queued == 11u && ak_usb_dropped() == 0u);
    expect("and nothing is pushed for a host that is not listening yet",
           ak_usb_ready() == 0 &&
               (OTG_DIEPCTL(1u) & OTG_DEPCTL_EPENA) == 0u &&
               OTG_DIEPTSIZ(1u) == 0u);

    /* A request nobody implements is stalled rather than ignored, which is how
     * a device says "not me" without hanging the bus. */
    usb_setup_packet(0x80u, 0xFFu, 0u, 0u, 0u); /* a request nothing implements */
    expect("a request that is not understood is stalled",
           (OTG_DIEPCTL0 & OTG_DEPCTL_STALL) != 0u &&
               (OTG_DOEPCTL0 & OTG_DEPCTL_STALL) != 0u);
    OTG_DIEPCTL0 &= ~OTG_DEPCTL_STALL;
    OTG_DOEPCTL0 &= ~OTG_DEPCTL_STALL;

    usb_setup_packet(0x80u, 0x06u, 0x0100u, 0u, 64u); /* GET_DESCRIPTOR, device */
    expect("no request is answered before the core closes the setup stage",
           usb_answered_early == 0);
    expect("the device descriptor goes out as eighteen bytes",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 18u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);
    expect("and its last two bytes are the ones the host reads last",
           OTG_FIFO(0u) == 0x00000103u);

    /*
     * The configuration descriptor is 67 bytes on this port too, so it needs
     * two packets - and the second one is where both ports had the same bug:
     * the continuation sent nothing, from past the end of the one packet the
     * driver keeps. A host that cannot read this descriptor cannot configure
     * the device, which on this board means no console at all.
     */
    usb_setup_packet(0x80u, 0x06u, 0x0200u, 0u, 255u); /* configuration */
    expect("a descriptor longer than a packet starts with a full one",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 64u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);

    OTG_DIEPINT0 |= OTG_DEPINT_XFRC;
    ak_usb_poll();
    expect("and the rest of it follows in the next packet",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 3u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);
    expect("carrying the end of the last endpoint descriptor",
           OTG_FIFO(0u) == 0x000040u);

    usb_setup_packet(0x80u, 0x06u, 0x0301u, 0u, 255u); /* string 1 */
    expect("a descriptor that fits is sent in one packet, not continued",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 20u);

    /* SET_ADDRESS, which every host sends before it asks for anything else -
     * and which this test had never sent either. What has to happen is one
     * write to the device's address field; the failure it prevents is a device
     * that answers at address 0 for ever, which the host gives up on with
     * "Device not responding to setup address". */
    usb_setup_packet(0x00u, 0x05u, 7u, 0u, 0u); /* SET_ADDRESS(7) */
    expect("the address the host gives the device is the one it answers at",
           (OTG_DCFG & OTG_DCFG_DAD(0x7Fu)) == OTG_DCFG_DAD(7u));
    expect("and the request is acknowledged with a zero-length packet",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 0u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);

    /* SET_CONFIGURATION(1): the host has a driver, and both bulk endpoints are
     * armed - the console's output above all, which is what the banner has been
     * waiting for. */
    usb_setup_packet(0x00u, 0x09u, 1u, 0u, 0u); /* SET_CONFIGURATION(1) */
    expect("the host configuring the device is noticed", ak_usb_ready() == 1);
    expect("the bulk out endpoint is armed, 64-byte packets",
           (OTG_DOEPCTL(2u) & (OTG_DEPCTL_EPENA | OTG_DEPCTL_USBAEP |
                               OTG_DEPCTL_EPTYP_BULK |
                               OTG_DEPCTL_MPS(0x7FFu))) ==
               (OTG_DEPCTL_EPENA | OTG_DEPCTL_USBAEP | OTG_DEPCTL_EPTYP_BULK |
                OTG_DEPCTL_MPS(64u)));
    expect("and the banner that was waiting goes out on the bulk in endpoint",
           (OTG_DIEPCTL(1u) & OTG_DEPCTL_EPENA) != 0u &&
               (OTG_DIEPTSIZ(1u) & 0x7FFFFu) == 11u);

    /* Typing: two packets on the bulk OUT endpoint, read back in the order they
     * arrived - which is the whole point of the ring. */
    usb_receive(2u, OTG_GRXSTS_DATA, 0x0A6B61u, 3u); /* "ak\n" */
    ak_usb_poll();
    usb_consumed();
    usb_receive(2u, OTG_GRXSTS_DATA, 0x74697571u, 4u); /* "quit" */
    ak_usb_poll();
    usb_consumed();

    char typed[8] = {0};
    unsigned got = 0u;
    while (got < sizeof typed && ak_usb_read(&typed[got])) {
        got++;
    }
    expect("what was typed arrives in the order it was typed",
           got == 7u && memcmp(typed, "ak\nquit", 7u) == 0);
    expect("and the console's reader is empty once it has been read",
           ak_usb_read(&typed[0]) == 0);
    expect("with nothing counted as dropped", ak_usb_rx_dropped() == 0u);

    /* A packet for an endpoint this device does not use is read out of the FIFO
     * - it has to be, or the queue fills - and then goes nowhere. */
    usb_receive(1u, OTG_GRXSTS_DATA, 0x41414141u, 4u);
    ak_usb_poll();
    usb_consumed();
    expect("a packet for somebody else's endpoint is not console input",
           ak_usb_read(&typed[0]) == 0);

    /* And the bulk OUT endpoint's own completion, which is what arms it for the
     * next packet. */
    OTG_DOEPCTL(2u) &= ~OTG_DEPCTL_EPENA;
    OTG_DOEPINT(2u) |= OTG_DEPINT_XFRC;
    ak_usb_poll();
    expect("a completed bulk out packet arms the endpoint for the next one",
           (OTG_DOEPCTL(2u) & (OTG_DEPCTL_EPENA | OTG_DEPCTL_CNAK)) ==
               (OTG_DEPCTL_EPENA | OTG_DEPCTL_CNAK) &&
               (OTG_DOEPTSIZ(2u) & 0x7FFFFu) == 64u);
    OTG_DOEPINT(2u) = 0u;

    /* And the ring's own limit: five 64-byte packets is more than it holds, and
     * what does not fit is counted rather than silently lost. */
    uint32_t before_dropped = ak_usb_rx_dropped();
    for (unsigned packet = 0; packet < 5u; packet++) {
        uint8_t full[64];

        memset(full, 0x41, sizeof full);
        usb_receive_words(2u, OTG_GRXSTS_DATA, full, sizeof full);
        ak_usb_poll();
        usb_consumed();
    }
    unsigned drained = 0u;
    while (ak_usb_read(&typed[0])) {
        drained++;
    }
    expect("a burst longer than the ring keeps what fits and counts the rest",
           drained == 255u && ak_usb_rx_dropped() == before_dropped + 65u);

    /*
     * The way out, which had the same hole as the way in on this part too: the
     * console's ring is two kilobytes and the endpoint carries 64 bytes at a
     * time, so a long line - `params` is one - is a sequence of packets that
     * only continues when the host acknowledges the one before it.
     */
    {
        static char flood[2200];

        usb_take_in_packet(1u); /* the banner, which the host has taken */

        memset(flood, '.', sizeof flood);
        unsigned before_tx_dropped = ak_usb_dropped();

        (void)ak_usb_write(flood, sizeof flood);
        expect("a line longer than one packet goes out as a full packet first",
               (OTG_DIEPTSIZ(1u) & 0x7FFFFu) == 64u &&
                   (OTG_DIEPCTL(1u) & OTG_DEPCTL_EPENA) != 0u);
        expect("and a write longer than the ring counts what it could not keep",
               ak_usb_dropped() > before_tx_dropped);

        usb_take_in_packet(1u);
        expect("and the next packet follows when the host takes the first",
               (OTG_DIEPTSIZ(1u) & 0x7FFFFu) == 64u &&
                   (OTG_DIEPCTL(1u) & OTG_DEPCTL_EPENA) != 0u);
    }

    /* A status stage the host completes, and the interrupt bits the core sets
     * for its own reasons - each of them a line in the poll that had never
     * run. */
    OTG_DOEPINT0 |= OTG_DEPINT_XFRC;
    ak_usb_poll();
    expect("a status stage the host completes arms endpoint 0 again",
           (OTG_DOEPCTL0 & (OTG_DEPCTL_EPENA | OTG_DEPCTL_CNAK)) ==
               (OTG_DEPCTL_EPENA | OTG_DEPCTL_CNAK));
    OTG_DOEPINT0 = 0u;

    OTG_DOEPINT0 |= OTG_DEPINT_STUP;
    ak_usb_poll();
    /* What the setup-phase line *clears* is not checkable here: on the part
     * writing the bit back is what clears it, and a page of memory has no such
     * rule. */
    expect("the setup-phase interrupt is handled without disturbing anything",
           ak_usb_ready() == 1u);
    OTG_DOEPINT0 = 0u;

    /* Cleared first so the handler below is the only thing that can set it. */
    OTG_DCTL &= ~OTG_DCTL_CGNPINNAK;
    OTG_GINTSTS |= OTG_GINT_ENUMDNE;
    ak_usb_poll();
    expect("a speed interrupt is handled without disturbing the configuration",
           ak_usb_ready() == 1);
    /* And it releases the global IN NAK the reset left in force - endpoint 0
     * with the rest. Until it is written the host's descriptor read is NAKed
     * for as long as the host keeps asking, and enumeration never completes.
     * The write is what is checkable here, not its effect: on the part the bit
     * is a self-clearing strobe and a page of memory does not model that. The
     * F405 suite carries the longer note. */
    expect("and it releases the global IN NAK a reset leaves in force",
           (OTG_DCTL & OTG_DCTL_CGNPINNAK) != 0u);
    OTG_DCTL &= ~OTG_DCTL_CGNPINNAK;
    OTG_GINTSTS &= ~OTG_GINT_ENUMDNE;

    /* SET_CONTROL_LINE_STATE: the other class request a serial driver sends,
     * and the one with no data stage at all. */
    usb_setup_packet(0x21u, 0x22u, 0x03u, 0u, 0u);
    expect("set control line state is acknowledged with nothing to carry",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 0u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);

    /* And a descriptor type this device does not implement: a stall rather than
     * an empty packet. */
    usb_setup_packet(0x80u, 0x06u, 0x0400u, 0u, 64u); /* interface descriptor */
    expect("a descriptor type nothing here has is stalled",
           (OTG_DIEPCTL0 & OTG_DEPCTL_STALL) != 0u &&
               (OTG_DOEPCTL0 & OTG_DEPCTL_STALL) != 0u);
    OTG_DIEPCTL0 &= ~OTG_DEPCTL_STALL;
    OTG_DOEPCTL0 &= ~OTG_DEPCTL_STALL;

    /* The class requests a serial driver makes when the port is opened.
     * SET_LINE_CODING carries seven bytes in a data stage, and the transfer only
     * ends when the device answers the status stage - so what is checked here is
     * the zero-length packet that finishes it, and then that GET_LINE_CODING is
     * answered with the seven bytes the host set. */
    usb_setup_packet(0x21u, 0x20u, 0u, 0u, 7u); /* SET_LINE_CODING */
    {
        /* The seven bytes of the line coding a host would send. */
        static const uint8_t coding[7] = { 0x80u, 0x25u, 0x00u, 0x00u,
                                           0x00u, 0x00u, 0x08u };
        usb_receive_words(0u, OTG_GRXSTS_DATA, coding, 7u);
    }
    ak_usb_poll();
    usb_consumed();
    expect("a data stage is answered with a zero-length status packet",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 0u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);

    usb_setup_packet(0xA1u, 0x21u, 0u, 0u, 7u); /* GET_LINE_CODING */
    expect("the line coding the host set is the line coding it is told",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 7u && OTG_FIFO(0u) == 0x00080000u);

    /*
     * What is *not* checked here, and why: the F405's file ends this test at
     * the board's own console reader, because that is the wire the console
     * actually pulls on and it has to take a USB byte before it looks at the
     * UART. That call is not reachable from this binary - `aerialkit-tests`
     * links exactly one board file, and it is the F405's, so the symbol here
     * would be a *different* board's reader answering. The AT32's board file is
     * compiled and linked by the target build and its reader is three lines
     * long, but that is the honest state of it: nothing on this host executes
     * the third board file, and building a second test binary for it means
     * compiling the whole test set twice (see the Makefile).
     */
    char byte = 0;
    usb_receive(2u, OTG_GRXSTS_DATA, 0x00000058u, 1u); /* "X" */
    ak_usb_poll();
    usb_consumed();
    expect("a byte typed over usb comes back out of ak_usb_read",
           ak_usb_read(&byte) == 1 && byte == 'X');

    /* A bus reset: the host is about to describe itself again, the configuration
     * is gone, and half a line from before it is not a command. */
    OTG_GINTSTS |= OTG_GINT_USBRST;
    ak_usb_poll();
    expect("a bus reset un-configures the device and empties the console",
           ak_usb_ready() == 0 && ak_usb_read(&byte) == 0);
    usb_receive(2u, OTG_GRXSTS_DATA, 0x41414141u, 4u);
    ak_usb_poll();
    usb_consumed();
    expect("and input that arrives before the next configuration is refused",
           ak_usb_read(&byte) == 0);
}

/*
 * And the third board's own file, which until now was compiled and linked by
 * the target build and executed by nothing on this host. It is the file that
 * decides what this board *is* - which UART is the console, which pins the
 * receiver and the GPS are on, where the saved configuration lives, how the
 * blackbox is cut into regions, which parts are fitted - so it is the file a
 * first flash depends on most, and the one whose mistakes look like absent
 * hardware.
 *
 * It runs here because the Makefile links *two* board files into this binary
 * and renames this one's entry points for the host build alone (see
 * src/boards/AERIALKIT_GHF435/board.h). Everything it calls underneath is the
 * real driver: the I2C model is the barometer, the flash model is the
 * controller, the mapped registers are the pins and the buses.
 */
static void test_the_boards_own_file(void)
{
    map_registers();
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    memset((void *)USB_BASE, 0, USB_SIZE);
    /* An erased part: all ones, which is what a factory part reads as and what
     * makes "nothing saved" a fact rather than a guess. */
    memset((void *)FLASH_REGION_BASE, 0xFF, FLASH_REGION_SIZE);
    host_at32_flash_model_reset();
    host_i2c_at32_reset();
    ak_test_at32_clock_up();

    ak_board_init();

    expect("the board names itself and the part it is",
           strstr(ak_board_name(), "GHF435") != 0 &&
               strstr(ak_board_name(), "AT32F435") != 0);
    expect("the console is attached to the port the board says it is on",
           ak_board_console_attached_port() == ak_board_console_port() &&
               ak_board_console_port() == AK_BOARD_CONSOLE_USART);
    expect("the clock the board reports is the one its code asked for",
           ak_board_clock_ok() == 1 && ak_board_clock_sysclk_hz() == 288000000u &&
               ak_board_clock_apb1_hz() == 144000000u);
    expect("the status pin is an output",
           ((AK_GPIO_CFGR(GPIOC_BASE) >> (13u * 2u)) & 0x3u) ==
               AK_GPIO_MODE_OUTPUT);

    /*
     * The barometer, end to end: the board says a DPS310 is fitted, hands over
     * a bus, and a register read through that bus comes back from the device
     * this test attached. That is the whole chain in one check - the board's
     * glue, the port's I2C sequences and the modelled part - and it is the
     * chain the navigator's altitude comes in on.
     */
    const ak_bus_t *baro = ak_board_baro_bus();
    expect("the board hands over a barometer bus, because this board has one",
           baro != 0 && baro->read != 0 && baro->write != 0);

    static const uint8_t dps310[8] = { 0x50u, 0x01u, 0x12u, 0x34u,
                                       0xAAu, 0xBBu, 0xCCu, 0xDDu };
    int device = host_i2c_at32_attach(AK_BOARD_BARO_ADDRESS, dps310,
                                      sizeof dps310);
    uint8_t who = 0u;

    expect("and a register read through it reaches the device on that address",
           device >= 0 && baro->read(0, 0x00u, &who, 1u) == 0 && who == 0x50u);
    expect("a write through it reaches the device too",
           baro->write(0, 0x01u, 0x5Au) == 0 &&
               host_i2c_at32_register((unsigned)device, 0x01u) == 0x5Au);

    /* The two buses this board does *not* have, which is an answer and not a
     * gap: nothing is fitted to the rangefinder, and the gyro's bus is there
     * because the part is. */
    expect("the rangefinder's bus is absent, because nothing is fitted",
           ak_board_range_bus() == 0);
    expect("the gyro's bus is present, because the board has a gyro",
           ak_board_imu_bus() != 0);

    /*
     * The saved configuration: written by the board, read back by the board,
     * through this part's flash controller - and the erased case first, because
     * a part that has never been saved to must read as "nothing saved" rather
     * than as a configuration.
     */
    char record[64] = {0};

    expect("an erased part holds nothing to load",
           ak_board_config_read(record, sizeof record) == 0);

    const char *saved = "airframe=wing\nrate_kp_roll=25.0\n";
    unsigned saved_len = (unsigned)strlen(saved);

    expect("a save writes the record and a load gives it back",
           ak_board_config_write(saved, saved_len) == 0 &&
               ak_board_config_read(record, sizeof record) == (int)saved_len &&
               memcmp(record, saved, saved_len) == 0);

    /* And the record is where the linker script says the configuration is, so
     * that a log wrap cannot eat it and a save cannot eat the image. */
    expect("the record lands in the page the board names for it",
           flash_word_at(0x080E0000u) == 0x414B4346u /* "AKCF" */);

    /*
     * And the second save, which is where this part's ring earns its keep:
     * every save erases only *its own* slot here (2 KB pages, four to a slot),
     * so the record that was already there is not inside the erase at any
     * point. Thirty-two slots come round before the first one is reused.
     */
    const char *second = "airframe=wing\nrate_kp_roll=26.0\n";
    unsigned second_len = (unsigned)strlen(second);
    expect("a second save goes into the next slot and becomes the one that "
           "loads",
           ak_board_config_write(second, second_len) == 0 &&
               ak_board_config_read(record, sizeof record) == (int)second_len &&
               memcmp(record, second, second_len) == 0 &&
               flash_word_at(0x080E0000u) == 0x414B4346u &&
               flash_word_at(0x080E1000u) == 0x414B4346u);

    /* An interrupted save is not a lost configuration: the newest slot's text
     * does not match its own sum - the state a power cut leaves - and the
     * record before it is what loads. Written straight into the mapped flash,
     * because the controller refuses to program a word that is not erased. */
    *(volatile char *)(0x080E0000u + 0x1000u + 16u) = (char)0x00;
    expect("a torn newest record falls back to the one before it",
           ak_board_config_read(record, sizeof record) == (int)saved_len &&
               memcmp(record, saved, saved_len) == 0);

    /* And a controller that erases and refuses to program - which on this part
     * cannot touch the record that is already there, because the erase is only
     * the slot the new record was going into. */
    host_at32_flash_model_set_program_error(1);
    expect("a save whose program is refused is reported",
           ak_board_config_write(second, second_len) < 0);
    expect("and the record it was replacing is still the one that loads",
           ak_board_config_read(record, sizeof record) == (int)saved_len &&
               memcmp(record, saved, saved_len) == 0);
    host_at32_flash_model_set_program_error(0);

    /* And a part whose slots are all damaged says so rather than pretending
     * nobody has saved here. */
    *(volatile uint32_t *)(0x080E0000u) = 0x00000000u;
    *(volatile uint32_t *)(0x080E1000u) = 0x00000000u;
    expect("a part of damaged records is an error, not an empty one",
           ak_board_config_read(record, sizeof record) < 0);

    /*
     * The blackbox: six regions of 128 KB, which is the layout the linker
     * script and the board have to agree about - the image stops before the
     * first, the record starts after the last - and the erase is this part's
     * pages, sixty-four of them per region.
     */
    const ak_flashlog_store_t *store = ak_board_log_store();

    expect("the blackbox is six regions of 128 KB from 0x08020000",
           store != 0 && store->count == 6u &&
               store->regions[0].base == 0x08020000u &&
               store->regions[0].bytes == 0x20000u &&
               store->regions[5].base == 0x080C0000u);
    expect("and the last region stops where the configuration begins",
           store->regions[5].base + store->regions[5].bytes == 0x080E0000u);

    /* Erasing a region is sixty-four page erases through the modelled
     * controller, and what the log reads afterwards is the erased word it
     * expects. */
    ak_flash_program(store->regions[0].base, (const uint32_t[]){ 1u, 2u }, 8u);
    expect("erasing a region empties the pages under it",
           store->erase(0) == 0 &&
               store->read(store->regions[0].base) == 0xFFFFFFFFu &&
               store->read(store->regions[0].base + 0x20000u - 4u) ==
                   0xFFFFFFFFu);
    expect("a record written into it reads back",
           store->write(store->regions[1].base + 0x100u, (const uint32_t[]){ 7u }, 4u) == 0 &&
               store->read(store->regions[1].base + 0x100u) == 7u);
    expect("and the region beside it is untouched",
           store->read(store->regions[2].base) == 0xFFFFFFFFu);

    /*
     * The receiver and the GPS, which is where the two facts this board was
     * surveyed for live: USART2 with a function number on each pin (6 and 8),
     * and USART3 with both pins at 7 and the peripheral's swap bit set.
     */
    ak_board_rc_init();
    expect("the receiver's two pins take their own function numbers",
           (AK_GPIO_MUXH(GPIOA_BASE) & 0xFu) == 8u &&
               (AK_GPIO_MUXL(GPIOB_BASE) & 0xFu) == 6u);
    expect("and the receiver is not inverted: CRSF needs no transistor",
           ak_board_rc_inverted() == 0);

    ak_board_gps_init();
    expect("the gps port has the peripheral's pins swapped",
           (AK_USART_CTRL2(AK_BOARD_GPS_USART) & AK_USART_CTRL2_TRPSWAP) != 0u);

    /* The pack, and the outputs, are the two the preflight reports on: this
     * board has its own divider and it drives motors, so both answer ready. */
    ak_board_battery_init();
    expect("the pack is readable, because this board has a divider on it",
           ak_board_battery_ready() == 1);
    expect("and the outputs are the board's own",
           ak_board_output_ready() == 1);
    ak_board_output_init();
    expect("initialising them leaves a frame's worth of state to report",
           ak_board_output_dshot_period() != 0u &&
               ak_board_output_dshot_hz() == 300000u);

    /* No network on this board, which the core is allowed to ask about. */
    expect("the board has no network and says so",
           ak_board_net_ready() == 0 && ak_board_net_connected() == 0);

    /* And the one thing here that is *not* checked, and cannot be: the hand-over
     * to the ROM bootloader. Calling it jumps to an address this process has
     * nothing mapped at, so the call is the bench's - and there is deliberately
     * no reference to it here, because a board's entry point is enforced by the
     * *firmware's* link (main.c hands it to the console) rather than by this
     * file. The address it jumps to is pinned at the top of the file. */
}

void test_arch_at32(void)
{
    test_the_arithmetic();
    test_the_clock_tree();
    test_the_clock_without_a_crystal();
    test_the_clock_from_another_crystal();
    test_the_pins();
    test_the_tick();
    test_the_console();
    test_the_flash();
    test_the_flash_that_never_finishes();
    test_the_sensor_bus();
    test_the_barometer_bus();
    test_the_barometers_transfers();
    test_the_barometer_bus_that_stops_answering();
    test_the_receiver_ports();
    test_the_pack();
    test_the_outputs();
    test_the_usb_console_path();
    test_the_boards_own_file();
}
