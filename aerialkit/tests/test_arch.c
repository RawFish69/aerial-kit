/*
 * The F405 port, running.
 *
 * Everything else in this repository tests the core or a driver. This file
 * tests `src/arch/stm32f405/` - the clock, the pins, the UARTs, the tick - by
 * giving the firmware a *register block* instead of a chip: the peripheral
 * region is mapped at the addresses the code expects, so the real
 * `ak_clk_init()`, `ak_uart_init()` and `ak_arch_time_init()` execute, and
 * what they leave in the registers is what this test reads back.
 *
 * That is worth a file of its own because the port is the one part of this
 * firmware that no host test has ever touched: the tests build `src/core` and
 * the simulator brings its own console, its own clock and its own UARTs. So
 * the port's arithmetic and its bit positions were only ever checked as
 * constants against the manual, never as *behaviour*.
 *
 * It found a real one on the way in. The UART divisor was computed as
 * `(fCK * 16) / baud` where RM0090 30.6.4 wants `fCK / baud` - the multiply
 * belongs to USARTDIV's scale, not to the numerator. Every port would have run
 * at a sixteenth of its baud rate: at 42 MHz and 115200 that is 7200 baud
 * against a terminal listening at 115200, a console that prints nothing
 * legible, on the very first flash. The numbers below are that bug.
 *
 * What this is not: a chip. There are no analogue properties here, no
 * interrupt controller, no clock tree that settles, and the byte stream out of
 * a UART is checked at its last byte because a memory-mapped register has no
 * history. It proves the code reads and writes what it means to, in the places
 * the manual says, and nothing about whether the silicon agrees.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <sys/mman.h>

#include "../src/arch/stm32f405/arch.h"
#include "../src/boards/AERIALKIT_F405/board.h"
#include "ak_console.h"
#include "ak_board.h"
#include "ak_params.h"
#include "ak_time.h"
#include "host_flash_model.h"
#include "host_adc_model.h"
#include "host_i2c_model.h"
#include "host_spi_model.h"
#include "host_usb_model.h"

/* The port's register loop, under the name the host build keeps for it: the
 * public entry has a modelled device on the bus (src/arch/stm32f405/spi.c). */
int ak_spi_transfer_loop(uint32_t spi, const uint8_t *tx, uint8_t *rx,
                         unsigned len);
#include "tests.h"

/* Where the board puts the saved configuration: the last 128 KB sector of a
 * 1 MB part. The board file's own constant is private to it, so this is the
 * address the linker script reserves and the one the config checks depend on -
 * written here so that a board that moved its record has to change this line
 * too, loudly, rather than silently reading a sector nobody writes. */
#define AK_CONFIG_BASE 0x080E0000u

/* And the configuration's *other* bank, the one the log now stops at: the
 * second 128 KB sector from the top, which the configuration alternates with
 * so that the erase a recycle needs never lands on the bank holding the newest
 * record. Same reason for writing it out here - a board that moved it has to
 * change this line too. */
#define AK_CONFIG_BASE0 0x080C0000u

/* One mapping covers every peripheral this port touches: the whole APB and
 * AHB1 windows from 0x40000000. SysTick and the system control block live in
 * the private peripheral region and get their own. */
#define PERIPH_BASE 0x40000000UL
#define PERIPH_SIZE 0x40000UL
#define PRIVATE_BASE 0xE000E000UL
#define PRIVATE_SIZE 0x1000UL

/* Two clocks for the delay's guard: one that never moves, and one that counts
 * once per read. Passing them in is what makes the guard testable without
 * waiting for a SysTick the host does not have running. */
static uint32_t frozen_now(void)
{
    return 0u;
}

static uint32_t stepping_now(void)
{
    static uint32_t ticks;
    return ticks++;
}

/* The USB controller is a window of its own at 0x50000000 - outside both
 * blocks above - and its FIFOs are 0x1000 apart inside it. */
#define USB_BASE 0x50000000UL
#define USB_SIZE 0x4000UL

/* The part's own flash, where the saved configuration lives and where a
 * blackbox will one day. The model in tests/host_flash_model.c erases and
 * programs into this mapping, so a test reads back the bytes the controller
 * would really have put there rather than a story about them. */
#define FLASH_REGION_BASE AK_FLASH_BASE
#define FLASH_REGION_SIZE 0x00100000UL

/* The two interrupt handlers this test calls directly, which the vector table
 * points at on the target and which therefore have no header of their own. */
void USART1_IRQHandler(void);
void SysTick_Handler(void);
void DMA1_Stream4_IRQHandler(void);

static int mapped;
static int system_control_mapped;

static void map_registers(void); /* below, with the addresses it maps */

/* Whether the register pages are up: the peripheral block, the private block,
 * the USB window and the flash region, all four of them. */
int ak_test_registers_mapped(void)
{
    return mapped;
}

/*
 * Bring them up, and answer whether that worked.
 *
 * A test that runs against these pages calls this rather than asking whether
 * something earlier in the run happened to map them. That question - "has the
 * port's own test run yet?" - is a fact about the runner's order and not about
 * the firmware, and a test that gets it wrong reports *zero* checks, which
 * reads exactly like a test that passed. Both board tests did that: run
 * without tests/test_arch.c in front of them they checked nothing and said
 * nothing, and it took running the suite in two pieces to notice.
 *
 * It maps them again even when they are already up. `MAP_FIXED` replaces the
 * old mapping with a fresh zeroed one, which is what makes the page state the
 * same however the suite is run: a board test then starts from a part that has
 * just come out of reset, not from whatever the port's own tests left in the
 * registers.
 */
int ak_test_map_registers(void)
{
    map_registers();
    return mapped;
}

/* The system control block on its own, which is all the fault record reads:
 * CFSR, HFSR, MMFAR and BFAR live in the private page. A flag of its own rather
 * than the one above, because a caller told "the model is up" when one page of
 * it is would go on to write into the peripheral block and fault. */
int ak_test_system_control_mapped(void)
{
    return system_control_mapped;
}

int ak_test_map_system_control(void)
{
    void *priv = mmap((void *)PRIVATE_BASE, PRIVATE_SIZE,
                      PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

    system_control_mapped = priv != MAP_FAILED;
    return system_control_mapped;
}

static void map_registers(void)
{
    void *apb = mmap((void *)PERIPH_BASE, PERIPH_SIZE, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    void *priv = mmap((void *)PRIVATE_BASE, PRIVATE_SIZE,
                      PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    void *usb = mmap((void *)USB_BASE, USB_SIZE, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    void *flash = mmap((void *)FLASH_REGION_BASE, FLASH_REGION_SIZE,
                       PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

    mapped = apb != MAP_FAILED && priv != MAP_FAILED && usb != MAP_FAILED &&
             flash != MAP_FAILED;
}

/* The registers come back zeroed, which is what a chip looks like before
 * anything is configured - except for the two things the clock code waits on,
 * which a real board answers after a few hundred microseconds and this has to
 * answer immediately or the wait never ends. */
static void reset_registers(void)
{
    memset((void *)PERIPH_BASE, 0, PERIPH_SIZE);
    memset((void *)PRIVATE_BASE, 0, PRIVATE_SIZE);
    memset((void *)USB_BASE, 0, USB_SIZE);
    /* The part arrives erased and locked, and the modelled controller arrives
     * with it: there is no state a test can inherit from the one before. */
    host_flash_model_reset();
    /* And the modelled I²C bus: no slaves, no half-finished transaction. */
    host_i2c_model_reset();
    /* And the modelled converter: a conversion that finishes, counts unset. */
    host_f4adc_reset();
    /* And the modelled USB receive FIFO: no words waiting to be read. */
    host_usb_reset();
    /* And the modelled SPI device: no part on the bus until a test puts one. */
    host_spi_reset();
}

void ak_test_f405_crystal(int up)
{
    RCC_CR = RCC_CR_HSION | RCC_CR_HSIRDY | RCC_CR_PLLON | RCC_CR_PLLRDY;
    if (up) {
        RCC_CR |= RCC_CR_HSEON | RCC_CR_HSERDY;
        /* SWS reports which clock the part is on, and the clock code waits for
         * it to agree. It is a read-only field on the part, so the only way to
         * answer it here is to have it already saying what the code is about
         * to ask for - which is what a chip does, a few microseconds later. */
        RCC_CFGR = RCC_CFGR_SWS_PLL;
    }
}

static void test_the_clock_tree(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();

    expect("the crystal is believed", ak_clk_hse_ok() == 1);

    /* The PLL registers, decoded rather than compared: the *arithmetic* is the
     * thing that has to be right. 8 MHz / M(8) * N(336) / P(2) = 168 MHz, and
     * the same multiplier over Q(7) is the 48 MHz USB wants. */
    uint32_t pll = RCC_PLLCFGR;
    unsigned m = (unsigned)(pll & 0x3Fu);
    unsigned n = (unsigned)((pll >> 6) & 0x1FFu);
    unsigned p = (unsigned)((((pll >> 16) & 0x3u) + 1u) * 2u);
    unsigned q = (unsigned)((pll >> 24) & 0xFu);
    unsigned vco_mhz = (8u / m) * n;

    expect("the PLL is fed from the crystal", (pll & RCC_PLL_SRC_HSE) != 0u);
    expect("and the multiplier and divider give 168 MHz",
           vco_mhz / p == 168u);
    expect("and the USB divisor gives 48 MHz", vco_mhz / q == 48u);

    /* The bus dividers, checked by what they mean rather than by their
     * encodings: APB1 divides by four and APB2 by two. */
    uint32_t cfgr = RCC_CFGR;
    expect("the system clock is switched to the PLL",
           (cfgr & RCC_CFGR_SW_MASK) == RCC_CFGR_SW_PLL);
    expect("APB1 is the system clock over four",
           ((cfgr >> 10) & 0x7u) == (RCC_CFGR_PPRE1_DIV4 >> 10));
    expect("APB2 is the system clock over two",
           ((cfgr >> 13) & 0x7u) == (RCC_CFGR_PPRE2_DIV2 >> 13));
    expect("and the clock the firmware reports is the one it asked for",
           ak_clk_sysclk_hz() == 168000000u && ak_clk_apb1_hz() == 42000000u &&
               ak_clk_apb2_hz() == 84000000u);

    /* Five wait states at 168 MHz, and the caches on: the reference manual
     * pairs the two, and raising the latency *after* the clock is the classic
     * way to a board that faults a few instructions into its own clock change.
     */
    expect("flash is at five wait states with the prefetcher and caches on",
           (FLASH_ACR & 0x7u) == 5u && (FLASH_ACR & FLASH_ACR_PRFTEN) != 0u &&
               (FLASH_ACR & FLASH_ACR_ICEN) != 0u &&
               (FLASH_ACR & FLASH_ACR_DCEN) != 0u);
}

static void test_the_clock_without_a_crystal(void)
{
    /* A board whose crystal does not start must end up on a clock it can name
     * rather than on an unknown one, and it must say so - the preflight and
     * the banner both print the fallback. */
    reset_registers();
    ak_test_f405_crystal(0);
    ak_clk_init();

    expect("a crystal that never starts is reported as failed",
           ak_clk_hse_ok() == 0);
    expect("and the clock falls back to the internal oscillator, named",
           ak_clk_sysclk_hz() == 16000000u && ak_clk_apb1_hz() == 16000000u &&
               ak_clk_apb2_hz() == 16000000u);
    expect("at one wait state, which is what 16 MHz needs",
           (FLASH_ACR & 0x7u) == 1u);
}

static void test_the_uart_divisor(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();

    /*
     * The three rates this firmware actually uses, against the divisor the
     * manual's own example gives. BRR = fCK / baud, because BRR carries
     * USARTDIV shifted left four and USARTDIV is already fCK / (16 * baud):
     *
     *   console  42 MHz / 115200   = 364.58 -> 365   (RM0090 30.6.4 uses
     *                                                 exactly this example)
     *   gps      42 MHz /   9600   = 4375
     *   receiver 84 MHz / 420000   = 200
     *   sbus     84 MHz / 100000   = 840
     */
    ak_uart_init(AK_BOARD_CONSOLE_USART, AK_BOARD_CONSOLE_TX,
                 AK_BOARD_CONSOLE_RX, AK_BOARD_CONSOLE_BAUD,
                 AK_BOARD_CONSOLE_AF);
    expect("115200 baud on a 42 MHz APB is a divisor of 365",
           USART_BRR(AK_BOARD_CONSOLE_USART) == 365u);

    ak_uart_rx_init(AK_BOARD_GPS_USART, AK_BOARD_GPS_TX, AK_BOARD_GPS_RX,
                    AK_BOARD_GPS_BAUD, AK_BOARD_GPS_AF);
    expect("9600 baud on a 42 MHz APB is a divisor of 4375",
           USART_BRR(AK_BOARD_GPS_USART) == 4375u);

    ak_uart_rx_init(AK_BOARD_RC_USART, AK_BOARD_RC_TX, AK_BOARD_RC_RX,
                    AK_BOARD_RC_BAUD, AK_BOARD_RC_AF);
    expect("420000 baud on an 84 MHz APB is a divisor of 200",
           USART_BRR(AK_BOARD_RC_USART) == 200u);

    /* And the one port whose *frame* is not 8N1: SBUS is 100000 baud with even
     * parity and two stop bits, and neither of those changes the divisor. */
    ak_uart_rx_init_format(AK_BOARD_RC_USART, AK_BOARD_RC_TX, AK_BOARD_RC_RX,
                           AK_BOARD_RC_SBUS_BAUD, AK_BOARD_RC_AF, 1, 1);
    expect("sbus is 100000 baud, which is a divisor of 840",
           USART_BRR(AK_BOARD_RC_USART) == 840u);
    expect("with even parity enabled",
           (USART_CR1(AK_BOARD_RC_USART) & USART_CR1_PCE) != 0u &&
               (USART_CR1(AK_BOARD_RC_USART) & USART_CR1_PS) == 0u);
    expect("and two stop bits",
           USART_CR2(AK_BOARD_RC_USART) == USART_CR2_STOP_2);
}

static void test_the_pins_and_the_clock_enables(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_uart_init(AK_BOARD_CONSOLE_USART, AK_BOARD_CONSOLE_TX,
                 AK_BOARD_CONSOLE_RX, AK_BOARD_CONSOLE_BAUD,
                 AK_BOARD_CONSOLE_AF);

    /* The port has to be clocked, or every register write above went nowhere -
     * which is a failure that looks exactly like a working one. */
    expect("the console's port clock is enabled",
           (RCC_APB1ENR & RCC_APB1ENR_USART2EN) != 0u);
    expect("and its pins are clocked too",
           (RCC_AHB1ENR & RCC_AHB1ENR_GPIOAEN) != 0u);

    /* PA2 and PA3 as alternate function 7, transmit with no pull and receive
     * with a pull-up: an idle line that floats is a line that reads as noise. */
    unsigned tx = 2u, rx = 3u;
    expect("the transmit pin is an alternate function",
           ((GPIO_MODER(GPIOA_BASE) >> (tx * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOA_BASE) >> (rx * 2u)) & 0x3u) == GPIO_MODE_AF);
    expect("both are on alternate function 7",
           ((GPIO_AFRL(GPIOA_BASE) >> (tx * 4u)) & 0xFu) ==
               AK_BOARD_CONSOLE_AF &&
               ((GPIO_AFRL(GPIOA_BASE) >> (rx * 4u)) & 0xFu) ==
                   AK_BOARD_CONSOLE_AF);
    expect("the receive pin is pulled up and the transmit pin is not",
           ((GPIO_PUPDR(GPIOA_BASE) >> (rx * 2u)) & 0x3u) ==
               GPIO_PUPD_PULLUP &&
               ((GPIO_PUPDR(GPIOA_BASE) >> (tx * 2u)) & 0x3u) ==
                   GPIO_PUPD_NONE);
    expect("and the port is enabled with transmit and receive on",
           (USART_CR1(AK_BOARD_CONSOLE_USART) &
            (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE)) ==
               (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE));
}

static void test_the_console_path(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_uart_init(AK_BOARD_CONSOLE_USART, AK_BOARD_CONSOLE_TX,
                 AK_BOARD_CONSOLE_RX, AK_BOARD_CONSOLE_BAUD,
                 AK_BOARD_CONSOLE_AF);
    ak_console_attach(AK_BOARD_CONSOLE_USART);

    /* A working port has transmit-empty set: it is set at reset and again
     * after every byte. The fake has to say so, the same way it has to answer
     * the clock's ready bits. */
    USART_SR(AK_BOARD_CONSOLE_USART) = USART_SR_TXE;

    expect("the console is attached to the port the board says it is",
           ak_console_port() == AK_BOARD_CONSOLE_USART);

    /*
     * The whole write path - core formatting, the board sink, the data
     * register - with the last byte of what was sent left in the register,
     * because a memory-mapped register has no history to read. The *stream* is
     * what the simulator's console capture checks; this is that the bytes get
     * as far as the port at all, and in the right order for the last one.
     */
    ak_console_write("AerialKit\r\n");
    expect("a string written to the console ends in the data register",
           (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == '\n');

    ak_console_printf("len %u", 42u);
    expect("and so does a formatted line", 
           (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == '2');

    /*
     * The conversions and guards of the formatter that no line in the firmware
     * happens to use, which is exactly why nothing had ever run them: `%X`,
     * `%p`, the two justification flags, and a `%` that is not followed by
     * anything. The *length* the formatter returns and the last byte in the
     * register between them say what came out.
     *
     * The two guards matter more than the conversions do. A format string that
     * ends in a stray `%` - or carries a conversion this formatter does not
     * know - must not read an argument that is not there: on a board that is a
     * word of the caller's stack printed at a bench, or a crash, and both of
     * these calls are made with *no* varargs at all so a version that reached
     * for one could not pass by luck.
     */
    expect("uppercase hex is uppercase, and four digits long",
           ak_console_printf("%X", 0xABCDu) == 4 &&
               (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == 'D');
    expect("a pointer is 0x and eight digits, zero padded",
           ak_console_printf("%p", (void *)(uintptr_t)0x1234u) == 10 &&
               (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == '4');
    expect("a right-justified field pads on the left",
           ak_console_printf("%4s", "ab") == 4 &&
               (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == 'b');
    expect("and a left-justified one pads on the right",
           ak_console_printf("%-4s", "ab") == 4 &&
               (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == ' ');
    expect("as does a number, which pads after its digits",
           ak_console_printf("%-4d", 7) == 4 &&
               (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == ' ');
    /* The line stops where the format does - the `%` is not printed, and,
     * which is the point, no argument is read for it. */
    expect("a format that ends in % stops there and reads nothing",
           ak_console_printf("done%") == 4 &&
               (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == 'e');
    expect("and a conversion it does not know is printed, not consumed",
           ak_console_printf("x%z") == 3 &&
               (USART_DR(AK_BOARD_CONSOLE_USART) & 0xFFu) == 'z');

    expect("the port still has transmit enabled afterwards",
           (USART_CR1(AK_BOARD_CONSOLE_USART) & USART_CR1_TE) != 0u);

    /* And the way back: a byte waiting in the data register with the
     * receive-not-empty flag beside it. */
    uint8_t byte = 0;
    expect("an empty port offers no byte",
           ak_uart_poll_rx(AK_BOARD_CONSOLE_USART, &byte) == 0);

    USART_SR(AK_BOARD_CONSOLE_USART) = USART_SR_RXNE;
    USART_DR(AK_BOARD_CONSOLE_USART) = 'Z';
    expect("and a byte in it is one",
           ak_uart_poll_rx(AK_BOARD_CONSOLE_USART, &byte) == 1 && byte == 'Z');

    /*
     * And the port that never accepts anything. This is the check that would
     * have hung: the sink used to wait for transmit-empty forever, so a port
     * whose clock was off stopped the firmware inside the first line of its
     * own banner - no LED, no fault record, a board that looks dead. It gives
     * up now, which costs the byte and leaves everything else running. The
     * check completing at all is the test.
     */
    USART_DR(AK_BOARD_CONSOLE_USART) = 0u;
    ak_console_write("this port is not listening");
    expect("a port that never accepts a byte is given up on, not waited for",
           USART_DR(AK_BOARD_CONSOLE_USART) == 0u);
}

static void test_the_receivers_interrupt_path(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_uart_rx_init(AK_BOARD_RC_USART, AK_BOARD_RC_TX, AK_BOARD_RC_RX,
                    AK_BOARD_RC_BAUD, AK_BOARD_RC_AF);

    /* The receiver's bytes do not come through the polled path: they arrive on
     * an interrupt, fill a ring, and the flight loop drains it. This is that
     * whole path - the interrupt handler the vector table points at, reading
     * the data register and pushing into the ring - and it is the path a CRSF
     * or SBUS frame takes before any parser sees it. */
    uint8_t byte = 0;
    expect("the ring starts empty", ak_uart_rx_pop(AK_BOARD_RC_USART, &byte) == 0);

    for (unsigned i = 0; i < 3u; i++) {
        USART_SR(USART1_BASE) = USART_SR_RXNE;
        USART_DR(USART1_BASE) = (uint32_t)('a' + i);
        USART1_IRQHandler();
    }

    expect("three interrupts leave three bytes in the ring",
           ak_uart_rx_pop(USART1_BASE, &byte) == 1 && byte == 'a');
    expect("in the order they arrived",
           ak_uart_rx_pop(USART1_BASE, &byte) == 1 && byte == 'b');
    expect("and the last of them is still there",
           ak_uart_rx_pop(USART1_BASE, &byte) == 1 && byte == 'c');
    expect("with nothing dropped", ak_uart_rx_dropped(USART1_BASE) == 0u);
}

static void test_the_tick(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_time_init();

    /* One millisecond per interrupt at 168 MHz, and the counter starts at
     * zero: the preflight's "the tick is running" check is the same two
     * registers read back. */
    expect("the reload is one millisecond of system clocks",
           SYSTICK_LOAD == (168000000u / 1000u) - 1u);
    expect("the counter is cleared and the interrupt is on",
           SYSTICK_VAL == 0u &&
               (SYSTICK_CTRL & (SYSTICK_CTRL_CLKSOURCE |
                                SYSTICK_CTRL_TICKINT |
                                SYSTICK_CTRL_ENABLE)) ==
                   (SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT |
                    SYSTICK_CTRL_ENABLE));

    expect("and no time has passed yet", ak_time_ms() == 0u);
    for (unsigned i = 0; i < 5u; i++) {
        SysTick_Handler();
    }
    expect("five ticks are five milliseconds", ak_time_ms() == 5u);

    /*
     * And the wait that a delay runs, with the clock handed in so both of its
     * endings can be driven from here: a clock that moves lets the wait finish,
     * and a clock that does not lets it go after its limit and *says so*. The
     * second case is the one the bench found - a delay with no bound is a board
     * that stops inside its own boot.
     */
    {
        int gave_up = 0;
        unsigned rounds;

        rounds = ak_wait_for_ticks(0u, 5u, 1000u, frozen_now, &gave_up);
        expect("a tick that never moves ends the wait at its limit",
               gave_up == 1 && rounds == 1000u);

        rounds = ak_wait_for_ticks(0u, 5u, 1000u, stepping_now, &gave_up);
        expect("a tick that moves lets the wait finish, and does not give up",
               gave_up == 0 && rounds == 5u);
        expect("and the stall counter is what a report can print",
               ak_delay_stalls() == 0u);
    }
}

/*
 * The outputs: two timers, four compare registers on one, two on the other,
 * and a DMA burst that writes all four motor compare values at once.
 *
 * This is the path whose failure is a motor that does not turn - or worse, one
 * that turns at the wrong time - and it has the most moving parts of anything
 * in the port: two peripherals, a DMA stream whose channel selection lives in
 * the top byte of its control register, a burst configuration that counts
 * words from the timer's base address, and an interrupt that has to clear the
 * right flag in the right half of the flag register. None of it had run.
 *
 * The servo bank is an argument rather than the arch's own choice since
 * 2026-09-29, because the two F405 boards put their servos on different timers
 * and different pads - the WeAct on TIM2 channels 1-2 at PA0/PA1, the Feather
 * on TIM4 channels 3-4 at PB8/PB9, whose breakout brings out neither of the
 * WeAct's pads. The two tables below are those two banks, so what each board
 * hands in is under test here rather than only the arithmetic behind it. The
 * board's own wrapper - and therefore its own table - is driven by
 * tests/test_board_f405.c.
 */
static const ak_servo_out_t arch_servos_tim2[AK_MAX_SERVOS] = {
    {GPIOA_BASE, 0u, 1u, 1u},
    {GPIOA_BASE, 1u, 1u, 2u},
};

static const ak_servo_out_t arch_servos_tim4[AK_MAX_SERVOS] = {
    {GPIOB_BASE, 8u, 2u, 3u},
    {GPIOB_BASE, 9u, 2u, 4u},
};

static void test_the_servo_timer(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_output_init(TIM2_BASE, arch_servos_tim2, AK_MAX_SERVOS);

    /* 1 MHz from an 84 MHz timer clock, counting microseconds, wrapping at
     * 20000 of them - which is 50 Hz, and the pulse the servos read is the
     * compare value itself. */
    expect("the servo timer counts microseconds",
           TIM_PSC(TIM2_BASE) == (84000000u / 1000000u) - 1u);
    expect("it wraps at twenty milliseconds",
           TIM_ARR(TIM2_BASE) == 20000u - 1u);
    expect("and both servos start centred at 1500 microseconds",
           TIM_CCR1(TIM2_BASE) == 1500u && TIM_CCR2(TIM2_BASE) == 1500u);

    expect("the timer clocks are on",
           (RCC_APB1ENR & RCC_APB1ENR_TIM2EN) != 0u &&
               (RCC_APB1ENR & RCC_APB1ENR_TIM3EN) != 0u);

    /* Both channels in PWM mode 1, both enabled, auto-reload preloaded, and
     * the counter running. */
    expect("both servo channels are in pwm mode 1",
           ((TIM_CCMR1(TIM2_BASE) >> 4) & 0x7u) == TIM_CCMR_OCM_PWM1 &&
               ((TIM_CCMR1(TIM2_BASE) >> 12) & 0x7u) == TIM_CCMR_OCM_PWM1);
    expect("and both are enabled",
           (TIM_CCER(TIM2_BASE) & (TIM_CCER_CCE(1) | TIM_CCER_CCE(2))) ==
               (TIM_CCER_CCE(1) | TIM_CCER_CCE(2)));
    expect("with the counter running and the reload preloaded",
           (TIM_CR1(TIM2_BASE) & TIM_CR1_CEN) != 0u &&
               (TIM_CR1(TIM2_BASE) & TIM_CR1_ARPE) != 0u);

    expect("and the bank it names is the one it was handed",
           strcmp(ak_output_servo_timer_name(), "TIM2") == 0);
}

/*
 * The same bank on the other timer, which is the whole of what the argument
 * bought: TIM4's free pair is channels 3 and 4, so the compare registers are
 * CCR3 and CCR4 rather than CCR1 and CCR2, the pads are PB8 and PB9 at
 * alternate function 2, and TIM2 is not touched at all. A board that named
 * TIM4 and got the channels wrong would put both pulse widths in one register
 * and leave the other servo dead - which is a servo that does not move, on a
 * pad that looks configured.
 */
static void test_the_servo_bank_on_another_timer(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_output_init(TIM4_BASE, arch_servos_tim4, AK_MAX_SERVOS);

    expect("the second timer counts microseconds as the first does",
           TIM_PSC(TIM4_BASE) == (84000000u / 1000000u) - 1u &&
               TIM_ARR(TIM4_BASE) == 20000u - 1u);
    expect("channels three and four start centred",
           TIM_CCR3(TIM4_BASE) == 1500u && TIM_CCR4(TIM4_BASE) == 1500u);
    expect("and channels one and two, which this bank does not use, stay zero",
           TIM_CCR1(TIM4_BASE) == 0u && TIM_CCR2(TIM4_BASE) == 0u);

    /* Channels 3 and 4 live in CCMR2, whose halves are at bit 4 and bit 12 of
     * that register - the same nibble pattern as CCMR1, one register over. */
    expect("both of them are in pwm mode 1, in the second compare register",
           ((TIM_CCMR2(TIM4_BASE) >> 4) & 0x7u) == TIM_CCMR_OCM_PWM1 &&
               ((TIM_CCMR2(TIM4_BASE) >> 12) & 0x7u) == TIM_CCMR_OCM_PWM1);
    expect("and enabled, with the counter running",
           (TIM_CCER(TIM4_BASE) & (TIM_CCER_CCE(3) | TIM_CCER_CCE(4))) ==
               (TIM_CCER_CCE(3) | TIM_CCER_CCE(4)) &&
               (TIM_CR1(TIM4_BASE) & TIM_CR1_CEN) != 0u);
    expect("and its own clock gate is on",
           (RCC_APB1ENR & RCC_APB1ENR_TIM4EN) != 0u);

    expect("the pads are on the second timer's alternate function",
           ((GPIO_MODER(GPIOB_BASE) >> (8u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOB_BASE) >> (9u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_AFRH(GPIOB_BASE) >> 0) & 0xFu) == 2u &&
               ((GPIO_AFRH(GPIOB_BASE) >> 4) & 0xFu) == 2u);
    expect("and the other timer's pads were not configured by this bank",
           ((GPIO_MODER(GPIOA_BASE) >> (0u * 2u)) & 0x3u) == GPIO_MODE_INPUT &&
               ((GPIO_MODER(GPIOA_BASE) >> (1u * 2u)) & 0x3u) == GPIO_MODE_INPUT);

    expect("and it says which timer the servos went to",
           strcmp(ak_output_servo_timer_name(), "TIM4") == 0);

    /* A frame written now lands in this bank's compare registers, which is the
     * end-to-end half: the channel the board named is the one the write path
     * uses, and not the arch's old fixed pair.
     *
     * It is the same frame path the test above drives, so it starts a motor
     * burst too - one port, one write. The frame counters are cumulative over
     * the whole process, so they are read as a delta here; the assertions above
     * are about one frame and this is a second one. */
    ak_output_frame_t frame;
    memset(&frame, 0, sizeof frame);
    frame.servo_us[0] = 1100u;
    frame.servo_us[1] = 1900u;

    uint32_t sent_before = ak_output_frames_sent();
    ak_output_write(&frame);
    expect("and a frame's pulse widths land in the channels it named",
           TIM_CCR3(TIM4_BASE) == 1100u && TIM_CCR4(TIM4_BASE) == 1900u &&
               TIM_CCR1(TIM4_BASE) == 0u && TIM_CCR2(TIM4_BASE) == 0u);
    expect("and the burst that same frame started is the motor path, running",
           ak_output_busy() != 0 && ak_output_frames_sent() == sent_before);

    /* Ended the way every frame on hardware is ended, by the controller's own
     * interrupt. Leaving it running would leave the next test's first frame
     * dropped as one that arrived while the dma was busy. */
    DMA1_Stream4_IRQHandler();
    expect("which completes it and counts it once",
           ak_output_busy() == 0 && ak_output_frames_sent() == sent_before + 1u);

    /* A board with no servo outputs at all is a third answer, and it is the
     * one a report has to be able to say rather than naming a timer it is not
     * driving. */
    ak_output_init(TIM2_BASE, NULL, 0u);
    expect("a board with no servo bank says none rather than naming a timer",
           strcmp(ak_output_servo_timer_name(), "none") == 0);
}

static void test_the_motor_timer_and_the_burst(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_output_init(TIM2_BASE, arch_servos_tim2, AK_MAX_SERVOS);

    /*
     * One timer period per DShot bit, so the period is the timer clock over
     * the bit rate, which is 84 MHz:
     *
     *   150 kHz -> 560 ticks -> ARR 559
     *   300 kHz -> 280 ticks -> ARR 279   (the reset default)
     *   600 kHz -> 140 ticks -> ARR 139
     */
    expect("300 kHz is the rate it boots at",
           ak_output_dshot_hz() == 300000u &&
               TIM_ARR(TIM3_BASE) == 279u);

    ak_output_set_rate(600u);
    expect("600 kHz is a different period",
           ak_output_dshot_hz() == 600000u &&
               TIM_ARR(TIM3_BASE) == 139u);
    ak_output_set_rate(150u);
    expect("and 150 kHz another", ak_output_dshot_hz() == 150000u &&
                                      TIM_ARR(TIM3_BASE) == 559u);
    ak_output_set_rate(250u);
    expect("a rate the ESCs do not speak is refused rather than approximated",
           ak_output_dshot_hz() == 150000u);

    ak_output_set_rate(300u);
    /* Channels one and two live in CCMR1 and three and four in CCMR2, each as
     * three bits of output mode at bit 4 and bit 12 of its half. */
    expect("the first two motor channels are in pwm mode 1",
           ((TIM_CCMR1(TIM3_BASE) >> 4) & 0x7u) == TIM_CCMR_OCM_PWM1 &&
               ((TIM_CCMR1(TIM3_BASE) >> 12) & 0x7u) == TIM_CCMR_OCM_PWM1);
    expect("and the other two, in the second register",
           ((TIM_CCMR2(TIM3_BASE) >> 4) & 0x7u) == TIM_CCMR_OCM_PWM1 &&
               ((TIM_CCMR2(TIM3_BASE) >> 12) & 0x7u) == TIM_CCMR_OCM_PWM1);
    expect("and all four are enabled",
           (TIM_CCER(TIM3_BASE) & (TIM_CCER_CCE(1) | TIM_CCER_CCE(2) |
                                   TIM_CCER_CCE(3) | TIM_CCER_CCE(4))) ==
               (TIM_CCER_CCE(1) | TIM_CCER_CCE(2) | TIM_CCER_CCE(3) |
                TIM_CCER_CCE(4)));

    /* The pins, because a timer channel is no use if its pin is still an
     * input: servos are alternate function 1 on PA0 and PA1, motors are
     * alternate function 2 on PA6, PA7, PB0 and PB1. */
    expect("the servo pins are on the servo timer's alternate function",
           ((GPIO_MODER(GPIOA_BASE) >> (0u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOA_BASE) >> (1u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               (GPIO_AFRL(GPIOA_BASE) & 0xFu) == 1u &&
               ((GPIO_AFRL(GPIOA_BASE) >> 4) & 0xFu) == 1u);
    expect("and the motor pins on the motor timer's",
           (GPIO_AFRL(GPIOA_BASE) & 0xF0000000u) == 0x20000000u &&
               ((GPIO_AFRL(GPIOA_BASE) >> 28) & 0xFu) == 2u &&
               (GPIO_AFRL(GPIOB_BASE) & 0xFFu) == 0x22u);

    /* The burst: DCR.DBA counts 32-bit words from the timer's base, and CCR1
     * is at 0x34, which is word 13. DBL is the number of transfers *minus
     * one*, and four compare registers is three. Getting either wrong writes
     * four values somewhere other than the compare registers, which is a
     * DShot frame no ESC will ever see. */
    expect("the burst starts at the first compare register",
           (TIM_DCR(TIM3_BASE) & 0x1Fu) == (0x34u / 4u));
    expect("and moves four half-words",
           ((TIM_DCR(TIM3_BASE) >> 8) & 0x1Fu) == 3u);
    expect("the burst runs on the first channel's compare event",
           (TIM_DIER(TIM3_BASE) & TIM_DIER_CC1DE) != 0u);

    /* And the stream that does it. */
    uint32_t cr = DMA_SxCR(DMA1_BASE, 4u);
    expect("the dma clock is on", (RCC_AHB1ENR & RCC_AHB1ENR_DMA1EN) != 0u);
    expect("the stream is memory to peripheral, incrementing, half-words",
           (cr & DMA_SxCR_DIR_M2P) != 0u && (cr & DMA_SxCR_MINC) != 0u &&
               (cr & DMA_SxCR_MSIZE_16) != 0u &&
               (cr & DMA_SxCR_PSIZE_16) != 0u);
    expect("on the right channel of the dma controller, at high priority",
           (cr & 0x7u << 25) == (5u << 25) && (cr & DMA_SxCR_PL_HIGH) != 0u);
    expect("with the transfer-complete interrupt enabled",
           (cr & DMA_SxCR_TCIE) != 0u);
    expect("writing the timer's data register",
           DMA_SxPAR(DMA1_BASE, 4u) == (uint32_t)(uintptr_t)&TIM_DMAR(TIM3_BASE));
    expect("and the interrupt is enabled in the controller",
           (NVIC_ISER0 & (1u << DMA1_STREAM4_IRQ)) != 0u);
}

static void test_a_frame(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_output_init(TIM2_BASE, arch_servos_tim2, AK_MAX_SERVOS);
    ak_output_set_rate(300u);

    ak_output_frame_t frame;
    memset(&frame, 0, sizeof frame);
    frame.servo_us[0] = 1000u;
    frame.servo_us[1] = 2000u;
    /* 0x7D0 is the DShot value for a throttle of 1000, which the encoder's own
     * tests pin; the pattern below is deliberately not all zeros, so the first
     * bit of the burst is a '1' for one motor and a '0' for another. */
    /* The first bit of each frame, which is the bit this hand-writes: the most
     * significant bit of a 16-bit DShot value. The four below alternate it, so
     * a burst that wrote the same value four times would be visible. */
    frame.dshot[0] = 0x8000u; /* bit 15 set */
    frame.dshot[1] = 0x7D0u;  /* bit 15 clear */
    frame.dshot[2] = 0xFFFFu;
    frame.dshot[3] = 0x0100u;

    ak_output_write(&frame);

    expect("the servo pulse widths land in the compare registers",
           TIM_CCR1(TIM2_BASE) == 1000u && TIM_CCR2(TIM2_BASE) == 2000u);

    /* The first four entries of the burst are bit one of each motor's frame,
     * written by hand; the DMA starts one period behind and writes from bit
     * two on. */
    /* 300 kHz is 280 ticks a bit, and the duty the reference uses is 35% for a
     * '0' and 70% for a '1' - 98 and 196, which is also the row in the table in
     * 07-outputs.md. Written as literals because a check that recomputes the
     * implementation's own expression agrees with its own bug. */
    uint16_t zero = 98u;
    uint16_t one = 196u;
    expect("the first bit of each motor's frame is in its compare register",
           TIM_CCR1(TIM3_BASE) == one && TIM_CCR2(TIM3_BASE) == zero &&
               TIM_CCR3(TIM3_BASE) == one &&
               TIM_CCR4(TIM3_BASE) == zero);

    expect("and the burst was handed to the dma",
           (DMA_SxCR(DMA1_BASE, 4u) & DMA_SxCR_EN) != 0u &&
               ak_output_busy() == 1);
    expect("the transfer counts the groups after the first one",
           DMA_SxNDTR(DMA1_BASE, 4u) == (18u - 1u) * 4u);

    /* A second frame while the burst is still running is dropped and counted,
     * not queued: the compare registers are the ESC's clock, and a frame that
     * arrived late must not be interleaved into one that is being sent. */
    frame.dshot[0] = 0x0200u;
    ak_output_write(&frame);
    expect("a frame that arrives while the dma is busy is dropped and counted",
           ak_output_frames_skipped() == 1u && TIM_CCR1(TIM3_BASE) == one);

    /* The interrupt the vector table points at. */
    DMA1_Stream4_IRQHandler();
    expect("the transfer-complete interrupt clears the stream's flags",
           (DMA_HIFCR(DMA1_BASE) & DMA_IFCR_CLEAR(4u)) == DMA_IFCR_CLEAR(4u));
    expect("stops the stream, counts the frame and is ready again",
           (DMA_SxCR(DMA1_BASE, 4u) & DMA_SxCR_EN) == 0u &&
               ak_output_frames_sent() == 1u && ak_output_busy() == 0);
}

/*
 * The flash controller - the guards, and the honest boundary.
 *
 * This is the one port file that *cannot* be fully executed against a mapped
 * register block, and it is worth saying why rather than writing checks that
 * look like more than they are. The success path needs two things a plain
 * region cannot do: the key sequence in FLASH_KEYR has to *clear* the lock bit
 * in FLASH_CR, and programming has to make the part's own writes appear where
 * they were addressed. Both need either silicon or a write trap, and a trap
 * means decoding the faulting store instruction on whichever architecture the
 * test is running on.
 *
 * So what runs here is everything up to that point, which is the half that
 * decides whether the controller is ours at all. That half matters more than
 * it sounds: the failure it guards against is asking a controller somebody
 * else has open to erase a sector, and the file's whole job is to be the one
 * component that cannot half-work.
 */
static void test_the_flash_guards(void)
{
    reset_registers();

    /* A 1 MB part has sectors 0 to 11, and the sector number is a parameter a
     * person can type. */
    expect("a sector past the end of the part is refused",
           ak_flash_erase_sector(12u) < 0);
    expect("and nothing was sent to the controller for it",
           host_flash_model_keys() == 0u &&
               (host_flash_reg_read(FLASH_OFF_CR) & FLASH_CR_SER) == 0u);

    /* Word programming, because the write granularity is set to 32 bits: a
     * request the controller cannot honour is refused rather than rounded. */
    uint32_t word = 0x12345678u;
    expect("an address that is not word aligned is refused",
           ak_flash_program(0x08000001u, &word, 4u) < 0);
    expect("and a length that is not a whole number of words",
           ak_flash_program(0x08000000u, &word, 2u) < 0);
    expect("with nothing sent to the controller either time",
           host_flash_model_keys() == 0u);

    /*
     * The lock. A controller that is already unlocked belongs to somebody
     * else - the only way to get there is a fault between another operation's
     * unlock and its lock - and this refuses to touch it.
     */
    host_flash_model_set_locked(0);
    expect("a controller somebody else has open is refused",
           ak_flash_erase_sector(11u) < 0 &&
               ak_flash_program(0x080E0000u, &word, 4u) < 0);
    expect("and no key was written to it", host_flash_model_keys() == 0u);

    /*
     * And a controller that is locked and does not accept the keys. This is
     * the one behaviour of the model that no real part has, and it is here so
     * that the branch stays reachable: the driver has to notice a lock bit
     * that did not clear, report the operation as failed, and fail *before*
     * the erase bit is set - a controller that is not ours is never asked to
     * erase anything.
     */
    host_flash_model_set_locked(1);
    host_flash_model_set_unlock_refused(1);
    expect("a controller that will not unlock is a failure, not a success",
           ak_flash_erase_sector(11u) < 0);
    expect("the key sequence was sent, in the reference manual's order",
           host_flash_model_keys() == FLASH_KEY2);
    expect("and the erase was never armed",
           (host_flash_reg_read(FLASH_OFF_CR) & FLASH_CR_SER) == 0u);
    expect("nor programmed", ak_flash_program(0x080E0000u, &word, 4u) < 0 &&
                                 (host_flash_reg_read(FLASH_OFF_CR) &
                                  FLASH_CR_PG) == 0u);

    /*
     * What this file could not reach until the modelled controller arrived -
     * a sector that erases, a word that lands where it was addressed, the
     * flags after either - is the next test along. These are its refusals;
     * that one is its success path.
     */
}

/*
 * The controller that never finishes.
 *
 * Every other refusal in this file is the part saying *no*: it will not
 * unlock, it will not program over a word, the address is not aligned. This one
 * is the part saying nothing at all - the operation was accepted and the busy
 * bit never clears - and it is the one failure that does not look like a
 * failure on a board: the firmware waits, and waits, and the aircraft never
 * says anything again. The wait is bounded on purpose (`AK_FLASH_TIMEOUT`),
 * and a bound nothing ever reaches is a bound nobody has ever run, so this
 * makes the controller stick and runs it.
 *
 * What the checks are really about is what is left behind. The flash controller
 * is a part somebody else's code will use next: a driver that gave up but left
 * the erase armed, or left the controller unlocked, or worse - erased the
 * sector and then failed to program it - hands the next caller or the next boot
 * a state that is worse than the failure it reported. The last of those is why
 * the record is read back: "the save failed" and "the save failed and your
 * settings are gone" are different sentences to a person at a bench.
 */
static void test_the_flash_that_never_finishes(void)
{
    reset_registers();

    /* A sector with a record already in it, so that "the erase did not happen"
     * is answered by reading the flash rather than by asking the model. */
    uint32_t pattern[2] = { 0x414B0007u, 0xC0FFEE00u };
    expect("a sector to write into, with a record already in it",
           ak_flash_erase_sector(11u) == 0 &&
               ak_flash_program(0x080E0000u, pattern, sizeof pattern) == 0 &&
               AK_REG32(0x080E0000u) == pattern[0]);

    host_flash_model_set_stuck_busy(1);
    expect("a controller whose operation never finishes is given up on rather "
           "than waited for",
           ak_flash_erase_sector(11u) < 0);
    expect("and it is left locked, because the operation is the part's now",
           (host_flash_reg_read(FLASH_OFF_CR) & FLASH_CR_LOCK) != 0u);
    expect("with no erase left armed for the next caller",
           (host_flash_reg_read(FLASH_OFF_CR) & FLASH_CR_SER) == 0u);
    expect("and the record that was there is still there, not half erased",
           AK_REG32(0x080E0000u) == pattern[0] &&
               AK_REG32(0x080E0004u) == pattern[1]);

    uint32_t word = 0x12345678u;
    expect("programming a controller that never finishes is given up on too",
           ak_flash_program(0x080E0000u, &word, 4u) < 0);
    expect("with the programming bit cleared rather than left armed",
           (host_flash_reg_read(FLASH_OFF_CR) & FLASH_CR_PG) == 0u);
    expect("and the same record, not the first word of a new one",
           AK_REG32(0x080E0000u) == pattern[0]);

    /* And it is a state the part comes back from: clearing the stick is what a
     * controller that finishes after all looks like, and the driver has to be
     * usable again rather than wedged by the failure it survived. */
    host_flash_model_set_stuck_busy(0);
    expect("a controller that starts answering again is usable again",
           ak_flash_erase_sector(11u) == 0 &&
               ak_flash_program(0x080E0000u, pattern, sizeof pattern) == 0 &&
               AK_REG32(0x080E0000u) == pattern[0]);
}

/*
 * The flash controller's success path, which is the one thing the register
 * block could not reach: it needs the part to *act* on what is written rather
 * than to store it. tests/host_flash_model.c is that part, behind the seam in
 * flash.c, and the flash itself is the region mapped at 0x08000000.
 *
 * The check that matters is on the flash's contents. An earlier attempt at
 * this model erased the wrong bytes and reported success - it had assumed
 * every sector was the same size - and the thing that caught it was reading
 * the flash back, not asking the model how it had done. So the tests below
 * read the bytes: before, after, and in the sector next door.
 */
static void test_the_flash_controllers_success_path(void)
{
    reset_registers();

    /* A part that has never been written, at both ends of the region. */
    expect("a part that has never been written is all ones",
           AK_REG32(FLASH_REGION_BASE) == 0xFFFFFFFFu &&
               AK_REG32(FLASH_REGION_BASE + FLASH_REGION_SIZE - 4u) ==
                   0xFFFFFFFFu);

    /* The pair an erase has to be: the sector is emptied first, and the words
     * then land where they were addressed. */
    uint32_t pattern[3] = { 0x414B0001u, 0x00000002u, 0xDEADBEEFu };
    expect("a sector erases", ak_flash_erase_sector(11u) == 0);
    expect("and a program puts the words where they were addressed",
           ak_flash_program(0x080E0000u, pattern, sizeof pattern) == 0 &&
               AK_REG32(0x080E0000u) == pattern[0] &&
               AK_REG32(0x080E0004u) == pattern[1] &&
               AK_REG32(0x080E0008u) == pattern[2]);

    /* Programming can only clear bits in an erased word. A second write to the
     * same address is what a save that never erased looks like, and the part
     * refuses it - which the driver reports rather than assuming, and which
     * leaves the word that was there alone. */
    expect("programming over a word that is already programmed is refused",
           ak_flash_program(0x080E0000u, pattern, 4u) < 0 &&
               AK_REG32(0x080E0000u) == pattern[0]);

    /*
     * The geometry, which is what the withdrawn attempt got wrong. A 1 MB
     * STM32F405 is four sectors of 16 KB, one of 64 KB and seven of 128 KB:
     * erasing sector 3 must stop at 0x0800C000 + 16 KB and leave sector 4
     * alone. A model with one sector size erases somewhere else entirely and
     * says it did the job, and the driver cannot tell - it passes a sector
     * number and the part decodes it.
     */
    expect("a 16 KB sector holds a word and its neighbour holds another",
           ak_flash_erase_sector(3u) == 0 &&
               ak_flash_erase_sector(4u) == 0 &&
               ak_flash_program(0x0800C000u, pattern, 4u) == 0 &&
               ak_flash_program(0x08010000u, pattern, 4u) == 0 &&
               AK_REG32(0x0800C000u) == pattern[0] &&
               AK_REG32(0x08010000u) == pattern[0]);
    expect("erasing sector 3 empties it",
           ak_flash_erase_sector(3u) == 0 &&
               AK_REG32(0x0800C000u) == 0xFFFFFFFFu &&
               AK_REG32(0x0800C000u + 0x3FFCu) == 0xFFFFFFFFu);
    expect("and leaves sector 4 where it was",
           AK_REG32(0x08010000u) == pattern[0]);

    /*
     * And the whole configuration record over that pair - the thing the
     * saved settings have always depended on and that no test could reach:
     * the sector emptied, the record programmed into it, and the rest of the
     * sector still one, which is what proves the erase happened before the
     * write rather than instead of it.
     */
    char buffer[AK_PARAMS_TEXT_MAX + 1];
    const char *text = "airframe=1\nrth_enable=1\n";
    /* The length of the text, taken from the text. It was a literal 26 here
     * and the string is 24 bytes plus its terminator, so the save read two
     * bytes past the end of a string literal - which passed, because the byte
     * after the terminator went into a field nothing compared. The suite's
     * address- and undefined-behaviour-sanitizer run is what named it; the
     * AT32's version of this check has always used `strlen`, which is why this
     * is the one that had it. */
    const uint32_t length = (uint32_t)strlen(text);

    expect("an erased part has nothing saved",
           ak_flash_erase_sector(11u) == 0 &&
               ak_board_config_read(buffer, sizeof buffer) == 0);
    expect("a save erases the sector it writes into and reads back",
           ak_board_config_write(text, length) == 0 &&
               ak_board_config_read(buffer, sizeof buffer) == (int)length &&
               strcmp(buffer, text) == 0);
    expect("with the rest of the sector still erased",
           AK_REG32(AK_CONFIG_BASE + 4096u) == 0xFFFFFFFFu &&
               AK_REG32(AK_CONFIG_BASE + 0xFFFCu) == 0xFFFFFFFFu);

    /*
     * And a save the second time over, in the same sector, without anything
     * having touched it in between: this is the case that needs the erase,
     * because every word of the record is now programmed and the part would
     * refuse every one of them.
     */
    expect("saving again over a sector that is not erased still works, "
           "because the save erases it",
           ak_board_config_write("airframe=2\n", 11u) == 0 &&
               ak_board_config_read(buffer, sizeof buffer) == 11 &&
               strcmp(buffer, "airframe=2\n") == 0);
}

/*
 * The blackbox, written into the part's own flash.
 *
 * This is the whole path in one place: the board's region and its three
 * adapters, the flash driver, and the modelled controller underneath them.
 * The log's own logic - the ring, the wrap, the checksums - is tested against
 * a store the test owns in tests/test_log.c; what this adds is that the thing
 * the board hands over is the region this file believes it is, and that a
 * record written through the real driver lands in the real bytes.
 */

static unsigned dump_lines;

/* Counts the lines of the CSV rather than the calls the dump makes: the
 * formatter is free to build a line out of two calls, and a test that counted
 * calls would be testing that instead. */
static int count_lines(const char *fmt, ...)
{
    char line[256];
    va_list ap;

    va_start(ap, fmt);
    int written = vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);

    for (int i = 0; i < written && (unsigned)i < sizeof line; i++) {
        if (line[i] == '\n') {
            dump_lines++;
        }
    }
    return written;
}

static ak_log_record_t arch_log_record(uint32_t time_ms, uint8_t state)
{
    ak_log_record_t record;

    memset(&record, 0, sizeof record);
    record.time_ms = time_ms;
    record.state = state;
    record.accel[0] = (int16_t)(time_ms & 0x7FFFu);
    record.motor[0] = (uint8_t)(time_ms & 0xFFu);
    return record;
}

static void test_the_blackbox_in_flash(void)
{
    reset_registers();
    ak_flashlog_t log;
    const ak_flashlog_store_t *store = ak_board_log_store();

    /* The region the board hands over: five 128 KB sectors starting where the
     * image ends, stopping where the saved configuration's first bank begins.
     * It was six before the configuration took one for a second bank - see the
     * board's own section comment - and the number is here so that another
     * sector moving has to change this line. */
    expect("the board offers a region for the log",
           store != 0 && store->count == 5u);
    expect("it starts where the image ends",
           store->regions[0].base == 0x08020000u &&
               store->regions[0].bytes == 0x20000u);
    expect("and it stops at the configuration's first bank",
           store->regions[4].base + store->regions[4].bytes == AK_CONFIG_BASE0);

    /* A record, through the driver, into the mapped flash. */
    expect("an erased part is an empty log",
           ak_flashlog_resume(&log, store) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&log) == 0u && log.slots == 2184u);

    ak_log_record_t record = arch_log_record(4242u, 1u);
    expect("a record goes into flash",
           ak_flashlog_push(&log, &record) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&log) == 1u);
    expect("the sector header is the one the format describes",
           AK_REG32(0x08020000u) == AK_FLASHLOG_MAGIC &&
               AK_REG32(0x08020008u) == 0u &&      /* sector index */
               AK_REG32(0x0802000Cu) == 1u);       /* sequence */
    expect("and the slot holds the record, marker first",
           AK_REG32(0x08020020u) == AK_FLASHLOG_SLOT_MAGIC &&
               AK_REG32(0x08020028u) == record.time_ms);

    dump_lines = 0u;
    expect("reading it back finds the record where it was written",
           ak_flashlog_dump(&log, count_lines) == 1u && dump_lines == 5u);

    /* The two sectors above are the saved configuration's banks, and the log
     * must not touch either: one word programmed into each before the log ran,
     * still there after a record, a dump and a clear. The lower one is the
     * interesting boundary now - the log's region ends exactly at it. */
    uint32_t marker = 0x434F4E46u; /* "CONF" */
    expect("both configuration banks are written",
           ak_flash_erase_sector(10u) == 0 &&
               ak_flash_erase_sector(11u) == 0 &&
               ak_flash_program(AK_CONFIG_BASE0, &marker, 4u) == 0 &&
               ak_flash_program(AK_CONFIG_BASE, &marker, 4u) == 0);
    ak_log_record_t next = arch_log_record(4243u, 1u);
    (void)ak_flashlog_push(&log, &next);
    (void)ak_flashlog_dump(&log, count_lines);
    expect("and the log has not touched either",
           AK_REG32(AK_CONFIG_BASE0) == marker &&
               AK_REG32(AK_CONFIG_BASE) == marker);

    expect("clearing the log erases the whole region and nothing else",
           ak_flashlog_clear(&log) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&log) == 0u &&
               AK_REG32(0x08020000u) == 0xFFFFFFFFu &&
               AK_REG32(0x080A0000u) == 0xFFFFFFFFu &&
               AK_REG32(AK_CONFIG_BASE0) == marker &&
               AK_REG32(AK_CONFIG_BASE) == marker);

    /*
     * And the wrap, with the real sector size: 2184 records fill a sector
     * exactly, the next one needs the sector after it - which on a part that
     * has been somebody else's is not erased - and the log stops until it is
     * given a safe moment to erase it.
     */
    expect("a fresh log starts again", ak_flashlog_clear(&log) == AK_FLASHLOG_OK);
    for (uint32_t i = 0u; i < 2184u; i++) {
        ak_log_record_t each = arch_log_record(i, 1u);

        if (ak_flashlog_push(&log, &each) != AK_FLASHLOG_OK) {
            break;
        }
    }
    expect("a sector holds exactly 2184 records",
           ak_flashlog_count(&log) == 2184u && log.slot == 2184u);

    /* Something in the next sector that the log has to get rid of first. */
    uint32_t other = 0x12345678u;
    expect("the next sector holds somebody else's data",
           ak_flash_program(0x08040000u, &other, 4u) == 0);
    next = arch_log_record(9999u, 1u);
    expect("so the log stops rather than erasing in flight",
           ak_flashlog_push(&log, &next) == AK_FLASHLOG_NEEDS_ERASE);
    expect("and it stays stopped while the aircraft is flying",
           ak_flashlog_service(&log, 0) == AK_FLASHLOG_NEEDS_ERASE);
    expect("on the ground it erases the sector it needs",
           ak_flashlog_service(&log, 1) == AK_FLASHLOG_OK &&
               AK_REG32(0x08040000u) == 0xFFFFFFFFu);
    next = arch_log_record(10000u, 2u);
    expect("and the next record goes into it",
           ak_flashlog_push(&log, &next) == AK_FLASHLOG_OK &&
               log.sector == 1u && log.slot == 1u &&
               AK_REG32(0x0804000Cu) == 2u); /* the second sector's sequence */

    /* Reading it all back: 2184 records went into the first sector and one
     * into the second, and the dump finds every one of them. */
    dump_lines = 0u;
    expect("the dump counts both sectors' records",
           ak_flashlog_dump(&log, count_lines) == 2185u && dump_lines == 2189u);

    /* And by index, which is how a tool pulls it: the arithmetic has to cross
     * a sector boundary, because that is where an off-by-one would hide - the
     * last record of the first sector and the first of the second are 2183 and
     * 2184 in the log, and 2183 and 0 in their sectors. */
    ak_log_record_t got;
    expect("index 0 is the oldest record in the first sector",
           ak_flashlog_record_at(&log, 0u, &got) == 1 && got.time_ms == 0u);
    expect("and the index keeps counting across the boundary",
           ak_flashlog_record_at(&log, 2183u, &got) == 1 &&
               got.time_ms == 2183u &&
               ak_flashlog_record_at(&log, 2184u, &got) == 1 &&
               got.time_ms == 10000u);
    expect("with nothing past the end",
           ak_flashlog_record_at(&log, 2185u, &got) == 0);
}

/*
 * The sensor bus and the flight pack's ADC.
 *
 * Both are pure register configuration, so both execute against a mapped
 * region without any of the flash controller's trouble. The SPI side is the
 * bus the inertial sensor answers on - no IMU is no arming, so a bit wrong
 * here is an aircraft that never leaves the ground - and the ADC side is the
 * flight pack, where the trap is that the sample-time register is chosen by
 * channel number and the two registers split at channel ten.
 */
static void test_the_sensor_bus(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_spi_init(AK_BOARD_IMU_SPI, AK_BOARD_IMU_SCK, AK_BOARD_IMU_MISO,
                AK_BOARD_IMU_MOSI, AK_BOARD_IMU_AF);

    expect("the spi port's clock is on",
           (RCC_APB1ENR & (1u << 14)) != 0u);

    /*
     * One register, and every bit of it is load-bearing: master, mode 3 (both
     * CPOL and CPHA), software slave management with the internal select held
     * high, the clock at APB1 over eight, and the peripheral enabled last.
     * The total is asserted rather than the fields, so a bit somebody adds
     * later is a failure rather than a silent change of bus behaviour.
     */
    expect("spi2 is a master, mode 3, software-selected, APB1/8, enabled",
           SPI_CR1(SPI2_BASE) == 0x0357u);
    expect("and it is a 16-bit-free, 8-bit, motorola frame",
           SPI_CR2(SPI2_BASE) == 0u);

    /* The pins, on the alternate function the sensor bus uses; MISO is pulled
     * up because nothing drives it when the sensor is not selected. */
    expect("sck, miso and mosi are on the spi alternate function",
           ((GPIO_MODER(GPIOB_BASE) >> (13u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOB_BASE) >> (14u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOB_BASE) >> (15u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_AFRH(GPIOB_BASE) >> 20) & 0xFFFu) == 0x555u);
    expect("miso is pulled up and the clock and data lines are not",
           ((GPIO_PUPDR(GPIOB_BASE) >> (14u * 2u)) & 0x3u) ==
               GPIO_PUPD_PULLUP &&
               ((GPIO_PUPDR(GPIOB_BASE) >> (13u * 2u)) & 0x3u) ==
                   GPIO_PUPD_NONE &&
               ((GPIO_PUPDR(GPIOB_BASE) >> (15u * 2u)) & 0x3u) ==
                   GPIO_PUPD_NONE);

    /* A bus whose flags never move. There is no sensor on the bench board, and
     * the failure that matters is the firmware waiting for one: the transfer
     * gives up and says so rather than spinning in a bring-up step. */
    uint8_t out[4] = { 1u, 2u, 3u, 4u };
    uint8_t in[4] = { 0u, 0u, 0u, 0u };
    expect("a transfer on a bus that never reports a flag gives up",
           ak_spi_transfer_loop(SPI2_BASE, out, in, 4u) < 0);
    expect("and a transfer of nothing is not a transfer",
           ak_spi_transfer_loop(SPI2_BASE, out, in, 0u) == 0);

    /*
     * And the same loop with a part that answers, which is the half the model
     * does not reach: on a target `ak_spi_transfer()` *is* this loop, and the
     * host build routes that entry point to the modelled device instead - so
     * on this port the loop had only ever been executed down its failure path.
     * (The AT32's test has driven it through the mapped block since it was
     * written, which is the asymmetry this closes.)
     *
     * The flags stand in for the part: a page of memory cannot shift a byte
     * out and in, but the one thing the loop needs from a bus is that the data
     * register it wrote is the one it reads back - which is exactly what a
     * wire from MOSI to MISO gives a person, and the loopback the bring-up
     * checklist asks somebody to fit. What the sensor bus *does* answer with
     * is the model's business (tests/host_spi_model.c).
     */
    SPI_SR(SPI2_BASE) = SPI_SR_TXE | SPI_SR_RXNE;
    expect("the loop a target runs moves every byte through the data register",
           ak_spi_transfer_loop(SPI2_BASE, out, in, sizeof out) == 0 &&
               in[0] == out[0] && in[1] == out[1] && in[2] == out[2] &&
               in[3] == out[3]);
    expect("and the last byte written is the one left in the register",
           (SPI_DR(SPI2_BASE) & 0xFFu) == out[3]);
    expect("and a read-only transfer still sends something to shift out",
           ak_spi_transfer_loop(SPI2_BASE, 0, in, 1u) == 0 &&
               (SPI_DR(SPI2_BASE) & 0xFFu) == 0xFFu);

    /* A part that takes the byte and never says it has one back: the transfer
     * is lost and reported, not completed early. */
    SPI_SR(SPI2_BASE) = SPI_SR_TXE;
    expect("a byte that is never acknowledged costs the transfer, not the "
           "firmware",
           ak_spi_transfer_loop(SPI2_BASE, out, in, 1u) < 0);
    SPI_SR(SPI2_BASE) = SPI_SR_TXE | SPI_SR_RXNE;

    /* And the other two controllers on the part, whose clock bits had never
     * been written: this board uses SPI2, and a port that can only bring up the
     * one its own board happens to use fails on the next board rather than
     * here. */
    ak_spi_init(SPI1_BASE, AK_BOARD_IMU_SCK, AK_BOARD_IMU_MISO,
                AK_BOARD_IMU_MOSI, AK_BOARD_IMU_AF);
    expect("the first controller's clock can be turned on",
           (RCC_APB2ENR & (1u << 12)) != 0u);
    ak_spi_init(SPI3_BASE, AK_BOARD_IMU_SCK, AK_BOARD_IMU_MISO,
                AK_BOARD_IMU_MOSI, AK_BOARD_IMU_AF);
    expect("and the third's",
           (RCC_APB1ENR & (1u << 15)) != 0u);

    /* And a base address that is not one of the three: refused before anything
     * is written anywhere, which is the same guard the I2C port grew. */
    ak_spi_init(0x40000000u, AK_BOARD_IMU_SCK, AK_BOARD_IMU_MISO,
                AK_BOARD_IMU_MOSI, AK_BOARD_IMU_AF);
    expect("a controller that is not one of the three changes nothing",
           (RCC_APB1ENR & (1u << 15)) != 0u);
}

static void test_the_flight_packs_adc(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_adc_init(ADC1_BASE, AK_BOARD_VBAT_CHANNEL);

    expect("the adc clock is on and its prescaler is four",
           (RCC_APB2ENR & RCC_APB2ENR_ADC1EN) != 0u &&
               ((ADC_CCR >> 16) & 0x3u) == 1u);
    expect("the sequence is one long and holds the battery's channel",
           ADC_SQR1(ADC1_BASE) == 0u &&
               (ADC_SQR3(ADC1_BASE) & 0x1Fu) == AK_BOARD_VBAT_CHANNEL);
    expect("end of conversion is on each conversion, not each sequence",
           (ADC_CR2(ADC1_BASE) & ADC_CR2_EOCS) != 0u);
    expect("and the adc is switched on",
           (ADC_CR2(ADC1_BASE) & ADC_CR2_ADON) != 0u);

    /*
     * The trap: the sample time register is chosen by channel number, and the
     * two split at ten. Channel ten lives in SMPR1; a driver that wrote SMPR2
     * for it would leave the part sampling with the reset value, which is the
     * shortest time it has and the wrong one for a divider this weak.
     */
    expect("channel ten's sample time is in the first register",
           ADC_SMPR1(ADC1_BASE) == ADC_SMPR1_SMP(10u) &&
               ADC_SMPR2(ADC1_BASE) == 0u);

    /* And a channel on the other side of the split, to show the choice is a
     * choice. */
    ak_adc_init(ADC2_BASE, 3u);
    expect("a channel below ten is configured in the second register",
           ADC_SMPR2(ADC2_BASE) == ADC_SMPR2_SMP(3u) &&
               ADC_SMPR1(ADC2_BASE) == 0u);
    expect("and its own controller's clock is the one that was enabled",
           (RCC_APB2ENR & (1u << 9)) != 0u);

    /*
     * A conversion that finishes, which is the path a mapped block cannot
     * produce - a write to a register does not make the part finish - and the
     * one this board's pack voltage is read on. The counts go in the data
     * register the way the hardware would put them there.
     */
    uint16_t counts = 0xFFFFu;
    ADC_DR(ADC1_BASE) = 2000u;
    expect("a finished conversion is read from the data register",
           ak_adc_read_counts(ADC1_BASE, &counts) == 0 && counts == 2000u);

    /* And one that never finishes is a timeout, not a reading of the last value
     * in the data register: what a converter that is not running looks like. */
    host_f4adc_set_never_finishes(1);
    counts = 0xFFFFu;
    expect("a conversion that never finishes is reported, not read",
           ak_adc_read_counts(ADC1_BASE, &counts) < 0 && counts == 0xFFFFu);
    host_f4adc_set_never_finishes(0);
    expect("and it is a reading again once the part answers",
           ak_adc_read_counts(ADC1_BASE, &counts) == 0 && counts == 2000u);
    expect("and a controller this port does not know is refused",
           ak_adc_read_counts(0x40012010u, &counts) < 0);

    /* And the third controller on the part, whose clock bit had never been
     * written: this board uses ADC1, and a port that can only bring up the one
     * its board happens to use fails on the next board rather than here. */
    ak_adc_init(ADC3_BASE, 4u);
    expect("the third controller's clock can be turned on",
           (RCC_APB2ENR & (1u << 10)) != 0u);
}

/*
 * The saved configuration, which is board code and the one piece of the
 * firmware whose failure is "your settings were not kept".
 *
 * The record is four bytes of magic, a length, a checksum and the text, in the
 * last sector of the part. What matters is not that it saves - that needs the
 * flash controller - but that it *refuses*. An erased sector must read as
 * "nothing saved" rather than as a configuration of zeros, and a record whose
 * bytes do not match its checksum must be rejected rather than half-applied,
 * because the write that produced it was interrupted by a power failure or a
 * reset and what is in the sector is a prefix of what was meant to be there.
 *
 * The checksum here is written out from the algorithm's own constants - FNV-1a,
 * offset basis 2166136261, prime 16777619 - rather than borrowed from the
 * board file, so the two agreeing means the record's fields are where the
 * caller thinks they are.
 */
static uint32_t fnv1a(const char *text, uint32_t length)
{
    uint32_t sum = 2166136261u;
    for (uint32_t i = 0; i < length; i++) {
        sum = (sum ^ (uint8_t)text[i]) * 16777619u;
    }
    return sum;
}

/* One record, in one slot of the ring, written in the order the board programs
 * it: magic, serial, length, sum, text. */
static void put_record(unsigned slot, uint32_t serial, uint32_t magic,
                       uint32_t length, uint32_t sum, const char *text,
                       uint32_t text_length)
{
    uint32_t *word = (uint32_t *)(uintptr_t)(AK_CONFIG_BASE + slot * 4096u);
    word[0] = magic;
    word[1] = serial;
    word[2] = length;
    word[3] = sum;
    char *out = (char *)(uintptr_t)(AK_CONFIG_BASE + slot * 4096u + 16u);
    for (uint32_t i = 0; i < text_length; i++) {
        out[i] = text[i];
    }
}

static void test_the_saved_configuration(void)
{
    reset_registers();
    /* The record lives in the last sector of the part, which is inside the
     * whole flash region this file maps - so a record written here by hand is
     * a record the board's own reader finds, the same way the modelled
     * controller's writes are. */

    char buffer[AK_PARAMS_TEXT_MAX + 1];
    const char *text = "airframe=1\nrth_enable=1\n";
    /* From the text, as above and for the same reason: the literal that used
     * to be here was two bytes longer than the string, so the checksum read
     * past the end of it. */
    uint32_t length = (uint32_t)strlen(text);

    /*
     * The ring, in the order the states happen on a board.
     */

    /* An erased sector: every slot all ones, which is what a part looks like
     * after an erase and what a board nobody has saved to looks like. It reads
     * as "none" rather than as a configuration, which is the whole reason the
     * magic exists. */
    memset((void *)AK_CONFIG_BASE, 0xFF, 128u * 1024u);
    expect("an erased sector is nothing saved, not a configuration",
           ak_board_config_read(buffer, sizeof buffer) == 0);

    /* A record that was written completely, in the ring's first slot. */
    put_record(0u, 1u, 0x414B4346u, length, fnv1a(text, length), text, length);
    expect("a complete record reads back",
           ak_board_config_read(buffer, sizeof buffer) == (int)length &&
               strcmp(buffer, text) == 0);

    /* And the save after it: two records in the ring, and the *newer* one is
     * what loads. The serial is what decides that - the order in the ring is
     * not the order of time once it has wrapped. */
    const char *second = "airframe=2\n";
    uint32_t second_length = (uint32_t)strlen(second);
    put_record(1u, 2u, 0x414B4346u, second_length,
               fnv1a(second, second_length), second, second_length);
    expect("and the newest record in the ring is the one that loads",
           ak_board_config_read(buffer, sizeof buffer) == (int)second_length &&
               strcmp(buffer, second) == 0);

    /* A save that was interrupted: the newest slot's text does not match its
     * own sum, which is what a power cut or a failed program leaves. That is
     * one invalid slot, not a damaged board - the record before it is still a
     * configuration, and *that* is the difference this layout is for. */
    ((char *)(AK_CONFIG_BASE + 4096u + 16u))[2] ^= 0x20;
    expect("a torn newest record falls back to the one before it",
           ak_board_config_read(buffer, sizeof buffer) == (int)length &&
               strcmp(buffer, text) == 0);

    /* And a part whose records are all damaged says so rather than pretending
     * nobody has saved here: the magic is neither ours nor erased. */
    memset((void *)AK_CONFIG_BASE, 0x00, 128u * 1024u);
    expect("a sector of damaged records is an error, not an empty part",
           ak_board_config_read(buffer, sizeof buffer) < 0);

    /* The other ways a record can be wrong, each in an erased sector. */
    memset((void *)AK_CONFIG_BASE, 0xFF, 128u * 1024u);
    put_record(0u, 1u, 0x414B4346u, 0u, 0u, text, 0u);
    expect("a record of no length is refused",
           ak_board_config_read(buffer, sizeof buffer) < 0);
    memset((void *)AK_CONFIG_BASE, 0xFF, 128u * 1024u);
    put_record(0u, 1u, 0x414B4346u, AK_PARAMS_TEXT_MAX + 1u, 0u, text, 0u);
    expect("a length past the end of the record is refused",
           ak_board_config_read(buffer, sizeof buffer) < 0);
    memset((void *)AK_CONFIG_BASE, 0xFF, 128u * 1024u);
    put_record(0u, 1u, 0x414B4346u, length, fnv1a(text, length), text, length);
    expect("and a buffer too small for it is refused rather than truncated",
           ak_board_config_read(buffer, 4u) < 0);

    /*
     * And the guarantee the ring exists for, through the board's own save: a
     * write the flash controller refuses leaves the record that was there
     * readable. The old single-record save erased the sector first, so this
     * failure - a refused program, a sagging pack, somebody pulling the cable -
     * cost the aircraft its settings.
     */
    memset((void *)AK_CONFIG_BASE, 0xFF, 128u * 1024u);
    expect("a save writes the record and a load gives it back",
           ak_board_config_write(text, length) == 0 &&
               ak_board_config_read(buffer, sizeof buffer) == (int)length &&
               strcmp(buffer, text) == 0);
    host_flash_model_set_locked(0); /* somebody else has the controller open */
    expect("a save the flash controller refuses is reported, not assumed",
           ak_board_config_write(second, second_length) < 0);
    expect("and the configuration it could not replace is still readable",
           ak_board_config_read(buffer, sizeof buffer) == (int)length &&
               strcmp(buffer, text) == 0);
    host_flash_model_set_locked(1);

    /* And the failure the ring is *for*: this controller erases happily and
     * refuses to program, which is what a sagging supply or a worn cell looks
     * like from the driver's side. The single-record save erased the only copy
     * first, so this failure used to cost the aircraft its settings - the
     * check that this is not that is the one below. */
    host_flash_model_set_program_refused(1);
    expect("a save whose program is refused is reported", 
           ak_board_config_write(second, second_length) < 0);
    expect("and the record it was replacing is still the one that loads",
           ak_board_config_read(buffer, sizeof buffer) == (int)length &&
               strcmp(buffer, text) == 0);
    host_flash_model_set_program_refused(0);

    /* And the ring comes round: thirty-three saves, one bulk erase when the
     * thirty-two slots are used, and the last record still loads. */
    memset((void *)AK_CONFIG_BASE, 0xFF, 128u * 1024u);
    int saved = 1;
    char latest[16] = {0};
    for (unsigned i = 0; i < 33u; i++) {
        snprintf(latest, sizeof latest, "n=%u\n", i);
        if (ak_board_config_write(latest, (uint32_t)strlen(latest)) != 0) {
            saved = 0;
        }
    }
    expect("thirty-three saves come round the ring and the last one loads",
           saved == 1 &&
               ak_board_config_read(buffer, sizeof buffer) == 5 &&
               strcmp(buffer, "n=32\n") == 0);
}

/*
 * The sensor bus, running transactions.
 *
 * The timing arithmetic is pinned in tests/test_i2c.c against RM0090, and this
 * is the other half: the driver's *sequences*, against a modelled bus with a
 * device on it. There is no barometer on the bench board, so until this
 * existed the start, the address, the register number, the repeated start and
 * the acknowledge that moves for a two-byte read had never run anywhere -
 * tests/test_i2c.c computes divisors, it does not talk to anything.
 *
 * The device is shaped like the barometer the board would carry: a who-am-i, a
 * sixteen-bit coefficient, a burst of three, and a register file underneath.
 */
static void test_the_sensor_bus_transactions(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_i2c_init(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);

    static const uint8_t device_registers[8] = {
        0x50u, /* 0x00: who am i */
        0x01u,
        0x12u, 0x34u, /* 0x02: a sixteen-bit coefficient */
        0xAAu, 0xBBu, 0xCCu, /* 0x04: a burst of three */
    };
    int device = host_i2c_attach(AK_BOARD_BARO_ADDRESS, device_registers,
                                 sizeof device_registers);
    expect("a device can be attached to the modelled bus", device >= 0);

    /* One byte: the who-am-i, which is the read with the NACK and the STOP
     * before the byte arrives. */
    uint8_t value = 0u;
    expect("a one-byte read returns the register it named",
           ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x00u,
                           &value, 1) == 0 &&
               value == 0x50u);
    expect("and the device saw its own address",
           host_i2c_last_address() == AK_BOARD_BARO_ADDRESS &&
               host_i2c_addressed((unsigned)device));

    /* Two bytes: the POS case, where the acknowledge has to move or the
     * master waits for a third byte the slave was told not to send. */
    uint8_t pair[2] = { 0u, 0u };
    expect("a two-byte read returns both bytes in order",
           ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x02u,
                           pair, 2) == 0 &&
               pair[0] == 0x12u && pair[1] == 0x34u);

    /* Three: the burst path, where the acknowledge is held until two bytes
     * are left and the register pointer walks on its own. */
    uint8_t burst[3] = { 0u, 0u, 0u };
    expect("a longer read walks the register pointer",
           ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x04u,
                           burst, 3) == 0 &&
               burst[0] == 0xAAu && burst[1] == 0xBBu && burst[2] == 0xCCu);
    expect("and the count follows",
           host_i2c_last_read() == 3u && host_i2c_last_register() == 0x07u);

    /* A write: register number, then the value, ending on BTF rather than TXE
     * so the STOP does not arrive while the last byte is still going out. */
    expect("a write reaches the device's register",
           ak_i2c_write_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x01u,
                            0x5Au) == 0 &&
               host_i2c_register((unsigned)device, 0x01u) == 0x5Au);
    expect("and the device was told which register",
           host_i2c_last_written() == 2u);

    /* An address nobody answers: the driver has to notice, clear the flag, and
     * leave the bus able to try again - which is the path a barometer that is
     * not fitted exercises every time. */
    expect("an address nobody answers is an error",
           ak_i2c_read_reg(AK_BOARD_BARO_I2C, 0x60u, 0x00u, &value, 1) < 0);
    expect("and the bus still works afterwards",
           ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x00u,
                           &value, 1) == 0 &&
               value == 0x50u);
}

/*
 * The bus that stops answering.
 *
 * i2c.c promises two things about a sensor that dies while the aircraft is
 * flying: every wait is bounded, so it cannot stop the flight loop, and
 * reaching the bound resets the peripheral, so the next transaction is not
 * taken as a continuation of the half-finished one. Neither promise had ever
 * been *reached* here - the modelled bus always answered, and the one failure
 * the tests above exercise is an address nobody answers, which the part itself
 * reports with AF.
 *
 * So the model stops answering after a chosen number of status flags, and this
 * walks the count up: zero is a bus that never comes up at all, and each step
 * beyond it lets one more wait succeed before it dies - the start, the address,
 * the register number, the repeated start, the byte a one-byte read is waiting
 * for, the pair a two-byte read waits for, the bytes of a burst. Each of them
 * has to come back as an error rather than a hang, and each has to leave a bus
 * that works when it answers again - which is what the sensor coming back, or
 * the next boot, needs it to be.
 */
static void test_the_bus_that_stops_answering(void)
{
    static const uint8_t device_registers[8] = {
        0x50u, 0x01u, 0x12u, 0x34u, 0xAAu, 0xBBu, 0xCCu, 0x00u,
    };
    unsigned died = 0u;
    unsigned answered = 0u;

    for (unsigned flags = 0u; flags <= 14u; flags++) {
        char what[112];
        uint8_t value = 0u;
        uint8_t pair[2] = { 0u, 0u };
        uint8_t burst[3] = { 0u, 0u, 0u };

        reset_registers();
        ak_test_f405_crystal(1);
        ak_clk_init();
        host_i2c_model_reset();
        host_i2c_model_stall_after(flags);
        ak_i2c_init(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                    AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);
        (void)host_i2c_attach(AK_BOARD_BARO_ADDRESS, device_registers,
                              sizeof device_registers);

        snprintf(what, sizeof what,
                 "a bus that answers %u flag%s and stops gives an error on a "
                 "one-byte read, not a hang",
                 flags, flags == 1u ? "" : "s");
        if (ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x00u,
                            &value, 1) < 0) {
            died++;
            expect(what, 1);
        } else {
            /* Far enough along the sequence the transaction completes, which
             * is the control this sweep needs: a read that could never succeed
             * would make every line above pass for the wrong reason. */
            answered++;
            expect("a bus that answers every flag gives the reading instead",
                   value == 0x50u);
        }

        /* The same bus, the other two shapes of read and a write: each of them
         * waits on a different flag in a different place, and each has to give
         * up rather than return a number nobody sent.
         *
         * The stall is re-armed before each one, which is not tidiness: once a
         * transaction has failed at a given budget, every *later* transaction
         * on the same bus fails at its first wait, so without this the two-byte
         * and burst paths would never be reached - their waits are further
         * along the sequence than the one-byte read's. */
        host_i2c_model_stall_after(flags);
        snprintf(what, sizeof what,
                 "and a two-byte read with %u flag%s answered is an error or "
                 "the right pair, never a wrong reading",
                 flags,
                 flags == 1u ? "" : "s");
        expect(what, ak_i2c_read_reg(AK_BOARD_BARO_I2C,
                                     AK_BOARD_BARO_ADDRESS, 0x02u, pair,
                                     2) < 0 || (pair[0] == 0x12u &&
                                                pair[1] == 0x34u));
        host_i2c_model_stall_after(flags);
        snprintf(what, sizeof what,
                 "and a burst of three with %u flag%s answered is an error or "
                 "the right bytes",
                 flags, flags == 1u ? "" : "s");
        expect(what, ak_i2c_read_reg(AK_BOARD_BARO_I2C,
                                     AK_BOARD_BARO_ADDRESS, 0x04u, burst,
                                     3) < 0 || (burst[0] == 0xAAu &&
                                                burst[1] == 0xBBu &&
                                                burst[2] == 0xCCu));
        host_i2c_model_stall_after(flags);
        snprintf(what, sizeof what,
                 "and a write with %u flag%s answered is an error or the "
                 "value that arrived",
                 flags,
                 flags == 1u ? "" : "s");
        expect(what, ak_i2c_write_reg(AK_BOARD_BARO_I2C,
                                      AK_BOARD_BARO_ADDRESS, 0x01u, 0x5Au) < 0 ||
                         host_i2c_register(0u, 0x01u) == 0x5Au);

        /* The promise that matters after the error: the peripheral was reset,
         * so with the bus answering again the next transaction is a new one -
         * this is the sensor coming back, or the next boot, or the second half
         * of a flight after a wire was knocked. */
        host_i2c_model_stall_after(HOST_I2C_ANSWERS_ALL);
        value = 0u;
        snprintf(what, sizeof what,
                 "and the bus works again after %u flag%s", flags,
                 flags == 1u ? "" : "s");
        expect(what, ak_i2c_read_reg(AK_BOARD_BARO_I2C,
                                     AK_BOARD_BARO_ADDRESS, 0x00u, &value,
                                     1) == 0 &&
                         value == 0x50u);
        expect("and the peripheral it left behind is enabled",
               (host_i2c_read(AK_BOARD_BARO_I2C, I2C_OFF_CR1) & I2C_CR1_PE) !=
                   0u);
    }

    expect("the sweep reached the sequence dying part-way, not only at once",
           died >= 4u);
    expect("and ended on a bus that answers the whole transaction",
           answered >= 1u);

    /* And the two controllers this board does not use, because the enable bit
     * for each of the three is one line and two of them had never run: a port
     * that can only bring up the bus its own board happens to use is a port
     * that will fail on the next board rather than here. */
    ak_i2c_init(I2C2_BASE, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);
    expect("the second controller's clock can be turned on",
           (RCC_APB1ENR & RCC_APB1ENR_I2C2EN) != 0u);
    ak_i2c_init(I2C3_BASE, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);
    expect("and the third's",
           (RCC_APB1ENR & RCC_APB1ENR_I2C3EN) != 0u);
    ak_i2c_init(0x40000000u, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);
    expect("and a base address that is not one of the three changes nothing",
           (RCC_APB1ENR & RCC_APB1ENR_I2C2EN) != 0u);
}

/*
 * The barometer's bus, which is the last port file that can run here.
 *
 * Its timing *arithmetic* is tested in tests/test_i2c.c against RM0090, and
 * what is missing there is the register side: the prescaler and the pins, and
 * what the driver does when a bus does not answer. Both are reachable with a
 * plain region, because neither needs the part to act on a write - the flags
 * simply never move, which is what a bus nobody has plugged into looks like.
 *
 * The pins are the check worth having. SCL and SDA are pulled up by resistors
 * and every device on the bus can only pull them *down*; a push-pull output
 * would be two devices driving one wire in opposite directions the first time
 * two of them spoke.
 */
static void test_the_barometers_bus(void)
{
    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    ak_i2c_init(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);

    expect("the i2c port's clock is on",
           (RCC_APB1ENR & RCC_APB1ENR_I2C1EN) != 0u);

    /*
     * The three numbers the peripheral derives its timing from, at 42 MHz: the
     * frequency register, the fast-mode period for 400 kHz (42 MHz / 3 /
     * 400 kHz is 35), and the rise time (300 ns of 42 MHz is 12, plus one).
     * The arithmetic is pinned in test_i2c.c; this is that it reached the
     * registers at all.
     */
    expect("the frequency register is the apb1 clock in whole megahertz",
           (host_i2c_read(AK_BOARD_BARO_I2C, I2C_OFF_CR2) &
            I2C_CR2_FREQ_MASK) == 42u);
    expect("the period is the fast-mode one for 400 khz",
           host_i2c_read(AK_BOARD_BARO_I2C, I2C_OFF_CCR) == (35u | I2C_CCR_FS));
    expect("and the rise time is the fast-mode one",
           host_i2c_read(AK_BOARD_BARO_I2C, I2C_OFF_TRISE) == 13u);
    expect("the peripheral is enabled and acknowledging",
           (host_i2c_read(AK_BOARD_BARO_I2C, I2C_OFF_CR1) &
            (I2C_CR1_PE | I2C_CR1_ACK)) ==
               (I2C_CR1_PE | I2C_CR1_ACK));

    expect("scl and sda are on the i2c alternate function",
           ((GPIO_MODER(GPIOB_BASE) >> (6u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOB_BASE) >> (7u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               (GPIO_AFRL(GPIOB_BASE) & 0xFF000000u) == 0x44000000u);
    expect("and both are open drain, because nothing on this bus drives high",
           (GPIO_OTYPER(GPIOB_BASE) & ((1u << 6) | (1u << 7))) ==
               ((1u << 6) | (1u << 7)));

    /*
     * The way it fails. A bus with nothing on it never raises a start, so
     * every wait in the driver runs out; what matters is that it says so and
     * puts the peripheral back rather than returning half a reading or
     * spinning. This is the path a board with no barometer fitted takes at
     * every boot.
     */
    uint8_t data = 0;
    expect("a read on a bus that never answers is an error, not a value",
           ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x00u,
                           &data, 1u) < 0);
    expect("and the peripheral was left ready to try again",
           host_i2c_read(AK_BOARD_BARO_I2C, I2C_OFF_CR1) ==
               (I2C_CR1_PE | I2C_CR1_ACK) &&
               host_i2c_read(AK_BOARD_BARO_I2C, I2C_OFF_CCR) ==
                   (35u | I2C_CCR_FS));
    expect("a write on the same bus is an error too",
           ak_i2c_write_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x00u,
                            0x42u) < 0);
    /*
     * And the one place a port is validated. `ak_i2c_init` refuses a base
     * address that is not one of the three controllers - it enables no clock
     * and configures no pin - which is the check the read and write functions
     * *do not* make: they would write start bits into whatever lives at the
     * address they were given and fail on the timeout. Every caller in this
     * firmware passes a board constant, so it is a note rather than a bug, and
     * it is written here so the next person reads it rather than finds it.
     */
    uint32_t enables = RCC_APB1ENR;
    ak_i2c_init(ADC1_BASE, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);
    expect("initialising a controller that is not one of the three does nothing",
           RCC_APB1ENR == enables);

    /*
     * And the read and write functions refuse one too, *before* they touch
     * anything. They used to skip that check: a wrong base address got a start
     * bit and an address byte written into whatever lives there, and then a
     * timeout - the ADC's status register is at exactly the offset I2C's
     * control register one would have been written to, so the sentinel below
     * is what the old code would have scribbled on.
     */
    ADC_SR(ADC1_BASE) = 0x1234u;
    expect("a read from a controller that is not one of the three is refused",
           ak_i2c_read_reg(ADC1_BASE, AK_BOARD_BARO_ADDRESS, 0x00u, &data,
                           1u) < 0 &&
               ADC_SR(ADC1_BASE) == 0x1234u);
    expect("and a write is refused before it reaches the register",
           ak_i2c_write_reg(ADC1_BASE, AK_BOARD_BARO_ADDRESS, 0x00u,
                            0x42u) < 0 &&
               ADC_SR(ADC1_BASE) == 0x1234u);
}

/*
 * A bus a slave is holding down, and what the port does about it.
 *
 * This is the fault the Feather's LSM6DSO produced on 2026-10-01, and it is
 * worth being exact about why it needs a test rather than a bench: it looks
 * *identical* to a dead sensor. SCL reads high, SDA reads low, no address
 * answers, every read times out - the board says `nothing answered on the bus`
 * about a part that is plugged in and working, because the part is stuck
 * half-way through a byte and waiting for a clock edge that never comes.
 *
 * Nothing in the port could reach it. Every timeout path calls SWRST, which
 * resets the *master*; the master was never the problem. The cure is the one
 * in every I2C application note - clock the bus until the slave lets go, then
 * send a STOP - and it is reachable here only because the wires are modelled
 * separately from the peripheral (host_i2c_model.h).
 *
 * The three answers are all checked, because the third is the one that keeps
 * this honest: a recovery that reported success on a shorted wire would turn
 * "the sensor is dead" into "the sensor is fine", which is worse than the
 * fault it was written for.
 */
static void test_the_bus_a_slave_is_holding(void)
{
    /* A part on the bus with anything in its registers: the values are a
     * stand-in for whatever a real one would answer, and what the checks below
     * are about is whether it can be reached at all. Register 0x0F is the one
     * read back, because that is where every one of these parts keeps its
     * identity byte. */
    uint8_t regs[16] = {0};

    regs[0x0Fu] = 0x5Au;

    reset_registers();
    ak_test_f405_crystal(1);
    ak_clk_init();
    host_i2c_model_reset();
    host_i2c_gpio_attach(AK_BOARD_BARO_SCL.port, AK_BOARD_BARO_SCL.pin,
                         AK_BOARD_BARO_SDA.port, AK_BOARD_BARO_SDA.pin);
    host_i2c_attach(AK_BOARD_BARO_ADDRESS, regs, sizeof regs);
    ak_i2c_init(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);

    /* --- the bus is idle, and asking costs nothing ---------------------- */

    expect("a healthy bus is left alone, and the answer says so",
           ak_i2c_bus_recover(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL,
                              AK_BOARD_BARO_SDA, AK_BOARD_BARO_AF,
                              AK_BOARD_BARO_SPEED) == 0);
    expect("and nothing was clocked onto it",
           host_i2c_gpio_clocks() == 0u);

    /* --- SCL held down is not the recoverable case ----------------------- */

    /*
     * Clocking cannot fix a bus whose clock line is being held: the line that
     * would carry the clocks is the one that is stuck. Reporting 0 here rather
     * than burning nine pulses into a line nobody is listening to is what
     * stops the console's health output from claiming a repair it did not
     * make.
     */
    host_i2c_gpio_drive(AK_BOARD_BARO_SCL.port, AK_BOARD_BARO_SCL.pin, 0);
    expect("a bus whose clock is held is not a case clocking can fix",
           ak_i2c_bus_recover(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL,
                              AK_BOARD_BARO_SDA, AK_BOARD_BARO_AF,
                              AK_BOARD_BARO_SPEED) == 0);
    host_i2c_gpio_drive(AK_BOARD_BARO_SCL.port, AK_BOARD_BARO_SCL.pin, 1);

    /* --- the real case: a slave holding SDA, freed on the ninth clock ---- */

    host_i2c_gpio_hold_sda(9u);
    expect("a slave holding sda down is the case this exists for",
           ak_i2c_bus_recover(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL,
                              AK_BOARD_BARO_SDA, AK_BOARD_BARO_AF,
                              AK_BOARD_BARO_SPEED) == 1);
    expect("it took the whole nine clocks to let go, one more than a byte",
           host_i2c_gpio_clocks() == 9u);
    expect("and the stop that ends the frame went out after them",
           host_i2c_gpio_saw_stop() == 1);
    expect("so the part answers afterwards, which is the point of it all",
           ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x0Fu,
                           regs, 1u) == 0 &&
               regs[0] == 0x5Au);

    /* --- a slave that gives up early is not clocked nine times ----------- */

    /*
     * The loop stops as soon as the line comes back. Eight more clocks on a
     * bus that is already fine are eight edges of noise - and on a shared bus,
     * eight edges somebody else's device is trying to interpret.
     */
    host_i2c_model_reset();
    host_i2c_gpio_attach(AK_BOARD_BARO_SCL.port, AK_BOARD_BARO_SCL.pin,
                         AK_BOARD_BARO_SDA.port, AK_BOARD_BARO_SDA.pin);
    host_i2c_gpio_hold_sda(3u);
    expect("a slave that lets go on the third clock frees the bus",
           ak_i2c_bus_recover(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL,
                              AK_BOARD_BARO_SDA, AK_BOARD_BARO_AF,
                              AK_BOARD_BARO_SPEED) == 1);
    expect("and is not clocked the other six times",
           host_i2c_gpio_clocks() == 3u);

    /* --- a wire shorted to ground, which no amount of clocking fixes ----- */

    /*
     * The answer that matters most. A short is not a slave mid-byte, and nine
     * clocks do not touch it; saying -1 is what keeps "the board recovered"
     * from being a claim the firmware makes about hardware that is broken.
     */
    host_i2c_model_reset();
    host_i2c_gpio_attach(AK_BOARD_BARO_SCL.port, AK_BOARD_BARO_SCL.pin,
                         AK_BOARD_BARO_SDA.port, AK_BOARD_BARO_SDA.pin);
    host_i2c_gpio_hold_sda(0u);
    expect("a wire held down and never released is reported as not fixed",
           ak_i2c_bus_recover(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL,
                              AK_BOARD_BARO_SDA, AK_BOARD_BARO_AF,
                              AK_BOARD_BARO_SPEED) == -1);
    expect("after the full nine clocks, not fewer",
           host_i2c_gpio_clocks() == 9u);

    /* --- and the transaction paths ask by themselves --------------------- */

    /*
     * The whole reason this is wired into ak_i2c_read_reg rather than left for
     * a caller: the first access that meets a stuck bus is the one that should
     * free it. A board whose console `imu` verb had to remember to ask would
     * be a board where the fix depends on which verb you typed.
     */
    host_i2c_model_reset();
    host_i2c_gpio_attach(AK_BOARD_BARO_SCL.port, AK_BOARD_BARO_SCL.pin,
                         AK_BOARD_BARO_SDA.port, AK_BOARD_BARO_SDA.pin);
    host_i2c_attach(AK_BOARD_BARO_ADDRESS, regs, sizeof regs);
    host_i2c_gpio_hold_sda(9u);
    expect("a read that meets a stuck bus frees it and then reads",
           ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, 0x0Fu,
                           regs, 1u) == 0);
    expect("and the read came back through the bus it had just freed",
           host_i2c_gpio_clocks() == 9u &&
               host_i2c_last_address() == AK_BOARD_BARO_ADDRESS);
}

/*
 * The USB console.
 *
 * The port's other unusual device: the controller at 0x50000000, where the
 * word that says whose packet arrived and the payload itself live in two
 * different registers. A mapped page is a word rather than a queue, so a
 * payload comes back the same however many times it is read - and that is
 * still enough to drive the whole driver, because it is the status entry, not
 * the FIFO, that says whose packet it is and how long: four bytes of payload
 * are one word, and a setup packet is two words the test fills with the same
 * eight bytes. Enumeration, the descriptors, the class requests a serial driver
 * makes and typing at the console are all reachable that way, which is the
 * difference between this file and the constants in tests/test_regs.c.
 *
 * What this is not: a bus. There is no timing, no DATA0/DATA1 toggle, no
 * retry when the device NAKs, and the core's own side of the contract - that
 * reading the bytes is what drops the entry, that EPENA clears when a transfer
 * completes - is written here as the test's premise rather than observed. The
 * bench is where the two meet.
 */

/* One entry in the receive status queue: the status word in GRXSTSP, the bytes
 * in FIFO 0, and the flag that says there is something to read. The words the
 * bytes make go into the modelled FIFO first, in order, because that order is
 * part of the protocol - reading them out one at a time is what the driver
 * does, and what makes the core drop the entry (tests/host_usb_model.h). */
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

/* Four bytes or fewer, which is one FIFO word - the shape most of the checks
 * below are about. */
static void usb_receive(unsigned endpoint, unsigned kind, uint32_t payload,
                        unsigned count)
{
    host_usb_push(payload);
    OTG_GRXSTSP = endpoint | (count << 4) | (kind << 17);
    OTG_GINTSTS |= OTG_GINT_RXFLVL;
}

/* The host took a packet from an IN endpoint. On the part the transfer-complete
 * interrupt arrives with the endpoint already disabled; a page of memory has to
 * be told that, or the next push sees a transfer still in flight and does
 * nothing - which is what a test that never completed one would conclude. */
static void usb_take_in_packet(unsigned endpoint)
{
    OTG_DIEPCTL(endpoint) &= ~OTG_DEPCTL_EPENA;
    OTG_DIEPINT(endpoint) |= OTG_DEPINT_XFRC;
    ak_usb_poll();
    /* And the register reads clear, which on the part is what writing the bit
     * back to it does - a page of memory has to be told. */
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
 * queued as the two words the FIFO hands back.
 *
 * The first version of this helper wrote the *request* into the byte a host
 * uses for bmRequestType and left the same word in both halves of the packet,
 * which is a packet no host sends. The driver decoded the request out of the
 * wrong byte in a way that matched that, so the two agreed with each other and
 * with nothing else: the board appeared on the host's bus and stalled every
 * request it was sent (`device descriptor read/64, error -32`), and the test
 * could not see it. Every packet below is now the one a host actually sends.
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
    reset_registers();
    ak_usb_init(AK_BOARD_USB_DM, AK_BOARD_USB_DP);

    expect("the usb controller's clock is on",
           (RCC_AHB2ENR & RCC_AHB2ENR_OTGFSEN) != 0u);

    /* PA11 and PA12 are above pin 7, so their alternate function lives in the
     * high register. AF10 is the OTG FS transceiver; anything else is a device
     * that never appears on the bus. */
    expect("both data pins are alternate function 10",
           ((GPIO_MODER(GPIOA_BASE) >> (11u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOA_BASE) >> (12u * 2u)) & 0x3u) ==
                   GPIO_MODE_AF &&
               ((GPIO_AFRH(GPIOA_BASE) >> ((11u - 8u) * 4u)) & 0xFu) == 10u &&
               ((GPIO_AFRH(GPIOA_BASE) >> ((12u - 8u) * 4u)) & 0xFu) == 10u);
    /*
     * The soft disconnect, which is where the bench reading came from.
     *
     * SDIS high holds D+ down, so the device is not on the bus at all. That is
     * the invariant: init leaves it off, and the first poll - the first moment
     * anything is driving the controller - is what puts it on. A device that
     * attached in init instead is one the host may reset and ask for a
     * descriptor while the boot path is still running selftests and blinking an
     * LED, with nothing polling USB to answer it.
     *
     * This assertion required the opposite until the F405 bench session of
     * 2026-09-20: it read `== 0u` and so stated the defect as a requirement.
     */
    expect("the device is held off the bus until something drives it",
           (OTG_DCTL & OTG_DCTL_SDIS) != 0u);
    ak_usb_poll();
    expect("and the first poll is what puts it on the bus",
           (OTG_DCTL & OTG_DCTL_SDIS) == 0u);

    /*
     * The two registers a host reads before it reads anything else, and which
     * this test did not look at until the board said why it had to.
     *
     * GCCFG is the pair that decides whether a host sees a device at all, and
     * they were missing until 2026-09-17 - the driver wrote nothing here, so
     * every check in this file was green while the real device never appeared.
     * The F405's twin in tests/test_arch_at32.c has asserted them since the
     * AT32 needed them; this file is the one that was driving the same core
     * without the guard.
     */
    expect("the core is out of power-down", (OTG_GCCFG & OTG_GCCFG_PWRDWN) != 0u);
    expect("and is not waiting for VBUS sensing",
           (OTG_GCCFG & OTG_GCCFG_NOVBUSSENS) != 0u);

    /*
     * And the turn-around time, which is the same defect one register along.
     * ST's driver writes TRDTIM in `DCD_HandleEnumDone_ISR` - an interrupt
     * handler - and this stack is polled with nothing enabled in the NVIC, so
     * that write never happens here. The AT32 port writes the field itself at
     * init and enumerates; this driver left it at zero, and a host that sees
     * "new full-speed USB device" followed by "device descriptor read/8, error
     * -71" is that zero. See src/arch/stm32f405/usb.c.
     */
    expect("the turn-around time is the reference's five for 48 MHz",
           ((OTG_GUSBCFG >> 10) & 0xFu) == OTG_GUSBCFG_TRDTIM_48MHZ);
    expect("and the core is forced into device mode, full speed",
           (OTG_GUSBCFG & (OTG_GUSBCFG_PHYSEL | OTG_GUSBCFG_FDMOD)) ==
               (OTG_GUSBCFG_PHYSEL | OTG_GUSBCFG_FDMOD));

    /* The FIFOs, in words: 320 is all the part has, and going over it is a
     * device that misbehaves in ways that look like anything but a FIFO. */
    unsigned rx_words = OTG_GRXFSIZ;
    unsigned ep0_words = (OTG_DIEPTXF(0u) >> 16) & 0xFFFFu;
    unsigned ep1_words = (OTG_DIEPTXF(1u) >> 16) & 0xFFFFu;
    unsigned ep2_words = (OTG_DIEPTXF(2u) >> 16) & 0xFFFFu;
    expect("the receive fifo is 128 words, as the budget says", rx_words == 128u);
    expect("and the four fifos fit the 320 words the controller has",
           rx_words + ep0_words + ep1_words + ep2_words <= 320u);
    expect("each transmit fifo starts where the one below it ends",
           (OTG_DIEPTXF(0u) & 0xFFFFu) == rx_words &&
               (OTG_DIEPTXF(1u) & 0xFFFFu) == rx_words + ep0_words &&
               (OTG_DIEPTXF(2u) & 0xFFFFu) ==
                   rx_words + ep0_words + ep1_words);

    /*
     * And where those registers actually are, checked **at the address** rather
     * than through the symbol.
     *
     * That distinction is the point of these two checks. The three above read
     * back through the same macro the driver writes through, so when the macro
     * pointed endpoint 0 at the wrong register, the driver and the test moved
     * together and the suite stayed green. A test that asks a symbol where it
     * put something cannot tell you the symbol points at the wrong place.
     *
     * Endpoint 0's transmit fifo has its own register at 0x028 - RM0090's
     * `OTG_FS_GNPTXFSIZ`, ST's `DIEPTXF0_HNPTXFSIZ`, "EP0 / Non Periodic Tx
     * FIFO Size Register" - and the run at 0x104 is endpoints 1 and up. 0x100
     * is `HPTXFSIZ`, the **host** periodic fifo, and in device mode nothing
     * here has any business writing it. ST's device branch writes the two
     * separately (`usb_core.c`, `USB_OTG_CoreInitDev`); this driver used to
     * send endpoint 0 to 0x100 and leave 0x028 at its reset value of zero,
     * which is endpoint 0 with no transmit buffer at all - and endpoint 0 is
     * the one every host reads the device descriptor from first.
     */
    expect("endpoint 0's transmit fifo is written where the core keeps it",
           AK_REG32(USB_BASE + 0x028u) == ((16u << 16) | 128u));
    expect("and the host's periodic fifo register is left alone",
           AK_REG32(USB_BASE + 0x100u) == 0u);

    /*
     * Endpoint 0 armed for setup packets, with STUPCNT written. This is the
     * field ST's own driver programs to three on every return from a request
     * and this firmware did not write at all: a device that never sets it
     * answers the first descriptor request and goes deaf, which on a bench
     * looks like a cable.
     */
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

    /*
     * GET_DESCRIPTOR for the device: eighteen bytes of it go into endpoint 0's
     * fifo, which is FIFO 0 - and the last word written is therefore the last
     * two bytes of the descriptor, bSerialNumberIndex and bNumConfigurations.
     * A descriptor whose length and contents disagree is a device the host
     * walks off the end of.
     */
    usb_setup_packet(0x80u, 0x06u, 0x0100u, 0u, 64u); /* GET_DESCRIPTOR, device */
    expect("no request is answered before the core closes the setup stage",
           usb_answered_early == 0);
    expect("the device descriptor goes out as eighteen bytes",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 18u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);
    expect("and its last two bytes are the ones the host reads last",
           OTG_FIFO(0u) == 0x00000103u);

    /*
     * The configuration descriptor, which is **67 bytes** - one packet more
     * than endpoint 0 can carry. Every host reads it that way: it asks for the
     * first nine bytes, then for the whole thing, and the whole thing arrives
     * as 64 bytes and then 3. This is the check that was missing while the
     * driver's continuation sent a second packet of *zero* bytes from past the
     * end of the one packet it keeps, which is a device a host cannot
     * configure - and it is the console's only door on this board.
     */
    /* 255 is what a host asks for: more than the descriptor, so the driver is
     * the thing that decides the length is 67 and splits it across packets. */
    usb_setup_packet(0x80u, 0x06u, 0x0200u, 0u, 255u);
    expect("a descriptor longer than a packet starts with a full one",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 64u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);

    /* The host takes those 64 bytes and acknowledges them, which is what the
     * transfer-complete flag is; the last 3 have to follow. */
    OTG_DIEPINT0 |= OTG_DEPINT_XFRC;
    ak_usb_poll();
    expect("and the rest of it follows in the next packet",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 3u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);
    /* Those three bytes are the tail of the last endpoint descriptor: the
     * bulk OUT endpoint's 64-byte maximum packet and the two bytes after it,
     * which are the numbers a host cannot configure the port without. */
    expect("carrying the end of the last endpoint descriptor",
           OTG_FIFO(0u) == 0x000040u);

    /* And a request whose answer fits in one packet is still one packet: the
     * continuation must not fire for it. */
    usb_setup_packet(0x80u, 0x06u, 0x0301u, 0u, 255u);
    expect("a descriptor that fits is sent in one packet, not continued",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 20u);

    /*
     * String index 3 - the serial - which is the one descriptor this suite had
     * never asked for, and the one that was wrong.
     *
     * bLength is a descriptor's first byte and has to be the length of the whole
     * descriptor, not of the text inside it. This one declared 12 over a
     * fourteen-byte array. Nothing on the bench could have reached it: a host
     * asks for index 3 only after it has read the device and configuration
     * descriptors, and this board has never got that far.
     *
     * Nor would the size assertion above have caught it, which is the part worth
     * keeping in mind: the transfer is sized from `sizeof`, so XFRSIZ was always
     * right and the wrong byte was only ever read by the host. A check on the
     * length of a transfer cannot see a defect in the bytes being transferred,
     * so the check has to be on a byte.
     *
     * Four bytes is deliberate. OTG_FIFO(0) is a single mapped word holding the
     * *last* word written, so asking for the whole descriptor leaves the padding
     * of its final word behind - bytes 12 and 13, both zero - and the assertion
     * would be reading the wrong end. Four bytes is exactly one word, so the word
     * left in the FIFO is the first one, which is where bLength lives. The first
     * spelling of this check asked for 255 and failed for exactly that reason.
     */
    usb_setup_packet(0x80u, 0x06u, 0x0303u, 0u, 4u); /* GET_DESCRIPTOR, string 3 */
    expect("the serial string's bLength byte is the whole descriptor",
           (OTG_FIFO(0u) & 0xFFu) == 14u);
    expect("and the string descriptor type follows it",
           ((OTG_FIFO(0u) >> 8) & 0xFFu) == 0x03u);
    usb_setup_packet(0x80u, 0x06u, 0x0303u, 0u, 255u);
    expect("and asking for all of it transfers fourteen bytes",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 14u);

    /*
     * SET_ADDRESS, which every host sends before it asks for anything else -
     * and which this test had never sent. What has to happen is one write to
     * the device's address field, and the failure it prevents is a device that
     * answers at address 0 for ever: the host gives up with "Device not
     * responding to setup address", which is what the board reported on
     * 2026-09-17.
     */
    usb_setup_packet(0x00u, 0x05u, 7u, 0u, 0u); /* SET_ADDRESS(7) */
    expect("the address the host gives the device is the one it answers at",
           (OTG_DCFG & OTG_DCFG_DAD(0x7Fu)) == OTG_DCFG_DAD(7u));
    expect("and the request is acknowledged with a zero-length packet",
           (OTG_DIEPTSIZ0 & 0x7Fu) == 0u &&
               (OTG_DIEPTSIZ0 & OTG_DIEPTSIZ0_PKTCNT1) != 0u);

    /* SET_CONFIGURATION(1): the host has a driver, and both bulk endpoints are
     * armed - the console's output above all, which is what the banner has
     * been waiting for. */
    usb_setup_packet(0x00u, 0x09u, 1u, 0u, 0u);
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

    /*
     * Typing: two packets on the bulk OUT endpoint, read back in the order
     * they arrived - which is the whole point of the ring, and the thing that
     * was missing while the OUT endpoint's packets were read and dropped.
     */
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

    /* A packet for an endpoint this device does not use is read out of the
     * FIFO - it has to be, or the queue fills - and then goes nowhere. */
    usb_receive(1u, OTG_GRXSTS_DATA, 0x41414141u, 4u);
    ak_usb_poll();
    usb_consumed();
    expect("a packet for somebody else's endpoint is not console input",
           ak_usb_read(&typed[0]) == 0);

    /* And the bulk OUT endpoint's own completion, which is what arms it for the
     * next packet: a re-arm that never happens is a console that takes one word
     * from the host and then NAKs everything after it. */
    OTG_DOEPCTL(2u) &= ~OTG_DEPCTL_EPENA;
    OTG_DOEPINT(2u) |= OTG_DEPINT_XFRC;
    ak_usb_poll();
    expect("a completed bulk out packet arms the endpoint for the next one",
           (OTG_DOEPCTL(2u) & (OTG_DEPCTL_EPENA | OTG_DEPCTL_CNAK)) ==
               (OTG_DEPCTL_EPENA | OTG_DEPCTL_CNAK) &&
               (OTG_DOEPTSIZ(2u) & 0x7FFFFu) == 64u);
    OTG_DOEPINT(2u) = 0u;

    /* And the ring's own limit: five 64-byte packets is more than it holds,
     * and what does not fit is counted rather than silently lost. */
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
     * The way out, which had the same hole as the way in: the console's ring is
     * two kilobytes and the endpoint carries 64 bytes at a time, so a long line
     * - `params` is one - is a sequence of packets that only continues when the
     * host acknowledges the one before it. Nothing had ever written more than a
     * packet, so the continuation was never reached.
     */
    {
        static char flood[2200];

        /* The banner is still in flight: take it, the way the host does, so the
         * endpoint is free for what comes next. */
        usb_take_in_packet(1u);

        memset(flood, '.', sizeof flood);
        unsigned before_tx_dropped = ak_usb_dropped();

        (void)ak_usb_write(flood, sizeof flood);
        expect("a line longer than one packet goes out as a full packet first",
               (OTG_DIEPTSIZ(1u) & 0x7FFFFu) == 64u &&
                   (OTG_DIEPCTL(1u) & OTG_DEPCTL_EPENA) != 0u);
        expect("and a write longer than the ring counts what it could not keep",
               ak_usb_dropped() > before_tx_dropped);

        /* The host takes that packet, and the rest follows - which is the
         * branch a console that prints a parameter dump depends on. */
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
    /* The setup-phase interrupt is driven through the poll for the line that
     * handles it. What it *clears* is not checkable here: on the part writing
     * the bit back to the register is what clears it, and a page of memory has
     * no such rule - the write leaves the bit set, which is the one thing this
     * model cannot say. */
    expect("the setup-phase interrupt is handled without disturbing anything",
           ak_usb_ready() == 1u);
    OTG_DOEPINT0 = 0u;

    /* Cleared first so the handler below is the only thing that can set it. */
    OTG_DCTL &= ~OTG_DCTL_CGNPINNAK;
    OTG_GINTSTS |= OTG_GINT_ENUMDNE;
    ak_usb_poll();
    expect("a speed interrupt is handled without disturbing the configuration",
           ak_usb_ready() == 1);
    /* And it releases the global IN NAK that the reset before it left in force,
     * endpoint 0 included. Until that bit is written the host's read of the
     * device descriptor is answered NAK for as long as it keeps asking, so
     * enumeration never completes.
     *
     * What this pins is the *write*. On the part CGNPINNAK is a strobe - the 1
     * is what clears the NAK, and the bit reads back zero afterwards - and a
     * page of memory has no such rule, so the model cannot be asked whether the
     * NAK lifted, only whether the handler wrote the bit. That is exactly the
     * defect it is here to catch: nothing wrote it. */
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
     * an empty packet, because a host that gets nothing cannot tell "no such
     * descriptor" from "the device is asleep". */
    usb_setup_packet(0x80u, 0x06u, 0x0400u, 0u, 64u); /* interface descriptor */
    expect("a descriptor type nothing here has is stalled",
           (OTG_DIEPCTL0 & OTG_DEPCTL_STALL) != 0u &&
               (OTG_DOEPCTL0 & OTG_DEPCTL_STALL) != 0u);
    OTG_DIEPCTL0 &= ~OTG_DEPCTL_STALL;
    OTG_DOEPCTL0 &= ~OTG_DEPCTL_STALL;

    /*
     * The class requests a serial driver makes when the port is opened.
     * SET_LINE_CODING carries seven bytes in a data stage, and the transfer
     * only ends when the device answers the status stage - so what is checked
     * here is the zero-length packet that finishes it, and then that
     * GET_LINE_CODING is answered with the seven bytes the host set.
     */
    usb_setup_packet(0x21u, 0x20u, 0u, 0u, 7u); /* SET_LINE_CODING */
    {
        /* The seven bytes of the line coding a host would send: 9600 baud
         * little-endian, one stop bit, no parity, eight data bits. */
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
     * The board's own reader: this is the wire the console actually pulls on,
     * and it has to take a USB byte before it looks at the UART - which has
     * nothing in it here at all.
     */
    usb_receive(2u, OTG_GRXSTS_DATA, 0x00000058u, 1u); /* "X" */
    char byte = 0;
    expect("the board's console reader hands back what was typed over usb",
           ak_board_console_poll_rx(&byte) == 1 && byte == 'X');

    /* A bus reset: the host is about to describe itself again, the
     * configuration is gone, and half a line from before it is not a command.
     */
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

void test_arch(void)
{
    map_registers();
    if (!mapped) {
        /* Said rather than skipped silently: a test that could not set itself
         * up and passed anyway is worse than one that fails. */
        expect("the peripheral region can be mapped at its real addresses",
               0);
        return;
    }
    expect("the peripheral region can be mapped at its real addresses", 1);

    test_the_clock_tree();
    test_the_clock_without_a_crystal();
    test_the_uart_divisor();
    test_the_pins_and_the_clock_enables();
    test_the_console_path();
    test_the_receivers_interrupt_path();
    test_the_tick();
    test_the_servo_timer();
    test_the_motor_timer_and_the_burst();
    test_a_frame();
    /* After the frame test, because the frame counters are cumulative over the
     * process and that test is the one that pins a first frame's counts. */
    test_the_servo_bank_on_another_timer();
    test_the_flash_guards();
    test_the_flash_that_never_finishes();
    test_the_flash_controllers_success_path();
    test_the_blackbox_in_flash();
    test_the_sensor_bus();
    test_the_flight_packs_adc();
    test_the_saved_configuration();
    test_the_sensor_bus_transactions();
    test_the_bus_that_stops_answering();
    test_the_barometers_bus();
    test_the_bus_a_slave_is_holding();
    test_the_usb_console_path();
}
