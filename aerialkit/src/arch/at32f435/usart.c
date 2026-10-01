#include "arch.h"

#include "ak_console.h"
#include "ak_ring.h"

/*
 * The console's port, and the two arithmetic traps in it.
 *
 * The first is the divisor. `baudr` holds fCK / baud - the same *value* the
 * F405's BRR holds, and for the same reason: a UART divides its peripheral
 * clock by sixteen and then by USARTDIV, and fCK/baud is USARTDIV written out
 * at its own scale. Artery's own driver computes exactly this and then *rounds*
 * it; the F405 port's stops half a baud short instead. The difference is a
 * fraction of a per cent at every rate this firmware uses - and the reason the
 * arithmetic is written down here rather than copied is that the version of it
 * that multiplies by sixteen as well is a console that prints nothing legible,
 * which is a bug this project has already had once.
 *
 * The second is `trpswap`, which this part has and the F405 does not: it swaps
 * the transmit and receive pins in the peripheral, which is how the wing's GPS
 * port is wired (USART3, PB10 as receive and PB11 as transmit, which the
 * reference target sets the swap bit for). The console does not need it; it is
 * in the register map above so that the port that does need it is a flag rather
 * than a discovery.
 */

static uint32_t console_usart;

#define AK_UART_TXE_GUARD 100000u

static int usart_clock_enable(uint32_t usart)
{
    switch (usart) {
    case USART1_BASE:
        CRM_APB2EN |= 1u << 4; /* CRM_USART1_PERIPH_CLOCK = MAKE_VALUE(0x44, 4) */
        return 1;
    case USART2_BASE:
        CRM_APB1EN |= 1u << 17; /* MAKE_VALUE(0x40, 17) */
        return 1;
    case USART3_BASE:
        CRM_APB1EN |= 1u << 18; /* MAKE_VALUE(0x40, 18) */
        return 1;
    default:
        return 0;
    }
}

static uint32_t usart_pclk(uint32_t usart)
{
    /* USART1 and USART6 are on the fast bus on this part too; there is no
     * USART6 on the boards this port targets, so only the one case is written. */
    return usart == USART1_BASE ? ak_clk_apb2_hz() : ak_clk_apb1_hz();
}

/*
 * The two pins take their function numbers separately, and that is not
 * decoration: on this part a pin's function number is a per-pin field, and the
 * wing's receiver is the port that proves it - USART2 listens on PB0 with
 * function 6 and talks on PA8 with function 8. The F405 port takes one number
 * for both pins because every F4 USART pin it uses is on the same alternate
 * function; the API here carries two because this hardware does.
 */
static void usart_configure(uint32_t usart, ak_pin_t tx, uint8_t tx_af,
                            ak_pin_t rx, uint8_t rx_af, uint32_t baud,
                            int even_parity, int two_stop_bits, int swap)
{
    usart_clock_enable(usart);

    ak_pin_af(tx, tx_af, AK_GPIO_PULL_NONE);
    ak_pin_af(rx, rx_af, AK_GPIO_PULL_UP);

    AK_USART_CTRL1(usart) = 0u;

    /* fCK / baud, rounded to the nearest count - see the top of this file for
     * what the value means and what it costs to get it wrong. */
    AK_USART_BAUDR(usart) =
        (uint32_t)(((uint64_t)usart_pclk(usart) + baud / 2u) / baud) & 0xFFFFu;

    AK_USART_CTRL2(usart) =
        ((two_stop_bits ? AK_USART_CTRL2_STOP_2 : AK_USART_CTRL2_STOP_1)
         << AK_USART_CTRL2_STOP_SHIFT) |
        (swap ? AK_USART_CTRL2_TRPSWAP : 0u);

    AK_USART_CTRL1(usart) = AK_USART_CTRL1_UEN | AK_USART_CTRL1_TEN |
                            AK_USART_CTRL1_REN |
                            (even_parity ? AK_USART_CTRL1_PEN : 0u);
}

void ak_uart_init(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                  uint8_t af)
{
    usart_configure(usart, tx, af, rx, af, baud, 0, 0, 0);
}

/* The same, for the one port in this firmware with a swap in front of it. */
void ak_uart_init_swap(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                       uint8_t af)
{
    usart_configure(usart, tx, af, rx, af, baud, 0, 0, 1);
}

/* And the same again when the two pins' function numbers differ. */
void ak_uart_init_af(uint32_t usart, ak_pin_t tx, uint8_t tx_af, ak_pin_t rx,
                     uint8_t rx_af, uint32_t baud)
{
    usart_configure(usart, tx, tx_af, rx, rx_af, baud, 0, 0, 0);
}

void ak_console_attach(uint32_t usart)
{
    console_usart = usart;
}

uint32_t ak_console_port(void)
{
    return console_usart;
}

int ak_uart_write_bytes(uint32_t usart, const char *data, unsigned len)
{
    for (unsigned i = 0; i < len; i++) {
        uint32_t guard = AK_UART_TXE_GUARD;

        while ((AK_USART_STS(usart) & AK_USART_STS_TDBE) == 0u) {
            if (guard-- == 0u) {
                return -1;
            }
        }
        AK_USART_DT(usart) = (uint32_t)(unsigned char)data[i];
    }
    return 0;
}

/*
 * The console's sink: the same bytes, on the port the board attached.
 *
 * The wait is bounded, and the bound is the whole point of the comment: a port
 * whose clock never got enabled, or whose pins belong to something else, never
 * sets the transmit-empty flag - and an unbounded wait there is not a silent
 * console, it is a firmware that stops inside the first line of its own banner,
 * with no fault recorded and the LED never lit. Giving up costs a byte and
 * leaves everything else running. 115200 baud is 87 microseconds a byte, and a
 * hundred thousand trips round this loop is several milliseconds on this part,
 * so a working port has far more time than it needs.
 */
void ak_console_write_raw(const char *data, unsigned len)
{
    if (console_usart == 0u) {
        return;
    }
    (void)ak_uart_write_bytes(console_usart, data, len);
}

int ak_uart_poll_rx(uint32_t usart, uint8_t *byte)
{
    if ((AK_USART_STS(usart) & AK_USART_STS_RDBF) == 0u) {
        return 0;
    }
    *byte = (uint8_t)(AK_USART_DT(usart) & 0xFFu);
    /* Reading the data register clears the flag, and an overrun is cleared the
     * same way: a faster talker than the loop costs bytes rather than wedging
     * the port. */
    (void)AK_USART_STS(usart);
    return 1;
}

/*
 * The interrupt-driven receive, which is what the receiver and the GPS need:
 * each port gets its own ring, emptied from the main loop, so a receiver
 * talking at 420000 baud and a GPS at 9600 cannot lose each other's bytes. The
 * console keeps its polling - one person typing is not worth an interrupt - and
 * both paths read the same data register, which is why the flag read and the
 * data read are written the same way here as they are above.
 *
 * Only the byte-arrived flag is enabled. The error flags are not separately
 * handled: the ring is small and a lost byte costs one frame, so the simple
 * thing is also the right thing, and reading the status register and then the
 * data register clears an overrun in the same two accesses. What is counted is
 * what the ring could not keep, which the console prints - a receiver losing
 * bytes and a receiver saying nothing are different problems.
 */
#define AK_UART_RX_PORTS 3

static struct {
    uint32_t  usart;
    ak_ring_t ring;
} rx_ports[AK_UART_RX_PORTS];

static ak_ring_t *ring_for(uint32_t usart)
{
    for (unsigned i = 0; i < AK_UART_RX_PORTS; i++) {
        if (rx_ports[i].usart == usart) {
            return &rx_ports[i].ring;
        }
    }
    return 0;
}

static void rx_enable_interrupt(uint32_t usart)
{
    AK_USART_CTRL1(usart) |= AK_USART_CTRL1_RDBFIEN;

    unsigned irq = 0u;

    if (usart == USART1_BASE) {
        irq = AK_USART1_IRQ;
    } else if (usart == USART2_BASE) {
        irq = AK_USART2_IRQ;
    } else if (usart == USART3_BASE) {
        irq = AK_USART3_IRQ;
    } else {
        return;
    }
    AK_NVIC_ISER(irq / 32u) = 1u << (irq % 32u);
}

/* Give the port a ring and turn its receive interrupt on. Split out of the
 * entry points below because all of them end here. */
static void rx_setup(uint32_t usart)
{
    unsigned slot = AK_UART_RX_PORTS;

    for (unsigned i = 0; i < AK_UART_RX_PORTS; i++) {
        if (rx_ports[i].usart == usart || rx_ports[i].usart == 0u) {
            slot = i;
            break;
        }
    }
    if (slot >= AK_UART_RX_PORTS) {
        return; /* more ports than buffers: no receive, and no silent mix-up */
    }

    rx_ports[slot].usart = usart;
    ak_ring_init(&rx_ports[slot].ring);
    rx_enable_interrupt(usart);
}

/*
 * The general form, and the one the board file calls: every pin's function
 * number, the frame, and whether the peripheral swaps its two pins.
 *
 * Everything else here is a wrapper over this. It exists because the wing's
 * receiver needs all three of the things the simpler entry points leave out -
 * two different function numbers (PB0 is 6, PA8 is 8), and nothing else - and
 * its GPS needs a third (the swap bit on a pair that shares function 7). Two
 * more wrappers would have been two more combinations to keep in step.
 *
 * `even_parity` with two stop bits is SBUS, which this board cannot hear
 * without an inverting transistor in front of the pin - a fact that lives in
 * ak_board_rc_inverted() and not here, because it is a wire and not a register.
 */
void ak_uart_rx_init_af(uint32_t usart, ak_pin_t tx, uint8_t tx_af, ak_pin_t rx,
                        uint8_t rx_af, uint32_t baud, int even_parity,
                        int two_stop_bits, int swap)
{
    usart_configure(usart, tx, tx_af, rx, rx_af, baud, even_parity,
                    two_stop_bits, swap);
    rx_setup(usart);
}

void ak_uart_rx_init(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                     uint8_t af)
{
    ak_uart_rx_init_af(usart, tx, af, rx, af, baud, 0, 0, 0);
}

/* The same, for a receiver that does not speak 8N1. */
void ak_uart_rx_init_format(uint32_t usart, ak_pin_t tx, ak_pin_t rx,
                            uint32_t baud, uint8_t af, int even_parity,
                            int two_stop_bits)
{
    ak_uart_rx_init_af(usart, tx, af, rx, af, baud, even_parity, two_stop_bits,
                       0);
}

int ak_uart_rx_pop(uint32_t usart, uint8_t *byte)
{
    ak_ring_t *ring = ring_for(usart);

    return ring != 0 ? ak_ring_pop(ring, byte) : 0;
}

uint32_t ak_uart_rx_dropped(uint32_t usart)
{
    ak_ring_t *ring = ring_for(usart);

    return ring != 0 ? ring->dropped : 0;
}

static void uart_rx_interrupt(uint32_t usart)
{
    ak_ring_t *ring = ring_for(usart);

    if (ring != 0 && (AK_USART_STS(usart) & AK_USART_STS_RDBF) != 0u) {
        ak_ring_push(ring, (uint8_t)(AK_USART_DT(usart) & 0xFFu));
    }
    /* Reading the status register and then the data register is also what
     * clears an overrun, so a sender faster than the loop costs bytes rather
     * than wedging the port. */
}

void USART1_IRQHandler(void)
{
    uart_rx_interrupt(USART1_BASE);
}

void USART2_IRQHandler(void)
{
    uart_rx_interrupt(USART2_BASE);
}

void USART3_IRQHandler(void)
{
    uart_rx_interrupt(USART3_BASE);
}
