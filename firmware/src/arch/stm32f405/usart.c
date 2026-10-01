#include "arch.h"
#include "ak_console.h"
#include "ak_ring.h"

/*
 * Polled USART TX. Enough for a console; interrupts and DMA arrive with the RC
 * link milestone, where they are actually needed.
 */

static uint32_t console_usart;

#define AK_UART_RX_PORTS 2

static struct {
    uint32_t  usart;
    ak_ring_t ring;
} rx_ports[AK_UART_RX_PORTS];

static int usart_clock_enable(uint32_t usart)
{
    switch (usart) {
    case USART1_BASE:
        RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
        return 1;
    case USART6_BASE:
        RCC_APB2ENR |= RCC_APB2ENR_USART6EN;
        return 1;
    case USART2_BASE:
        RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
        return 1;
    case USART3_BASE:
        RCC_APB1ENR |= RCC_APB1ENR_USART3EN;
        return 1;
    default:
        return 0;
    }
}

static uint32_t usart_pclk(uint32_t usart)
{
    return usart == USART1_BASE || usart == USART6_BASE ? ak_clk_apb2_hz()
                                                        : ak_clk_apb1_hz();
}

/*
 * The line settings are the one place a receiver is not like the other two
 * ports. CRSF runs at 420000 baud, the GPS at 9600, and both 8N1; SBUS runs at
 * 100000 baud with even parity and two stop bits, which is a frame one bit
 * longer than the others and a different BRR *only* by way of the baud rate -
 * the divisor does not change with the frame format, which is why this is two
 * register writes rather than a second copy of the arithmetic.
 */
static void usart_configure(uint32_t usart, ak_pin_t tx, ak_pin_t rx,
                            uint32_t baud, uint8_t af, int even_parity,
                            int two_stop_bits)
{
    usart_clock_enable(usart);

    ak_pin_af(tx, af, GPIO_PUPD_NONE);
    ak_pin_af(rx, af, GPIO_PUPD_PULLUP);

    USART_CR1(usart) = 0;

    /*
     * The divisor, and the one arithmetic in this port that had it wrong for
     * its whole life without anything failing.
     *
     * RM0090 30.6.4: with oversampling by 16, baud = fCK / (16 * USARTDIV),
     * and BRR holds USARTDIV with four fractional bits - so BRR = USARTDIV *
     * 16 = fCK / baud. The multiply by sixteen belongs to USARTDIV's *scale*,
     * not to the numerator: writing (fCK * 16) / baud asks for BRR = fCK/baud
     * times sixteen, and a divisor sixteen times too large is a baud rate
     * sixteen times too slow.
     *
     * At 42 MHz and 115200 that is 7200 baud where a terminal is listening at
     * 115200 - a console that prints nothing legible at all, which is the
     * first thing anybody would have met on the bench, and which no check in
     * this repository could see: the port is not in the host build, the
     * simulator brings its own console and its own UARTs, and the image check
     * looks at the vector table. tests/test_arch.c now runs this code against
     * a register block and pins the divisor for all three ports.
     */
    uint32_t div = (uint32_t)(((uint64_t)usart_pclk(usart) + baud / 2u) /
                              baud);
    USART_BRR(usart) = div;

    USART_CR2(usart) = two_stop_bits ? USART_CR2_STOP_2 : 0u;

    USART_CR1(usart) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE |
                       (even_parity ? USART_CR1_PCE : 0u);
}

void ak_uart_init(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                  uint8_t af)
{
    usart_configure(usart, tx, rx, baud, af, 0, 0);
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
        uint32_t guard = 100000u;
        while ((USART_SR(usart) & USART_SR_TXE) == 0 && guard-- > 0) {
        }
        if (guard == 0) {
            return -1;
        }
        USART_DR(usart) = (uint32_t)(unsigned char)data[i];
    }
    return 0;
}

/*
 * The console's own sink, and the last byte of every line this firmware
 * prints.
 *
 * The transmit-empty flag is set at reset and again after each byte - so on a
 * working port this loop passes immediately - but the wait is still bounded,
 * and the bound is the whole of what this comment is for. A port whose clock
 * is off, or whose pins were configured by something else, never sets TXE. An
 * unbounded wait there is not a silent console: it is a firmware that stops
 * inside the first line of its own banner, with the LED never lit and no fault
 * recorded, which is a board that looks dead. Giving up costs a missing byte
 * and leaves everything else running, which is both easier to diagnose and
 * true: the port really did not accept it.
 *
 * 115200 baud is 87 microseconds a byte, and a hundred thousand trips round
 * this loop is about two milliseconds at 168 MHz - so a working port has
 * twenty times the time it needs, and a dead one costs a fifth of a second per
 * line rather than the aircraft.
 */
#define AK_UART_TXE_GUARD 100000u

void ak_console_write_raw(const char *data, unsigned len)
{
    if (console_usart == 0) {
        return;
    }
    for (unsigned i = 0; i < len; i++) {
        uint32_t guard = AK_UART_TXE_GUARD;
        while ((USART_SR(console_usart) & USART_SR_TXE) == 0) {
            if (guard-- == 0u) {
                return; /* the port is not accepting: give the rest up */
            }
        }
        USART_DR(console_usart) = (uint32_t)(unsigned char)data[i];
    }

    /* And the same bytes go to USB, if this board has it. The console is one
     * interface with two doors: somebody with a USB-TTL adapter on PA2/PA3
     * reads the port, somebody with only the USB cable reads this, and neither
     * has to know about the other. The queue never blocks and never waits for
     * a host - it counts what it cannot keep. */
    (void)ak_usb_write(data, len);
}

int ak_uart_poll_rx(uint32_t usart, uint8_t *byte)
{
    if ((USART_SR(usart) & USART_SR_RXNE) == 0) {
        return 0;
    }
    /* Reading DR clears RXNE. An overrun flag is cleared the same way, so a
     * faster talker than the loop costs bytes rather than wedging the port. */
    *byte = (uint8_t)(USART_DR(usart) & 0xFFu);
    (void)USART_SR(usart);
    return 1;
}

void ak_uart_rx_init(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                     uint8_t af)
{
    ak_uart_rx_init_format(usart, tx, rx, baud, af, 0, 0);
}

void ak_uart_rx_init_format(uint32_t usart, ak_pin_t tx, ak_pin_t rx,
                            uint32_t baud, uint8_t af, int even_parity,
                            int two_stop_bits)
{
    usart_configure(usart, tx, rx, baud, af, even_parity, two_stop_bits);

    unsigned slot = AK_UART_RX_PORTS;
    for (unsigned i = 0; i < AK_UART_RX_PORTS; i++) {
        if (rx_ports[i].usart == usart) {
            slot = i;
            break;
        }
        if (rx_ports[i].usart == 0) {
            slot = i;
            break;
        }
    }
    if (slot >= AK_UART_RX_PORTS) {
        return; /* more ports than buffers: no receive, and no silent mix-up */
    }

    rx_ports[slot].usart = usart;
    ak_ring_init(&rx_ports[slot].ring);

    /* Receive-not-empty only. The error flags are not separately enabled: the
     * ring buffer is small and a lost byte costs one frame, so the simple
     * thing is also the right thing here. */
    USART_CR1(usart) |= USART_CR1_RXNEIE;

    if (usart == USART1_BASE) {
        NVIC_ISER1 = 1u << (USART1_IRQ - 32u);
    } else if (usart == USART3_BASE) {
        NVIC_ISER1 = 1u << (USART3_IRQ - 32u);
    }
}

static ak_ring_t *ring_for(uint32_t usart)
{
    for (unsigned i = 0; i < AK_UART_RX_PORTS; i++) {
        if (rx_ports[i].usart == usart) {
            return &rx_ports[i].ring;
        }
    }
    return 0;
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
    if (ring != 0 && (USART_SR(usart) & USART_SR_RXNE) != 0) {
        ak_ring_push(ring, (uint8_t)(USART_DR(usart) & 0xFFu));
    }
    /* Reading SR then DR also clears the overrun flag, so a receiver that
     * outruns the loop costs bytes rather than wedging the port. */
}

void USART1_IRQHandler(void)
{
    uart_rx_interrupt(USART1_BASE);
}

void USART3_IRQHandler(void)
{
    uart_rx_interrupt(USART3_BASE);
}
