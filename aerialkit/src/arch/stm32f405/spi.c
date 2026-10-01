#include "arch.h"

/*
 * SPI master for sensor buses, mode 3 (CPOL = CPHA = 1), 8 bits, software chip
 * select. The InvenSense parts want mode 3 and a chip select the master owns,
 * which is the case this covers; a device with something else in mind gets
 * another function rather than a flag soup.
 *
 * The clock is APB1 / 8. Nothing here is verified against a device yet - the
 * loopback check on the console is what turns it into a fact, and it needs one
 * jumper wire from MOSI to MISO.
 */

static int clock_enable(uint32_t spi)
{
    switch (spi) {
    case SPI1_BASE:
        RCC_APB2ENR |= (1u << 12);
        return 1;
    case SPI2_BASE:
        RCC_APB1ENR |= (1u << 14);
        return 1;
    case SPI3_BASE:
        RCC_APB1ENR |= (1u << 15);
        return 1;
    default:
        return 0;
    }
}

void ak_spi_init(uint32_t spi, ak_pin_t sck, ak_pin_t miso, ak_pin_t mosi,
                 uint8_t af)
{
    if (!clock_enable(spi)) {
        return;
    }

    ak_pin_af(sck, af, GPIO_PUPD_NONE);
    ak_pin_af(mosi, af, GPIO_PUPD_NONE);
    /* MISO floats when nothing is driving it, so it gets a pull-up. */
    ak_pin_af(miso, af, GPIO_PUPD_PULLUP);

    SPI_CR1(spi) = 0;
    SPI_CR1(spi) = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA |
                   SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_DIV8;
    SPI_CR2(spi) = 0;
    SPI_CR1(spi) |= SPI_CR1_SPE;
}

static int wait_flag(uint32_t spi, uint32_t mask, uint32_t wanted)
{
    uint32_t guard = 100000u;
    while (guard-- > 0) {
        if ((SPI_SR(spi) & mask) == wanted) {
            return 0;
        }
    }
    return -1;
}

#ifdef AK_HOST_SPI
#include "host_spi_model.h"
/*
 * Two entries, and the names say which is which.
 *
 * `ak_spi_transfer()` is what the board and the sensor drivers call. On a
 * target it is the register loop below; in the host build it hands the frame to
 * the modelled device (tests/host_spi_model.c), because what a page of memory
 * cannot do is *answer as a part* - and the board's own IMU bus and its SPI
 * loopback are only reachable with a part on the other end.
 *
 * `ak_spi_transfer_loop()` is the register loop itself, kept under its own name
 * so the test that pins it against the mapped block (tests/test_arch.c, and the
 * AT32's) can still run it. Without this the seam would have hidden the loop
 * from the coverage map - the same trade the F405's `usb.c` makes in the
 * opposite direction, where the queue is behind the seam and the loop is not.
 */
int ak_spi_transfer(uint32_t spi, const uint8_t *tx, uint8_t *rx, unsigned len)
{
    return host_spi_transfer(spi, tx, rx, len);
}

int ak_spi_transfer_loop(uint32_t spi, const uint8_t *tx, uint8_t *rx,
                         unsigned len)
{
#else
int ak_spi_transfer(uint32_t spi, const uint8_t *tx, uint8_t *rx, unsigned len)
{
#endif
    for (unsigned i = 0; i < len; i++) {
        if (wait_flag(spi, SPI_SR_TXE, SPI_SR_TXE) != 0) {
            return -1;
        }
        SPI_DR(spi) = tx != 0 ? tx[i] : 0xFFu;

        if (wait_flag(spi, SPI_SR_RXNE, SPI_SR_RXNE) != 0) {
            return -1;
        }
        uint8_t value = (uint8_t)(SPI_DR(spi) & 0xFFu);
        if (rx != 0) {
            rx[i] = value;
        }
    }

    /* Leave the bus idle rather than returning while the last bit is still
     * shifting: the next transfer's chip select must not cut it short. */
    return wait_flag(spi, SPI_SR_BSY, 0);
}
