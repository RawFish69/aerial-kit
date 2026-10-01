#include "arch.h"

/*
 * SPI master for the sensor bus: mode 3 (CPOL = CPHA = 1), eight bits, software
 * chip select owned by the caller. That is what the inertial parts on this
 * board want, and it is the same choice the F405 port made for the same parts -
 * a device with something else in mind gets another function rather than a flag
 * soup.
 *
 * The divisor is the one number that is not the same on the two ports. The
 * F405's SPI1 sits on an 84 MHz APB and this one on a 144 MHz APB, so the same
 * setting would be a bus 1.7 times faster here; the setting is chosen to land
 * near the same 10 MHz - which every part in this project's list tolerates, and
 * which a scope is the only thing that can confirm.
 *
 * This part splits its divider between two fields: `mdiv_l` in ctrl1 and
 * `mdiv_h` in ctrl2, with a separate `mdiv3en` for a divide-by-three. Only the
 * power-of-two range below 256 is used here, so the high bit and the third
 * divider are cleared explicitly rather than assumed to be - a register that
 * resets to zero is not a register that is still zero after somebody else has
 * configured the bus.
 */

static int clock_enable(uint32_t spi)
{
    switch (spi) {
    case SPI1_BASE:
        CRM_APB2EN |= 1u << 12; /* CRM_SPI1_PERIPH_CLOCK = MAKE_VALUE(0x44, 12) */
        return 1;
    case SPI2_BASE:
        CRM_APB1EN |= 1u << 14; /* MAKE_VALUE(0x40, 14) */
        return 1;
    case SPI3_BASE:
        CRM_APB1EN |= 1u << 15; /* MAKE_VALUE(0x40, 15) */
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

    ak_pin_af(sck, af, AK_GPIO_PULL_NONE);
    ak_pin_af(mosi, af, AK_GPIO_PULL_NONE);
    /* MISO floats when no device is driving it, so it gets a pull-up - the
     * same choice the F405 port makes, for the same reason: a floating input
     * reads as noise, and it is the byte that arrives while nothing is
     * answering that looks like a part. */
    ak_pin_af(miso, af, AK_GPIO_PULL_UP);

    AK_SPI_CTRL1(spi) = 0u;
    AK_SPI_CTRL2(spi) = 0u;

    AK_SPI_CTRL1(spi) = AK_SPI_CTRL1_MSTEN | AK_SPI_CTRL1_CLKPOL |
                        AK_SPI_CTRL1_CLKPHA | AK_SPI_CTRL1_SWCSEN |
                        AK_SPI_CTRL1_ORA |
                        (AK_SPI_MDIV_DIV16 << AK_SPI_CTRL1_MDIV_SHIFT);
    AK_SPI_CTRL1(spi) |= AK_SPI_CTRL1_SPIEN;
}

static int wait_flag(uint32_t spi, uint32_t mask, uint32_t wanted)
{
    uint32_t guard = 100000u;

    while (guard-- > 0u) {
        if ((AK_SPI_STS(spi) & mask) == wanted) {
            return 0;
        }
    }
    return -1;
}

#ifdef AK_HOST_SPI_AT32
#include "host_spi_model.h"
/* Two entries, for the same reason the F405 port has two: the public one has a
 * modelled device on the bus, and the loop below it keeps its own name so the
 * test that pins it against the mapped block can still run it. See that file's
 * comment in src/arch/stm32f405/spi.c. */
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
        if (wait_flag(spi, AK_SPI_STS_TDBE, AK_SPI_STS_TDBE) != 0) {
            return -1;
        }
        AK_SPI_DT(spi) = tx != 0 ? tx[i] : 0xFFu;

        if (wait_flag(spi, AK_SPI_STS_RDBF, AK_SPI_STS_RDBF) != 0) {
            return -1;
        }
        uint8_t value = (uint8_t)(AK_SPI_DT(spi) & 0xFFu);

        if (rx != 0) {
            rx[i] = value;
        }
    }

    /* Leave the bus idle rather than returning while the last bit is still
     * shifting: the next transfer's chip select must not cut it short. */
    return wait_flag(spi, AK_SPI_STS_BF, 0u);
}
