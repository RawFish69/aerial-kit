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
        /* SPI1 is the one bus here with a DMA stream (the table at the bottom
         * of this file), and that stream lives on DMA2 - which nothing else in
         * this port has ever turned on. The outputs enable DMA1 for the DShot
         * burst and that is the only DMA clock anybody asks for. Without this
         * line a sensor read over DMA is the first thing to touch a controller
         * whose gate is still shut: the stream registers take the writes and
         * the engine does nothing, which looks exactly like a part that never
         * answers. */
        RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;
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

/* --- the DMA path (roadmap phase 1.2) -------------------------------------
 *
 * The sensor read is the one transfer whose latency is the control loop's, and
 * the loop above costs a poll of the status register per byte. Here the DMA
 * engine moves the bytes and the receive stream's transfer-complete interrupt
 * is what says the sample is in the caller's buffer.
 *
 * WHICH STREAM, AND WHY THAT IS NOT THIS FILE'S CHOICE
 *
 * Betaflight's generated table has it, as DMA(controller, stream, channel):
 *
 *   SPI1_TX DMA(2,3,3)   SPI1_RX DMA(2,0,3)
 *   SPI2_TX DMA(1,4,0)   SPI2_RX DMA(1,3,0)
 *   SPI3_TX DMA(1,5,0)   SPI3_RX DMA(1,0,0)
 *
 * (upstream/betaflight-2026.6.1/src/platform/STM32/dma_reqmap_mcu.c:957-963,
 * the STM32F4 branch; the macro those three numbers expand from is at line
 * 945.) INAV has no SPI DMA table to cross-check against - its F4 SPI driver
 * is a polling LL one - so the pinned Betaflight tree is the only source used,
 * and it is quoted rather than recalled.
 *
 * TWO ROWS ARE ABSENT, AND THEY ARE THE PART WORTH READING TWICE
 *
 * SPI2 is refused. Its transmit stream is DMA1 Stream 4 channel 0, and the
 * outputs already own DMA1 Stream 4: the DShot burst, channel 5, TIM3_CH1
 * (regs.h's AK_DMA_TIM3_STREAM, from INAV's generated timer table). A stream
 * serves one channel at a time, so an SPI2 transfer over DMA would take the
 * motor frame's stream out from under it - and what that looks like in flight
 * is a motor that stops answering. The board it costs is the one bench board
 * whose IMU bus would want it: src/boards/AERIALKIT_F405 declares
 * AK_BOARD_IMU_SPI as SPI2_BASE. Refused rather than scheduled, and
 * tests/test_arch.c checks the refusal by name.
 *
 * SPI3 is refused because nothing in this tree drives a sensor on it. Its
 * streams are free and its row is above; the day a board puts a part there,
 * adding the row is the work.
 *
 * So this port has exactly one DMA bus, SPI1 - which is the bus the wing's own
 * board fits its gyro on (src/boards/AERIALKIT_GHF435/board.h). */

typedef struct {
    uint32_t spi;
    uint32_t dma;
    unsigned rx_stream;
    unsigned tx_stream;
    unsigned channel;
    uint32_t irq; /* the receive stream's interrupt number */
} spi_dma_t;

static const spi_dma_t spi_dma_buses[] = {
    { SPI1_BASE, DMA2_BASE, 0u, 3u, 3u, DMA2_STREAM0_IRQ },
};

/* One transfer at a time. The sensor bus is read by the fast task and by
 * nothing else, so a second caller arriving while one is running is a caller
 * this port refuses rather than a queue it grows. */
static volatile int dma_busy;
static volatile int dma_complete;

static const spi_dma_t *dma_bus_for_spi(uint32_t spi)
{
    for (unsigned i = 0u; i < sizeof spi_dma_buses / sizeof spi_dma_buses[0];
         i++) {
        if (spi_dma_buses[i].spi == spi) {
            return &spi_dma_buses[i];
        }
    }
    return 0;
}

/* Flags live in LISR/LIFCR for streams 0-3 and in the H registers for 4-7, and
 * a ternary over the two would not be an lvalue - so the two cases are written
 * out. The mask itself comes from the same 6-bit-per-stream pattern the DShot
 * path cleared before this existed. */
static void dma_clear_stream_flags(uint32_t dma, unsigned stream)
{
    if (stream < 4u) {
        DMA_LIFCR(dma) = DMA_IFCR_CLEAR(stream);
    } else {
        DMA_HIFCR(dma) = DMA_IFCR_CLEAR(stream);
    }
}

/*
 * What the receive stream's interrupt does: the bytes are in the buffer, so
 * stop the streams, drop the SPI's DMA requests, and open the port for the
 * next transfer. Both the interrupt and the timeout path come through here, so
 * the two cannot leave the bus in different states - which matters, because
 * the timeout path is the one nothing else would catch.
 */
static void dma_complete_transfer(uint32_t dma, unsigned stream)
{
    const spi_dma_t *bus = 0;

    for (unsigned i = 0u; i < sizeof spi_dma_buses / sizeof spi_dma_buses[0];
         i++) {
        if (spi_dma_buses[i].dma == dma && spi_dma_buses[i].rx_stream == stream) {
            bus = &spi_dma_buses[i];
        }
    }
    if (bus == 0) {
        return;
    }

    dma_clear_stream_flags(bus->dma, bus->rx_stream);
    DMA_SxCR(bus->dma, bus->rx_stream) &= ~DMA_SxCR_EN;
    DMA_SxCR(bus->dma, bus->tx_stream) &= ~DMA_SxCR_EN;
    SPI_CR2(bus->spi) &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    dma_busy = 0;
    dma_complete = 1;
}

/* The receive stream's vector. The name fixes which stream this is; which bus
 * that stream belongs to is the table's answer, not a second copy here. */
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void)
{
    dma_complete_transfer(DMA2_BASE, 0u);
}

/*
 * Start a transfer and return. Zero means the streams are running and the
 * interrupt is armed; the bytes are in `rx` when it fires. A null `tx` shifts
 * 0xFF for every byte the way the loop above does, and the source is a single
 * byte with the memory increment off, because a buffer that does not exist
 * cannot be walked.
 */
int ak_spi_transfer_dma_start(uint32_t spi, const uint8_t *tx, uint8_t *rx,
                              unsigned len)
{
    static uint8_t filler = 0xFFu;
    const spi_dma_t *bus = dma_bus_for_spi(spi);

    if (bus == 0 || rx == 0 || len == 0u || len > 0xFFFFu || dma_busy) {
        return -1;
    }

    dma_busy = 1;
    dma_complete = 0;

    /* A running stream takes writes to its registers as "keep going", so both
     * are stopped before either is reprogrammed. */
    DMA_SxCR(bus->dma, bus->rx_stream) &= ~DMA_SxCR_EN;
    DMA_SxCR(bus->dma, bus->tx_stream) &= ~DMA_SxCR_EN;

    /* Clear the flag the last transfer left, or the first wait returns on the
     * previous sample's interrupt. */
    dma_clear_stream_flags(bus->dma, bus->rx_stream);

    DMA_SxPAR(bus->dma, bus->tx_stream) = (uint32_t)(uintptr_t)&SPI_DR(spi);
    DMA_SxM0AR(bus->dma, bus->tx_stream) =
        (uint32_t)(uintptr_t)(tx != 0 ? tx : &filler);
    DMA_SxNDTR(bus->dma, bus->tx_stream) = len;
    DMA_SxCR(bus->dma, bus->tx_stream) =
        DMA_SxCR_DIR_M2P | DMA_SxCR_PL_HIGH | DMA_SxCR_CHSEL(bus->channel) |
        (tx != 0 ? DMA_SxCR_MINC : 0u);

    DMA_SxPAR(bus->dma, bus->rx_stream) = (uint32_t)(uintptr_t)&SPI_DR(spi);
    DMA_SxM0AR(bus->dma, bus->rx_stream) = (uint32_t)(uintptr_t)rx;
    DMA_SxNDTR(bus->dma, bus->rx_stream) = len;
    DMA_SxCR(bus->dma, bus->rx_stream) = DMA_SxCR_MINC | DMA_SxCR_PL_HIGH |
                                         DMA_SxCR_TCIE |
                                         DMA_SxCR_CHSEL(bus->channel);

    /* The interrupt number is the architectural one; only the low half of the
     * NVIC's set-enable registers is reachable from this port's interrupts, and
     * this stream's 56 lands in ISER1 at bit 24. */
    NVIC_ISER1 = 1u << (bus->irq - 32u);

    /* The receive stream first: on this part enabling transmit with nothing
     * draining the receive register is how a byte is lost before the engine
     * that would take it is running. */
    DMA_SxCR(bus->dma, bus->rx_stream) |= DMA_SxCR_EN;
    DMA_SxCR(bus->dma, bus->tx_stream) |= DMA_SxCR_EN;
    SPI_CR2(spi) |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

#ifdef AK_HOST_SPI
    /*
     * A mapped page has these registers and no engine, so the model on the
     * other end supplies the bytes now and the interrupt below is run by
     * whoever would have had it - the blocking wrapper here, or a test. The
     * register writes above are the same on both builds, so the test that pins
     * the stream programming is checking the code the target runs.
     */
    if (host_spi_transfer(spi, tx, rx, len) != 0) {
        dma_complete_transfer(bus->dma, bus->rx_stream);
        return -1;
    }
#endif

    return 0;
}

/*
 * The blocking form: start, then wait for the interrupt rather than for the
 * status register to say each byte has arrived. That wait is the whole change
 * phase 1.2 makes - the bytes still take the same time on the wire, and what
 * goes away is the loop asking between them.
 *
 * A guard rather than an unbounded wait: a stream that never completes is a
 * bus fault or a wrong channel, and a control loop that spins on it forever is
 * worse than one that reports a failed read and tries again next period.
 */
int ak_spi_transfer_dma(uint32_t spi, const uint8_t *tx, uint8_t *rx,
                        unsigned len)
{
    const spi_dma_t *bus = dma_bus_for_spi(spi);

    if (ak_spi_transfer_dma_start(spi, tx, rx, len) != 0) {
        return -1;
    }

#ifdef AK_HOST_SPI
    DMA2_Stream0_IRQHandler();
    (void)bus;
    return 0;
#else
    uint32_t guard = 1000000u;

    while (!dma_complete && guard-- > 0u) {
    }

    if (!dma_complete) {
        /* Tear down the same way the interrupt would, so a timed-out transfer
         * does not leave a stream armed to write into a buffer that has gone
         * out of scope. */
        dma_complete_transfer(bus->dma, bus->rx_stream);
        dma_complete = 0;
        return -1;
    }

    /* The last bit may still be shifting: the next transfer's chip select must
     * not cut it short. */
    return wait_flag(spi, SPI_SR_BSY, 0);
#endif
}
