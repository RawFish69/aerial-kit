#include "arch.h"

#include "ak_dshot_timing.h"
#include "ak_output.h"

/*
 * Motors and servos on the AT32F435 - the piece that makes this port an
 * aircraft rather than a console.
 *
 * **The timers run at the system clock.** This part's clock tree doubles an APB
 * clock into its timers exactly as the STM32F4's does, and 288 MHz is what that
 * gives on a /2 APB - which is what the reference's own `timerClock()` returns
 * for these timers. So the servo tick and every DShot bit time are counted in
 * 288 MHz periods, which is a different set of numbers from the F405 port's
 * 84 MHz and the first thing a copy of that file would get wrong.
 *
 * **Servos** are ordinary PWM: TMR2 at 1 MHz (prescaler 287), a 20 ms period,
 * and a compare value that is the pulse width in microseconds. On this board
 * they are PB8 and PB9, which the reference's pin table gives as mux 1 on
 * TMR2's first two channels.
 *
 * **Motors** are DShot: one timer period per bit, so the period is the timer
 * clock over the bit rate - 960 ticks for DShot300. This board's two motors are
 * PB6 and PB7, mux 2 on TMR4's first two channels.
 *
 * The bit values reach the compare registers **one DMA channel per motor**,
 * each triggered by its own channel's compare event, rather than the single
 * burst the F405 port uses. That is not a preference: this part *does* have the
 * burst registers (`dmactrl` and `dmadt`, the F4's DCR and DMAR renamed), but
 * the one implementation of DShot on this chip that exists to read - INAV's -
 * uses per-channel requests through the DMAMUX, and the burst's length field
 * has no reference here to check an encoding against. A guess about how many
 * transfers a burst performs is a guess about the length of every frame.
 *
 * The first bit is loaded by hand, and the DMA carries the rest: the compare
 * event that starts a transfer arrives at the beginning of a period, which is a
 * period too late for the value that period needs. The F405 port has the same
 * off-by-one and the same note, and on both it is the kind of thing that shows
 * up as an ESC reading a throttle one bit out.
 */

/* This board's airframe: the twin-motor elevon wing. A four-motor board needs
 * four planes and four DMA channels; the arithmetic below is the same and the
 * arrays grow. */
#define AK_MOTOR_COUNT 2u

#define AK_TIMER_HZ        288000000u /* the timers run at the system clock */
#define AK_SERVO_TICK_HZ   1000000u   /* 1 us of resolution */
#define AK_SERVO_PERIOD_US 20000u     /* 50 Hz */

/* The interleaved form the core's encoder produces, and the per-motor planes
 * the DMA channels read. Both are here because the transpose is cheaper than a
 * second bit encoder: the framing, the CRC and the bit order stay in the core,
 * where they are checked on a host. */
static uint16_t dshot_entries[AK_DSHOT_ENTRIES];
static uint16_t dshot_plane[AK_MOTOR_COUNT][AK_DSHOT_GROUPS];

static uint16_t dshot_period;
static uint16_t dshot_ccr_zero;
static uint16_t dshot_ccr_one;
static uint32_t dshot_hz = 300000u;
static uint32_t dshot_frames;
static uint32_t dshot_skipped;
/* How many of this frame's channels have finished: the frame is over when both
 * have, and only then is another one allowed to start. */
static uint32_t dshot_channels_done;
static volatile int dshot_busy;

static void pwm_channel(uint32_t tmr, uint8_t channel)
{
    volatile uint32_t *ccm = channel <= 2u ? &AK_TMR_CCM1(tmr)
                                           : &AK_TMR_CCM2(tmr);
    uint32_t shift = AK_TMR_CCM_MODE_SHIFT(channel);

    /* PWM mode A with the preload on, which is what makes a write land at the
     * next period boundary rather than in the middle of the bit it is in. */
    *ccm = (*ccm & ~(0x7u << shift)) | (AK_TMR_CCM_MODE_PWM_A << shift) |
           AK_TMR_CCM_PRELOAD(channel);
}

static void dma_channel_init(unsigned index, uint32_t ccr_address,
                             const uint16_t *first_value)
{
    uint32_t n = index + 1u;

    AK_DMA_CH_CTRL(DMA1_BASE, n) = 0u;
    AK_DMA_CH_PADDR(DMA1_BASE, n) = (uint32_t)(uintptr_t)ccr_address;
    AK_DMA_CH_MADDR(DMA1_BASE, n) = (uint32_t)(uintptr_t)first_value;
    /* One transfer per bit, from the second one on: the first was loaded by
     * hand. */
    AK_DMA_CH_DTCNT(DMA1_BASE, n) = AK_DSHOT_GROUPS - 1u;
    AK_DMA_CH_CTRL(DMA1_BASE, n) =
        AK_DMA_CTRL_DTD | AK_DMA_CTRL_MINC |
        (AK_DMA_WIDTH_HALFWORD << AK_DMA_CTRL_PWIDTH_SHIFT) |
        (AK_DMA_WIDTH_HALFWORD << AK_DMA_CTRL_MWIDTH_SHIFT) |
        (AK_DMA_PRIORITY_HIGH << AK_DMA_CTRL_CHPL_SHIFT) |
        AK_DMA_CTRL_FDTIEN;

    /* And the request: which peripheral event this channel is wired to. On the
     * F4 that is a fixed table entry; here it is a multiplexer, and the
     * controller has to be told to use it before the id means anything. */
    AK_DMA_MUXSEL(DMA1_BASE) |= 1u;
    AK_DMAMUX_CH(DMA1_BASE, n) =
        (AK_DMAMUX_CH(DMA1_BASE, n) & ~(uint32_t)AK_DMAMUX_REQSEL_MASK) |
        (index == 0u ? AK_DMAMUX_REQ_TMR4_CH1 : AK_DMAMUX_REQ_TMR4_CH2);
}

void ak_output_init(void)
{
    CRM_APB1EN |= 1u << 0;  /* TMR2: MAKE_VALUE(0x40, 0) */
    CRM_APB1EN |= 1u << 2;  /* TMR4: MAKE_VALUE(0x40, 2) */
    CRM_AHBEN1 |= 1u << 22; /* DMA1: MAKE_VALUE(0x30, 22) */

    /* --- servos: TMR2, PB8 and PB9, mux 1 --- */
    ak_pin_af(AK_PIN(GPIOB_BASE, 8), AK_GPIO_MUX_1, AK_GPIO_PULL_NONE);
    ak_pin_af(AK_PIN(GPIOB_BASE, 9), AK_GPIO_MUX_1, AK_GPIO_PULL_NONE);

    AK_TMR_DIV(TMR2_BASE) = (AK_TIMER_HZ / AK_SERVO_TICK_HZ) - 1u;
    AK_TMR_PR(TMR2_BASE) = AK_SERVO_PERIOD_US - 1u;
    AK_TMR_C1DT(TMR2_BASE) = AK_SERVO_CENTER_US;
    AK_TMR_C2DT(TMR2_BASE) = AK_SERVO_CENTER_US;
    pwm_channel(TMR2_BASE, 1u);
    pwm_channel(TMR2_BASE, 2u);
    AK_TMR_CCE(TMR2_BASE) |= AK_TMR_CCE_C1EN | AK_TMR_CCE_C2EN;
    AK_TMR_CTRL1(TMR2_BASE) = AK_TMR_CTRL1_OCMEN;
    AK_TMR_EVEG(TMR2_BASE) = AK_TMR_EVEG_UG;
    AK_TMR_CTRL1(TMR2_BASE) |= AK_TMR_CTRL1_TMREN;

    /* --- motors: TMR4, PB6 and PB7, mux 2 --- */
    ak_pin_af(AK_PIN(GPIOB_BASE, 6), AK_GPIO_MUX_2, AK_GPIO_PULL_NONE);
    ak_pin_af(AK_PIN(GPIOB_BASE, 7), AK_GPIO_MUX_2, AK_GPIO_PULL_NONE);

    AK_TMR_DIV(TMR4_BASE) = 0u; /* one tick per timer clock */
    AK_TMR_C1DT(TMR4_BASE) = 0u;
    AK_TMR_C2DT(TMR4_BASE) = 0u;
    pwm_channel(TMR4_BASE, 1u);
    pwm_channel(TMR4_BASE, 2u);
    AK_TMR_CCE(TMR4_BASE) |= AK_TMR_CCE_C1EN | AK_TMR_CCE_C2EN;
    AK_TMR_CTRL1(TMR4_BASE) = AK_TMR_CTRL1_OCMEN;

    /* The per-channel DMA requests: channel 1's compare event for motor 1,
     * channel 2's for motor 2. */
    AK_TMR_IDEN(TMR4_BASE) = AK_TMR_IDEN_C1DEN | (AK_TMR_IDEN_C1DEN << 1u);

    dma_channel_init(0u, (uint32_t)(uintptr_t)&AK_TMR_C1DT(TMR4_BASE),
                     &dshot_plane[0][1]);
    dma_channel_init(1u, (uint32_t)(uintptr_t)&AK_TMR_C2DT(TMR4_BASE),
                     &dshot_plane[1][1]);
    AK_NVIC_ISER(AK_DMA1_CH1_IRQ / 32u) = (1u << (AK_DMA1_CH1_IRQ % 32u)) |
                                          (1u << (AK_DMA1_CH2_IRQ % 32u));

    AK_TMR_EVEG(TMR4_BASE) = AK_TMR_EVEG_UG;

    ak_output_set_rate(dshot_hz / 1000u);
    AK_TMR_CTRL1(TMR4_BASE) |= AK_TMR_CTRL1_TMREN;
}

void ak_output_set_rate(uint32_t khz)
{
    if (khz != 150u && khz != 300u && khz != 600u) {
        return;
    }
    dshot_hz = khz * 1000u;

    /* One timer period per bit: 288 MHz over 300 kHz is 960 ticks. */
    uint32_t ticks = AK_TIMER_HZ / dshot_hz;

    dshot_period = (uint16_t)(ticks - 1u);
    dshot_ccr_zero = ak_dshot_ccr_zero((uint16_t)ticks);
    dshot_ccr_one = ak_dshot_ccr_one((uint16_t)ticks);

    AK_TMR_PR(TMR4_BASE) = dshot_period;
}

void ak_output_write(const ak_output_frame_t *frame)
{
    /* The servos: the compare value is the pulse width in microseconds,
     * because the timer counts microseconds. */
    AK_TMR_C1DT(TMR2_BASE) = frame->servo_us[0];
    AK_TMR_C2DT(TMR2_BASE) = frame->servo_us[1];

    if (dshot_busy) {
        dshot_skipped++;
        return;
    }

    /* The frames first, in the core's own interleaved layout, then transposed
     * into one plane per motor - because a DMA channel walks its memory in
     * order and cannot stride over the other motors' values. */
    ak_dshot_fill(dshot_entries, frame->dshot, dshot_ccr_zero, dshot_ccr_one);
    for (unsigned m = 0; m < AK_MOTOR_COUNT; m++) {
        for (unsigned g = 0; g < AK_DSHOT_GROUPS; g++) {
            dshot_plane[m][g] = dshot_entries[g * AK_MAX_MOTORS + m];
        }
    }

    /* Bit 1 by hand, with the DMA starting at bit 2: the compare event that
     * starts a transfer is already one period late for the value it carries. */
    AK_TMR_C1DT(TMR4_BASE) = dshot_plane[0][0];
    AK_TMR_C2DT(TMR4_BASE) = dshot_plane[1][0];

    dshot_channels_done = 0u;
    dshot_busy = 1;
    AK_DMA_CH_CTRL(DMA1_BASE, 1u) |= AK_DMA_CTRL_CHEN;
    AK_DMA_CH_CTRL(DMA1_BASE, 2u) |= AK_DMA_CTRL_CHEN;
}

static void dshot_channel_finished(unsigned index)
{
    uint32_t n = index + 1u;

    AK_DMA_CLR(DMA1_BASE) = 1u << ((n - 1u) * 4u + 1u);
    AK_DMA_CH_CTRL(DMA1_BASE, n) &= ~(uint32_t)AK_DMA_CTRL_CHEN;

    if (++dshot_channels_done >= AK_MOTOR_COUNT) {
        /* The last group written was a blank one, so every compare value is
         * zero and the outputs idle low until the next frame. */
        dshot_busy = 0;
        dshot_frames++;
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    dshot_channel_finished(0u);
}

void DMA1_Channel2_IRQHandler(void)
{
    dshot_channel_finished(1u);
}

uint32_t ak_output_dshot_hz(void)
{
    return dshot_hz;
}

uint16_t ak_output_dshot_period(void)
{
    return dshot_period;
}

uint16_t ak_output_ccr_zero(void)
{
    return dshot_ccr_zero;
}

uint16_t ak_output_ccr_one(void)
{
    return dshot_ccr_one;
}

uint32_t ak_output_frames_sent(void)
{
    return dshot_frames;
}

uint32_t ak_output_frames_skipped(void)
{
    return dshot_skipped;
}

int ak_output_busy(void)
{
    return dshot_busy;
}
