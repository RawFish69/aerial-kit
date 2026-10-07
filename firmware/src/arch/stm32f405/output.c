#include "arch.h"

#include "ak_output.h"
#include "ak_dshot_timing.h"

/*
 * Motors and servos on the STM32F405.
 *
 * Servos are ordinary timer PWM: a timer at 1 MHz, a 20 ms period, and a
 * compare value that is the pulse width in microseconds. **Which timer and
 * which pads are board facts**, handed in by `ak_output_init()` - the WeAct
 * board's servos are TIM2 channels 1-2 on PA0/PA1 and the Feather's are TIM4
 * channels 3-4 on PB8/PB9, because the Feather's breakout does not bring PA0 or
 * PA1 out. Both timers are on APB1 and count at 84 MHz, so nothing below the
 * pin and register selection is different between them.
 *
 * Motors are DShot, one timer period per bit on TIM3, so the bit rate is the
 * timer rate: 84 MHz / 280 = 300 kHz for DShot300. The compare values for a
 * '0' and a '1' are the duty cycles the ESC reads. Four channels are updated at
 * once by a DMA burst into TIM3_DMAR, triggered by the compare event of
 * channel 1, and the compare preload is what makes that safe: a write lands in
 * the preload register and takes effect at the next period boundary, so a
 * transfer arriving mid-bit cannot stretch or clip the bit it arrives in.
 *
 * The first bit of every frame is loaded by hand. The DMA's first burst lands
 * one period later than the values it carries are supposed to be used, so the
 * buffer starts at bit 2 and bit 1 is written directly. That off-by-one is the
 * kind of thing that shows up as a one-bit shift in the ESC's throttle, which
 * is why it is written down here and checked by eye on a scope.
 *
 * Every number in this file is a claim about hardware. The scope check is the
 * only thing that turns it into a fact - see docs/07-outputs.md.
 */

#define AK_SERVO_TIMER_HZ   84000000u /* the servo timers are on APB1, whose timer clock is x2 */
#define AK_SERVO_TICK_HZ    1000000u  /* 1 us resolution */
#define AK_SERVO_PERIOD_US  20000u    /* 50 Hz */

#define AK_DSHOT_TIMER_HZ   84000000u /* TIM3, also APB1 */
#define AK_DSHOT_GAP_US     2u        /* idle between frames, at least a bit time */

/* The servo bank, from ak_output_init(). The channel is kept per servo because
 * it is not always 1 and 2: TIM4's free pair on the Feather is 3 and 4, so the
 * compare register ak_output_write() must write is a per-servo answer. */
static uint32_t servo_timer;
static uint8_t  servo_channel[AK_MAX_SERVOS];
static unsigned servo_count;

static uint16_t dshot_entries[AK_DSHOT_ENTRIES];
static uint16_t dshot_period;      /* timer ticks per bit */
static uint16_t dshot_ccr_zero;
static uint16_t dshot_ccr_one;
static uint32_t dshot_hz = 300000u;
static uint32_t dshot_frames;
static uint32_t dshot_skipped;     /* frames that arrived while DMA was busy */
static volatile int dshot_busy;

static void gpio_af(uint32_t port, uint8_t pin, uint8_t af, uint32_t speed)
{
    ak_pin_af(AK_PIN(port, pin), af, GPIO_PUPD_NONE);
    uint32_t shift = (uint32_t)pin * 2u;
    GPIO_OSPEEDR(port) = (GPIO_OSPEEDR(port) & ~(0x3u << shift)) |
                         ((uint32_t)speed << shift);
}

static void timer_channel_pwm(uint32_t timer, uint8_t channel, uint8_t invert)
{
    volatile uint32_t *ccmr = channel <= 2 ? &TIM_CCMR1(timer) : &TIM_CCMR2(timer);
    uint32_t shift = (channel == 1u || channel == 3u) ? 4u : 12u;

    /* PWM mode 1, compare preload on: the compare value a write puts here is
     * latched at the next update event, not used immediately. */
    *ccmr = (*ccmr & ~(0x7u << shift)) |
            ((uint32_t)TIM_CCMR_OCM_PWM1 << shift) |
            (1u << (shift - 1u));

    TIM_CCER(timer) |= TIM_CCER_CCE(channel);
    if (invert) {
        TIM_CCER(timer) |= TIM_CCER_CCE(channel) << 1u; /* CCxP */
    }
}

/* The compare register of one channel: the one ak_output_write() puts a pulse
 * width in, and the one the center value is loaded into at init. */
static volatile uint32_t *channel_ccr(uint32_t timer, uint8_t channel)
{
    switch (channel) {
    case 1u: return &TIM_CCR1(timer);
    case 2u: return &TIM_CCR2(timer);
    case 3u: return &TIM_CCR3(timer);
    default: return &TIM_CCR4(timer);
    }
}

/* The clock gate of the servo timer the board named. A mapping rather than a
 * guess: the two timers this tree's F405 boards put servos on, both on APB1. A
 * board asking for a third gets no gate here and says so through a servo that
 * does not move, which is the same answer a wrong pad gives. */
static void servo_timer_clock(uint32_t timer)
{
    if (timer == TIM2_BASE) {
        RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
    } else if (timer == TIM4_BASE) {
        RCC_APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
}

void ak_output_init(uint32_t timer, const ak_servo_out_t *servos, unsigned count)
{
    /* TIM3 and the servo timer are on APB1, and the timer clock on APB1 is
     * twice the bus clock when the prescaler is not 1 - 84 MHz here, which is
     * where the bit rates below come from. */
    RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* --- servos: the pads and the timer the board named --- */
    servo_timer = timer;
    servo_count = count > AK_MAX_SERVOS ? AK_MAX_SERVOS : count;

    servo_timer_clock(servo_timer);
    for (unsigned i = 0; i < servo_count; i++) {
        gpio_af(servos[i].port, servos[i].pin, servos[i].af, GPIO_SPEED_HIGH);
        servo_channel[i] = servos[i].channel;
    }

    TIM_PSC(servo_timer) = (AK_SERVO_TIMER_HZ / AK_SERVO_TICK_HZ) - 1u;
    TIM_ARR(servo_timer) = AK_SERVO_PERIOD_US - 1u;
    for (unsigned i = 0; i < servo_count; i++) {
        *channel_ccr(servo_timer, servo_channel[i]) = AK_SERVO_CENTER_US;
        timer_channel_pwm(servo_timer, servo_channel[i], 0);
    }
    TIM_CR1(servo_timer) = TIM_CR1_ARPE;
    TIM_EGR(servo_timer) = TIM_EGR_UG;
    TIM_CR1(servo_timer) |= TIM_CR1_CEN;

    /* --- motors: TIM3 CH1 PA6, CH2 PA7, CH3 PB0, CH4 PB1 --- */
    gpio_af(GPIOA_BASE, 6, 2, GPIO_SPEED_FAST);
    gpio_af(GPIOA_BASE, 7, 2, GPIO_SPEED_FAST);
    gpio_af(GPIOB_BASE, 0, 2, GPIO_SPEED_FAST);
    gpio_af(GPIOB_BASE, 1, 2, GPIO_SPEED_FAST);

    TIM_PSC(TIM3_BASE) = 0;
    TIM_CCR1(TIM3_BASE) = 0;
    TIM_CCR2(TIM3_BASE) = 0;
    TIM_CCR3(TIM3_BASE) = 0;
    TIM_CCR4(TIM3_BASE) = 0;
    for (uint8_t channel = 1; channel <= 4; channel++) {
        timer_channel_pwm(TIM3_BASE, channel, 0);
    }
    TIM_CR1(TIM3_BASE) = TIM_CR1_ARPE;

    /* --- the DMA burst that writes all four compare registers at once ---
     *
     * DCR.DBA is the burst base, counted in 32-bit words from the timer's base
     * address: CCR1 is at 0x34, which is word 13. DCR.DBL of 3 asks for four
     * transfers of 16 bits, which is CCR1, CCR2, CCR3 and CCR4 in that order -
     * the order the buffer in ak_dshot_timing.c is laid out in. */
    TIM_DCR(TIM3_BASE) = TIM_DCR_DBA(0x34u / 4u) | TIM_DCR_DBL(3u);

    DMA_SxCR(DMA1_BASE, AK_DMA_TIM3_STREAM) = 0;
    DMA_SxPAR(DMA1_BASE, AK_DMA_TIM3_STREAM) =
        (uint32_t)(uintptr_t)&TIM_DMAR(TIM3_BASE);
    DMA_SxM0AR(DMA1_BASE, AK_DMA_TIM3_STREAM) = (uint32_t)(uintptr_t)&dshot_entries[AK_MAX_MOTORS];
    DMA_SxNDTR(DMA1_BASE, AK_DMA_TIM3_STREAM) =
        (AK_DSHOT_GROUPS - 1u) * AK_MAX_MOTORS;
    DMA_SxCR(DMA1_BASE, AK_DMA_TIM3_STREAM) =
        DMA_SxCR_DIR_M2P | DMA_SxCR_MINC | DMA_SxCR_PSIZE_16 |
        DMA_SxCR_MSIZE_16 | DMA_SxCR_PL_HIGH | DMA_SxCR_TCIE |
        DMA_SxCR_CHSEL(AK_DMA_TIM3_CHANNEL);

    NVIC_ISER0 = 1u << DMA1_STREAM4_IRQ;

    /* The burst runs on the compare event of channel 1: one burst per bit
     * period, which is exactly one group of four compare values. The request
     * is *not* enabled here, only for the length of a frame (ak_output_write
     * turns it on, the interrupt off): with it on while the stream is off,
     * every period's compare queues a request, and the first one served when
     * the stream starts wrote bit 2 over the bit 1 just loaded by hand -
     * compare preload is on, so bit 1 had not been latched yet - and every
     * frame went out fifteen bits long. Betaflight's DShot driver gates the
     * timer's DMA request off between frames for the same reason. */
    TIM_DIER(TIM3_BASE) = 0u;
    TIM_EGR(TIM3_BASE) = TIM_EGR_UG;

    ak_output_set_rate(dshot_hz / 1000u);
    TIM_CR1(TIM3_BASE) |= TIM_CR1_CEN;
}

void ak_output_set_rate(uint32_t khz)
{
    if (khz != 150u && khz != 300u && khz != 600u) {
        return;
    }
    dshot_hz = khz * 1000u;

    /* One timer period per bit, so the period is the timer clock over the bit
     * rate. 84 MHz / 300 kHz = 280 ticks. */
    uint32_t ticks = AK_DSHOT_TIMER_HZ / dshot_hz;
    dshot_period = (uint16_t)(ticks - 1u);
    dshot_ccr_zero = ak_dshot_ccr_zero((uint16_t)ticks);
    dshot_ccr_one = ak_dshot_ccr_one((uint16_t)ticks);

    TIM_ARR(TIM3_BASE) = dshot_period;
}

void ak_output_write(const ak_output_frame_t *frame)
{
    /* Servos: the compare value is the pulse width in microseconds, because
     * the timer counts microseconds. Which compare register is the board's
     * channel, and a board with no servo outputs writes none of them. */
    for (unsigned i = 0; i < servo_count; i++) {
        *channel_ccr(servo_timer, servo_channel[i]) = frame->servo_us[i];
    }

    if (dshot_busy) {
        dshot_skipped++;
        return;
    }

    ak_dshot_fill(dshot_entries, frame->dshot, dshot_ccr_zero, dshot_ccr_one);

    /* Bit 1 by hand; the DMA starts at bit 2, one period behind. */
    TIM_CCR1(TIM3_BASE) = dshot_entries[0];
    TIM_CCR2(TIM3_BASE) = dshot_entries[1];
    TIM_CCR3(TIM3_BASE) = dshot_entries[2];
    TIM_CCR4(TIM3_BASE) = dshot_entries[3];

    dshot_busy = 1;
    /* No stale compare event, then the stream, then the request: the first
     * transfer is then the next compare, after the update that latches bit 1.
     * TIM_SR is write-zero-to-clear, so ones elsewhere leave the rest alone. */
    TIM_SR(TIM3_BASE) = ~TIM_SR_CC1IF;
    DMA_SxCR(DMA1_BASE, AK_DMA_TIM3_STREAM) |= DMA_SxCR_EN;
    TIM_DIER(TIM3_BASE) |= TIM_DIER_CC1DE;
}

void DMA1_Stream4_IRQHandler(void)
{
    /* Stream 4 lives in the high half of the flag registers. */
    DMA_HIFCR(DMA1_BASE) = DMA_IFCR_CLEAR(4u);
    TIM_DIER(TIM3_BASE) &= ~TIM_DIER_CC1DE; /* no requests between frames */
    DMA_SxCR(DMA1_BASE, AK_DMA_TIM3_STREAM) &= ~DMA_SxCR_EN;

    /* The last groups written were the blank ones, so the compare values are
     * zero and every output is low until the next frame. */
    dshot_busy = 0;
    dshot_frames++;
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

const char *ak_output_servo_timer_name(void)
{
    /* Named from the base the board handed in, not from a literal in a board's
     * report: the two F405 boards put their servos on different timers, and a
     * line that says TIM2 while the pulses come out of TIM4 is the banner bug
     * this codebase has paid for twice (traps 203). */
    if (servo_count == 0u) {
        return "none";
    }
    return servo_timer == TIM4_BASE ? "TIM4" : "TIM2";
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
