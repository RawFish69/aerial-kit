#include "arch.h"

/* Port clock enable bits follow the port base addresses: GPIOA at 0x40020000
 * and one 1 KB step per port, so the AHB1ENR bit is that index. */
static void port_clock_enable(uint32_t port)
{
    /* The cast is for the host build, where a `uintptr_t` difference is wider
     * than the register it lands in; on the target it is a no-op. */
    uint32_t bit = (uint32_t)((port - GPIOA_BASE) / 0x400u);
    RCC_AHB1ENR |= 1u << bit;
    (void)RCC_AHB1ENR;
}

void ak_pin_output(ak_pin_t pin, int open_drain, uint32_t speed)
{
    port_clock_enable(pin.port);

    uint32_t shift = (uint32_t)pin.pin * 2u;
    GPIO_MODER(pin.port) = (GPIO_MODER(pin.port) & ~(0x3u << shift)) |
                           (GPIO_MODE_OUTPUT << shift);
    GPIO_OTYPER(pin.port) = (GPIO_OTYPER(pin.port) & ~(0x1u << pin.pin)) |
                            ((open_drain ? GPIO_OTYPE_OPEN_DRAIN
                                         : GPIO_OTYPE_PUSH_PULL) << pin.pin);
    GPIO_OSPEEDR(pin.port) = (GPIO_OSPEEDR(pin.port) & ~(0x3u << shift)) |
                             (speed << shift);
}

void ak_pin_af(ak_pin_t pin, uint8_t af, uint32_t pull)
{
    port_clock_enable(pin.port);

    uint32_t shift = (uint32_t)pin.pin * 2u;
    GPIO_MODER(pin.port) = (GPIO_MODER(pin.port) & ~(0x3u << shift)) |
                           (GPIO_MODE_AF << shift);
    GPIO_OTYPER(pin.port) &= ~(0x1u << pin.pin);
    GPIO_OSPEEDR(pin.port) = (GPIO_OSPEEDR(pin.port) & ~(0x3u << shift)) |
                             (GPIO_SPEED_HIGH << shift);
    GPIO_PUPDR(pin.port) = (GPIO_PUPDR(pin.port) & ~(0x3u << shift)) |
                           (pull << shift);

    volatile uint32_t *afr = pin.pin < 8 ? &GPIO_AFRL(pin.port)
                                         : &GPIO_AFRH(pin.port);
    uint32_t nibble = (uint32_t)(pin.pin & 0x7u) * 4u;
    *afr = (*afr & ~(0xFu << nibble)) | ((uint32_t)af << nibble);
}

/*
 * The same, for a pin that is only ever allowed to pull down.
 *
 * I2C needs it and nothing else here does: the bus is pulled up by resistors
 * and every device on it can only pull it low, so a push-pull output is two
 * devices driving the same wire in opposite directions the first time two of
 * them speak. It is its own function rather than a flag on ak_pin_af() for the
 * same reason the UART's parity is: an argument at the call site that nobody
 * reads is an argument nobody gets right.
 */
void ak_pin_af_open_drain(ak_pin_t pin, uint8_t af, uint32_t pull)
{
    ak_pin_af(pin, af, pull);
    GPIO_OTYPER(pin.port) |= 0x1u << pin.pin;
}

/*
 * A pin an ADC is allowed to read.
 *
 * Analog mode is not a default: a pin left as a digital input has its Schmitt
 * trigger connected, and that draws current from whatever is driving it and
 * loads the very divider the reading is supposed to measure. The pull is off
 * for the same reason - a pull-up on an analog input is a resistor in parallel
 * with the top half of the divider.
 */
void ak_pin_analog(ak_pin_t pin)
{
    port_clock_enable(pin.port);

    uint32_t shift = (uint32_t)pin.pin * 2u;
    GPIO_MODER(pin.port) = (GPIO_MODER(pin.port) & ~(0x3u << shift)) |
                           (GPIO_MODE_ANALOG << shift);
    GPIO_PUPDR(pin.port) &= ~(0x3u << shift);
}

void ak_pin_set(ak_pin_t pin, int level)
{
    GPIO_BSRR(pin.port) = level ? (1u << pin.pin) : (1u << (pin.pin + 16u));
}

void ak_pin_toggle(ak_pin_t pin)
{
    GPIO_ODR(pin.port) ^= 1u << pin.pin;
}

int ak_pin_get(ak_pin_t pin)
{
    return (GPIO_ODR(pin.port) & (1u << pin.pin)) != 0;
}
