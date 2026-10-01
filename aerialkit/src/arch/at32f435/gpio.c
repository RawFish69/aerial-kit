#include "arch.h"

/*
 * Pins, on the part whose pins are not the F405's.
 *
 * Three things here are this part's rather than a re-naming:
 *
 *   - a pin's mode is two bits in `cfgr`, and the *function number* it takes in
 *     mux mode lives in a different register (`muxl` for pins 0-7, `muxh` for
 *     8-15) - there is no AFR;
 *   - output type, drive strength and pull are each their own per-pin field, so
 *     configuring one pin touches four registers rather than two;
 *   - a port's clock has to be enabled before any of that, and the enable bit
 *     is the port's index in a register of its own.
 *
 * What is *not* different is the shape of the API: a board file says "output",
 * "alternate function 7 with a pull-up", "analog", "set", "toggle", and the two
 * ports do whatever their part requires. That is the whole point of the layer.
 */

/* Port clock enable bits follow the port base addresses: GPIOA at 0x40020000
 * and one 0x400 step per port, so the bit is that index - the same trick the
 * F405 port uses, and the same trap if a port ever moves. */
static void port_clock_enable(uint32_t port)
{
    uint32_t bit = (uint32_t)((port - GPIOA_BASE) / AK_GPIO_PORT_STEP);

    CRM_AHBEN1 |= 1u << bit;
    (void)CRM_AHBEN1; /* the read back is what makes the write happen */
}

static volatile uint32_t *mux_register(uint32_t port, uint8_t pin)
{
    return pin < 8u ? &AK_GPIO_MUXL(port) : &AK_GPIO_MUXH(port);
}

static void set_mode(uint32_t port, uint8_t pin, uint32_t mode)
{
    uint32_t shift = (uint32_t)pin * 2u;

    AK_GPIO_CFGR(port) = (AK_GPIO_CFGR(port) & ~(0x3u << shift)) |
                         (mode << shift);
}

static void set_pull(uint32_t port, uint8_t pin, uint32_t pull)
{
    uint32_t shift = (uint32_t)pin * 2u;

    AK_GPIO_PULL(port) = (AK_GPIO_PULL(port) & ~(0x3u << shift)) |
                         (pull << shift);
}

static void set_output_type(uint32_t port, uint8_t pin, int open_drain)
{
    AK_GPIO_OMODE(port) = (AK_GPIO_OMODE(port) & ~(1u << pin)) |
                          ((open_drain ? 1u : 0u) << pin);
}

void ak_pin_output(ak_pin_t pin, int open_drain, uint32_t speed)
{
    port_clock_enable(pin.port);
    set_mode(pin.port, pin.pin, AK_GPIO_MODE_OUTPUT);
    set_output_type(pin.port, pin.pin, open_drain);
    /* The drive strength is the argument the F405 port calls `speed`: a board
     * asking for a fast pin gets the stronger driver, and zero gets the
     * moderate one, because there is no "weakest" on this part. */
    uint32_t shift = (uint32_t)pin.pin * 2u;
    AK_GPIO_ODRVR(pin.port) =
        (AK_GPIO_ODRVR(pin.port) & ~(0x3u << shift)) |
        ((speed != 0u ? AK_GPIO_DRIVE_STRONGER : AK_GPIO_DRIVE_MODERATE)
         << shift);
}

void ak_pin_af(ak_pin_t pin, uint8_t af, uint32_t pull)
{
    port_clock_enable(pin.port);
    set_mode(pin.port, pin.pin, AK_GPIO_MODE_MUX);
    set_output_type(pin.port, pin.pin, 0);
    set_pull(pin.port, pin.pin, pull);

    volatile uint32_t *mux = mux_register(pin.port, pin.pin);
    uint32_t nibble = (uint32_t)(pin.pin & 0x7u) * 4u;

    *mux = (*mux & ~(0xFu << nibble)) | ((uint32_t)(af & 0xFu) << nibble);
}

/*
 * And the drive strength on its own, for a pin whose mode is already chosen.
 *
 * It exists for one peripheral: the USB transceiver, whose two data pins want
 * the stronger driver - that part's own configuration says so
 * (`gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER` in the
 * USB library's `usb_gpio_config`), and on a 12 Mb/s differential pair the
 * weakest setting is the wrong place to save a milliamp. The F405 port does not
 * need an equivalent because its `ak_pin_af` writes a slew rate for every mux
 * pin; this part's has a separate drive field, and this is the one caller that
 * asks for it rather than taking whatever the reset value is.
 */
void ak_pin_drive(ak_pin_t pin, uint32_t speed)
{
    uint32_t shift = (uint32_t)pin.pin * 2u;

    AK_GPIO_ODRVR(pin.port) =
        (AK_GPIO_ODRVR(pin.port) & ~(0x3u << shift)) |
        ((speed != 0u ? AK_GPIO_DRIVE_STRONGER : AK_GPIO_DRIVE_MODERATE)
         << shift);
}

/*
 * The same, for a pin that is only ever allowed to pull down.
 *
 * I2C needs it and nothing else here does: the bus is pulled up by resistors
 * elsewhere, and a pin that drives it high at the wrong moment is a pin that
 * fights every other device on the wire. The F405 port has the same function
 * for the same reason.
 */
void ak_pin_af_open_drain(ak_pin_t pin, uint8_t af, uint32_t pull)
{
    ak_pin_af(pin, af, pull);
    set_output_type(pin.port, pin.pin, 1);
}

void ak_pin_analog(ak_pin_t pin)
{
    port_clock_enable(pin.port);
    set_mode(pin.port, pin.pin, AK_GPIO_MODE_ANALOG);
    set_pull(pin.port, pin.pin, AK_GPIO_PULL_NONE);
}

void ak_pin_set(ak_pin_t pin, int level)
{
    /* Set and clear are separate registers, and each takes a one written into
     * the pin's bit: writing the output register directly would be a read,
     * modify and write of every other pin on the port. */
    if (level) {
        AK_GPIO_SCR(pin.port) = 1u << pin.pin;
    } else {
        AK_GPIO_CLR(pin.port) = 1u << pin.pin;
    }
}

void ak_pin_toggle(ak_pin_t pin)
{
    /* The output register is readable on this part, so the level can be read
     * and inverted without keeping a shadow copy - which is what the F405 port
     * does too, and for the same reason: a shadow is a second source of truth. */
    ak_pin_set(pin, (AK_GPIO_ODT(pin.port) & (1u << pin.pin)) == 0u);
}

int ak_pin_get(ak_pin_t pin)
{
    return (AK_GPIO_IDT(pin.port) & (1u << pin.pin)) != 0u;
}
