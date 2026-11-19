#include <stdint.h>

/*
 * The ESP32-S3 GPIO input register. This code runs from the second-stage
 * bootloader, after GPIO is initialized and before either ExpressLRS app is
 * loaded. GPIO4 has an external pull-up on Rev A3: low selects TX, high RX.
 */
#define AERIALKIT_GPIO_IN_REG (*(volatile uint32_t *)0x6000403cU)
#define AERIALKIT_IO_MUX_GPIO4_REG (*(volatile uint32_t *)0x60009014U)
#define AERIALKIT_GPIO4_FUNCTION 1U
#define AERIALKIT_IO_MUX_FUNCTION_SHIFT 12U
#define AERIALKIT_IO_MUX_INPUT_ENABLE (1U << 9)

__attribute__((section(".iram.text.aerialkit_selector"), used, noinline))
int aerialkit_role_partition(const void *partition_state)
{
    (void)partition_state;
    AERIALKIT_IO_MUX_GPIO4_REG =
        (AERIALKIT_IO_MUX_GPIO4_REG & ~(7U << AERIALKIT_IO_MUX_FUNCTION_SHIFT)) |
        (AERIALKIT_GPIO4_FUNCTION << AERIALKIT_IO_MUX_FUNCTION_SHIFT) |
        AERIALKIT_IO_MUX_INPUT_ENABLE;
    return (AERIALKIT_GPIO_IN_REG & (1U << 4)) ? 1 : 0;
}
