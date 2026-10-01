/* Two I2C devices on the same two pins, naming *different* peripherals. The
 * pins are shared but the manifests say they are two buses, and two buses
 * cannot be one pair of wires. This is the legal-sharing rule read the other
 * way round: without it, "both are SCL" would be enough to allow anything. */
#define AK_BOARD_BARO_PORT 0
#define AK_BOARD_BARO_SCL_GPIO 22
#define AK_BOARD_BARO_SDA_GPIO 21
#define AK_BOARD_RANGE_PORT 1
#define AK_BOARD_RANGE_SCL_GPIO 22
#define AK_BOARD_RANGE_SDA_GPIO 21
