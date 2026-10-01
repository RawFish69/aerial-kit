/* A board with nothing wrong with it: two I2C devices on one pair, which is a
 * bus, and a set of signals that do not meet. It exists so that a validator
 * that reports nothing is told apart from one that reads nothing. */
#define AK_BOARD_MOTORS 4u
#define AK_BOARD_SERVOS 0u
#define AK_BOARD_LED_GPIO 2
#define AK_BOARD_MOTOR1_GPIO 4
#define AK_BOARD_MOTOR2_GPIO 5
#define AK_BOARD_RC_TX_GPIO 17
#define AK_BOARD_RC_RX_GPIO 16
#define AK_BOARD_BARO_PORT 0
#define AK_BOARD_BARO_SCL_GPIO 22
#define AK_BOARD_BARO_SDA_GPIO 21
#define AK_BOARD_RANGE_PORT 0
#define AK_BOARD_RANGE_SCL_GPIO 22
#define AK_BOARD_RANGE_SDA_GPIO 21
