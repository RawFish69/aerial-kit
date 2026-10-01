/* A sensor that is not there yet, on a pin the receiver already uses. The bare
 * board is fine and the fitted one is not, which is the ESP32C3DEV's shape and
 * the reason the FITTED flags are read rather than the pins alone. */
#define AK_BOARD_RC_RX_GPIO 10
#define AK_BOARD_IMU_CS_GPIO 10
#ifndef AK_BOARD_IMU_FITTED
#define AK_BOARD_IMU_FITTED 0
#endif
