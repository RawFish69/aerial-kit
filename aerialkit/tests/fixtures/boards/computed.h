/* `#if` with an expression in it. Same reason as elif.h: refuse, do not guess. */
#define AK_BOARD_LED_GPIO 2
#if AK_BOARD_MOTORS > 2
#define AK_BOARD_MOTOR1_GPIO 2
#endif
