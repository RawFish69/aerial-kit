/* A conditional this validator does not evaluate. It has to refuse rather than
 * read on with one arm of it, because the arm it skipped is where the
 * collision may be. */
#define AK_BOARD_LED_GPIO 2
#ifdef AK_SOMETHING
#define AK_BOARD_MOTOR1_GPIO 3
#elif defined(AK_OTHER)
#define AK_BOARD_MOTOR1_GPIO 2
#endif
