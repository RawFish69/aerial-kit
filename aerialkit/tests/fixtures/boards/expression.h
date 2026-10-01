/* A pin whose right-hand side is not a number or an AK_PIN: this could be
 * anything, so it is refused rather than compared as text. */
#define AK_BOARD_LED_GPIO 2
#define AK_BOARD_RC_RX_GPIO (AK_BOARD_LED_GPIO + 1)
