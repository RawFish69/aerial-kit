/*
 * Where IDF starts, and the whole of what this file does.
 *
 * AerialKit's entry point is a plain main() that never returns, and IDF's is
 * app_main() running inside a FreeRTOS task. One calls the other: there is no
 * port-specific flight code, no second initialisation order, and no reason for
 * the firmware to know which of the two it is running under.
 */

#include "ak_main.h"

void app_main(void)
{
    (void)ak_firmware_main();
}
