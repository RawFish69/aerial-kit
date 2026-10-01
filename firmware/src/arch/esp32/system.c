#include "esp.h"

#include "esp_system.h"
#include "sdkconfig.h"

void ak_arch_reset(void)
{
    esp_restart();
}

uint32_t ak_esp_cpu_mhz(void)
{
    /* From the build configuration rather than a query: nothing in this port
     * changes the clock at runtime, and the value IDF was configured with is
     * the one the firmware is running at. */
    return (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
}
