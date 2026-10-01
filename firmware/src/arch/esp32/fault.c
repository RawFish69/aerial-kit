#include "esp.h"

#include "ak_fault.h"

#include "esp_system.h"

/*
 * Faults on the ESP32, told differently.
 *
 * The STM32 port catches the exception itself: it reads the fault status
 * registers and the stacked registers out of its own handler and keeps them in
 * RAM that startup does not clear. On the ESP32 the panic path belongs to
 * IDF - it prints a backtrace, can dump core to flash, and resets - so the
 * honest equivalent is to ask IDF afterwards *why* the chip restarted. That is
 * one bit of information instead of a register dump, and it is the bit that
 * matters on a first boot: "it restarted because it panicked" against "it
 * restarted because somebody rebooted it".
 */

ak_fault_record_t ak_fault;

int ak_fault_present(void)
{
    return ak_fault.magic == AK_FAULT_MAGIC;
}

void ak_fault_clear(void)
{
    ak_fault.magic = 0;
    ak_fault.count = 0;
}

/* Called from the board's init, which is the first thing our code runs: by then
 * IDF has already worked out why it started. */
void ak_esp_note_reset_reason(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    if (reason == ESP_RST_PANIC || reason == ESP_RST_TASK_WDT ||
        reason == ESP_RST_INT_WDT || reason == ESP_RST_WDT) {
        ak_fault.count = 1;
        /* The field is a fault status register on the STM32 and a reset reason
         * here; both are "which kind of failure was it", which is what the
         * reader wants from it. */
        ak_fault.cfsr = (uint32_t)reason;
        ak_fault.pc = 0; /* IDF printed the backtrace; we did not keep it */
        ak_fault.magic = AK_FAULT_MAGIC;
    }
}
