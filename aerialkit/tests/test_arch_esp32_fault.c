/*
 * The ESP32's fault story, and its LED, on the host.
 *
 * Three files on that target had never run anywhere with anything checked:
 *
 *   `fault.c`   - the STM32 port catches the exception itself and keeps a
 *                 record. IDF owns the panic path, so this port's honest
 *                 equivalent is to ask IDF *why* the chip restarted on the next
 *                 boot. That branch runs when a crash happened - which is to
 *                 say never, on an emulator that boots cleanly - so what the
 *                 model does is let the reason be set.
 *   `system.c`  - one call (`esp_restart()`), and the CPU clock the image was
 *                 built for.
 *   `led.c`     - the heartbeat, which a person at a bench reads before they
 *                 read anything else: two of the four tell-tale patterns are
 *                 this file.
 *
 * The port's entry points are renamed for this build (`-Dak_fault=ak_esp_fault`
 * and friends) because the *tests* binary already holds the ARM port's fault
 * record and its reset - one firmware has one of each, and this is the same
 * trick the third target's board file uses to live in one binary with the F405's.
 */

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "esp.h"

#include "driver/gpio.h"
#include "sdkconfig.h"

#include "ak_fault.h"
#include "host_esp32_system_model.h"
#include "tests.h"

/* The renamed entry points of src/arch/esp32/fault.c and system.c. */
extern ak_fault_record_t ak_esp_fault;
int ak_esp_fault_present(void);
void ak_esp_fault_clear(void);
void ak_esp_arch_reset(void);

void test_arch_esp32_fault(void)
{
    host_esp_reset();

    /* --- a clean boot is not a fault, however the chip got there ---------- */

    ak_esp_fault_clear();
    host_esp_set_reset_reason(ESP_RST_POWERON);
    ak_esp_note_reset_reason();
    expect("a chip that was switched on has no fault recorded",
           !ak_esp_fault_present());

    host_esp_set_reset_reason(ESP_RST_SW);
    ak_esp_note_reset_reason();
    expect("and neither has one that restarted because somebody said so",
           !ak_esp_fault_present());

    host_esp_set_reset_reason(ESP_RST_DEEPSLEEP);
    ak_esp_note_reset_reason();
    expect("nor one that woke out of deep sleep", !ak_esp_fault_present());

    /* --- and the three ways a chip can crash are a fault ------------------ */

    host_esp_set_reset_reason(ESP_RST_PANIC);
    ak_esp_note_reset_reason();
    expect("a panic is recorded as a fault", ak_esp_fault_present());
    expect("with the reason IDF gave, in the field a reader looks at",
           ak_esp_fault.cfsr == (uint32_t)ESP_RST_PANIC);
    expect("counted once", ak_esp_fault.count == 1u);
    expect("and with no pc, because IDF printed the backtrace and this port "
           "did not keep it", ak_esp_fault.pc == 0u);

    ak_esp_fault_clear();
    host_esp_set_reset_reason(ESP_RST_TASK_WDT);
    ak_esp_note_reset_reason();
    expect("the task watchdog is a fault too",
           ak_esp_fault_present() &&
               ak_esp_fault.cfsr == (uint32_t)ESP_RST_TASK_WDT);

    ak_esp_fault_clear();
    host_esp_set_reset_reason(ESP_RST_INT_WDT);
    ak_esp_note_reset_reason();
    expect("and so is the interrupt watchdog",
           ak_esp_fault_present() &&
               ak_esp_fault.cfsr == (uint32_t)ESP_RST_INT_WDT);

    /* One more watchdog IDF has, so the port's list is checked rather than
     * assumed to be two of them. */
    ak_esp_fault_clear();
    host_esp_set_reset_reason(ESP_RST_WDT);
    ak_esp_note_reset_reason();
    expect("and the other watchdog",
           ak_esp_fault_present() &&
               ak_esp_fault.cfsr == (uint32_t)ESP_RST_WDT);

    ak_esp_fault_clear();
    expect("clearing it is what makes the next boot quiet",
           !ak_esp_fault_present());

    /* --- the reset itself, and the clock the image was built for ---------- */

    expect("the CPU clock is the one the build was configured with",
           ak_esp_cpu_mhz() == (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    expect("and it is a plausible ESP32 clock, not zero",
           ak_esp_cpu_mhz() >= 80u && ak_esp_cpu_mhz() <= 240u);

    expect("a reboot asks IDF for one", host_esp_restarts() == 0u);
    ak_esp_arch_reset();
    expect("exactly once", host_esp_restarts() == 1u);

    /* --- the LED, which is two of the tell-tale's four patterns ----------- */

    host_esp_reset();
    expect("a pin nothing has touched is not being driven",
           host_gpio_level(AK_BOARD_LED_GPIO) < 0 &&
               host_gpio_pin_config_count() == 0u);

    ak_esp_led_init(AK_BOARD_LED_GPIO);
    expect("the LED's pin is configured as an output",
           host_gpio_pin_config_count() == 1u &&
               host_gpio_pin_mask() == (1ULL << (uint32_t)AK_BOARD_LED_GPIO) &&
               host_gpio_mode() == GPIO_MODE_OUTPUT);
    expect("with no pull and no interrupt, because it is an LED",
           host_gpio_pull_up() == GPIO_PULLUP_DISABLE &&
               host_gpio_pull_down() == GPIO_PULLDOWN_DISABLE &&
               host_gpio_intr() == GPIO_INTR_DISABLE);
    expect("and it starts out dark",
           host_gpio_level(AK_BOARD_LED_GPIO) == 0 && ak_esp_led_state() == 0);

    ak_esp_led_set(1);
    expect("setting it lights the pin and says so",
           host_gpio_level(AK_BOARD_LED_GPIO) == 1 && ak_esp_led_state() == 1);
    ak_esp_led_set(0);
    expect("and clearing it does the same the other way",
           host_gpio_level(AK_BOARD_LED_GPIO) == 0 && ak_esp_led_state() == 0);

    /* The port keeps the pin it was given, so a second set drives the same
     * wire rather than inventing one - which is what the tell-tale's blink
     * loop depends on. */
    ak_esp_led_set(1);
    expect("and the pin it drives is the one it was initialised with",
           host_gpio_level(AK_BOARD_LED_GPIO) == 1 &&
               host_gpio_pin_config_count() == 1u);
}
