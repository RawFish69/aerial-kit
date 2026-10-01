#include "board.h"

#include "ak_board.h"
#include "ak_battery.h"
#include "ak_console.h"
#include "ak_dshot_timing.h"
#include "ak_log.h"
#include "ak_params.h"
#include "ak_time.h"
#include "ak_version.h"

static int led_lit;
static char clock_summary[96];

/* Defined with the barometer's bus further down, and wanted by the init above
 * it: the bus is configured at boot whether or not a part is fitted. */
static void baro_bus_init(void);

static void put_dec(char *buf, unsigned *len, uint32_t value)
{
    char digits[10];
    unsigned n = 0;

    if (value == 0) {
        buf[(*len)++] = '0';
        return;
    }
    while (value > 0) {
        digits[n++] = (char)('0' + (value % 10u));
        value /= 10u;
    }
    while (n > 0) {
        buf[(*len)++] = digits[--n];
    }
}

static void put_text(char *buf, unsigned *len, const char *text)
{
    while (*text != '\0') {
        buf[(*len)++] = *text++;
    }
}

static void build_clock_summary(void)
{
    unsigned n = 0;

    put_dec(clock_summary, &n, ak_clk_sysclk_hz() / 1000000u);
    put_text(clock_summary, &n, " MHz sysclk (");
    if (ak_clk_hse_ok()) {
        /* The crystal as measured at boot (clk.c), or this board's declared
         * 8 MHz when the measurement did not fire.
         *
         * The fallback read the literal 8u until 2026-09-29, when it became
         * this header's macro - and the header then said 12, so for a day the
         * declared fallback and the measurement's honest value were the two
         * different numbers this board has been called. The 8 was right: see
         * `board.h`, which carries the whole correction, and traps 212.
         *
         * Whoever reads this line next should read the `, assumed` suffix
         * too. `HSE 8 MHz x PLL` with no suffix is the *measured* branch, and
         * the measurement is printed in whole megahertz, so it cannot name a
         * crystal more precisely than that. */
        uint32_t hse = ak_clk_hse_hz();

        put_text(clock_summary, &n, "HSE ");
        put_dec(clock_summary, &n,
                hse != 0u ? (hse + 500000u) / 1000000u : AK_BOARD_HSE_MHZ);
        put_text(clock_summary, &n, hse != 0u ? " MHz x PLL" : " MHz x PLL, assumed");
    } else {
        put_text(clock_summary, &n, "HSI, HSE FAILED");
    }
    put_text(clock_summary, &n, "), apb1 ");
    put_dec(clock_summary, &n, ak_clk_apb1_hz() / 1000000u);
    put_text(clock_summary, &n, " MHz, apb2 ");
    put_dec(clock_summary, &n, ak_clk_apb2_hz() / 1000000u);
    put_text(clock_summary, &n, " MHz");
    clock_summary[n] = '\0';
}

void ak_board_init(void)
{
    ak_clk_init();

    /*
     * The status pin is set up here, before the buses and the console, because
     * it is the instrument the tell-tale build blinks: a stage that runs
     * before the pin exists is a stage nobody can see. It is turned off at
     * once - the pin's reset state is low and this LED is active low, so a
     * board that left it there would be a board that looks lit from reset.
     */
    ak_pin_output(AK_BOARD_LED_PIN, 0, GPIO_SPEED_LOW);
    ak_board_led_set(0);
    ak_boot_mark(AK_BOOT_CLOCK);

    baro_bus_init();
    ak_boot_mark(AK_BOOT_BARO_BUS);

    ak_uart_init(AK_BOARD_CONSOLE_USART, AK_BOARD_CONSOLE_TX,
                 AK_BOARD_CONSOLE_RX, AK_BOARD_CONSOLE_BAUD,
                 AK_BOARD_CONSOLE_AF);
    ak_console_attach(AK_BOARD_CONSOLE_USART);
    ak_boot_mark(AK_BOOT_CONSOLE);

    /* The console's second road. Bringing USB up here means every byte the
     * firmware prints from now on also waits for a host on the USB port, which
     * is the one a laptop already has a cable for. */
    ak_usb_init(AK_BOARD_USB_DM, AK_BOARD_USB_DP);
    ak_boot_mark(AK_BOOT_USB);

    build_clock_summary();
}

const char *ak_board_name(void)
{
    return AK_BOARD_STR " / WeAct STM32F405RGT6";
}

/*
 * Quad-X: the core's default, and what this board has always flown. Said out
 * loud rather than left out, because the entry point is the board contract and
 * a board that cannot fly the default has to be able to say so - see the note
 * in ak_board.h, and src/boards/FEATHER_F405/board.c, which is the board that
 * does.
 */
uint32_t ak_board_default_airframe(void)
{
    return 0u;
}

void ak_board_led_set(int on)
{
    led_lit = on ? 1 : 0;
#if AK_BOARD_LED_ACTIVE_LOW
    ak_pin_set(AK_BOARD_LED_PIN, !on);
#else
    ak_pin_set(AK_BOARD_LED_PIN, on);
#endif
}

void ak_board_led_toggle(void)
{
    ak_board_led_set(!led_lit);
}

int ak_board_led_state(void)
{
    return led_lit;
}

/*
 * The tell-tale, in the one build that wants it (`AK_BOOT_STAGE=1`).
 *
 * A flight image does not: its status pin is a heartbeat, and a boot that spent
 * a minute and a half blinking thirteen stage patterns would be a boot nobody
 * would flash twice. The diagnostic image is the same sources compiled with the
 * flag, which is what keeps it from going stale - see docs/05-bringup.md 6a.
 */
#if AK_BOOT_STAGE
static volatile int told_stage;

/*
 * A spin, not ak_delay_ms.
 *
 * Stages 1-4 run before the millisecond tick exists - and a tick that is not
 * moving is one of the two failures this instrument exists to tell apart - so
 * the wait cannot be the firmware's own. It does not have to be exact, since a
 * person is reading the result, but it should not be an order of magnitude out:
 * thirty thousand rounds of a five-cycle loop is about a millisecond at
 * 168 MHz.
 */
static void boot_spin(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 30000u; i++) {
    }
}

static void boot_led(int on)
{
    ak_board_led_set(on);
}

void ak_board_boot_mark(int stage)
{
    /*
     * Static rather than on the stack, because the fault path is one of the two
     * callers and a fault caused by a full stack is exactly the fault a
     * three-hundred-byte frame would turn into a lockup. The diagnostic build
     * has the RAM to spare.
     */
    static ak_boot_beat_t beats[AK_BOOT_BEATS_MAX];
    int count;

    if (stage == AK_BOOT_FAULT) {
        count = ak_boot_fault_plan(told_stage, beats, AK_BOOT_BEATS_MAX);
    } else {
        told_stage = stage;
        count = ak_boot_blink_plan(stage, beats, AK_BOOT_BEATS_MAX);
    }
    if (count > 0) {
        ak_boot_run(beats, count, boot_led, boot_spin);
    }
    /* Left dark: the gap is what separates one stage's pattern from the next
     * one's first blink. */
    ak_board_led_set(0);
}
#else
void ak_board_boot_mark(int stage)
{
    (void)stage;
}
#endif

const char *ak_board_clock_summary(void)
{
    return clock_summary;
}

/* --- saved configuration --------------------------------------------------
 *
 * The last two 128 KB sectors of the part, which linker/stm32f405rg.ld asserts
 * the image never reaches. On a 1 MB F405 those are sectors 10 and 11, at
 * 0x080C0000 and 0x080E0000.
 *
 * **A ring of records rather than one.** Each save goes into the next *erased*
 * slot and carries a serial number; a load takes the newest valid one. The old
 * shape - erase the sector, then program the record - had one copy, so the
 * failure it could not survive was the one that matters: the erase succeeding
 * and the program failing (or the pack sagging, or somebody pulling the cable)
 * left no configuration at all, and the aircraft booted on defaults with
 * nothing but a console line to say so. Now a failed save costs the new
 * settings and keeps the old ones, which is the difference between a save that
 * did not happen and an aircraft whose tuning is gone.
 *
 * The record still carries a magic and a length so that an erased slot - all
 * 0xFF - reads as "nothing saved" rather than as a configuration, and a sum so
 * that a write interrupted half way is rejected instead of applied in part.
 * Both of those are per slot now: a torn record is one invalid slot, and the
 * slot before it is still a configuration.
 *
 * **The erase is the whole sector, so there are two of them.** The F405 erases
 * 16, 64 or 128 KB at once - never one slot - so a full ring has to erase its
 * whole sector before it can write again, and that erase (about a second, with
 * the instructions coming out of the same bank) is the one window a power cut
 * can lose everything in. One sector cannot remove that window: the records
 * that would be the fallback are inside the erase, every time.
 *
 * So the ring alternates between two sectors, and **the erase always lands on
 * the bank the newest record is *not* in** - the one holding the generation
 * before, which is stale by definition because the current bank was filled
 * after it. A power cut inside that erase leaves the current bank untouched
 * with every record it holds, newest included. What it costs is one 128 KB
 * sector of the blackbox: the log is sectors 5 to 9 rather than 5 to 10.
 *
 * The window was not hypothetical. `make config-recycle` drives the real board
 * through it: thirty-two saves, then a recycle whose program the controller
 * refuses, which is a supply that sagged or somebody pulling the cable. With
 * one bank the sector reads all-0xFF afterwards - the same answer a part nobody
 * has ever saved to gives, so `preflight` prints "none stored" and counts no
 * problem while the airframe, the mixer and the tuning are gone. The AT32F435
 * beside this port never had the window: its erase unit is 2 KB and it erases
 * only the slot it is about to write.
 *
 * The erase stalls the CPU for about a second, because the instructions doing
 * the erasing come from the same flash bank. That is fine on a bench with the
 * props off. It is not fine in the air, which is why `help` says so - and why
 * `ak_flight_config_writable()` refuses a save at all while armed.
 */

#define AK_CONFIG_MAGIC  0x414B4346u /* "AKCF" */
/* Two 128 KB sectors, the last two on the part. Bank 1 is the one a part
 * nobody has saved to starts in, so `AK_CONFIG_BASE` is still where a reader
 * should look first and every address a test already knows is unchanged. */
#define AK_CONFIG_BANKS   2u
#define AK_CONFIG_SECTOR  11u
#define AK_CONFIG_BASE    0x080E0000u
#define AK_CONFIG_SECTOR0 10u
#define AK_CONFIG_BASE0   0x080C0000u
/* One slot per 4 KB, so thirty-two saves between erases - and the slot is four
 * times the record, which leaves the record room to grow without a layout
 * change. Per bank: the ring is sixty-four saves long now, thirty-two in each
 * sector, and the two are written alternately rather than as one long run. */
#define AK_CONFIG_SLOT   0x1000u
#define AK_CONFIG_SLOTS  (128u * 1024u / AK_CONFIG_SLOT)

typedef struct {
    uint32_t magic;
    uint32_t serial;   /* which save this is: the newest valid one wins */
    uint32_t length;
    uint32_t sum;
    char     text[AK_PARAMS_TEXT_MAX];
} ak_config_record_t;

_Static_assert(sizeof(ak_config_record_t) <= AK_CONFIG_SLOT,
               "the configuration record does not fit one slot");

static uint32_t text_sum(const char *text, uint32_t length)
{
    uint32_t sum = 0x811C9DC5u;
    for (uint32_t i = 0; i < length; i++) {
        sum = (sum ^ (uint8_t)text[i]) * 16777619u;
    }
    return sum;
}

/* A bank is a sector: bank 0 is the 128 KB below, bank 1 the last one on the
 * part. Every address in this file goes through here, so which bank a record
 * is in is never an arithmetic detail at a call site. */
static uintptr_t config_bank_base(unsigned bank)
{
    return bank == 0u ? AK_CONFIG_BASE0 : AK_CONFIG_BASE;
}

static uint8_t config_bank_sector(unsigned bank)
{
    return (uint8_t)(bank == 0u ? AK_CONFIG_SECTOR0 : AK_CONFIG_SECTOR);
}

static const volatile ak_config_record_t *config_slot(unsigned bank,
                                                      unsigned index)
{
    return (const volatile ak_config_record_t *)
        (config_bank_base(bank) + index * AK_CONFIG_SLOT);
}

static int slot_is_erased(const volatile ak_config_record_t *record)
{
    return record->magic == 0xFFFFFFFFu;
}

static int slot_is_valid(const volatile ak_config_record_t *record)
{
    if (record->magic != AK_CONFIG_MAGIC) {
        return 0;
    }
    if (record->length == 0 || record->length > AK_PARAMS_TEXT_MAX) {
        return 0;
    }
    if (record->sum != text_sum((const char *)record->text, record->length)) {
        return 0;
    }
    return 1;
}

/* The newest valid slot across both banks, if there is one. The serial decides
 * it rather than the bank or the index, because the banks are written
 * alternately and neither address order nor bank number is the order of time.
 * The serial wraps every four billion saves, which is why the comparison is a
 * signed difference rather than `>`: 0 - 0xFFFFFFFF is +1, not a regression to
 * the oldest record. */
static int newest_slot(unsigned *bank, unsigned *index, uint32_t *serial)
{
    int found = 0;

    for (unsigned b = 0; b < AK_CONFIG_BANKS; b++) {
        for (unsigned i = 0; i < AK_CONFIG_SLOTS; i++) {
            const volatile ak_config_record_t *record = config_slot(b, i);

            if (!slot_is_valid(record)) {
                continue;
            }
            if (!found || (int32_t)(record->serial - *serial) > 0) {
                *bank = b;
                *index = i;
                *serial = record->serial;
                found = 1;
            }
        }
    }
    return found;
}

/* Every slot in both banks erased, which is what a part nobody has saved to
 * looks like and what a sector looks like after an erase. */
static int all_erased(void)
{
    for (unsigned b = 0; b < AK_CONFIG_BANKS; b++) {
        for (unsigned i = 0; i < AK_CONFIG_SLOTS; i++) {
            if (!slot_is_erased(config_slot(b, i))) {
                return 0;
            }
        }
    }
    return 1;
}

int ak_board_config_read(void *buf, uint32_t len)
{
    unsigned bank = 0u;
    unsigned index = 0u;
    uint32_t serial = 0u;

    if (!newest_slot(&bank, &index, &serial)) {
        /* Nothing valid, and there are two very different reasons for that: a
         * part nobody has saved to (every slot in both banks erased) and a part
         * whose records are all torn or scribbled over. The core says something
         * different about each. */
        return all_erased() ? 0 : -1;
    }

    const volatile ak_config_record_t *record = config_slot(bank, index);
    if (record->length + 1u > len) {
        return -1; /* the caller's buffer, not the record */
    }

    char *out = buf;
    for (uint32_t i = 0; i < record->length; i++) {
        out[i] = (char)record->text[i];
    }
    out[record->length] = '\0';
    return (int)record->length;
}

int ak_board_config_write(const void *buf, uint32_t len)
{
    static ak_config_record_t record;
    const char *text = buf;

    /* One byte short of the record, not exactly it: a reader is handed the
     * record's length and writes a terminator after it, so a record that fills
     * the array is one no reader with that array can take back. The bound is
     * the same on the ESP32's store and in the simulator, which is why the
     * three cannot disagree about what "saved" means. */
    if (len == 0 || len >= AK_PARAMS_TEXT_MAX) {
        return -1;
    }

    record.magic = AK_CONFIG_MAGIC;
    record.length = len;
    record.sum = text_sum(text, len);
    for (uint32_t i = 0; i < AK_PARAMS_TEXT_MAX; i++) {
        record.text[i] = i < len ? text[i] : 0;
    }

    unsigned bank = 1u;     /* the last sector, where a blank part starts */
    unsigned index = 0u;
    uint32_t serial = 0u;
    int have = newest_slot(&bank, &index, &serial);

    record.serial = have ? serial + 1u : 1u;

    if (have) {
        /* The next erased slot in the bank that holds the newest record. Not
         * simply the one after it: a save whose program was refused leaves a
         * torn slot behind, and stepping over it costs one slot where erasing
         * to get past it would cost every record in the bank. */
        for (index += 1u; index < AK_CONFIG_SLOTS; index++) {
            if (slot_is_erased(config_slot(bank, index))) {
                break;
            }
        }

        if (index >= AK_CONFIG_SLOTS) {
            /* This bank is full, so the record goes into the other one - and
             * the erase lands there, which is the whole point of having two.
             * That bank holds the generation before this one: it was filled
             * earlier and this one was filled after it, so nothing in it is
             * the newest record and erasing it costs no configuration. A power
             * cut inside this erase leaves the bank we just filled intact,
             * newest record included, which is what one sector could not do. */
            bank ^= 1u;
            index = 0u;
            if (ak_flash_erase_sector(config_bank_sector(bank)) != 0) {
                return -1;
            }
        }
    }

    return ak_flash_program(config_bank_base(bank) + index * AK_CONFIG_SLOT,
                            &record, sizeof record);
}

int ak_board_console_poll_rx(char *byte)
{
    /* The USB device is polled here rather than from the portable core, for
     * the same reason it is initialised here: enumeration and the bulk
     * endpoints are hardware, and the core only knows about console bytes.
     * USB is asked first because it is the only door that has to be driven -
     * the UART's byte is already waiting in a register - and either door may
     * be the one somebody is typing at. */
    ak_usb_poll();
    if (ak_usb_read(byte)) {
        return 1;
    }
    return ak_uart_poll_rx(AK_BOARD_CONSOLE_USART, (uint8_t *)byte);
}

/*
 * No network on this board, and that is an answer rather than a stub: the
 * F405 target is the one with a wire to the console, and the ESP32 is the one
 * with a radio. The core asks, is told no, and serves the protocol on the
 * console alone - which is what it has always done.
 */
int ak_board_net_ready(void) { return 0; }
int ak_board_net_connected(void) { return 0; }
int ak_board_net_poll_rx(char *byte) { (void)byte; return 0; }
void ak_board_net_write(const char *data, unsigned len) { (void)data; (void)len; }
void ak_board_net_report(ak_printf_fn out) { out("net:       none on this board\n"); }
void ak_board_net_start(void) {}

/* No parameters of this board's own: everything the F405 has to be told is in
 * the flight core's table or in `set` commands that already cover it. */
unsigned ak_board_param_table(ak_param_t *items, unsigned count)
{
    (void)items;
    return count;
}

/*
 * Retained RAM, for the log that has to survive the reset it is explaining.
 *
 * The section is the one the fault record already uses: it is not cleared by
 * startup, so whatever was there before the reset is still there after it. A
 * log is fifteen kilobytes and the part has a hundred and twenty-eight, so
 * this is cheap; what it is *not* is flash, so it survives a reset and not a
 * battery going flat.
 */
__attribute__((section(".noinit"), used))
static ak_log_t retained_log;

void *ak_board_retained_ram(unsigned *bytes)
{
    *bytes = sizeof retained_log;
    return &retained_log;
}

/* --- the blackbox in flash ------------------------------------------------
 *
 * Sectors 5 to 9 - five 128 KB sectors, 640 KB, from 0x08020000 to 0x080C0000
 * - are the log. Sectors 10 and 11 above them are the saved configuration, its
 * two banks, and sectors below are the image; both of those are enforced rather
 * than hoped for: the linker script asserts that the image ends before
 * 0x08020000, and the static assertion below is that this region stops before
 * the *first* configuration bank. Neither is a comment somebody has to
 * remember.
 *
 * The log gave up one of its six sectors for the configuration's second bank.
 * That is where the space came from and it is worth saying plainly: the log is
 * 640 KB where it was 768 KB, and what it bought is that no power cut can leave
 * the aircraft with no configuration at all. See the section above.
 *
 * The log erases a sector only when the core asks it to, and the core asks
 * only when the aircraft is disarmed: the erase stops the CPU for about a
 * second, so it belongs on the ground, and a log that runs out of erased
 * sectors stops rather than taking that second in the air.
 */

#define AK_LOG_FIRST_SECTOR 5u
#define AK_LOG_SECTOR_BYTES 0x20000u /* sectors 5..9 are all 128 KB */

static const ak_flashlog_region_t log_regions[] = {
    { 0x08020000u, AK_LOG_SECTOR_BYTES },
    { 0x08040000u, AK_LOG_SECTOR_BYTES },
    { 0x08060000u, AK_LOG_SECTOR_BYTES },
    { 0x08080000u, AK_LOG_SECTOR_BYTES },
    { 0x080A0000u, AK_LOG_SECTOR_BYTES },
};

_Static_assert(sizeof log_regions / sizeof log_regions[0] +
                       AK_LOG_FIRST_SECTOR <=
                   AK_CONFIG_SECTOR0,
               "the blackbox region runs into the configuration's first bank");

static uint32_t log_read(uint32_t address)
{
    return AK_REG32(address);
}

static int log_erase(unsigned index)
{
    return ak_flash_erase_sector((uint8_t)(AK_LOG_FIRST_SECTOR + index));
}

static int log_write(uint32_t address, const void *bytes, unsigned len)
{
    return ak_flash_program(address, bytes, len);
}

/* Named fields rather than positional: the store grew a field once (the block
 * read, which this board does not need - its flash is memory, and the word
 * path is a load), and a positional initialiser would have taken a new
 * function pointer for the erase. */
static const ak_flashlog_store_t log_store = {
    .regions = log_regions,
    .count = (unsigned)(sizeof log_regions / sizeof log_regions[0]),
    .read = log_read,
    .read_block = 0, /* memory-mapped: a word is a load, and a span is a loop */
    .erase = log_erase,
    .write = log_write,
};

const ak_flashlog_store_t *ak_board_log_store(void)
{
    return &log_store;
}

void ak_board_reboot(void)
{
    ak_arch_reset();
}

/*
 * And the other way out, which is the one that matters when the firmware is
 * healthy and somebody wants to flash something else: the part's ROM DFU. On
 * this board that is normally a BOOT0 button - it is how the image gets written
 * the first time - and this is the same thing without a hand on the board.
 *
 * Run once, 2026-09-27, and it did not take: the ROM's device never appeared
 * and the application's own came back instead, because the ROM reads BOOT0 at
 * reset and a branch is not a reset. The full reading - the 170 s watch, the
 * 145 s second attach - is in docs/05-bringup.md 6c. BOOT0 plus a reset is
 * what gets to DFU on this part.
 */
int ak_board_enter_bootloader(void)
{
    ak_arch_bootloader();
    return 0; /* not reached: the ROM bootloader is running by the time it is */
}

#if AK_USB_TRACE
/* --- where the trace goes, so that nobody has to count it -----------------
 *
 * The LED report costs a person a steady eye and a stopwatch, and a miscount
 * is indistinguishable from a different number. So the same four numbers are
 * also written down, in flash, where a host reads them back with the tool it
 * already flashes the board with: the ROM bootloader's DFU *upload* reads
 * memory over the same wire the image arrived on.
 *
 * The page is the last 4 KB of the sector the image lives in - 0x0801F000 -
 * and every part of that choice is a boundary rather than a preference:
 *
 *   below it   the image, which is 107 KB today
 *   above it   0x08020000, the first sector of the blackbox log
 *   neither    sectors 10 and 11 are the configuration, and each is a ring
 *              that uses all of its slots; the log sectors are the log's
 *
 * So this is the only flash on the part nothing else claims, and it is claimed
 * without an erase: the page is blank until this writes to it, `dfu-util`
 * erases sector 4 with every image it writes, and a page that is *not* blank
 * is refused rather than programmed over. That refusal is also the collision
 * detector - if the image ever grows past 0x0801F000 the page stops being
 * blank and this returns -1 instead of corrupting the image - which is the
 * honest failure for a page whose address is written down in two places.
 *
 * **A ring rather than one record**, because a boot that writes and then hands
 * the part to the ROM bootloader can be followed by another boot: slots fill
 * from the bottom, a slot is written only while it is still blank, and the
 * slot number *is* the sequence - so the newest record is the valid one in the
 * highest slot, and the host takes the last one it finds. A torn record - a
 * supply that sagged mid-word - is a record whose sum does not match, which
 * the host rejects rather than half a number it believes.
 *
 * All of it is diagnostic, and it is compiled only when the trace is.
 */
#define AK_TRACE_PAGE       0x0801F000u
#define AK_TRACE_PAGE_BYTES 0x1000u
#define AK_TRACE_MAGIC      0x52544B41u /* "AKTR" */
#define AK_TRACE_WORDS      16u
#define AK_TRACE_SLOT_BYTES (AK_TRACE_WORDS * 4u)
#define AK_TRACE_SLOTS      (AK_TRACE_PAGE_BYTES / AK_TRACE_SLOT_BYTES)

/* The record, word by word. Named offsets rather than a struct, because the
 * reader is a program on a host and not this file: what has to be true is the
 * layout, and a struct is a promise about padding that only one compiler
 * makes. */
#define AK_TRACE_W_MAGIC    0u
#define AK_TRACE_W_SLOT     1u /* which slot this is: the newest is the last */
#define AK_TRACE_W_SYSCLK   2u
#define AK_TRACE_W_RESETS   3u
#define AK_TRACE_W_SETUPS   4u
#define AK_TRACE_W_SENDS    5u
/* The second reading, for the question the first one left open: a core that
 * sees resets and is handed no SETUP packets has stopped somewhere between the
 * wire and the stack, and these say where. The registers go in raw - a host
 * neither encodes nor decodes them, it looks them up - and the sum stays the
 * last word so a reader can take it without being told the width. */
#define AK_TRACE_W_PLLCFGR   6u
#define AK_TRACE_W_DSTS      7u
#define AK_TRACE_W_DCTL      8u
#define AK_TRACE_W_GINTSTS   9u
#define AK_TRACE_W_PASSES    10u
#define AK_TRACE_W_RX        11u
#define AK_TRACE_W_PKT_SETUP 12u
#define AK_TRACE_W_PKT_DATA  13u
#define AK_TRACE_W_PKT_OTHER 14u
#define AK_TRACE_W_SUM       15u

static uint32_t trace_sum(const uint32_t *words)
{
    uint32_t sum = 0x811C9DC5u;

    for (unsigned i = 0u; i < AK_TRACE_W_SUM; i++) {
        sum = (sum ^ words[i]) * 16777619u;
    }
    return sum;
}

static void trace_slot_read(unsigned slot, uint32_t *words)
{
    const volatile uint32_t *at =
        (const volatile uint32_t *)(AK_TRACE_PAGE + slot * AK_TRACE_SLOT_BYTES);

    for (unsigned i = 0u; i < AK_TRACE_WORDS; i++) {
        words[i] = at[i];
    }
}

int ak_board_trace_save(void)
{
    uint32_t record[AK_TRACE_WORDS];
    ak_usb_trace_t trace;
    unsigned slot;

    for (slot = 0u; slot < AK_TRACE_SLOTS; slot++) {
        uint32_t words[AK_TRACE_WORDS];
        int blank = 1;

        trace_slot_read(slot, words);
        for (unsigned i = 0u; i < AK_TRACE_WORDS; i++) {
            if (words[i] != 0xFFFFFFFFu) {
                blank = 0;
            }
        }
        if (blank) {
            break; /* where this boot writes */
        }
        if (words[AK_TRACE_W_MAGIC] != AK_TRACE_MAGIC ||
            words[AK_TRACE_W_SUM] != trace_sum(words)) {
            return -1; /* not ours: something else is living in this page */
        }
    }

    if (slot == AK_TRACE_SLOTS) {
        return -1; /* full, and an erase here would take the image with it */
    }

    ak_usb_trace_get(&trace);

    record[AK_TRACE_W_MAGIC]    = AK_TRACE_MAGIC;
    record[AK_TRACE_W_SLOT]     = slot;
    record[AK_TRACE_W_SYSCLK]   = trace.sysclk_hz;
    record[AK_TRACE_W_RESETS]   = trace.resets;
    record[AK_TRACE_W_SETUPS]   = trace.setups;
    record[AK_TRACE_W_SENDS]    = trace.sends;
    record[AK_TRACE_W_PLLCFGR]  = trace.pllcfgr;
    record[AK_TRACE_W_DSTS]     = trace.dsts;
    record[AK_TRACE_W_DCTL]     = trace.dctl;
    record[AK_TRACE_W_GINTSTS]  = trace.gintsts_or;
    record[AK_TRACE_W_PASSES]   = trace.passes;
    record[AK_TRACE_W_RX]       = trace.rx_entries;
    record[AK_TRACE_W_PKT_SETUP] = trace.pktsts_setup;
    record[AK_TRACE_W_PKT_DATA]  = trace.pktsts_data;
    record[AK_TRACE_W_PKT_OTHER] = trace.pktsts_other;
    record[AK_TRACE_W_SUM]      = trace_sum(record);

    /* `record` is on the stack deliberately: programming stalls every read of
     * flash, so the words being written cannot come out of flash. */
    return ak_flash_program(AK_TRACE_PAGE + slot * AK_TRACE_SLOT_BYTES,
                            record, sizeof record);
}
#endif /* AK_USB_TRACE */

uint32_t ak_board_console_port(void)
{
    return AK_BOARD_CONSOLE_USART;
}

uint32_t ak_board_console_attached_port(void)
{
    return ak_console_port();
}

int ak_board_clock_ok(void)
{
    return ak_clk_hse_ok();
}

uint32_t ak_board_clock_sysclk_hz(void)
{
    return ak_clk_sysclk_hz();
}

uint32_t ak_board_clock_apb1_hz(void)
{
    return ak_clk_apb1_hz();
}

uint32_t ak_board_output_dshot_hz(void)
{
    return ak_output_dshot_hz();
}

uint32_t ak_board_output_dshot_period(void)
{
    return ak_output_dshot_period();
}

uint32_t ak_board_output_ccr_zero(void)
{
    return ak_output_ccr_zero();
}

uint32_t ak_board_output_ccr_one(void)
{
    return ak_output_ccr_one();
}

uint32_t ak_board_output_frames_sent(void)
{
    return ak_output_frames_sent();
}

/* The servo outputs this board has, handed to the arch layer rather than
 * reached for by it: TIM2 channels 1 and 2 on PA0 and PA1, AF1.
 *
 * The pads are read from board.h rather than written twice, so the pair
 * `make boards-check` validates for conflicts is the pair the firmware drives -
 * a table that named a third pad would be checked in one place and used in
 * another (traps 204). The channel and the alternate function sit here because
 * neither can collide with anything: they are properties of this timer's pin
 * mapping, and the channels are what `ak_output_write()` addresses.
 *
 * The table is a local rather than a file-scope constant for one reason worth
 * writing down: `AK_PIN` is a compound literal, and a compound literal is not a
 * constant expression, so `AK_BOARD_SERVO1_PIN.port` cannot initialise a static
 * at file scope ("initializer element is not constant"). Inside a function it is
 * an ordinary lvalue and the initialisation is a few stores at init. */
void ak_board_output_init(void)
{
    const ak_servo_out_t servo_outputs[AK_MAX_SERVOS] = {
        {AK_BOARD_SERVO1_PIN.port, AK_BOARD_SERVO1_PIN.pin, 1u, 1u},
        {AK_BOARD_SERVO2_PIN.port, AK_BOARD_SERVO2_PIN.pin, 1u, 2u},
    };

    ak_output_init(AK_BOARD_SERVO_TIMER, servo_outputs, AK_MAX_SERVOS);
}

int ak_board_output_ready(void)
{
    return 1;
}

/* Four timer channels of motors and two of servos are what this port drives:
 * `ak_output_init` sets TIM3 channels 1-4 up as motors and TIM2 channels 1-2 as
 * servos, and the mixer a wing needs is two of each. A quadrotor's mix needs
 * four motors, which this board has - every one of the six is a header pin on a
 * bare dev board, so the timer's count is the board's count. */
void ak_board_output_shape(unsigned *motors, unsigned *servos)
{
    *motors = 4u;
    *servos = 2u;
}

void ak_board_output_write(const ak_output_frame_t *frame)
{
    ak_output_write(frame);
}

void ak_board_output_set_rate(uint32_t khz)
{
    ak_output_set_rate(khz);
}

void ak_board_output_report(ak_printf_fn out)
{
    out("dshot:     %u kHz, one timer period per bit\n",
        ak_output_dshot_hz() / 1000u);
    out("           TIM3 ARR %u ticks, ccr '0' %u, ccr '1' %u\n",
        ak_output_dshot_period(), ak_output_ccr_zero(), ak_output_ccr_one());
    out("           burst: DMA1 stream 4 channel 5 into TIM3_DMAR, %u entries\n",
        (AK_DSHOT_GROUPS - 1u) * AK_MAX_MOTORS);
    out("           frames sent %u, skipped %u, busy %d\n",
        ak_output_frames_sent(), ak_output_frames_skipped(), ak_output_busy());
    out("servos:    50 Hz, 1000-2000 us, 1 us resolution on %s\n",
        ak_output_servo_timer_name());
}

void ak_board_rc_init(void)
{
    ak_uart_rx_init(AK_BOARD_RC_USART, AK_BOARD_RC_TX, AK_BOARD_RC_RX,
                    AK_BOARD_RC_BAUD, AK_BOARD_RC_AF);
}

void ak_board_rc_set_protocol(uint32_t protocol)
{
    if (protocol == 1u) {
        /* SBUS: 100000 baud, even parity, two stop bits. The pin is the same
         * wire as CRSF's; only the format changes. */
        ak_uart_rx_init_format(AK_BOARD_RC_USART, AK_BOARD_RC_TX,
                               AK_BOARD_RC_RX, AK_BOARD_RC_SBUS_BAUD,
                               AK_BOARD_RC_AF, 1, 1);
    } else {
        ak_uart_rx_init(AK_BOARD_RC_USART, AK_BOARD_RC_TX, AK_BOARD_RC_RX,
                        AK_BOARD_RC_BAUD, AK_BOARD_RC_AF);
    }
}

int ak_board_rc_inverted(void)
{
    return AK_BOARD_RC_INVERTER;
}

/* Telemetry out, on the same port the receiver's frames come in on: CRSF is a
 * two-way bus at the same 420000 baud, and PA9 has been sitting there wired
 * for this since the board file was written. SBUS has no return path at all,
 * and the frames are only ever sent when the protocol is CRSF - see main.c. */
int ak_board_rc_send(const char *data, unsigned len)
{
    return ak_uart_write_bytes(AK_BOARD_RC_USART, data, len);
}

int ak_board_rc_poll(uint8_t *byte)
{
    return ak_uart_rx_pop(AK_BOARD_RC_USART, byte);
}

uint32_t ak_board_rc_dropped(void)
{
    return ak_uart_rx_dropped(AK_BOARD_RC_USART);
}

void ak_board_gps_init(void)
{
    ak_uart_rx_init(AK_BOARD_GPS_USART, AK_BOARD_GPS_TX, AK_BOARD_GPS_RX,
                    AK_BOARD_GPS_BAUD, AK_BOARD_GPS_AF);
}

int ak_board_gps_poll(uint8_t *byte)
{
    return ak_uart_rx_pop(AK_BOARD_GPS_USART, byte);
}

uint32_t ak_board_gps_dropped(void)
{
    return ak_uart_rx_dropped(AK_BOARD_GPS_USART);
}

int ak_board_gps_send(const char *data, unsigned len)
{
    return ak_uart_write_bytes(AK_BOARD_GPS_USART, data, len);
}

/* --- the IMU's bus --------------------------------------------------------
 *
 * The drivers ask for a register number and get bytes back; the read flag, the
 * chip select and the SPI port all live here. InvenSense parts use bit 7 of
 * the address byte as the read flag, which is why 0x80 appears exactly once.
 */

static int imu_bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    uint8_t address = (uint8_t)(reg | 0x80u);
    int result;

    ak_pin_set(AK_BOARD_IMU_CS, 0);
    result = ak_spi_transfer(AK_BOARD_IMU_SPI, &address, 0, 1);
    if (result == 0) {
        result = ak_spi_transfer(AK_BOARD_IMU_SPI, 0, buf, len);
    }
    ak_pin_set(AK_BOARD_IMU_CS, 1);
    return result;
}

static int imu_bus_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    uint8_t bytes[2] = { (uint8_t)(reg & 0x7Fu), value };
    int result;

    ak_pin_set(AK_BOARD_IMU_CS, 0);
    result = ak_spi_transfer(AK_BOARD_IMU_SPI, bytes, 0, 2);
    ak_pin_set(AK_BOARD_IMU_CS, 1);
    return result;
}

/*
 * A burst, which is a chip select held low across the whole write.
 *
 * The one caller is the BMI270's configuration upload: eight kilobytes through
 * a single register, which is one frame to the part and would be eight
 * thousand chip-select cycles the other way. The chunks here are only a
 * bound on the buffer - the select never lifts between them - which is the
 * difference between a longer transfer and a different one.
 */
#define AK_IMU_BURST_CHUNK 64u

static int imu_bus_write_burst(void *ctx, uint8_t reg, const uint8_t *buf,
                               unsigned len)
{
    (void)ctx;
    uint8_t first = (uint8_t)(reg & 0x7Fu);
    int result;

    ak_pin_set(AK_BOARD_IMU_CS, 0);
    result = ak_spi_transfer(AK_BOARD_IMU_SPI, &first, 0, 1);
    for (unsigned at = 0; at < len && result == 0; at += AK_IMU_BURST_CHUNK) {
        unsigned chunk = len - at;
        if (chunk > AK_IMU_BURST_CHUNK) {
            chunk = AK_IMU_BURST_CHUNK;
        }
        result = ak_spi_transfer(AK_BOARD_IMU_SPI, &buf[at], 0, chunk);
    }
    ak_pin_set(AK_BOARD_IMU_CS, 1);
    return result;
}

static void imu_bus_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    ak_delay_ms(ms);
}

static const ak_bus_t imu_bus = {
    .read = imu_bus_read,
    .write = imu_bus_write,
    .write_burst = imu_bus_write_burst,
    .delay_ms = imu_bus_delay,
    .ctx = 0,
};

const ak_bus_t *ak_board_imu_bus(void)
{
    return &imu_bus;
}

void ak_board_imu_init(void)
{
    ak_pin_output(AK_BOARD_IMU_CS, 0, GPIO_SPEED_HIGH);
    ak_pin_set(AK_BOARD_IMU_CS, 1); /* idle high: a low select is a frame */
    ak_spi_init(AK_BOARD_IMU_SPI, AK_BOARD_IMU_SCK, AK_BOARD_IMU_MISO,
                AK_BOARD_IMU_MOSI, AK_BOARD_IMU_AF);
}

/* --- the barometer's bus --------------------------------------------------
 *
 * I2C, which is where a barometer lives on the boards that have one. The bus
 * is a plain register read and write to the driver, exactly like the SPI one
 * the IMU uses, so the same driver code runs on either - which is the whole
 * reason ak_bus.h exists.
 *
 * Nothing is fitted to the bench board, so `fitted` decides whether the bus is
 * handed over at all. A probe against a bus with nothing on it is not
 * dangerous - every wait in the driver has a bound and a reset - but it is
 * also not useful, and a board that says "no barometer" is more honest than
 * one that spends a second finding out.
 */

static int baro_bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    return ak_i2c_read_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, reg, buf,
                           len);
}

static int baro_bus_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    return ak_i2c_write_reg(AK_BOARD_BARO_I2C, AK_BOARD_BARO_ADDRESS, reg,
                            value);
}

static void baro_bus_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    ak_delay_ms(ms);
}

static const ak_bus_t baro_bus = {
    .read = baro_bus_read,
    .write = baro_bus_write,
    .delay_ms = baro_bus_delay,
    .ctx = 0,
};

static const ak_bus_t *baro_bus_fitted = 0;

/* Called by the board's own init: the pins and the peripheral are configured
 * whether or not a part is fitted, so that fitting one is one line rather than
 * a bring-up. */
static void baro_bus_init(void)
{
    ak_i2c_init(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);

    if (AK_BOARD_BARO_FITTED) {
        baro_bus_fitted = &baro_bus;
    }
}

const ak_bus_t *ak_board_baro_bus(void)
{
    return baro_bus_fitted;
}

/* --- the rangefinder's bus -------------------------------------------------
 *
 * The barometer's port, a different address, and the same three functions: a
 * driver never learns that the two parts share a pair of wires, which is the
 * point of ak_bus.h. The pins are already configured by baro_bus_init() - it
 * brings the port up whether or not a part is fitted - so there is nothing to
 * initialise here, only an address to answer to.
 */

static int range_bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    return ak_i2c_read_reg(AK_BOARD_RANGE_I2C, AK_BOARD_RANGE_ADDRESS, reg, buf,
                           len);
}

static int range_bus_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    return ak_i2c_write_reg(AK_BOARD_RANGE_I2C, AK_BOARD_RANGE_ADDRESS, reg,
                            value);
}

static const ak_bus_t range_bus = {
    .read = range_bus_read,
    .write = range_bus_write,
    .delay_ms = baro_bus_delay,
    .ctx = 0,
};

const ak_bus_t *ak_board_range_bus(void)
{
    return AK_BOARD_RANGE_FITTED ? &range_bus : 0;
}

void ak_board_spi_loopback(ak_printf_fn out)
{
    static const uint8_t pattern[6] = { 0x55, 0xAA, 0x00, 0xFF, 0x5A, 0xA5 };
    uint8_t read_back[6] = { 0 };

    ak_pin_set(AK_BOARD_IMU_CS, 1); /* nothing to select: this is the wires */
    if (ak_spi_transfer(AK_BOARD_IMU_SPI, pattern, read_back, sizeof pattern) != 0) {
        out("spi:       transfer timed out\n");
        return;
    }

    int same = 1;
    for (unsigned i = 0; i < sizeof pattern; i++) {
        same = same && read_back[i] == pattern[i];
    }

    out("spi:       sent    ");
    for (unsigned i = 0; i < sizeof pattern; i++) {
        out("%02x ", pattern[i]);
    }
    out("\nspi:       read    ");
    for (unsigned i = 0; i < sizeof pattern; i++) {
        out("%02x ", read_back[i]);
    }
    out("\nspi:       %s\n",
        same ? "loopback matches - MOSI reaches MISO"
             : "no match - what a floating MISO does; jumper PB15 to PB14");
}

/* --- the flight pack ------------------------------------------------------
 *
 * Two resistors and an ADC pin, and the whole of this board's part in it is
 * turning counts into volts. The divider's ratio, the cell count and the
 * thresholds are the flight core's, because they are the pilot's to set and
 * the same arithmetic on any board.
 *
 * The reading is taken when it is asked for rather than cached: a conversion
 * is 23 microseconds and there is one of them ten times a second, so a cache
 * would cost more in staleness than the conversion costs in time.
 */

static int vbat_ready;

void ak_board_battery_init(void)
{
    ak_pin_analog(AK_BOARD_VBAT_PIN);
    ak_adc_init(AK_BOARD_VBAT_ADC, AK_BOARD_VBAT_CHANNEL);
    vbat_ready = AK_BOARD_VBAT_FITTED;
}

int ak_board_battery_ready(void)
{
    return vbat_ready;
}

float ak_board_battery_pin_volts(void)
{
    uint16_t counts = 0u;

    if (!vbat_ready || ak_adc_read_counts(AK_BOARD_VBAT_ADC, &counts) != 0) {
        /* Negative, which the core reads as "no reading" and not as a pack
         * that is not there. A conversion that never finishes and an empty
         * pad want different words on the console. */
        return -1.0f;
    }
    return ak_battery_pin_volts(counts, AK_BOARD_VBAT_VREF,
                                AK_BOARD_VBAT_FULL_SCALE);
}

void ak_board_battery_report(ak_printf_fn out)
{
    uint16_t counts = 0u;
    int got = vbat_ready &&
              ak_adc_read_counts(AK_BOARD_VBAT_ADC, &counts) == 0;

    out("adc:       PC0, ADC1 input %u; 10k over 1k against %.2f V, %u counts full scale\n",
        (unsigned)AK_BOARD_VBAT_CHANNEL, (double)AK_BOARD_VBAT_VREF,
        (unsigned)AK_BOARD_VBAT_FULL_SCALE);
    if (got) {
        out("raw:       %u counts = %d.%03d V at the pin\n", (unsigned)counts,
            (int)ak_battery_pin_volts(counts, AK_BOARD_VBAT_VREF,
                                      AK_BOARD_VBAT_FULL_SCALE),
            (int)(ak_battery_pin_volts(counts, AK_BOARD_VBAT_VREF,
                                       AK_BOARD_VBAT_FULL_SCALE) *
                      1000.0f) %
                1000);
    } else {
        out("raw:       no conversion - a timeout, not an empty battery\n");
    }
#if AK_BOARD_VBAT_FITTED
    out("           the divider is fitted: this is the pack, times 11\n");
#else
    out("           no divider is fitted, so the counts above are a pad and not\n"
        "           a pack. Set AK_BOARD_VBAT_FITTED in src/boards/AERIALKIT_F405\n"
        "           /board.h when the 10k and the 1k are soldered in\n");
#endif
}
