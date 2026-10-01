#include "board.h"

/*
 * The Adafruit Feather F405's board file: the glue between this breakout and
 * the core.
 *
 * It is the WeAct board's `board.c` carried across with the pin map changed,
 * and every place the two boards differ is a place a copy from the other one is
 * wrong: the console is USART6 on APB2 rather than USART2 on APB1, the status
 * pin is active *high* on PC1 rather than active low, the IMU is an LSM6DSO on
 * I2C1 rather than a part on SPI2, the servos are TIM4 channels 3-4 rather than
 * TIM2's 1-2, and there is no GPS at all. Above all, the *shape* is 2 motors
 * and 2 servos rather than 4 and 2, because only two of the motor timer's
 * channels reach this header.
 *
 * The crystal is *not* on that list and was on it until 2026-09-29. Both boards
 * are 12 MHz; the 8 MHz this file carried was the WeAct header's assumption,
 * inherited rather than measured, and it is the reason that board never
 * enumerated. See the note in the header and traps 206.
 *
 * **Nothing here has run on a Feather.** It builds (`make BOARD=FEATHER_F405
 * all`), its pins are checked conflict-free (`make boards-check`), and since
 * 2026-09-29 it runs in the host suite: `tests/test_board_feather.c` drives
 * this file's boot sequence and its wiring against the same mapped register
 * page the other two board tests use. That is what found the clock line and the
 * pack's report below, each of which was copied from the WeAct board and named
 * facts this board does not have - and it is the argument for a board file
 * having a test rather than only a compile (traps 204).
 *
 * What is still owed is the flash itself: the first one needs BOOT0 and a reset
 * by hand, and the divider's ratio needs a multimeter rather than a guess. See
 * the handoff's item 1.
 */

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
        /*
         * The crystal as measured at boot (clk.c), or **this board's** declared
         * 12 MHz when the measurement did not fire.
         *
         * This read a number back rather than printing one from 2026-09-29, and
         * it is worth saying why: it used to say "HSE 8 MHz x PLL" - the WeAct
         * header's *assumed* 8 MHz, carried across with the rest of the file -
         * on a board whose own file declares 12 MHz. Neither header said 12 in
         * the sense of having measured one, each thought the other was the
         * 8 MHz board, and each board's test pinned its own file's number. A
         * banner that names a crystal the board does not have is the exact
         * shape both F405 ports have already paid for (traps 203, and the
         * AT32's banner in traps 203's predecessor), and no host test could see
         * this one because this file had no test at all.
         *
         * What it says now is the header's own AK_BOARD_HSE_MHZ, and the
         * sentence this comment used to end on - "so the number on the line is
         * a fact about this board in both directions" - was too much. It is a
         * fact about the header. This board's crystal has never been measured,
         * and the 12 here rests on Adafruit's variant file rather than on an
         * instrument; board.h carries that, and traps 212 corrects the same
         * line on the WeAct without touching this one. The suffix says which
         * branch a reader is looking at, and it is `, assumed` on both F405
         * ports rather than one word each. */
        uint32_t hse = ak_clk_hse_hz();

        put_text(clock_summary, &n, "HSE ");
        put_dec(clock_summary, &n,
                hse != 0u ? (hse + 500000u) / 1000000u : AK_BOARD_HSE_MHZ);
        put_text(clock_summary, &n,
                 hse != 0u ? " MHz x PLL" : " MHz x PLL, assumed");
    } else {
        /* hse_ok is 0 in two cases and this line is right in both: the crystal
         * never came up, or it came up and the PLL it asked for did not lock
         * (clk.c falls back to HSI and clears the flag). */
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
     * once - the pin's reset state is low and this LED is active *high*, so a
     * board that left it there would be correct by accident and a board that
     * had the other polarity would look lit from reset. Turning it off
     * explicitly is what makes the two boards' difference a fact rather than a
     * coin flip.
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
    return AK_BOARD_STR " / Adafruit Feather STM32F405 Express";
}

/*
 * The aircraft this breakout goes into: airframe 7, the single-motor elevon
 * wing, whose mix is one motor and two elevons - and this header carries
 * exactly one motor pair and one elevon pair.
 *
 * The default it replaces is 0, quad-X, which needs four motors this breakout
 * cannot reach: only two of TIM3's four channels come out to pads (see the
 * output block in board.h). So a bare Feather flew the quad-X mix, failed the
 * arming gate's fit check, and reported it - `this mix needs 4 motors and 0
 * servos; the board drives 2 and 2`. That refusal is correct and unclearable at
 * the same time: nothing a person does to a Feather's pads changes the count.
 * The board is the only place that knows the count, so the board answers.
 *
 * Which wing it is, is the owner's answer rather than the header's - the same
 * two pads would fly the twin-motor elevon wing (airframe 1) just as well. See
 * the note on the entry point in ak_board.h, docs/07-outputs.md for the table
 * of what each board answers, and trap 219.
 */
uint32_t ak_board_default_airframe(void)
{
    return 7u;
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
 * Not run yet. If it is ever wrong it is wrong in a way the console will show:
 * the USB device disappears and the host sees the ROM's own instead.
 */
int ak_board_enter_bootloader(void)
{
    ak_arch_bootloader();
    return 0; /* not reached: the ROM bootloader is running by the time it is */
}

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

/* The servo bank, as a board fact. The arch layer's own pads are the WeAct's -
 * TIM2 channels 1 and 2 on PA0 and PA1 - and this breakout brings out neither
 * pin, so a Feather built against the arch's choice drives two pulses into
 * nothing. These are the free timer channels the header does bring out: TIM4
 * CH3 on PB8 and CH4 on PB9, Adafruit pins 9 and 10, both listed in the
 * variant file this port's pin map comes from and both at AF2.
 *
 * The channel numbers matter as much as the pins: TIM4's free pair is 3 and 4,
 * so `ak_output_write()` has to put the compare value in CCR3 and CCR4 rather
 * than CCR1 and CCR2. That is what the channel field is for.
 *
 * The pads themselves are read from board.h rather than written twice, so the
 * pair `make boards-check` validates for conflicts is the pair the firmware
 * drives (traps 204). */
void ak_board_output_init(void)
{
    /* A local, not a file-scope constant: `AK_PIN` is a compound literal and a
     * compound literal is not a constant expression, so reading a pin out of
     * board.h in a static initialiser is "initializer element is not constant".
     * Here it is an ordinary lvalue and this is a few stores at init. */
    const ak_servo_out_t servo_outputs[AK_MAX_SERVOS] = {
        {AK_BOARD_SERVO1_PIN.port, AK_BOARD_SERVO1_PIN.pin, 2u, 3u},
        {AK_BOARD_SERVO2_PIN.port, AK_BOARD_SERVO2_PIN.pin, 2u, 4u},
    };

    ak_output_init(AK_BOARD_SERVO_TIMER, servo_outputs, AK_MAX_SERVOS);
}

int ak_board_output_ready(void)
{
    return 1;
}

/* What can actually be plugged in, which on this breakout is not what the
 * timers drive. The motors are TIM3 channels 1-4 and only PA6 and PA7 reach the
 * header; the servos are TIM4 channels 3-4 on PB8/PB9 and both do. So a
 * quadrotor's four-motor mix is refused at the arming gate - correctly, there
 * are two pads - and a wing's two motors and two elevons is not.
 *
 * This used to say four, which was the timer's number rather than the header's,
 * and it is the difference between an aircraft that will not arm and one that
 * arms with two dead ESCs. */
void ak_board_output_shape(unsigned *motors, unsigned *servos)
{
    *motors = 2u;
    *servos = AK_MAX_SERVOS;
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
    /* Named from the timer the board handed to ak_output_init(), not from a
     * literal: this board's servos are on TIM4 and the WeAct's are on TIM2, and
     * a line that says one while the pulses come out of the other is the banner
     * bug both ports have already paid for (traps 203). */
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

/* No GPS: the two usable UARTs are the receiver's and the console's (board.h),
 * so the contract is answered with no-ops, as on the ESP32-C3 devkit. */
void ak_board_gps_init(void) {}

int ak_board_gps_poll(uint8_t *byte)
{
    (void)byte;
    return 0;
}

uint32_t ak_board_gps_dropped(void) { return 0; }

int ak_board_gps_send(const char *data, unsigned len)
{
    (void)data;
    (void)len;
    return 0;
}

/* --- the IMU's bus --------------------------------------------------------
 *
 * I2C1 to the LSM6DSO on the Qwiic connector (board.h). The driver asks for a
 * register number and gets bytes back; the address and the port live here.
 * No burst: the LSM6DSO has no configuration upload, and a null burst is
 * "one byte at a time" to ak_bus.h.
 */

static int imu_bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    return ak_i2c_read_reg(AK_BOARD_IMU_I2C, AK_BOARD_IMU_ADDRESS, reg, buf,
                           len);
}

static int imu_bus_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    return ak_i2c_write_reg(AK_BOARD_IMU_I2C, AK_BOARD_IMU_ADDRESS, reg, value);
}

static void imu_bus_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    ak_delay_ms(ms);
}

static const ak_bus_t imu_bus = {
    .read = imu_bus_read,
    .write = imu_bus_write,
    .write_burst = 0,
    .delay_ms = imu_bus_delay,
    .ctx = 0,
};

const ak_bus_t *ak_board_imu_bus(void)
{
    return &imu_bus;
}

void ak_board_imu_init(void)
{
    ak_i2c_init(AK_BOARD_IMU_I2C, AK_BOARD_IMU_SCL, AK_BOARD_IMU_SDA,
                AK_BOARD_IMU_AF, AK_BOARD_IMU_SPEED);
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
    out("spi:       this board has no SPI bus - its IMU is on I2C1, PB6/PB7\n");
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

/* The pad's name, built from the header rather than typed again.
 *
 * "PC0" beside AK_BOARD_VBAT_PIN is the drift this file has already paid for
 * twice - the servo pads and the crystal - and here it reads as a contradiction:
 * this board's pin is PA3, which is ADC1 input 3, and the report was printing
 * "PC0, ADC1 input 3", a pad and a channel that cannot both be true. A report
 * that names the wrong pad is worse than one that names none, because the
 * reader goes and puts a meter on it. */
static void pad_name(char *buf, unsigned size, uint32_t port, unsigned pin)
{
    unsigned n = 0;

    buf[n++] = (char)('A' + (unsigned)((port - GPIOA_BASE) / 0x400u));
    put_dec(buf, &n, pin);
    buf[n < size ? n : size - 1u] = '\0';
}

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
    char pad[4];

    pad_name(pad, sizeof pad, AK_BOARD_VBAT_PIN.port, AK_BOARD_VBAT_PIN.pin);

    out("adc:       %s, ADC1 input %u; %.2f V reference, %u counts full scale\n",
        pad, (unsigned)AK_BOARD_VBAT_CHANNEL, (double)AK_BOARD_VBAT_VREF,
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
    out("           the divider is fitted: the pack is this pin voltage times\n"
        "           vbat_ratio, which is this board's own - a multimeter, not a\n"
        "           guess\n");
#else
    /* Not "no divider is fitted", which is the WeAct board's sentence and is
     * false here: this board *has* a divider on the pad Adafruit labels VDIV.
     * What it does not have is a measured ratio, so the honest half is that no
     * reading is taken - see the header, and note that the flag's own name
     * means "there is a number to multiply by" rather than "there are two
     * resistors". */
    out("           no divider reading is taken: %s is labelled VDIV on this\n"
        "           board and the ratio is unmeasured, so the counts above are a\n"
        "           pad and not a pack. Set AK_BOARD_VBAT_FITTED in\n"
        "           src/boards/FEATHER_F405/board.h once it has been measured\n",
        pad);
#endif
}
