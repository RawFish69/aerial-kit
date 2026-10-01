#include "board.h"

#include "ak_board.h"
#include "ak_battery.h"
#include "ak_console.h"
#include "ak_dshot_timing.h"
#include "ak_log.h"
#include "ak_params.h"
#include "ak_time.h"
#include "ak_version.h"

/*
 * The wing's board, as the core sees it.
 *
 * The shape is the F405's - the core asks the same questions and gets the same
 * kinds of answer - and one thing this hardware does not have at all: a
 * network, which is an answer and not a gap, because the ESP32 is the target
 * with a radio.
 *
 * Everything else is real: the console on USART1 **and on USB** (this board is
 * on a USB port whenever it is being flashed, so that is the door a first boot
 * is read through), the clock, the tick, the sensor bus the gyro is on, the
 * barometer's bus (this board has a DPS310 on it, and the port's I2C transfers
 * landed with the board file's own host model), the outputs, the receiver, the
 * GPS, the pack, the configuration record and the blackbox.
 */

static int led_lit;
static char clock_summary[96];

/* Defined with the barometer's bus further down, and wanted by the init above
 * it: the pins are configured at boot whether or not a part is fitted. */
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

/* Built once, at boot, so that a console that comes up before the clock is
 * right says the wrong number rather than no number. */
static void build_clock_summary(void)
{
    unsigned n = 0;

    put_dec(clock_summary, &n, ak_clk_sysclk_hz() / 1000000u);
    put_text(clock_summary, &n, " MHz sysclk (");
    if (ak_clk_hse_ok()) {
        /*
         * The crystal this clock was actually built on, read back rather than
         * spelled out - the line said "HEXT 8 MHz" as a literal while clk.c
         * built the PLL from whatever the board handed it, which is the same
         * shape as the F405's banner naming a crystal that board did not have.
         * Rounding to whole megahertz is the F405's banner's form as well, and
         * every can either board carries is a whole number of them.
         */
        uint32_t hse = ak_clk_hse_hz();

        put_text(clock_summary, &n, "HEXT ");
        put_dec(clock_summary, &n, (hse + 500000u) / 1000000u);
        put_text(clock_summary, &n, " MHz x PLL");
    } else {
        put_text(clock_summary, &n, "HICK, HEXT FAILED");
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
    /* The crystal this board carries, declared in board.h and handed over
     * here - see the note above AK_BOARD_HEXT_HZ for why this part is told
     * what the F405 measures. */
    ak_clk_init(AK_BOARD_HEXT_HZ);

    /* The status pin first, before the buses and the console: it is the
     * instrument the tell-tale build blinks, and a stage that runs before the
     * pin exists is a stage nobody can see. Dark at once - the pin's reset
     * state is low and this LED is active low. */
    ak_pin_output(AK_BOARD_LED_PIN, 0, GPIO_SPEED_LOW);
    ak_board_led_set(0);
    ak_boot_mark(AK_BOOT_CLOCK);

    /* The barometer's pins before anything that could print: a part on that
     * bus is a line on the console, and a bus that is not configured is not
     * something to find out about later. */
    baro_bus_init();
    ak_boot_mark(AK_BOOT_BARO_BUS);

    ak_uart_init(AK_BOARD_CONSOLE_USART, AK_BOARD_CONSOLE_TX,
                 AK_BOARD_CONSOLE_RX, AK_BOARD_CONSOLE_BAUD,
                 AK_BOARD_CONSOLE_AF);
    ak_console_attach(AK_BOARD_CONSOLE_USART);
    ak_boot_mark(AK_BOOT_CONSOLE);

    /* The console's second road, and on this board the one a first boot is
     * most likely to be read through: the cable that flashes it is already on
     * the USB port. Every byte the firmware prints from here on also waits for
     * a host there. */
    ak_usb_init(AK_BOARD_USB_DM, AK_BOARD_USB_DP);
    ak_boot_mark(AK_BOOT_USB);

    build_clock_summary();
}

const char *ak_board_name(void)
{
    return AK_BOARD_STR " / JHEMCU GHF435 AIO V2 (AT32F435)";
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
 * The tell-tale, in the one build that wants it - the same arrangement as the
 * F405's (docs/05-bringup.md 6a), because the two boards have the same failure
 * mode: no console until the USB core is up, and an LED that is the only thing
 * left to read. See src/boards/AERIALKIT_F405/board.c for what the pattern is.
 */
#if AK_BOOT_STAGE
static volatile int told_stage;

static void boot_spin(uint32_t ms)
{
    /* This part runs at 288 MHz rather than 168, so the same round count is
     * a shorter wait - and it is a person reading it, not a stopwatch. */
    for (volatile uint32_t i = 0; i < ms * 50000u; i++) {
    }
}

static void boot_led(int on)
{
    ak_board_led_set(on);
}

void ak_board_boot_mark(int stage)
{
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
 * The last 128 KB of the second bank: 0x080E0000, which linker/at32f435rg.ld
 * asserts the image never reaches (the image is 128 KB at the base of bank 0
 * and this is the top of bank 1).
 *
 * **A ring of records, and on this part each save erases only its own slot.**
 * That is the property the F405 cannot have: this part's erase unit is 2 KB
 * and a slot is four, so the slot a new record goes into is erased on its own
 * while the record that is there now sits untouched two pages away. A failed
 * program, a torn record or a power cut between the two therefore costs the
 * *new* settings and never the old ones - there is no window in which the only
 * copy is inside an erase. (The F405's port explains what it does instead: its
 * erase unit is the whole 128 KB sector, so it erases only when the ring is
 * full, once every thirty-two saves.)
 *
 * The record carries a magic and a length so that an erased slot - all 0xFF -
 * reads as "nothing saved" rather than as a configuration, and a sum so that a
 * write interrupted half way is rejected instead of applied in part. Both are
 * per slot now: a torn record is one invalid slot, and the slot before it is
 * still a configuration.
 *
 * **The erase here is a page, not a sector, and it is not the F405's second.**
 * This part's erase unit is 2 KB and the whole page takes single-digit
 * milliseconds - the F405's 128 KB sector takes about a second, which is why
 * that board's comment explains the stall and this one does not have one to
 * explain. It is still an erase, so it still belongs on the ground.
 */

#define AK_CONFIG_MAGIC 0x414B4346u /* "AKCF" */
#define AK_CONFIG_PAGE  0x080E0000u
/* **Two** pages, and the reason is the record's own size: a magic word, a
 * length, a sum and two thousand and forty-eight bytes of text is 2060 bytes,
 * and this part erases 2 KB at a time. The F405 needs no such arithmetic - its
 * sector is 128 KB and the same record sits in it with room to spare - which is
 * exactly the kind of difference that a config write never notices until
 * something tries one. The check below the record's declaration is what makes
 * this a build failure if the record ever grows past its two pages. */
#define AK_CONFIG_PAGES 2u
/* One slot is one erase unit's worth of pages, and the ring fills the 128 KB
 * region: thirty-two saves before a slot is reused, and every one of them
 * erases only its own slot. */
#define AK_CONFIG_SLOT  (AK_CONFIG_PAGES * AK_FLASH_PAGE_BYTES)
#define AK_CONFIG_SLOTS (128u * 1024u / AK_CONFIG_SLOT)

typedef struct {
    uint32_t magic;
    uint32_t serial;   /* which save this is: the newest valid one wins */
    uint32_t length;
    uint32_t sum;
    char     text[AK_PARAMS_TEXT_MAX];
} ak_config_record_t;

_Static_assert(sizeof(ak_config_record_t) <= AK_CONFIG_SLOT,
               "the configuration record does not fit the pages it is given");

static const volatile ak_config_record_t *config_slot(unsigned index)
{
    return (const volatile ak_config_record_t *)
        (uintptr_t)(AK_CONFIG_PAGE + index * AK_CONFIG_SLOT);
}

static uint32_t text_sum(const char *text, uint32_t length)
{
    uint32_t sum = 0x811C9DC5u;
    for (uint32_t i = 0; i < length; i++) {
        sum = (sum ^ (uint8_t)text[i]) * 16777619u;
    }
    return sum;
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

/* The newest valid slot, if there is one - a signed difference so that the
 * serial wrapping at four billion saves reads as "newer", not "older". */
static int newest_slot(unsigned *index, uint32_t *serial)
{
    int found = 0;

    for (unsigned i = 0; i < AK_CONFIG_SLOTS; i++) {
        const volatile ak_config_record_t *record = config_slot(i);

        if (!slot_is_valid(record)) {
            continue;
        }
        if (!found || (int32_t)(record->serial - *serial) > 0) {
            *index = i;
            *serial = record->serial;
            found = 1;
        }
    }
    return found;
}

int ak_board_config_read(void *buf, uint32_t len)
{
    unsigned index = 0u;
    uint32_t serial = 0u;

    if (!newest_slot(&index, &serial)) {
        /* Nothing valid: "nobody has saved here" and "what was saved is
         * damaged" are different answers for the core to print. */
        for (unsigned i = 0; i < AK_CONFIG_SLOTS; i++) {
            if (!slot_is_erased(config_slot(i))) {
                return -1;
            }
        }
        return 0;
    }

    const volatile ak_config_record_t *record = config_slot(index);
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
     * the array is one no reader with that array can take back. */
    if (len == 0 || len >= AK_PARAMS_TEXT_MAX) {
        return -1;
    }
    record.magic = AK_CONFIG_MAGIC;
    record.length = len;
    record.sum = text_sum(text, len);
    for (uint32_t i = 0; i < AK_PARAMS_TEXT_MAX; i++) {
        record.text[i] = i < len ? text[i] : 0;
    }

    unsigned index = 0u;
    uint32_t serial = 0u;
    int have = newest_slot(&index, &serial);

    record.serial = have ? serial + 1u : 1u;

    /* The slot after the newest one, whether or not it has been used before:
     * this part erases it here and now, and an old record in it is not a copy
     * anybody needs - the newest record is two pages away and stays there
     * through the erase. */
    unsigned target = have ? (index + 1u) % AK_CONFIG_SLOTS : 0u;
    uint32_t base = AK_CONFIG_PAGE + target * AK_CONFIG_SLOT;

    /* Every page the record lives in, erased before it is written: this part's
     * controller refuses a program into a page that was not erased, and a
     * half-erased record is a configuration that loads in part. */
    for (unsigned page = 0; page < AK_CONFIG_PAGES; page++) {
        if (ak_flash_erase_page(base + page * AK_FLASH_PAGE_BYTES) != 0) {
            return -1;
        }
    }
    return ak_flash_program(base, &record, sizeof record);
}

int ak_board_console_poll_rx(char *byte)
{
    /* The USB device is polled here rather than from the portable core, for the
     * same reason it is initialised here: enumeration and the bulk endpoints are
     * hardware, and the core only knows about console bytes. USB is asked first
     * because it is the only door that has to be driven - the UART's byte is
     * already waiting in a register - and either door may be the one somebody
     * is typing at. */
    ak_usb_poll();
    if (ak_usb_read(byte)) {
        return 1;
    }
    return ak_uart_poll_rx(AK_BOARD_CONSOLE_USART, (uint8_t *)byte);
}

/*
 * No network on this board, and that is an answer rather than a stub: the
 * ESP32 is the target with a radio. The core asks, is told no, and serves the
 * protocol on the console alone.
 */
int ak_board_net_ready(void) { return 0; }
int ak_board_net_connected(void) { return 0; }
int ak_board_net_poll_rx(char *byte) { (void)byte; return 0; }
void ak_board_net_write(const char *data, unsigned len) { (void)data; (void)len; }
void ak_board_net_report(ak_printf_fn out) { out("net:       none on this board\n"); }
void ak_board_net_start(void) {}

/* No parameters of this board's own: everything this board has to be told is
 * in the flight core's table. */
unsigned ak_board_param_table(ak_param_t *items, unsigned count)
{
    (void)items;
    return count;
}

/* Retained RAM, for the log that has to survive the reset it is explaining.
 * The section is the one the fault record already uses: startup does not clear
 * it. A hundred and twenty-eight kilobytes is the region the linker gives this
 * part, and the log is fifteen of them. */
__attribute__((section(".noinit"), used))
static ak_log_t retained_log;

void *ak_board_retained_ram(unsigned *bytes)
{
    *bytes = sizeof retained_log;
    return &retained_log;
}

/* --- the blackbox in flash ------------------------------------------------
 *
 * Six 128 KB regions from 0x08020000 to 0x080E0000, which is where the F405's
 * log sits and how big it is - but for a different reason. There are no 128 KB
 * sectors here: this part erases 2 KB pages, so one region is 64 of them, and
 * the region exists because the log is a ring *over* something that can be
 * erased as a unit and whose size divides into whole records.
 *
 * 128 KB is not an arbitrary choice of that size. A slot is 60 bytes and the
 * header 32, so a region holds exactly (131072 - 32) / 60 = 2184 of them, with
 * no tail nobody accounts for - the same arithmetic the F405's sector has, by
 * construction: 128 KB was picked there so that a slot size would divide it.
 * That makes this the one number here that is the same on both parts for a
 * reason rather than by copying.
 *
 * The log erases a region only when the core asks, and the core asks only when
 * the aircraft is disarmed. Sixty-four page erases take longer than the F405's
 * single sector erase in total, so that rule matters more here, not less.
 */

#define AK_LOG_PAGE_BYTES   0x800u  /* 2 KB, this part's erase unit */
#define AK_LOG_REGION_BYTES 0x20000u /* 128 KB: 64 pages, 2184 records */
#define AK_LOG_PAGES_PER_REGION (AK_LOG_REGION_BYTES / AK_LOG_PAGE_BYTES)
#define AK_LOG_BASE         0x08020000u

static const ak_flashlog_region_t log_regions[] = {
    { AK_LOG_BASE + 0x00000u, AK_LOG_REGION_BYTES },
    { AK_LOG_BASE + 0x20000u, AK_LOG_REGION_BYTES },
    { AK_LOG_BASE + 0x40000u, AK_LOG_REGION_BYTES },
    { AK_LOG_BASE + 0x60000u, AK_LOG_REGION_BYTES },
    { AK_LOG_BASE + 0x80000u, AK_LOG_REGION_BYTES },
    { AK_LOG_BASE + 0xA0000u, AK_LOG_REGION_BYTES },
};

/* The regions and the configuration page cannot overlap, and this is checked
 * here rather than trusted: the linker script asserts the image stops before
 * the log, and this asserts the log stops before the record - which is the
 * pair of facts that would otherwise be a comment somebody has to remember. */
_Static_assert(AK_LOG_BASE +
                       sizeof log_regions / sizeof log_regions[0] *
                           AK_LOG_REGION_BYTES <=
                   AK_CONFIG_PAGE,
               "the blackbox region runs into the configuration page");

static uint32_t log_read(uint32_t address)
{
    return AK_REG32(address);
}

static int log_erase(unsigned index)
{
    uint32_t base = AK_LOG_BASE + (uint32_t)index * AK_LOG_REGION_BYTES;

    for (unsigned page = 0; page < AK_LOG_PAGES_PER_REGION; page++) {
        if (ak_flash_erase_page(base + page * AK_LOG_PAGE_BYTES) != 0) {
            return -1;
        }
    }
    return 0;
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
 * The way back in, and on this board it is not a convenience.
 *
 * To put this board in DFU by hand you hold its BOOT button *and* short a
 * solder joint - the board's own drawing says so - and neither has been tried
 * on this one: the firmware it arrived with (Betaflight, then INAV) could jump
 * to the ROM bootloader by command, and that is how it was flashed. AerialKit
 * has the same command, because without it the first successful flash would
 * also be the last one a terminal could perform.
 *
 * See src/arch/at32f435/system.c for the jump. It does not return.
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

void ak_board_output_init(void)
{
    ak_output_init();
}

int ak_board_output_ready(void)
{
    return 1;
}

/*
 * **Two motors and two servos, and that is the whole of this board's answer.**
 *
 * The AIO it is built from is a quadrotor's board - four motor pads, TMR4's two
 * channels and TMR2's other two - and the wing gives up two of those motors so
 * its elevons can have TMR2. So this firmware, on this board, drives a wing's
 * mix and cannot drive a quadrotor's: four motors need four timer channels and
 * TMR2's are servos here. Setting `airframe` to 0 on this board is a build
 * mistake, and the flight core refuses to arm rather than flying a quadrotor's
 * mix on two motors - see `ak_board_output_shape`'s contract in ak_board.h. */
void ak_board_output_shape(unsigned *motors, unsigned *servos)
{
    *motors = 2u;
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

/* What the outputs ended up being, rather than what they were meant to be -
 * the two are the same here only because the timer arithmetic was checked on a
 * host first, and the preflight prints both for the case where they are not. */
void ak_board_output_report(ak_printf_fn out)
{
    out("dshot:     %u kHz, one timer period per bit\n",
        ak_output_dshot_hz() / 1000u);
    out("           TMR4 ARR %u ticks at 288 MHz (no prescaler), ccr '0' %u,\n",
        ak_output_dshot_period(), ak_output_ccr_zero());
    out("           ccr '1' %u, DMA1 channel 1 (TMR4_CH1) and channel 2\n",
        ak_output_ccr_one());
    out("           (TMR4_CH2), one channel per motor through the DMAMUX\n");
    out("           frames sent %u, skipped %u, busy %d\n",
        ak_output_frames_sent(), ak_output_frames_skipped(), ak_output_busy());
    out("servos:    50 Hz, 1000-2000 us, 1 us resolution on TMR2 (PB8/PB9)\n");
}

/* --- the receiver ---------------------------------------------------------
 *
 * The onboard ELRS receiver, on USART2, which is the one fact on this board
 * that was measured rather than read: a bound handset produces CRSF channel
 * data here and produced nothing on USART1 whatever the manufacturer's drawing
 * says. See board.h.
 *
 * The port also carries SBUS, because the core owns the protocol - a parameter
 * - and the board follows it. What the board cannot do is invert the wire, and
 * ak_board_rc_inverted() says so, so the console warns rather than leaving
 * somebody to blame the baud rate.
 */

void ak_board_rc_init(void)
{
    ak_uart_rx_init_af(AK_BOARD_RC_USART, AK_BOARD_RC_TX, AK_BOARD_RC_TX_AF,
                       AK_BOARD_RC_RX, AK_BOARD_RC_RX_AF, AK_BOARD_RC_BAUD, 0, 0,
                       0);
}

void ak_board_rc_set_protocol(uint32_t protocol)
{
    if (protocol == 1u) {
        /* SBUS: 100000 baud, even parity, two stop bits, on the same wire. */
        ak_uart_rx_init_af(AK_BOARD_RC_USART, AK_BOARD_RC_TX,
                           AK_BOARD_RC_TX_AF, AK_BOARD_RC_RX,
                           AK_BOARD_RC_RX_AF, AK_BOARD_RC_SBUS_BAUD, 1, 1, 0);
    } else {
        ak_uart_rx_init_af(AK_BOARD_RC_USART, AK_BOARD_RC_TX, AK_BOARD_RC_TX_AF,
                           AK_BOARD_RC_RX, AK_BOARD_RC_RX_AF, AK_BOARD_RC_BAUD, 0,
                           0, 0);
    }
}

int ak_board_rc_inverted(void)
{
    return AK_BOARD_RC_INVERTER;
}

/* Telemetry out, on the same port the receiver's frames come in on: CRSF is a
 * two-way bus and PA8 has been wired for this since the board file was
 * written. SBUS has no return path at all, and the frames are only sent when
 * the protocol is CRSF - see main.c. */
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

/* --- the GPS ---------------------------------------------------------------
 *
 * USART3, with the peripheral's transmit/receive swap bit set. That bit is not
 * decoration: this board wires its GPS pads to the two pins the *other* way
 * round from the part's default, and the swap is what makes the firmware's
 * transmit land on the pad the GPS is listening to. The evidence is on the
 * board - Betaflight's resource map for it - and the reference target sets it
 * for the same reason.
 */

void ak_board_gps_init(void)
{
    ak_uart_rx_init_af(AK_BOARD_GPS_USART, AK_BOARD_GPS_TX, AK_BOARD_GPS_AF,
                       AK_BOARD_GPS_RX, AK_BOARD_GPS_AF, AK_BOARD_GPS_BAUD, 0, 0,
                       1);
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
 * SPI1, chip select by hand, function 5 on all four pins - and this is the
 * board whose gyro is a fact: an ICM-42688P, detected live, on the same bus the
 * F405 port's SPI was written for. The register protocol above this (the read
 * flag, the chip select, the burst) is the driver's and not the bus's, which is
 * the whole point of ak_bus.h.
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

/* A burst, which is one chip select held low across the whole write. The one
 * caller is the BMI270's configuration upload: eight kilobytes through a
 * single register, which is one frame to the part. This board has an ICM, so
 * the path is here for the driver's sake rather than for this part's. */
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
    return AK_BOARD_IMU_FITTED ? &imu_bus : 0;
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
 * I2C2 on PH2/PH3, 400 kHz, at the address a DPS310 answers to - and it is a
 * real bus now: the transfers landed with this board file, checked on a host
 * against a modelled device (tests/host_i2c_model_at32.c) because this part's
 * I2C does not move bytes the F405's way. A driver above it is unchanged: the
 * barometer in src/core/sensors/ asks for a register and gets bytes, and it has
 * never known which wire answered.
 *
 * `fitted` is a board fact and not a probe, for the same reason the F405's is:
 * the part is on this board, one part number, and a board that answers "no
 * barometer" is more useful than one that spends a second finding out. The
 * difference from the F405 is which answer this board gives.
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

static void baro_bus_init(void)
{
    ak_i2c_init(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);
}

const ak_bus_t *ak_board_baro_bus(void)
{
    return AK_BOARD_BARO_FITTED ? &baro_bus : 0;
}

/* The rangefinder is the same bus and a different address, and nothing is
 * fitted to it: a board may have either part, both or neither, which is what an
 * address is for. */
const ak_bus_t *ak_board_range_bus(void)
{
    return 0;
}

/* The loopback check: a jumper from MOSI to MISO is what turns the pin map
 * into a fact, and it is the same check on both this board and the F405. */
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
             : "no match - what a floating MISO does; jumper PA7 to PA6");
}

/* --- the flight pack ------------------------------------------------------
 *
 * Two resistors and an ADC pin, and the whole of this board's part in it is
 * turning counts into volts: the ratio, the cell count and the thresholds are
 * the flight core's, because they are the pilot's to set and the same
 * arithmetic on any board.
 *
 * Unlike the bench F405, the divider here is *fitted* - this is an AIO with a
 * battery lead and it measures its own pack - but the ratio is not known from
 * here, and it matters: `vbat_ratio` defaults to the F405 board's 10k/1k, so a
 * pack reading on this board is the right shape and possibly the wrong scale
 * until somebody puts a multimeter on it. That is a calibration, not a port.
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
    float volts = ak_battery_pin_volts(counts, AK_BOARD_VBAT_VREF,
                                       AK_BOARD_VBAT_FULL_SCALE);

    out("adc:       PA0, ADC1 input %u; the board's own divider against %.2f V,\n",
        (unsigned)AK_BOARD_VBAT_CHANNEL, (double)AK_BOARD_VBAT_VREF);
    out("           %u counts full scale\n", (unsigned)AK_BOARD_VBAT_FULL_SCALE);
    if (got) {
        out("raw:       %u counts = %d.%03d V at the pin\n", (unsigned)counts,
            (int)volts, (int)(volts * 1000.0f) % 1000);
    } else {
        out("raw:       no conversion - a timeout, not an empty battery\n");
    }
    out("           the divider's ratio is the core's `vbat_ratio` and its\n"
        "           default is the F405 bench board's: calibrate it against a\n"
        "           multimeter before trusting a cell count\n");
}
