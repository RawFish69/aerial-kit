/*
 * Tests for the text layer, the parameter table and the console.
 *
 * The console is the only way a person talks to this firmware, so it gets the
 * same treatment as the control code: every command is exercised, and the
 * failure paths - an out-of-range value, an unknown name, a line too long, a
 * board with no storage - are checked rather than assumed.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ak_board.h"
#include "ak_cli.h"
#include "ak_dshot_timing.h"
#include "ak_flight.h"
#include "ak_params.h"
#include "ak_text.h"
#include "ak_version.h"
#include "tests.h"

/* The subject under test. Declared up here because the fakes below refer to
 * it: the console's parameter-change callback is part of what is being
 * tested. */
static ak_flight_t  flight;
static ak_param_t   items[AK_PARAMS_MAX];
static ak_params_t  params;
static ak_cli_t     cli;

/* Two text parameters, one of them secret, because that is what the ESP32's
 * Wi-Fi credentials are and what has to be shown to be safe: a password that
 * round-trips through save and load without ever being printed. */
static char wifi_ssid[33] = "bench";
static char wifi_pass[64] = "letmein1";

/* --- a capturing console and a fake flash --------------------------------- */

static char   captured[8192];
static size_t captured_len;

static int capture_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(captured + captured_len, sizeof captured - captured_len,
                            fmt, ap);
    va_end(ap);
    if (written > 0 && captured_len + (size_t)written < sizeof captured) {
        captured_len += (size_t)written;
    }
    return written;
}

static void capture_reset(void)
{
    captured_len = 0;
    captured[0] = '\0';
}

static int capture_has(const char *needle)
{
    return strstr(captured, needle) != NULL;
}

static char fake_flash[AK_PARAMS_TEXT_MAX];
static int  fake_flash_valid;
static int  reboot_count;
static uint32_t fake_now;

static int fake_config_read(void *buf, uint32_t len)
{
    if (!fake_flash_valid) {
        return 0;
    }
    size_t n = strlen(fake_flash);
    if (n + 1u > len) {
        return -1;
    }
    memcpy(buf, fake_flash, n);
    ((char *)buf)[n] = '\0';
    return (int)n;
}

static int fake_config_write(const void *buf, uint32_t len)
{
    if (len + 1u > sizeof fake_flash) {
        return -1;
    }
    memcpy(fake_flash, buf, len);
    fake_flash[len] = '\0';
    fake_flash_valid = 1;
    return 0;
}

static void fake_reboot(void)
{
    reboot_count++;
}

/* The ROM bootloader, as a board that has one: the hook is called, and the
 * value it returns is what says whether it could (a board that jumps never
 * comes back here, so 0 is the "could not" answer the command prints about). */
static int bootloader_attempts;
static int bootloader_answers;

static int fake_bootloader(void)
{
    bootloader_attempts++;
    return bootloader_answers;
}

static int output_reports;
static int imu_reports;
static int spi_tests;
static int calibrations;
static int calibration_fails;
static int gps_reports;
static int home_sets;
static int home_clears;
static int preflights;

static int fake_output_report(ak_printf_fn out, int argc, const char *const *argv)
{
    (void)argc;
    (void)argv;
    output_reports++;
    out("fake outputs\n");
    return 0;
}

static void fake_imu_report(ak_printf_fn out)
{
    imu_reports++;
    out("fake imu\n");
}

static void fake_spi_test(ak_printf_fn out)
{
    spi_tests++;
    out("fake bus\n");
}

static int fake_calibrate(ak_printf_fn out, int argc, const char *const *argv)
{
    calibrations++;
    if (calibration_fails) {
        out("fake calibration failed\n");
        return -1;
    }
    out("fake calibration ok");
    for (int i = 1; i < argc; i++) {
        out("%s%s", i == 1 ? " for " : " ", argv[i]);
    }
    out("\n");
    return 0;
}

static void fake_gps_report(ak_printf_fn out)
{
    gps_reports++;
    out("fake gps\n");
}

static int fake_home_set(ak_printf_fn out)
{
    home_sets++;
    out("fake home\n");
    return 0;
}

static void fake_home_clear(void)
{
    home_clears++;
}

static int fake_preflight(ak_printf_fn out)
{
    preflights++;
    out("fake preflight: clean\n");
    return 0;
}

/* What the board does when a parameter moves. */
static void fake_on_change(void)
{
    ak_flight_apply_airframe(&flight);
}

static uint32_t fake_now_ms(void)
{
    return fake_now;
}

static void setup(void)
{
    ak_flight_init(&flight, &ak_mixer_quad_x);
    unsigned count = ak_flight_param_table(&flight, items, AK_PARAMS_MAX);
    count = ak_params_add_text(items, count, "wifi_ssid",
                               "the network to join", wifi_ssid, 32u, 0u,
                               "bench", AK_PARAM_GROUP_NETWORK);
    count = ak_params_add_text(items, count, "wifi_pass",
                               "and its password", wifi_pass, 63u,
                               AK_PARAM_SECRET, "", AK_PARAM_GROUP_NETWORK);
    ak_params_init(&params, items, count);

    ak_cli_io_t io = {
        .out = capture_printf,
        .now_ms = fake_now_ms,
        .config_read = fake_config_read,
        .config_write = fake_config_write,
        .reboot = fake_reboot,
        .bootloader = fake_bootloader,
        .output_report = fake_output_report,
        .imu_report = fake_imu_report,
        .spi_test = fake_spi_test,
        .calibrate = fake_calibrate,
        .gps_report = fake_gps_report,
        .home_set = fake_home_set,
        .home_clear = fake_home_clear,
        .preflight = fake_preflight,
        .on_change = fake_on_change,
    };
    ak_cli_init(&cli, &io, &params, &flight);

    fake_flash_valid = 0;
    fake_now = 1000;
    capture_reset();
}

static void run(const char *line)
{
    capture_reset();
    (void)ak_cli_run(&cli, line);
}

/* --- text ----------------------------------------------------------------- */

static void test_parse(void)
{
    int32_t i = 0;
    uint32_t u = 0;
    float f = 0.0f;

    expect("parse int accepts a plain number", ak_parse_int("123", &i) && i == 123);
    expect("parse int accepts a negative", ak_parse_int("-45", &i) && i == -45);
    expect("parse int rejects trailing junk", !ak_parse_int("12x", &i));
    expect("parse int rejects an empty string", !ak_parse_int("", &i));
    expect("parse int rejects overflow", !ak_parse_int("99999999999", &i));

    expect("parse uint accepts a plain number", ak_parse_uint("60000", &u) && u == 60000u);
    expect("parse uint rejects a negative", !ak_parse_uint("-1", &u));
    expect("parse uint rejects overflow", !ak_parse_uint("4294967296", &u));

    expect("parse float accepts decimals", ak_parse_float("0.25", &f) &&
           f > 0.249f && f < 0.251f);
    expect("parse float accepts an exponent", ak_parse_float("1e-3", &f) &&
           f > 0.0009f && f < 0.0011f);
    expect("parse float requires the whole string", !ak_parse_float("0.25x", &f));
    expect("parse float rejects a bare sign", !ak_parse_float("-", &f));
}

static void test_format(void)
{
    char buf[24];

    ak_format_fixed(0.25f, 3, buf, sizeof buf);
    expect("fixed formatting pads the fraction", strcmp(buf, "0.250") == 0);

    ak_format_fixed(-1.5f, 3, buf, sizeof buf);
    expect("fixed formatting keeps the sign", strcmp(buf, "-1.500") == 0);

    ak_format_fixed(1.999f, 2, buf, sizeof buf);
    expect("fixed formatting rounds", strcmp(buf, "2.00") == 0);

    ak_format_fixed(0.0f, 0, buf, sizeof buf);
    expect("fixed formatting with no decimals", strcmp(buf, "0") == 0);

    ak_format_uint(7u, 4, buf, sizeof buf);
    expect("uint formatting zero pads", strcmp(buf, "0007") == 0);

    /* Seven decimals, because that is how a position is written down: a
     * waypoint printed one decimal short is a waypoint ten metres out, and the
     * formatter used to stop at six. */
    ak_format_fixed(52.1234567f, 7, buf, sizeof buf);
    expect("fixed formatting goes to seven decimals",
           strcmp(buf, "52.1234550") == 0 || strcmp(buf, "52.1234560") == 0 ||
           strcmp(buf, "52.1234567") == 0);
    ak_format_fixed(-4.9876543f, 7, buf, sizeof buf);
    expect("and keeps the sign doing it", buf[0] == '-');

    /* A number too large for the fixed-point accumulator is clamped rather than
     * wrapped. Five million with three decimals is 5e9, which does not fit in
     * the 32 bits the fraction is accumulated in: it used to come back as
     * 705032.704, and a gain that reads back as a different gain is worse than
     * one that reads back as too big. */
    ak_format_fixed(5000000.0f, 3, buf, sizeof buf);
    expect("a number too large to scale is clamped, not wrapped",
           strcmp(buf, "4294967.040") == 0);

    /* A buffer too small must still be terminated, not overrun. */
    char tiny[4];
    ak_format_fixed(123.456f, 3, tiny, sizeof tiny);
    expect("a short buffer is terminated, not overrun", tiny[sizeof tiny - 1] == '\0');
}

/* --- parameters ----------------------------------------------------------- */

/*
 * Text parameters, which is what makes a network name something the aircraft
 * can be told rather than something that was compiled into it. Two of the
 * checks are the reason this is not just "cope with a string": a secret is
 * never *shown* - the console, the dump and the protocol all say "***" - and
 * it still has to come back from the saved configuration, because the aircraft
 * has to remember its own password across a reflash.
 */
static void test_text_params(void)
{
    char msg[64];
    char value[AK_PARAM_VALUE_MAX];

    run("set wifi_ssid bench 2");
    expect("a text parameter takes a value with a space in it",
           strcmp(wifi_ssid, "bench 2") == 0);

    run("get wifi_ssid");
    expect("and reads back as itself",
           capture_has("wifi_ssid = bench 2"));

    char too_long[40];
    for (unsigned i = 0; i < 33u; i++) {
        too_long[i] = 'a';
    }
    too_long[33] = '\0';
    msg[0] = '\0';
    expect("a value longer than the parameter holds is refused",
           ak_params_set(&params, "wifi_ssid", too_long, msg, sizeof msg) != 0 &&
               strcmp(msg, "at most 32 characters") == 0);
    expect("and the value that was there is untouched",
           strcmp(wifi_ssid, "bench 2") == 0);

    msg[0] = '\0';
    expect("a value with a control character is refused",
           ak_params_set(&params, "wifi_ssid", "bench\n2", msg, sizeof msg) !=
               0 &&
               strcmp(wifi_ssid, "bench 2") == 0);

    /* The secret: set it, and see that nothing prints it. */
    run("set wifi_pass hunter2ok");
    expect("a secret parameter is set like any other",
           strcmp(wifi_pass, "hunter2ok") == 0);
    const ak_param_t *pass = ak_params_find(&params, "wifi_pass");
    expect("but reading it back gives the mask",
           pass != 0 &&
               (ak_params_get_text(pass, value, sizeof value), 
                strcmp(value, "***") == 0));

    capture_reset();
    run("get wifi_pass");
    expect("and the console does not print it",
           capture_has("wifi_pass = ***") && !capture_has("hunter2ok"));

    capture_reset();
    run("params");
    expect("nor does a dump of the whole table",
           capture_has("wifi_pass") && !capture_has("hunter2ok"));

    /* And the value itself survives what it exists for: the round trip through
     * the configuration record. */
    char saved[AK_PARAMS_TEXT_MAX];
    unsigned length = ak_params_serialize(&params, saved, sizeof saved);
    expect("the saved record carries the real password",
           length > 0u && strstr(saved, "wifi_pass=hunter2ok") != 0);

    wifi_pass[0] = '\0';
    msg[0] = '\0';
    expect("and loading it puts the password back",
           ak_params_deserialize(&params, saved, msg, sizeof msg) == 0 &&
               strcmp(wifi_pass, "hunter2ok") == 0);

    /* `defaults` puts back what the parameter was registered with, which for a
     * text parameter is the string rather than a number. */
    run("defaults");
    expect("defaults restores the registered text",
           strcmp(wifi_ssid, "bench") == 0 && wifi_pass[0] == '\0');
}

/*
 * The two ways a table goes wrong when it is too big or too small, both of
 * which were true of this firmware on the day the ESP32 grew Wi-Fi
 * parameters: the table was full at 64 and the fourth credential was dropped
 * without a word, and the record was 1024 bytes while the table serialised to
 * 1135 - so `save` wrote three quarters of a configuration and said it had
 * saved one.
 */
static void test_table_limits(void)
{
    /* A table filled to the brim, and one registration past it. The count
     * stops, and the counter says a parameter was lost rather than leaving it
     * to be noticed later - which is how the first one was found. */
    static ak_param_t full[AK_PARAMS_MAX];
    static uint32_t filler;
    unsigned before = ak_params_overflow();
    unsigned n = 0;

    for (unsigned i = 0; i < AK_PARAMS_MAX; i++) {
        n = ak_params_add_u32(full, n, "filler", "filler", &filler, 0u, 1u, AK_PARAM_GROUP_NONE);
    }
    expect("a table can be filled to its limit", n == AK_PARAMS_MAX &&
               ak_params_overflow() == before);

    n = ak_params_add_u32(full, n, "one_too_many", "filler", &filler, 0u, 1u, AK_PARAM_GROUP_NONE);
    expect("one past the limit is refused, and counted",
           n == AK_PARAMS_MAX && ak_params_overflow() == before + 1u);
    expect("and the text registration is refused the same way",
           ak_params_add_text(full, n, "also_too_many", "filler", wifi_ssid,
                              32u, 0u, "", AK_PARAM_GROUP_NONE) == AK_PARAMS_MAX &&
               ak_params_overflow() == before + 2u);

    /* And a name too long to be read back out of a saved record is refused
     * too, and counted separately - it is not an overflow, and the cure is a
     * different one. The name below is exactly one character too long, which
     * is the smallest case that fails: the loader truncates to
     * AK_PARAM_NAME_MAX - 1 before the exact-match lookup. */
    static char too_long[AK_PARAM_NAME_MAX + 1];
    memset(too_long, 'n', sizeof too_long - 1u);
    too_long[sizeof too_long - 1u] = '\0';

    unsigned names_before = ak_params_long_name();
    n = ak_params_add_u32(full, n, too_long, "filler", &filler, 0u, 1u, AK_PARAM_GROUP_NONE);
    expect("a name too long to load back is refused, and counted",
           n == AK_PARAMS_MAX && ak_params_long_name() == names_before + 1u);

    /* The last name that does fit still registers, so the limit is a boundary
     * and not an off-by-one in the other direction. It gets its own table,
     * because the one above is full and a refusal there would be the overflow
     * talking rather than the name. */
    static ak_param_t room[2];
    too_long[AK_PARAM_NAME_MAX - 1u] = '\0';
    expect("and the longest name that fits is taken",
           ak_params_add_u32(room, 0u, too_long, "filler", &filler, 0u, 1u,
                             AK_PARAM_GROUP_NONE) == 1u &&
               ak_params_long_name() == names_before + 1u);
}

/* Every parameter the board registers survives a save and a load.
 *
 * This is not a formality, and it is over the whole table rather than one
 * clever case, because the failure it exists to catch is a name-length miss
 * and only a walk finds those.
 *
 * `ak_params_deserialize` reads a name into a buffer of AK_PARAM_NAME_MAX and
 * looks it up by exact match. A longer name was truncated first, never
 * matched, and was counted as a name from a newer build - so the parameter
 * kept its default through every power cycle while the load returned success
 * and the boot report blamed the record. Eleven of this board's names were
 * over the old limit of 16, `arm_accel_lpf_hz` and `arm_max_tilt_deg` among
 * them.
 *
 * So: move every parameter off its default, save, reset, load, and require all
 * of them back. A parameter that cannot be configured is found here or it is
 * found by an aircraft flying a number nobody chose.
 */
static void test_every_parameter_round_trips(void)
{
    static char record[AK_PARAMS_TEXT_MAX];
    static char wanted[AK_PARAMS_MAX][AK_PARAM_VALUE_MAX];
    static char value[AK_PARAM_VALUE_MAX];
    char message[64];

    unsigned names_before = ak_params_long_name();
    setup();
    expect("no name the board registers is too long to load back",
           ak_params_long_name() == names_before);

    unsigned moved = 0;
    for (unsigned i = 0; i < params.count; i++) {
        ak_param_t *item = &params.items[i];
        char target[AK_PARAM_VALUE_MAX];

        if (item->type == AK_PARAM_TEXT) {
            /* Anything inside the field's own limit that is not what it
             * already holds. */
            unsigned length = item->max_len < 10u ? item->max_len : 10u;
            for (unsigned k = 0; k < length; k++) {
                target[k] = (char)('a' + (int)(i % 26u));
            }
            target[length] = '\0';
        } else if (item->type == AK_PARAM_U32) {
            unsigned long pick = (item->min.u != *(const uint32_t *)item->value)
                                     ? (unsigned long)item->min.u
                                     : (unsigned long)item->max.u;
            snprintf(target, sizeof target, "%lu", pick);
        } else {
            float pick = (item->min.f != *(const float *)item->value)
                             ? item->min.f
                             : item->max.f;
            snprintf(target, sizeof target, "%.*f", (int)item->decimals,
                     (double)pick);
        }

        if (ak_params_set(&params, item->name, target, message,
                          sizeof message) != 0) {
            /* A parameter that will not take a value built from its own
             * declared bound is a different test's subject. Named rather than
             * quietly skipped, so it cannot pass by being absent. */
            printf("      %s refused its own %s: %s\n", item->name, target,
                   message);
            continue;
        }
        ak_params_get_value(item, value, sizeof value);
        snprintf(wanted[i], sizeof wanted[i], "%s", value);
        moved++;
    }
    expect("every parameter took a value other than its default",
           moved == params.count);

    unsigned length = ak_params_serialize(&params, record, sizeof record);
    expect("and the whole table fits the record it is saved in",
           length > 0u && length < sizeof record);

    ak_params_reset(&params);
    expect("the table loads its own record back",
           ak_params_deserialize(&params, record, message, sizeof message) == 0);

    unsigned lost = 0;
    for (unsigned i = 0; i < params.count; i++) {
        ak_params_get_value(&params.items[i], value, sizeof value);
        if (strcmp(value, wanted[i]) != 0) {
            if (lost == 0u) {
                printf("      %s came back as %s, not %s\n",
                       params.items[i].name, value, wanted[i]);
            }
            lost++;
        }
    }
    expect("and every one of them came back", lost == 0u);
}

/* A table that is bigger than the record it is saved in: `save` refuses rather
 * than writing the part that fits. */
static void test_save_refuses_a_table_that_does_not_fit(void)
{
    static ak_param_t big[AK_PARAMS_MAX];
    static char values[AK_PARAMS_MAX][24];
    ak_params_t wide;
    ak_cli_t wide_cli;
    unsigned n = 0;

    for (unsigned i = 0; i < AK_PARAMS_MAX; i++) {
        ak_strlcpy(values[i], "0123456789012345678", sizeof values[i]);
        n = ak_params_add_text(big, n, "long_parameter", "filler", values[i],
                               20u, 0u, "", AK_PARAM_GROUP_NONE);
    }
    ak_params_init(&wide, big, n);

    ak_cli_io_t io = {
        .out = capture_printf,
        .now_ms = fake_now_ms,
        .config_read = fake_config_read,
        .config_write = fake_config_write,
        .reboot = fake_reboot,
        .preflight = fake_preflight,
        .on_change = fake_on_change,
    };
    ak_cli_init(&wide_cli, &io, &wide, &flight);
    fake_flash_valid = 0;
    capture_reset();

    ak_cli_run(&wide_cli, "save");
    expect("a table too big for the record is not saved in part",
           capture_has("does not fit") && !fake_flash_valid);
}

static void test_params(void)
{
    char msg[64];
    /* The table is shared with the console tests below, and the last check in
     * here fills one of its own to the brim. */
    ak_params_t before = params;

    /* The flight core's own, plus the two text parameters this test adds where
     * the board would add its own - the Wi-Fi credentials. */
    expect("the flight core registers its tunables",
           params.count == AK_FLIGHT_PARAM_COUNT + 2u);

    msg[0] = '\0';
    expect("a valid value is accepted",
           ak_params_set(&params, "rate_kp_roll", "0.4", msg, sizeof msg) == 0);
    expect("and it lands in the live config",
           flight.cfg.rate_kp[0] > 0.399f && flight.cfg.rate_kp[0] < 0.401f);

    msg[0] = '\0';
    expect("an out-of-range value is rejected",
           ak_params_set(&params, "rate_kp_roll", "9", msg, sizeof msg) == -2);
    expect("with a message that says the range", strstr(msg, "out of") != NULL);
    expect("and the old value is left alone",
           flight.cfg.rate_kp[0] > 0.399f && flight.cfg.rate_kp[0] < 0.401f);

    msg[0] = '\0';
    expect("a value that is not a number is rejected",
           ak_params_set(&params, "rate_kp_roll", "sideways", msg, sizeof msg) == -2);

    msg[0] = '\0';
    expect("an unknown name is rejected",
           ak_params_set(&params, "rate_kp_roller", "1", msg, sizeof msg) == -1);

    /* Integer parameters take integers, and keep their exact value. */
    msg[0] = '\0';
    expect("a whole-number parameter takes a whole number",
           ak_params_set(&params, "rc_timeout_ms", "300", msg, sizeof msg) == 0 &&
           flight.cfg.rc_timeout_ms == 300u);

    char text[24];
    ak_param_t *item = ak_params_find(&params, "rate_kp_roll");
    ak_params_get_text(item, text, sizeof text);
    expect("a value reads back as text", strcmp(text, "0.400") == 0);

    /* Serialise, change everything, deserialise, and check it comes back. */
    char saved[AK_PARAMS_TEXT_MAX];
    unsigned length = ak_params_serialize(&params, saved, sizeof saved);
    expect("serialising writes something", length > 0 && length < sizeof saved);
    expect("and it is line oriented", strstr(saved, "rate_kp_roll=0.400") != NULL);

    msg[0] = '\0';
    ak_params_set(&params, "rate_kp_roll", "0.1", msg, sizeof msg);
    expect("changing a value moves it", flight.cfg.rate_kp[0] < 0.11f);

    msg[0] = '\0';
    expect("deserialising restores the file",
           ak_params_deserialize(&params, saved, msg, sizeof msg) == 0 &&
           flight.cfg.rate_kp[0] > 0.399f && flight.cfg.rate_kp[0] < 0.401f);

    msg[0] = '\0';
    expect("an unknown name in a file is skipped, not an error",
           ak_params_deserialize(&params, "from_the_future=5\nrate_kp_pitch=0.3\n",
                                 msg, sizeof msg) == 0 &&
           flight.cfg.rate_kp[1] > 0.29f && flight.cfg.rate_kp[1] < 0.31f);

    msg[0] = '\0';
    expect("but a bad value for a known name is an error",
           ak_params_deserialize(&params, "rate_kp_pitch=99\n", msg, sizeof msg) == -1);

    ak_params_reset(&params);
    expect("defaults restores the built-in values",
           flight.cfg.rate_kp[0] > 0.249f && flight.cfg.rate_kp[0] < 0.251f);

    /* The table is a fixed array, and registering past the end of it used to
     * write over whatever followed - which is exactly what happened when a
     * mission's nine parameters arrived. Registering into a full table now
     * stops, and says so by not growing. */
    static ak_param_t full[AK_PARAMS_MAX];
    static float full_value;
    unsigned n = 0;
    for (unsigned i = 0; i < AK_PARAMS_MAX; i++) {
        n = ak_params_add_float(full, n, "filler", "filler", &full_value, 0,
                                0.0f, 1.0f, AK_PARAM_GROUP_NONE);
    }
    expect("the table fills to its limit", n == AK_PARAMS_MAX);
    expect("and registering past the end stops there rather than running on",
           ak_params_add_float(full, n, "one_too_many", "filler", &full_value,
                               0, 0.0f, 1.0f, AK_PARAM_GROUP_NONE) == AK_PARAMS_MAX);

    /* And put the shared table back, because the console tests below use it. */
    params = before;
}

/* --- the console ---------------------------------------------------------- */

static void test_cli_commands(void)
{
    run("help");
    expect("help lists the commands", capture_has("commands:") && capture_has("set <name>"));
    expect("help says what the calibration commands are",
           capture_has("calibrate accel <0-5>") && capture_has("calibrate rc"));
    expect("help says what is still missing, which is the aircraft",
           capture_has("nothing has turned a motor"));

    /* The product this build *is*, not a literal: these tests run for three
     * targets now, and the third one is what found the two places that had the
     * first one's name written into them. */
    char expected[64];
    snprintf(expected, sizeof expected, "product:  %s", AK_PRODUCT_STR);
    run("version");
    expect("version names the product", capture_has(expected));
    /*
     * The board line, and not the product line, is the one that names the
     * hardware: `AK_BOARD_STR` is the image's target, so a Feather flashed with
     * the F405 image answers AERIALKIT_F405 and is believed (trap 209). Both
     * commands now print ak_board_name(), which is the board's own answer.
     */
    expect("version names the board, not just the image's target",
           capture_has(ak_board_name()));

    /*
     * The boot banner is printed before a host can be attached, so it cannot be
     * read off a reset however long the host waits - measured, six resets, a
     * late attach got 0, 0, 0, 16, 0 and 14 bytes, and none of them the head.
     * The command is the way back to it, and it must print the same text.
     */
    run("banner");
    expect("banner prints the product", capture_has(expected));
    expect("banner names the board", capture_has(ak_board_name()));
    expect("banner reports the clocks", capture_has("clocks:"));
    expect("banner reports the revision", capture_has(AK_REV_STR));
    expect("banner says the flight core is in failsafe",
           capture_has("the flight loop runs in failsafe"));

    run("status");
    expect("status reports the state", capture_has("state:     disarmed"));
    expect("status reports the attitude estimate", capture_has("mrad"));
    expect("status reports the parameter count", capture_has("changed since the last save"));

    run("params");
    expect("params lists the table",
           capture_has("rate_kp_roll") && capture_has("max_tilt_deg"));

    run("get rate_kp_roll");
    expect("get prints one value", capture_has("rate_kp_roll = 0.250"));

    run("get nonsense");
    expect("get rejects an unknown name", capture_has("unknown parameter: nonsense"));

    run("set rate_kp_roll 0.5");
    expect("set prints the new value", capture_has("rate_kp_roll = 0.500"));
    expect("set reached the config", flight.cfg.rate_kp[0] > 0.49f);

    run("set rate_kp_roll 42");
    expect("set reports a range error", capture_has("out of"));

    run("set");
    expect("set with no arguments explains itself", capture_has("usage: set"));

    run("wibble");
    expect("an unknown command is reported", capture_has("unknown command: wibble"));

    /* The airframe parameter has to take effect without a reboot. */
    run("set airframe 1");
    expect("changing the airframe changes the mixer",
           flight.mixer == &ak_mixer_elevon_wing);
    run("set airframe 0");
    expect("and back again", flight.mixer == &ak_mixer_quad_x);

    run("defaults");
    expect("defaults is reported", capture_has("built-in values"));

    reboot_count = 0;
    run("reboot");
    expect("reboot calls the board's reset", reboot_count == 1);

    /* And the ROM bootloader, which is the command that decides whether a board
     * can be reflashed from a terminal: the hook is called once, and a board
     * that says it cannot is told so rather than left thinking it worked. */
    bootloader_attempts = 0;
    bootloader_answers = 0;
    run("dfu");
    expect("dfu asks the board for its rom bootloader",
           bootloader_attempts == 1);
    expect("and says so when the board cannot enter it",
           capture_has("cannot enter its rom bootloader"));

    output_reports = 0;
    run("output");
    expect("output asks the board to report", output_reports == 1 &&
           capture_has("fake outputs"));

    imu_reports = 0;
    run("imu");
    expect("imu asks the board to report", imu_reports == 1 &&
           capture_has("fake imu"));

    spi_tests = 0;
    run("spi");
    expect("spi runs the board's bus check", spi_tests == 1 &&
           capture_has("fake bus"));

    calibrations = 0;
    calibration_fails = 0;
    run("calibrate");
    expect("calibrate runs and reports success",
           calibrations == 1 && capture_has("fake calibration ok"));

    calibration_fails = 1;
    run("calibrate");
    expect("a failed calibration is reported as a failure",
           capture_has("fake calibration failed"));

    gps_reports = 0;
    run("gps");
    expect("gps asks the board to report", gps_reports == 1 &&
           capture_has("fake gps"));

    home_sets = 0;
    home_clears = 0;
    run("home");
    expect("home takes the current position", home_sets == 1 &&
           capture_has("fake home"));
    run("home clear");
    expect("and home clear forgets it", home_clears == 1 &&
           capture_has("home forgotten"));

    preflights = 0;
    run("preflight");
    expect("preflight asks the board to check itself", preflights == 1 &&
           capture_has("fake preflight"));
}

static void test_cli_persistence(void)
{
    setup();

    run("load");
    expect("load with nothing saved says so", capture_has("no saved configuration"));

    run("set rate_kp_yaw 0.9");
    run("save");
    expect("save reports the write", capture_has("saved") && fake_flash_valid);

    /* Change it, then load the saved file back. */
    run("set rate_kp_yaw 0.1");
    expect("the value really changed", flight.cfg.rate_kp[2] < 0.11f);
    run("load");
    expect("load restores what was saved",
           flight.cfg.rate_kp[2] > 0.89f && flight.cfg.rate_kp[2] < 0.91f);

    /* And a record written by a build that had a parameter this one does not:
     * the load still works, and the console says what it skipped - which is
     * the whole question after a flash. */
    strcpy(fake_flash, "rate_kp_yaw=0.80\nlong_gone_name=3\n");
    fake_flash_valid = 1;
    capture_reset();
    run("load");
    expect("a load names the parameter the record carried and this build has not",
           capture_has("1 gone (long_gone_name)"));
    expect("and still applies everything else",
           flight.cfg.rate_kp[2] > 0.79f && flight.cfg.rate_kp[2] < 0.81f);

    /* A board with no storage must say so rather than pretend. */
    ak_cli_io_t with_storage = cli.io;
    ak_cli_io_t io = cli.io;
    io.config_write = 0;
    io.config_read = 0;
    ak_cli_init(&cli, &io, &params, &flight);
    run("save");
    expect("save without storage says so", capture_has("no configuration storage"));
    run("load");
    expect("load without storage says so", capture_has("no configuration storage"));

    /* And a board *with* storage will not use it while the aircraft is flying.
     *
     * The state is set here rather than flown into because this test is about
     * the save policy, not about arming - tests/oracle/test_config_policy.c
     * arms a real aircraft through the real hold and measures the same refusal
     * end to end. What this pair adds is the control: the same command, on the
     * same board, with one field different. That is what says the refusal came
     * from the policy rather than from a write that was going to fail anyway. */
    ak_cli_init(&cli, &with_storage, &params, &flight);

    flight.state = AK_FLIGHT_DISARMED;
    fake_flash_valid = 0;
    run("save");
    expect("disarmed, save still writes", fake_flash_valid);

    /* Something to save, so "nothing was marked saved" is a claim with a
     * number behind it rather than one that is true of zero. */
    run("set rate_kp_yaw 0.7");
    unsigned before = params.changed;
    flight.state = AK_FLIGHT_ARMED;
    fake_flash_valid = 0;
    run("save");
    expect("armed, save refuses", !fake_flash_valid);
    expect("and says which state refused it", capture_has("refused while armed"));
    expect("and a refused save does not mark the table saved",
           before != 0u && params.changed == before);

    /* Failsafe and the navigator are flying states too, and the rule is one
     * function rather than a list each route writes for itself - so these are
     * the same refusal, not three. */
    flight.state = AK_FLIGHT_FAILSAFE;
    fake_flash_valid = 0;
    run("save");
    expect("failsafe is not a state to write flash in either", !fake_flash_valid);

    flight.state = AK_FLIGHT_MANAGED;
    fake_flash_valid = 0;
    run("save");
    expect("nor is the navigator's", !fake_flash_valid);

    flight.state = AK_FLIGHT_DISARMED;
}

/*
 * The saved record across a firmware upgrade.
 *
 * A record is name-keyed, so a parameter that this build does not have is
 * skipped and one it has just gained keeps its compiled-in value - both
 * deliberately, and both invisible until something says so. What says so is
 * the load's report: how much of this build's table the record carried, what
 * the record carried that is gone, and what this build has that the record
 * never saw. The interesting half is the *names*, because a count only says
 * that something happened.
 */
static void test_config_across_an_upgrade(void)
{
    static ak_param_t old_items[8];
    static ak_params_t old;
    static float old_roll, old_pitch, old_gone;
    static ak_param_t new_items[8];
    static ak_params_t updated;
    static float new_roll, new_pitch, new_added;
    char record[512];
    char msg[80];
    unsigned n;

    /* The build that saved it: two parameters that will still exist and one
     * that will not. */
    n = ak_params_add_float(old_items, 0, "gain_roll", "x", &old_roll, 3,
                            0.0f, 2.0f, AK_PARAM_GROUP_NONE);
    old_roll = 0.37f;
    n = ak_params_add_float(old_items, n, "gain_pitch", "x", &old_pitch, 3,
                            0.0f, 2.0f, AK_PARAM_GROUP_NONE);
    old_pitch = 0.51f;
    n = ak_params_add_float(old_items, n, "gain_removed", "x", &old_gone, 3,
                            0.0f, 2.0f, AK_PARAM_GROUP_NONE);
    old_gone = 1.25f;
    ak_params_init(&old, old_items, n);

    int length = ak_params_store(&old, record, sizeof record);
    expect("the old build saves a record", length > 0 &&
           strstr(record, "gain_removed") != 0);

    /* The build that loads it: one of them gone, one it has never seen. */
    n = ak_params_add_float(new_items, 0, "gain_roll", "x", &new_roll, 3,
                            0.0f, 2.0f, AK_PARAM_GROUP_NONE);
    new_roll = 0.1f;
    n = ak_params_add_float(new_items, n, "gain_pitch", "x", &new_pitch, 3,
                            0.0f, 2.0f, AK_PARAM_GROUP_NONE);
    new_pitch = 0.1f;
    n = ak_params_add_float(new_items, n, "gain_added", "x", &new_added, 3,
                            0.0f, 2.0f, AK_PARAM_GROUP_NONE);
    new_added = 0.9f;
    ak_params_init(&updated, new_items, n);

    msg[0] = '\0';
    expect("the record loads into a table that is not the one that wrote it",
           ak_params_deserialize(&updated, record, msg, sizeof msg) == 0);

    ak_params_load_report_t report;
    ak_params_load_report(&updated, &report);
    expect("and the values that are still named are the ones in force",
           new_roll > 0.369f && new_roll < 0.371f && new_pitch > 0.509f &&
               new_pitch < 0.511f);
    expect("and the report counts what came, what is gone and what is new",
           report.total == 3u && report.applied == 2u &&
               report.unknown == 1u && report.unmentioned == 1u);
    expect("with the name of the parameter this build does not have",
           strcmp(report.unknown_name, "gain_removed") == 0);
    expect("and the name of the one the record never saw",
           strcmp(report.unmentioned_name, "gain_added") == 0);
    expect("and the new one keeps the value it was compiled with",
           new_added > 0.899f && new_added < 0.901f);

    /* And a record written by *this* build says so plainly: everything
     * applied, nothing gone, nothing new. */
    char fresh[512];
    ak_params_store(&updated, fresh, sizeof fresh);
    ak_params_init(&updated, new_items, n);
    ak_params_deserialize(&updated, fresh, msg, sizeof msg);
    ak_params_load_report(&updated, &report);
    expect("a record from this same build reports no surprises",
           report.applied == 3u && report.unknown == 0u &&
               report.unmentioned == 0u);
}

static void test_cli_line_editing(void)
{
    setup();

    const char *typing = "heXlp\r";
    for (const char *p = typing; *p != '\0'; p++) {
        if (*p == 'X') {
            ak_cli_feed(&cli, 'X');  /* the typo */
            ak_cli_feed(&cli, '\b'); /* and the backspace that fixes it */
        } else {
            ak_cli_feed(&cli, *p);
        }
    }
    expect("backspace edits the line before it runs",
           capture_has("commands:"));

    capture_reset();
    for (int i = 0; i < AK_CLI_LINE_MAX + 10; i++) {
        ak_cli_feed(&cli, 'x');
    }
    ak_cli_feed(&cli, '\r');
    expect("an over-long line is discarded, not half run",
           capture_has("line too long"));

    capture_reset();
    ak_cli_feed(&cli, '\r'); /* an empty line must not run anything */
    expect("an empty line is harmless", !capture_has("unknown command"));
    expect("and the console says it is ready for the next one",
           capture_has("ak> "));

    capture_reset();
    ak_cli_prompt(&cli);
    expect("the prompt is what the boot report ends with",
           strcmp(captured, "ak> ") == 0);
}

void test_text(void)
{
    test_parse();
    test_format();
}

/* --- what a parameter belongs to ------------------------------------------ */

/*
 * The group field exists because a screen used to derive a structure from name
 * prefixes - a claim about the board that the board never made, and one that
 * silently misfiles anything the heuristic did not anticipate, which is most of
 * a ninety-parameter table. Three things have to hold for the field to be an
 * improvement rather than a second guess.
 */
static void test_param_groups(void)
{
    /*
     * First: the names. This list is spelled out here *as the check*, and it is
     * deliberately a second copy of the one in ak_params.c - that is the whole
     * mechanism. The day somebody renumbers the enum, or reorders the name
     * table, or drops a value out of it, this is what notices. It is not a
     * generator: nothing here writes the header, because a header that is only
     * right after someone remembers to run a script is a header that is wrong.
     */
    static const char *const spelled[AK_PARAM_GROUP_COUNT] = {
        [AK_PARAM_GROUP_NONE]       = "",
        [AK_PARAM_GROUP_RATES]      = "rates",
        [AK_PARAM_GROUP_ANGLE]      = "angle",
        [AK_PARAM_GROUP_ARMING]     = "arming",
        [AK_PARAM_GROUP_RECEIVER]   = "receiver",
        [AK_PARAM_GROUP_AIRFRAME]   = "airframe",
        [AK_PARAM_GROUP_OUTPUTS]    = "outputs",
        [AK_PARAM_GROUP_POWER]      = "power",
        [AK_PARAM_GROUP_FAILSAFE]   = "failsafe",
        [AK_PARAM_GROUP_NAVIGATION] = "navigation",
        [AK_PARAM_GROUP_SENSORS]    = "sensors",
        [AK_PARAM_GROUP_NETWORK]    = "network",
        [AK_PARAM_GROUP_TIMING]     = "timing",
    };

    int named = 1;
    char which[64];
    for (unsigned g = 0; g < AK_PARAM_GROUP_COUNT; g++) {
        if (strcmp(ak_param_group_name((uint8_t)g), spelled[g]) != 0) {
            named = 0;
            snprintf(which, sizeof which, "group %u is '%.32s'", g,
                     ak_param_group_name((uint8_t)g));
            expect(which, 0);
        }
    }
    expect("every group is spelled the way this test spells it", named);
    expect("and the count is the table's, not a second number",
           ak_param_group_count() == AK_PARAM_GROUP_COUNT);

    /*
     * A number the firmware does not know is unknown, and says so by naming
     * nothing. Folding it into a neighbour would put a parameter under a
     * heading that describes something else, which is worse than no heading:
     * a blank cell invites a question, a wrong heading does not.
     */
    expect("a group number this build does not know has no name",
           ak_param_group_name(AK_PARAM_GROUP_COUNT)[0] == '\0' &&
           ak_param_group_name(200)[0] == '\0');
    expect("and NONE has none either, so 'no group' is not a group",
           ak_param_group_name(AK_PARAM_GROUP_NONE)[0] == '\0');

    /* Two groups sharing a name would collapse into one heading in any client
     * that keys on the string, which is most of them. */
    int distinct = 1;
    for (unsigned a = 0; a < AK_PARAM_GROUP_COUNT; a++) {
        if (a == AK_PARAM_GROUP_NONE) {
            continue;
        }
        for (unsigned b = a + 1; b < AK_PARAM_GROUP_COUNT; b++) {
            if (b != AK_PARAM_GROUP_NONE &&
                strcmp(ak_param_group_name((uint8_t)a),
                       ak_param_group_name((uint8_t)b)) == 0) {
                distinct = 0;
            }
        }
    }
    expect("no two groups carry the same heading", distinct);

    /*
     * Second, and the reason the field could go in at all: a group is not a
     * value. It is filed under nothing that is written down or hashed.
     *
     * Two tables are built identical in every respect except their groups, and
     * this asserts they serialise to the same bytes and hash to the same
     * number. So a configuration saved by a build before this field existed
     * loads into one built after it, and a `config_hash` taken on the bench
     * today still identifies the same aircraft tomorrow - which is the property
     * the whole saved-record format rests on, since `ak_params_serialize`
     * writes `name=value\n` and never a struct dump.
     */
    static ak_param_t left[4];
    static ak_param_t right[4];
    static float left_f = 0.4f, right_f = 0.4f;
    static uint32_t left_u = 300u, right_u = 300u;
    static char left_t[33] = "bench", right_t[33] = "bench";
    static char left_s[64] = "letmein1", right_s[64] = "letmein1";

    unsigned n = ak_params_add_float(left, 0, "gain_roll", "a gain",
                                     &left_f, 3, 0.0f, 1.0f,
                                     AK_PARAM_GROUP_RATES);
    n = ak_params_add_u32(left, n, "rc_timeout_ms", "a timeout", &left_u,
                          0u, 1000u, AK_PARAM_GROUP_RECEIVER);
    n = ak_params_add_text(left, n, "wifi_ssid", "a network", left_t, 32u, 0u,
                           "bench", AK_PARAM_GROUP_NETWORK);
    n = ak_params_add_text(left, n, "wifi_pass", "a password", left_s, 63u,
                           AK_PARAM_SECRET, "", AK_PARAM_GROUP_NETWORK);

    unsigned m = ak_params_add_float(right, 0, "gain_roll", "a gain",
                                     &right_f, 3, 0.0f, 1.0f,
                                     AK_PARAM_GROUP_NONE);
    m = ak_params_add_u32(right, m, "rc_timeout_ms", "a timeout", &right_u,
                          0u, 1000u, AK_PARAM_GROUP_NAVIGATION);
    m = ak_params_add_text(right, m, "wifi_ssid", "a network", right_t, 32u, 0u,
                           "bench", AK_PARAM_GROUP_TIMING);
    m = ak_params_add_text(right, m, "wifi_pass", "a password", right_s, 63u,
                           AK_PARAM_SECRET, "", AK_PARAM_GROUP_NONE);

    expect("the two tables have the same parameters in them", n == m && n == 4);

    /*
     * And the group really is on the row. Without this the three assertions
     * below would pass just as well if the registration functions threw the
     * argument away, which is the failure mode a test written from the same
     * change as the code is most likely to have.
     */
    expect("a registration keeps the group it was given",
           left[0].group == AK_PARAM_GROUP_RATES &&
           left[1].group == AK_PARAM_GROUP_RECEIVER &&
           left[2].group == AK_PARAM_GROUP_NETWORK &&
           right[0].group == AK_PARAM_GROUP_NONE &&
           right[1].group == AK_PARAM_GROUP_NAVIGATION);
    expect("so the same parameter can sit under two headings and still be one "
           "parameter", left[0].group != right[0].group &&
           left[1].group != right[1].group);

    ak_params_t left_params, right_params;
    ak_params_init(&left_params, left, n);
    ak_params_init(&right_params, right, m);

    char left_text[AK_PARAMS_TEXT_MAX], right_text[AK_PARAMS_TEXT_MAX];
    unsigned left_len  = ak_params_serialize(&left_params, left_text,
                                             sizeof left_text);
    unsigned right_len = ak_params_serialize(&right_params, right_text,
                                             sizeof right_text);

    expect("a group does not reach the saved record",
           left_len == right_len && strcmp(left_text, right_text) == 0);
    expect("nor the configuration's identity",
           ak_params_hash(&left_params) == ak_params_hash(&right_params));

    /* And the record is what it always was: lines of name=value, with the
     * secret's value in it and its name where a dump would put the value. That
     * is asserted here rather than assumed, because the two claims above are
     * only worth anything if this is what serialising does. */
    expect("the record is still lines of name=value",
           strstr(left_text, "gain_roll=0.400\n") != NULL &&
           strstr(left_text, "rc_timeout_ms=300\n") != NULL &&
           strstr(left_text, "wifi_pass=letmein1\n") != NULL);

    /*
     * Third: the table this board actually builds is grouped. Every assertion
     * above would hold on a table of nothing but NONE, which is the shape a
     * field introduced one call site at a time tends to settle into - the
     * compiler is satisfied, because the argument is there, and the client
     * gets the flat list the field was meant to replace.
     *
     * A parameter registered with no group is a decision nobody made, so the
     * count of them is asserted rather than the presence of a few known ones.
     */
    unsigned ungrouped = 0;
    for (unsigned i = 0; i < params.count; i++) {
        if (params.items[i].group == AK_PARAM_GROUP_NONE) {
            ungrouped++;
        }
    }
    expect("every parameter this board registers states a group",
           ungrouped == 0);

    ak_param_t *gain = ak_params_find(&params, "rate_kp_roll");
    ak_param_t *ssid = ak_params_find(&params, "wifi_ssid");
    expect("and the heading is the one the firmware means, not the one the "
           "name suggests",
           gain != NULL && gain->group == AK_PARAM_GROUP_RATES &&
           ssid != NULL && ssid->group == AK_PARAM_GROUP_NETWORK);

    /* The case that made this necessary: `arm_accel_lpf_hz` is an arming
     * threshold, and the prefix-matching screen that used to derive groups had
     * no rule for it at all. It now says so itself. */
    ak_param_t *gate = ak_params_find(&params, "arm_accel_lpf_hz");
    expect("a parameter whose name follows no pattern still has a group",
           gate != NULL && gate->group == AK_PARAM_GROUP_ARMING);
}

void test_params_and_cli(void)
{
    setup();
    test_param_groups();
    setup();
    test_params();
    setup();
    test_text_params();
    setup();
    test_table_limits();
    test_save_refuses_a_table_that_does_not_fit();
    test_every_parameter_round_trips();
    setup();
    test_cli_commands();
    test_cli_persistence();
    test_config_across_an_upgrade();
    test_cli_line_editing();
}

/* --- the DShot bit timing, which is pure arithmetic ---------------------- */

void test_dshot_timing(void)
{
    /* DShot300 on an 84 MHz timer: 280 ticks per bit. The duty cycles are the
     * ones an ESC reads, so they have to be well apart and on the right side of
     * the middle. */
    uint16_t period = 280;
    uint16_t zero = ak_dshot_ccr_zero(period);
    uint16_t one = ak_dshot_ccr_one(period);
    expect("dshot '0' duty is about a third of the bit", zero == 98);
    expect("dshot '1' duty is about two thirds of the bit", one == 196);
    expect("the two duties are far apart", one - zero > period / 4);
    expect("and neither is ambiguous to an ESC",
           zero < period / 2 && one > period / 2);

    uint16_t frames[AK_MAX_MOTORS] = { 0x8001, 0x0000, 0xFFFF, 0xAAAA };
    static uint16_t entries[AK_DSHOT_ENTRIES];
    ak_dshot_fill(entries, frames, zero, one);

    /* Bit 15 first, one entry per motor per bit, motors in channel order. */
    expect("the first entry is motor 1's first bit",
           entries[0] == one && entries[1] == zero && entries[2] == one &&
           entries[3] == one);
    expect("the last data bit is bit 0 of each frame",
           entries[15 * AK_MAX_MOTORS] == one &&      /* 0x8001 bit 0 */
           entries[15 * AK_MAX_MOTORS + 1] == zero && /* 0x0000 */
           entries[15 * AK_MAX_MOTORS + 2] == one &&  /* 0xFFFF */
           entries[15 * AK_MAX_MOTORS + 3] == zero);  /* 0xAAAA */
    /* 0xAAAA is 1010 1010 1010 1010, so bit 15 is a '1' and bit 14 a '0'. */
    expect("alternating frames alternate their bits",
           entries[0 * AK_MAX_MOTORS + 3] == one &&
           entries[1 * AK_MAX_MOTORS + 3] == zero &&
           entries[2 * AK_MAX_MOTORS + 3] == one);

    /* The blank groups hold every output low between frames, which is what
     * stops a frame running into the next one. */
    int gap_clear = 1;
    for (unsigned group = AK_DSHOT_BITS; group < AK_DSHOT_GROUPS; group++) {
        for (unsigned motor = 0; motor < AK_MAX_MOTORS; motor++) {
            gap_clear = gap_clear && entries[group * AK_MAX_MOTORS + motor] == 0;
        }
    }
    expect("the frame is followed by a blank gap", gap_clear);
    expect("a frame is 16 bits plus the gap", AK_DSHOT_GROUPS == 18);

    /*
     * And the same frame as a duration list, which is what a transmitter told
     * "change now" wants - the ESP32's RMT, for one. The check that matters is
     * the one an ESC would make: read the high times back and see the frame.
     */
    ak_dshot_edge_t edges[AK_DSHOT_EDGES];
    uint16_t frame = 0x8001u; /* bit 15 and bit 0 set, so both ends are tested */
    uint16_t ticks = ak_dshot_ticks_per_bit(600u, 100u); /* DShot600, 100 ns */

    expect("a dshot bit is whole ticks of the transmitter's clock",
           ticks == 17u && ak_dshot_ticks_per_bit(300u, 100u) == 33u &&
               ak_dshot_ticks_per_bit(150u, 100u) == 67u);
    expect("and a rate dshot does not have is refused",
           ak_dshot_ticks_per_bit(250u, 100u) == 0u &&
               ak_dshot_ticks_per_bit(600u, 0u) == 0u);

    ak_dshot_edges(frame, ticks, edges);

    /* Every bit is a high and a low that together are one bit time. */
    int pairs_ok = 1;
    for (unsigned bit = 0; bit < AK_DSHOT_BITS; bit++) {
        pairs_ok = pairs_ok && edges[bit * 2u].level == 1u &&
                   edges[bit * 2u + 1u].level == 0u &&
                   (unsigned)edges[bit * 2u].ticks +
                           edges[bit * 2u + 1u].ticks == ticks;
    }
    expect("sixteen bits are thirty-two edges that fill their bit times",
           pairs_ok && edges[AK_DSHOT_BITS * 2u].level == 0u);
    expect("and the frame is followed by two bit times of low",
           edges[AK_DSHOT_BITS * 2u].ticks == ticks * AK_DSHOT_GAP_GROUPS);

    /* Reading the bits back the way an ESC does: a long high is a one. */
    uint16_t decoded = 0u;
    for (unsigned bit = 0; bit < AK_DSHOT_BITS; bit++) {
        uint16_t high = edges[bit * 2u].ticks;
        if (high > ticks / 2u) {
            decoded |= (uint16_t)(0x8000u >> bit);
        }
    }
    expect("the durations decode back to the frame that went in",
           decoded == frame);

    /* And the two encoders agree about *which* bits are ones, which is the
     * cross-check that would catch one of them drifting: the compare-value
     * path for a timer, this path for a duration-based one, same frame. */
    uint16_t more_frames[AK_MAX_MOTORS] = { frame, 0u, 0xFFFFu, 0xAAAAu };
    uint16_t entries2[AK_DSHOT_ENTRIES];
    uint16_t zero2 = ak_dshot_ccr_zero(1000u);
    uint16_t one2 = ak_dshot_ccr_one(1000u);
    ak_dshot_fill(entries2, more_frames, zero2, one2);

    int agree = 1;
    for (unsigned motor = 0; motor < AK_MAX_MOTORS; motor++) {
        ak_dshot_edges(more_frames[motor], ticks, edges);
        for (unsigned bit = 0; bit < AK_DSHOT_BITS; bit++) {
            int timer_bit = entries2[bit * AK_MAX_MOTORS + motor] == one2;
            int edge_bit = edges[bit * 2u].ticks > ticks / 2u;
            agree = agree && timer_bit == edge_bit;
        }
    }
    expect("both encoders send the same bits, for every motor", agree);

    /* A transmitter with no ticks to give sends nothing rather than a wrong
     * frame: every item zero, and no high anywhere. */
    ak_dshot_edges(frame, 0u, edges);
    int silent = 1;
    for (unsigned i = 0; i < AK_DSHOT_EDGES; i++) {
        silent = silent && edges[i].level == 0u && edges[i].ticks == 0u;
    }
    expect("no ticks per bit is silence, not a malformed frame", silent);
}
