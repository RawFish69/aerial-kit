#include "ak_params.h"

#include "ak_text.h"

/* Registrations the table was too small to take. See ak_params.h: this is
 * counted because a dropped parameter is otherwise indistinguishable from one
 * that was never added. */
static unsigned params_overflow;

unsigned ak_params_overflow(void)
{
    return params_overflow;
}

/* Registrations whose name is longer than AK_PARAM_NAME_MAX can hold.
 *
 * Counted for the same reason as the overflow above and one more: this failure
 * does not look like a failure. The parameter registers, reads, sets and saves
 * perfectly - it is only the *load* that cannot match it, and a load that
 * cannot match a name treats it as a name from a newer build and skips it. So
 * the aircraft flies the default, the boot report blames the record, and
 * nothing anywhere says the two are the same problem. */
static unsigned params_long_name;

unsigned ak_params_long_name(void)
{
    return params_long_name;
}

/* Whether a name can be registered at all: it has to fit the buffer the saved
 * record is read back through. One predicate, used by all three registration
 * functions, so the three cannot come to disagree about what fits. */
static int name_fits(const char *name)
{
    return ak_strlen(name) < (unsigned)AK_PARAM_NAME_MAX;
}

/*
 * The one place the group names are spelled. Indexed by ak_param_group_t, so
 * the table and the enumeration cannot disagree about which number is which -
 * a name added in the wrong place here renames a group, and `test_params_cli`
 * asserts the two orders match.
 *
 * "" for anything out of range rather than a fallback to a neighbour: a client
 * that shows "rates" for a group number it does not know has been told
 * something false, and one that shows "" has been told nothing, which is the
 * better of the two.
 */
static const char *const group_names[AK_PARAM_GROUP_COUNT] = {
    [AK_PARAM_GROUP_NONE] = "",
    [AK_PARAM_GROUP_RATES] = "rates",
    [AK_PARAM_GROUP_ANGLE] = "angle",
    [AK_PARAM_GROUP_ARMING] = "arming",
    [AK_PARAM_GROUP_RECEIVER] = "receiver",
    [AK_PARAM_GROUP_AIRFRAME] = "airframe",
    [AK_PARAM_GROUP_OUTPUTS] = "outputs",
    [AK_PARAM_GROUP_POWER] = "power",
    [AK_PARAM_GROUP_FAILSAFE] = "failsafe",
    [AK_PARAM_GROUP_NAVIGATION] = "navigation",
    [AK_PARAM_GROUP_SENSORS] = "sensors",
    [AK_PARAM_GROUP_NETWORK] = "network",
    [AK_PARAM_GROUP_TIMING] = "timing",
};

const char *ak_param_group_name(uint8_t group)
{
    if (group >= AK_PARAM_GROUP_COUNT) {
        return "";
    }
    return group_names[group];
}

/* How many group names there are, for the protocol's group table and for a
 * check that wants to walk them. */
unsigned ak_param_group_count(void)
{
    return AK_PARAM_GROUP_COUNT;
}

void ak_params_init(ak_params_t *params, ak_param_t *items, unsigned count)
{
    params->items = items;
    /* Clamped rather than trusted: a caller that registered more than the table
     * holds has a bug, and reading past the end of it is not how to find out. */
    params->count = count < AK_PARAMS_MAX ? count : AK_PARAMS_MAX;
    params->changed = 0;
    params->load_applied = 0;
    params->load_unknown = 0;
    params->load_renamed = 0;
    params->load_unmentioned = params->count;
    params->load_unknown_name[0] = '\0';
    params->load_renamed_name[0] = '\0';
    params->load_unmentioned_name[0] = '\0';
    params->load_have_report = 0;
}

/*
 * The table is a fixed array, and these are its only writers. A registration
 * past the end used to write over whatever followed it in memory - which is
 * how a mission's nine new parameters silently corrupted the table's own
 * descriptor, and the console crashed an hour later doing something unrelated.
 * Now it stops, and the count stalls at the limit where a `params` dump shows
 * the table is short.
 */
unsigned ak_params_add_float(ak_param_t *items, unsigned n, const char *name,
                             const char *help, float *value, uint8_t decimals,
                             float low, float high, uint8_t group)
{
    /* Refused rather than accepted, because a name this long can be set and
     * saved and never loaded back - the parameter would look present and
     * behave as though it had never been configured. Refusing makes it absent,
     * which is the failure this table already knows how to report. */
    if (!name_fits(name)) {
        params_long_name++;
        return n;
    }
    if (n >= AK_PARAMS_MAX) {
        params_overflow++;
        return n;
    }
    items[n].name = name;
    items[n].help = help;
    items[n].type = AK_PARAM_FLOAT;
    items[n].group = group;
    items[n].decimals = decimals;
    items[n].value = value;
    items[n].def.f = *value; /* the value at registration is the default */
    items[n].min.f = low;
    items[n].max.f = high;
    items[n].max_len = 0u;
    items[n].flags = 0u;
    items[n].def_text = 0;
    return n + 1u;
}

unsigned ak_params_add_u32(ak_param_t *items, unsigned n, const char *name,
                           const char *help, uint32_t *value, uint32_t low,
                           uint32_t high, uint8_t group)
{
    /* Refused rather than accepted, because a name this long can be set and
     * saved and never loaded back - the parameter would look present and
     * behave as though it had never been configured. Refusing makes it absent,
     * which is the failure this table already knows how to report. */
    if (!name_fits(name)) {
        params_long_name++;
        return n;
    }
    if (n >= AK_PARAMS_MAX) {
        params_overflow++;
        return n;
    }
    items[n].name = name;
    items[n].help = help;
    items[n].type = AK_PARAM_U32;
    items[n].group = group;
    items[n].decimals = 0;
    items[n].value = value;
    items[n].def.u = *value;
    items[n].min.u = low;
    items[n].max.u = high;
    items[n].max_len = 0u;
    items[n].flags = 0u;
    items[n].def_text = 0;
    return n + 1u;
}

unsigned ak_params_add_text(ak_param_t *items, unsigned n, const char *name,
                            const char *help, char *value, unsigned max_len,
                            uint8_t flags, const char *default_text,
                            uint8_t group)
{
    /* Refused rather than accepted, because a name this long can be set and
     * saved and never loaded back - the parameter would look present and
     * behave as though it had never been configured. Refusing makes it absent,
     * which is the failure this table already knows how to report. */
    if (!name_fits(name)) {
        params_long_name++;
        return n;
    }
    if (n >= AK_PARAMS_MAX) {
        params_overflow++;
        return n;
    }
    if (max_len + 1u > AK_PARAM_VALUE_MAX) {
        max_len = AK_PARAM_VALUE_MAX - 1u;
    }
    items[n].name = name;
    items[n].help = help;
    items[n].type = AK_PARAM_TEXT;
    items[n].group = group;
    items[n].decimals = 0;
    items[n].value = value;
    items[n].def.u = 0u;
    items[n].min.u = 0u;
    items[n].max.u = 0u;
    items[n].max_len = (uint8_t)max_len;
    items[n].flags = flags;
    items[n].def_text = default_text != 0 ? default_text : "";
    return n + 1u;
}

ak_param_t *ak_params_find(ak_params_t *params, const char *name)
{
    for (unsigned i = 0; i < params->count; i++) {
        if (ak_str_eq(params->items[i].name, name)) {
            return &params->items[i];
        }
    }
    return 0;
}

static int in_range(const ak_param_t *item, ak_param_value_t value)
{
    if (item->type == AK_PARAM_U32) {
        return value.u >= item->min.u && value.u <= item->max.u;
    }
    return value.f >= item->min.f && value.f <= item->max.f;
}

/*
 * A text value, and the two things it may not contain. A newline would break
 * the "name=value" line the configuration is saved as, and a control character
 * has no business in a network name - it would be there by accident and would
 * then be invisible in every dump that shows it.
 */
static int text_ok(const char *text)
{
    for (const char *p = text; *p != '\0'; p++) {
        if ((unsigned char)*p < 0x20u || (unsigned char)*p > 0x7Eu) {
            return 0;
        }
    }
    return 1;
}

static void set_message(char *msg, unsigned msg_len, const char *first,
                        const char *second)
{
    if (msg == 0 || msg_len == 0) {
        return;
    }
    ak_strlcpy(msg, first, msg_len);
    if (second != 0 && *second != '\0') {
        unsigned used = ak_strlen(msg);
        if (used + 2u < msg_len) {
            msg[used] = ' ';
            ak_strlcpy(&msg[used + 1u], second, msg_len - used - 1u);
        }
    }
}

/* The one place a bound is spelled. `ak_params_format_value` is the same call
 * the wire makes, so a range a configurator shows and a range this message
 * prints are the same string by construction rather than by both being right -
 * which is the difference between one authority and two that agree today. */
static void describe_range(const ak_param_t *item, char *buf, unsigned len)
{
    ak_strlcpy(buf, "range ", len);
    unsigned used = ak_strlen(buf);

    used += ak_params_format_value(item, item->min, &buf[used], len - used);
    if (used + 2u < len) {
        buf[used] = '.';
        buf[used + 1u] = '.';
        used += 2u;
        used += ak_params_format_value(item, item->max, &buf[used], len - used);
    }
}

int ak_params_set(ak_params_t *params, const char *name, const char *text,
                  char *msg, unsigned msg_len)
{
    ak_param_t *item = ak_params_find(params, name);
    if (item == 0) {
        set_message(msg, msg_len, "unknown parameter:", name);
        return -1;
    }

    if (item->type == AK_PARAM_TEXT) {
        if (!text_ok(text)) {
            set_message(msg, msg_len, "not printable text:", "control characters");
            return -2;
        }
        if (ak_strlen(text) > item->max_len) {
            char range[40];
            ak_strlcpy(range, "at most ", sizeof range);
            unsigned used = ak_strlen(range);
            used += ak_format_uint(item->max_len, 0, &range[used],
                                   sizeof range - used);
            set_message(msg, msg_len, range, "characters");
            return -2;
        }
        ak_strlcpy((char *)item->value, text, (unsigned)item->max_len + 1u);
        params->changed++;
        return 0;
    }

    ak_param_value_t value;
    if (item->type == AK_PARAM_U32) {
        if (!ak_parse_uint(text, &value.u)) {
            set_message(msg, msg_len, "not a whole number:", text);
            return -2;
        }
    } else {
        if (!ak_parse_float(text, &value.f)) {
            set_message(msg, msg_len, "not a number:", text);
            return -2;
        }
    }

    if (!in_range(item, value)) {
        char range[40];
        describe_range(item, range, sizeof range);
        set_message(msg, msg_len, "out of", range);
        return -2;
    }

    if (item->type == AK_PARAM_U32) {
        *(uint32_t *)item->value = value.u;
    } else {
        *(float *)item->value = value.f;
    }
    params->changed++;
    return 0;
}

unsigned ak_params_format_value(const ak_param_t *item, ak_param_value_t value,
                                char *buf, unsigned len)
{
    /* A zero-length buffer is a caller saying there is no room for a
     * terminator, and the answer is to write nothing at all. This used to
     * write `buf[0]` anyway, which is one byte into a region of size zero by
     * the caller's own account - and it is the only one of the three functions
     * here that did: `ak_params_get_value` and `ak_params_get_default_text`
     * both return without touching the buffer, and `ak_format_fixed` does the
     * same one level down. No caller can reach this today, because all three
     * sites measure their own room before calling, so the cost so far is a
     * `-Wstringop-overflow` on the call in `describe_range` and a trap for the
     * next caller of a function this header publishes. ASan is not the
     * instrument for it: a caller that passes zero still owns the bytes at that
     * address, so the write is only out of bounds by the caller's declaration
     * rather than by the allocation. The test is a canary byte. */
    if (len == 0u) {
        return 0u;
    }
    if (item->type == AK_PARAM_U32) {
        ak_format_uint(value.u, 0, buf, len);
    } else {
        ak_format_fixed(value.f, item->decimals, buf, len);
    }
    return ak_strlen(buf);
}

unsigned ak_params_get_value(const ak_param_t *item, char *buf, unsigned len)
{
    if (len == 0u) {
        return 0u;
    }
    if (item->type == AK_PARAM_TEXT) {
        ak_strlcpy(buf, (const char *)item->value, len);
        return ak_strlen(buf);
    }

    /* The live value, read out of the field the parameter points at, and then
     * spelled by the one function that spells anything. This used to branch
     * here as well, which is how there came to be two format calls for one
     * concept. */
    ak_param_value_t value;
    if (item->type == AK_PARAM_U32) {
        value.u = *(const uint32_t *)item->value;
    } else {
        value.f = *(const float *)item->value;
    }
    return ak_params_format_value(item, value, buf, len);
}

void ak_params_get_default_text(const ak_param_t *item, char *buf, unsigned len)
{
    if (len == 0u) {
        return;
    }
    if ((item->flags & AK_PARAM_SECRET) != 0u) {
        ak_strlcpy(buf, "***", len);
        return;
    }
    if (item->type == AK_PARAM_TEXT) {
        ak_strlcpy(buf, item->def_text, len);
        return;
    }
    (void)ak_params_format_value(item, item->def, buf, len);
}

void ak_params_get_text(const ak_param_t *item, char *buf, unsigned len)
{
    if (item->type == AK_PARAM_TEXT && (item->flags & AK_PARAM_SECRET) != 0u) {
        ak_strlcpy(buf, "***", len);
        return;
    }
    (void)ak_params_get_value(item, buf, len);
}

void ak_params_dump(ak_params_t *params, ak_printf_fn out)
{
    char value[AK_PARAM_VALUE_MAX];

    out("params (%u):\n", params->count);
    for (unsigned i = 0; i < params->count; i++) {
        const ak_param_t *item = &params->items[i];
        ak_params_get_text(item, value, sizeof value);
        out("  %-16s %10s  %s\n", item->name, value, item->help);
    }
    if (params->changed > 0) {
        out("%u changed since the last save\n", params->changed);
    }
}

void ak_params_default_one(ak_params_t *params, unsigned index)
{
    if (index >= params->count) {
        return;
    }
    ak_param_t *item = &params->items[index];
    if (item->type == AK_PARAM_TEXT) {
        ak_strlcpy((char *)item->value, item->def_text,
                   (unsigned)item->max_len + 1u);
    } else if (item->type == AK_PARAM_U32) {
        *(uint32_t *)item->value = item->def.u;
    } else {
        *(float *)item->value = item->def.f;
    }
    params->changed++;
}

void ak_params_reset(ak_params_t *params)
{
    /* One row at a time, through the same function the wire's `default` calls,
     * so "reset everything" and "reset this one" cannot come to mean different
     * things - which is the failure mode a second implementation would have
     * had, and would have shown up only on the parameter type somebody forgot
     * to add to the copy. */
    for (unsigned i = 0; i < params->count; i++) {
        ak_params_default_one(params, i);
    }
}

void ak_params_mark_saved(ak_params_t *params)
{
    params->changed = 0;
}

/*
 * A float as the saved record carries it: with every decimal that is still
 * exact in the formatter's float arithmetic (the scaled value under 2^24), up
 * to seven, and then trailing zeros trimmed back to the display precision.
 *
 * The record used to be written at the display precision itself, so the table
 * flew one value until the next boot and another after it: `rate_ki` set to
 * 0.0004 at three decimals was saved as "0.000" and loaded back as zero, and a
 * whole-number trim of 12.4 came back as 12. What is displayed - and the
 * config hash, which is computed from the displayed text on purpose - are
 * unchanged; only the round trip through flash is.
 */
static void format_for_record(const ak_param_t *item, char *buf, unsigned len)
{
    const float value = *(const float *)item->value;
    const float magnitude = value < 0.0f ? -value : value;
    unsigned decimals = 7u;
    float scaled = magnitude * 10000000.0f;

    while (decimals > item->decimals && scaled >= 16000000.0f) {
        decimals--;
        scaled *= 0.1f;
    }
    ak_format_fixed(value, decimals, buf, len);

    /* Trim: "0.2500000" is "0.250" at three display decimals, and a value
     * that needed none of the extra digits reads exactly as it always did. */
    unsigned n = ak_strlen(buf);
    unsigned dot = n;
    for (unsigned i = 0; i < n; i++) {
        if (buf[i] == '.') {
            dot = i;
        }
    }
    if (dot < n) {
        unsigned keep = dot + 1u + item->decimals;
        while (n > keep && buf[n - 1u] == '0') {
            n--;
        }
        if (n == dot + 1u) {
            n = dot; /* no decimals at all: no point either */
        }
        buf[n] = '\0';
    }
}

unsigned ak_params_serialize(ak_params_t *params, char *buf, unsigned len)
{
    unsigned pos = 0;
    char value[AK_PARAM_VALUE_MAX];

    if (len == 0) {
        return 0;
    }

    for (unsigned i = 0; i < params->count; i++) {
        const ak_param_t *item = &params->items[i];
        const char *name = item->name;
        /* The real value, not the displayed one: the password has to survive
         * the round trip even though nothing prints it - and nor does a float
         * set more finely than it is displayed (format_for_record). */
        if (item->type == AK_PARAM_FLOAT) {
            format_for_record(item, value, sizeof value);
        } else {
            (void)ak_params_get_value(item, value, sizeof value);
        }

        while (*name != '\0' && pos + 1u < len) {
            buf[pos++] = *name++;
        }
        if (pos + 1u < len) {
            buf[pos++] = '=';
        }
        for (unsigned j = 0; value[j] != '\0' && pos + 1u < len; j++) {
            buf[pos++] = value[j];
        }
        if (pos + 1u < len) {
            buf[pos++] = '\n';
        }
    }

    buf[pos < len ? pos : len - 1u] = '\0';
    return pos < len ? pos : len - 1u;
}

int ak_params_store(ak_params_t *params, char *buf, unsigned len)
{
    unsigned length;

    if (len == 0u) {
        return -1;
    }
    length = ak_params_serialize(params, buf, len);
    /* `serialize` answers len - 1 when it filled the buffer, which is the one
     * answer that means "this is not all of it". */
    if (length + 1u >= len) {
        buf[0] = '\0';
        return -1;
    }
    return (int)length;
}

int ak_params_save(ak_params_t *params, char *buf, unsigned len,
                   int (*write)(const void *buf, uint32_t len), int writable)
{
    int length;

    /* Before the table is even serialised: a refused save should not do work,
     * and more to the point the answer must not depend on whether the table
     * happened to fit. */
    if (!writable) {
        return AK_PARAMS_SAVE_BLOCKED;
    }
    length = ak_params_store(params, buf, len);
    if (length < 0) {
        return AK_PARAMS_SAVE_TOO_BIG;
    }
    if (write == 0 || write(buf, (uint32_t)length) != 0) {
        return AK_PARAMS_SAVE_REFUSED;
    }
    /* Marked saved last, and only here. `changed` is what the console reports
     * and what the *next* save's answer rests on, so a table marked saved over
     * a write that failed is a configuration the aircraft believes it has and
     * does not - and the next `save` would say there was nothing to do. */
    ak_params_mark_saved(params);
    return length;
}

/* Parameters this build renamed, and what the record used to call them.
 *
 * A rename is not a cosmetic act here. `gyro_lpf_hz` becoming
 * `gyro_lpf1_static_hz` (roadmap 2.2, so that Betaflight tuning knowledge
 * transfers by name) means a configuration saved by the build before it
 * carries a name this one does not have - and the loader's answer to a name it
 * does not have is to skip it and count it. The aircraft would come up flying
 * the *default* gyro filter, with the console reporting one unknown parameter
 * and the person reading that as an upgrade note rather than as "your gyro
 * filter is gone".
 *
 * So the old spelling is translated rather than dropped, and the load reports
 * it as its own category: not `applied` (the record did not use this name),
 * not `unknown` (the parameter is not missing), but `renamed`. The value is
 * carried across in full, so nothing about the aircraft's behaviour changes at
 * the moment of the upgrade - which is the whole point of having this table
 * rather than a note in a release message.
 *
 * Entries are one-way, oldest name to current, and are never removed: a
 * record written three renames ago still loads. That is a few bytes of flash
 * in exchange for the property that a saved configuration cannot silently lose
 * a filter, and it is why this lives in the loader rather than in a migration
 * step somebody has to remember to run. */
typedef struct {
    const char *was;   /* the name the record may carry */
    const char *now;   /* the name this build registers */
} ak_param_rename_t;

static const ak_param_rename_t param_renames[] = {
    /* Roadmap 2.2: the gyro and D-term chains took Betaflight's names. */
    { "gyro_lpf_hz",  "gyro_lpf1_static_hz"  },
    { "d_cutoff_hz",  "dterm_lpf1_static_hz" },
};

#define PARAM_RENAME_COUNT \
    (sizeof param_renames / sizeof param_renames[0])

/* The current spelling of `name`, or null when this build never renamed it. */
static const char *renamed_to(const char *name)
{
    for (unsigned i = 0; i < PARAM_RENAME_COUNT; i++) {
        if (ak_str_eq(name, param_renames[i].was)) {
            return param_renames[i].now;
        }
    }
    return 0;
}

int ak_params_deserialize(ak_params_t *params, const char *text, char *msg,
                          unsigned msg_len)
{
    const char *p = text;

    /* A load is a question about two builds: what the record carried that this
     * one does not have, and what this one has that the record never saw. So
     * forget what the last load saw before starting this one. */
    for (unsigned i = 0; i < params->count; i++) {
        params->items[i].flags &= (uint8_t)~AK_PARAM_SEEN;
    }
    params->load_applied = 0;
    params->load_unknown = 0;
    params->load_renamed = 0;
    params->load_unknown_name[0] = '\0';
    params->load_renamed_name[0] = '\0';

    /* The first value refused, reported after the whole record has been read.
     * Stopping at it - which this did until 2026-10-06 - left the lines before
     * it applied and the lines after it at their defaults, and the caller then
     * skipped applying the change at all: a boot with one value an upgraded
     * build no longer accepts flew a mixture nobody had chosen. Now every value
     * that is valid is loaded, each refused one keeps its current value, and
     * the refusal is still an error the caller reports. */
    int refused = 0;
    char first_refusal[64];
    first_refusal[0] = '\0';

    while (*p != '\0') {
        const char *line_end = p;
        while (*line_end != '\0' && *line_end != '\n' && *line_end != '\r') {
            line_end++;
        }

        const char *equals = p;
        while (equals < line_end && *equals != '=') {
            equals++;
        }

        if (equals > p && equals < line_end) {
            char name[AK_PARAM_NAME_MAX];
            /* Long enough for the longest text value: a Wi-Fi password is 63
             * characters and this is the path it is read back through. */
            char value[AK_PARAM_VALUE_MAX];
            unsigned name_len = (unsigned)(equals - p);
            unsigned value_len = (unsigned)(line_end - equals - 1);

            if (name_len >= sizeof name) {
                name_len = sizeof name - 1u;
            }
            if (value_len >= sizeof value) {
                value_len = sizeof value - 1u;
            }
            for (unsigned i = 0; i < name_len; i++) {
                name[i] = p[i];
            }
            name[name_len] = '\0';
            for (unsigned i = 0; i < value_len; i++) {
                value[i] = equals[1 + i];
            }
            value[value_len] = '\0';

            ak_param_t *item = ak_params_find(params, name);
            int renamed = 0;

            /* A name this build renamed is not a name this build lacks. Try
             * the current spelling before deciding the record carried
             * something from a build we cannot speak to. */
            if (item == 0) {
                const char *now = renamed_to(name);

                if (now != 0) {
                    item = ak_params_find(params, now);
                    renamed = (item != 0);
                }
            }

            if (item != 0) {
                /* Set under the *current* name: the record's spelling is a
                 * historical fact and does not go back into the table. */
                char why[48];
                why[0] = '\0';
                if (ak_params_set(params, item->name, value, why, sizeof why) != 0) {
                    if (refused == 0) {
                        /* Named: "out of range 0.000..2.000" alone does not
                         * say which of forty-five parameters it was. */
                        unsigned at = 0;
                        for (const char *c = item->name;
                             *c != '\0' && at + 1u < sizeof first_refusal; c++) {
                            first_refusal[at++] = *c;
                        }
                        for (const char *c = ": ";
                             *c != '\0' && at + 1u < sizeof first_refusal; c++) {
                            first_refusal[at++] = *c;
                        }
                        for (const char *c = why;
                             *c != '\0' && at + 1u < sizeof first_refusal; c++) {
                            first_refusal[at++] = *c;
                        }
                        first_refusal[at] = '\0';
                    }
                    refused++;
                    goto next_line;
                }
                item->flags |= AK_PARAM_SEEN;
                params->load_applied++;
                if (renamed) {
                    params->load_renamed++;
                    if (params->load_renamed_name[0] == '\0') {
                        unsigned i = 0;

                        while (name[i] != '\0' &&
                               i + 1u < sizeof params->load_renamed_name) {
                            params->load_renamed_name[i] = name[i];
                            i++;
                        }
                        params->load_renamed_name[i] = '\0';
                    }
                }
            } else {
                /* An unknown name is skipped on purpose - and remembered,
                 * because "one parameter in the record is not in this build"
                 * is the thing a person wants to be told after an upgrade. */
                params->load_unknown++;
                if (params->load_unknown_name[0] == '\0') {
                    unsigned i = 0;

                    while (name[i] != '\0' &&
                           i + 1u < sizeof params->load_unknown_name) {
                        params->load_unknown_name[i] = name[i];
                        i++;
                    }
                    params->load_unknown_name[i] = '\0';
                }
            }
        }

    next_line:
        p = (*line_end == '\0') ? line_end : line_end + 1;
    }

    /* And the other direction: this build's parameters the record never
     * carried, which keep whatever they were compiled with. */
    params->load_unmentioned = 0;
    params->load_unmentioned_name[0] = '\0';
    for (unsigned i = 0; i < params->count; i++) {
        if ((params->items[i].flags & AK_PARAM_SEEN) == 0u) {
            params->load_unmentioned++;
            if (params->load_unmentioned_name[0] == '\0') {
                unsigned j = 0;

                while (params->items[i].name[j] != '\0' &&
                       j + 1u < sizeof params->load_unmentioned_name) {
                    params->load_unmentioned_name[j] =
                        params->items[i].name[j];
                    j++;
                }
                params->load_unmentioned_name[j] = '\0';
            }
        }
    }
    params->load_have_report = 1;
    if (refused > 0) {
        if (msg != 0 && msg_len > 0u) {
            unsigned at = 0;
            for (const char *c = first_refusal; *c != '\0' && at + 1u < msg_len; c++) {
                msg[at++] = *c;
            }
            msg[at] = '\0';
        }
        return -1;
    }
    return 0;
}

void ak_params_load_report(const ak_params_t *params,
                           ak_params_load_report_t *out)
{
    out->total = params->count;
    out->applied = params->load_applied;
    out->unknown = params->load_unknown;
    out->renamed = params->load_renamed;
    out->unmentioned = params->load_unmentioned;
    out->unknown_name = params->load_unknown_name;
    out->renamed_name = params->load_renamed_name;
    out->unmentioned_name = params->load_unmentioned_name;
}

/* FNV-1a, byte at a time. The offset basis and the prime are the same two
 * constants the F405's flash record uses (there is one of these in the tree
 * already); what is different here is only *which* bytes go in. */
static uint32_t hash_byte(uint32_t hash, char byte)
{
    hash ^= (uint32_t)(uint8_t)byte;
    return hash * 16777619u;
}

static uint32_t hash_text(uint32_t hash, const char *text)
{
    while (*text != '\0') {
        hash = hash_byte(hash, *text++);
    }
    return hash;
}

uint32_t ak_params_hash(const ak_params_t *params)
{
    char value[AK_PARAM_VALUE_MAX];
    uint32_t hash = 0x811C9DC5u;

    /* Table order, not sorted: the order is a property of the build, so two
     * builds that register the same parameters in a different order hash
     * differently, which is the honest answer - they are different builds. */
    for (unsigned i = 0; i < params->count; i++) {
        const ak_param_t *item = &params->items[i];

        hash = hash_text(hash, item->name);
        hash = hash_byte(hash, '=');
        /* get_text, not get_value: that is the one line that turns a secret
         * into "***", and it is why this function exists next to the two
         * printers rather than reaching into the value itself. */
        ak_params_get_text(item, value, sizeof value);
        hash = hash_text(hash, value);
        hash = hash_byte(hash, '\n');
    }

    return hash;
}
