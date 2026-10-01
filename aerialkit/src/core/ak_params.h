#ifndef AK_CORE_AK_PARAMS_H
#define AK_CORE_AK_PARAMS_H

#include <stdint.h>

#include "ak_console.h"

/*
 * The tunable numbers, as a table.
 *
 * A parameter points straight at the field it controls, so setting one cannot
 * drift out of step with what the control loop reads - there is no copy to
 * forget to refresh. Ranges are checked on the way in, and a value outside its
 * range is *rejected*, not clamped: a mistyped gain should look like an error,
 * not like a silently different aircraft.
 *
 * This table is also what gets written to flash and what a configurator will
 * talk to later, so there is one definition of "a parameter" in the firmware
 * rather than three.
 */

/*
 * A name has to fit this, and the reason is not tidiness.
 *
 * `ak_params_deserialize` reads a name out of the saved record into a buffer of
 * this size and looks it up *by exact match*. A name longer than this was
 * therefore truncated before the lookup, never matched the registered
 * parameter, and was counted as an unknown name in the record - so the
 * parameter kept whatever default it had been registered with and came back
 * from every power cycle reset, while the load reported success and the boot
 * report blamed the record for holding a parameter the build did not have.
 *
 * The bug was measured, not reasoned about: `set arm_accel_lpf_hz 40` took,
 * `save` took, and the value read back as 25.000 after a load. Eleven of the
 * ninety-two names the board registers were over the old limit of 16, and two
 * of them - arm_accel_lpf_hz and arm_max_tilt_deg - are what the arming gate
 * consults.
 *
 * 24 is the round number above the longest name the board has (22,
 * `wing_descend_pitch_deg`) with the terminator. It is a bound and not a
 * budget: `ak_params_add_*` refuses a longer name and counts it, and
 * tools/param_name_check.py fails the build on any registration site that
 * spells one, because a name that cannot be read back is a parameter that
 * cannot be configured no matter how well it is spelled everywhere else.
 */
#define AK_PARAM_NAME_MAX  24
/*
 * The table, and the record it is saved in. Both were raised after the ESP32's
 * Wi-Fi parameters went in: the table was *full at 64* and registering another
 * one was a silent no-op - it returned the same index and the parameter simply
 * did not exist, which is the worst possible way to find out. The F405's own
 * table reaches 64 today, so the old limit was not a future problem; it was
 * this one, and nothing had noticed because a dropped parameter looks exactly
 * like a parameter nobody added.
 *
 * `AK_PARAMS_TEXT_MAX` grew with it, because a table that fits in RAM but not
 * in the record is a configuration that saves itself by halves: `save` is
 * refused outright when the text does not fit (see ak_cli.c), rather than
 * writing a prefix that loads back as a mixture of two configurations.
 */
#define AK_PARAMS_MAX      96
#define AK_PARAMS_TEXT_MAX 2048
/* The longest value a text parameter can hold, including its terminator. A
 * Wi-Fi password is up to 63 characters and an SSID up to 32, and 64 is the
 * round number above both. */
#define AK_PARAM_VALUE_MAX 64

typedef union {
    float    f;
    uint32_t u;
} ak_param_value_t;

typedef enum {
    AK_PARAM_FLOAT = 0,
    AK_PARAM_U32 = 1,
    AK_PARAM_TEXT = 2,
} ak_param_type_t;

/*
 * What a parameter belongs to, so a configurator can group the table the way
 * the firmware means it rather than the way the names happen to read.
 *
 * This exists because a screen was doing the latter. It guessed a group from a
 * name prefix ("rate_" is the rate loop, "wp" is the mission), which is a claim
 * about a structure the board never stated - and it was already wrong: it had
 * no rule for `arm_accel_lpf_hz` and no idea what to do with the sixty-odd
 * parameters it had never seen. A guess that is right most of the time is worse
 * than a group that is stated, because the failures are silent and the successes
 * make it look trustworthy.
 *
 * The group is an enumeration rather than a string so it survives the wire as
 * one byte, and because the *names* are then a decision of this file rather than
 * a typo repeated in ninety places. `ak_param_group_name()` is the one place
 * that spells them; the protocol serves that table so a client needs no copy.
 *
 * **Adding a value here is a protocol change a client will not know about** in
 * the same way a new command is: an unknown group number renders as unknown,
 * which is honest, rather than being folded into a neighbour.
 */
typedef enum {
    AK_PARAM_GROUP_NONE = 0,
    AK_PARAM_GROUP_RATES,      /* the rate loop and its filters */
    AK_PARAM_GROUP_ANGLE,      /* the angle loop and the tilt limits */
    AK_PARAM_GROUP_ARMING,     /* what has to be true before it may arm */
    AK_PARAM_GROUP_RECEIVER,   /* channels, protocol, failsafe timing */
    AK_PARAM_GROUP_AIRFRAME,   /* which mix, and what it is made of */
    AK_PARAM_GROUP_OUTPUTS,    /* the servo and ESC outputs themselves */
    AK_PARAM_GROUP_POWER,      /* the pack: divider, cells, thresholds */
    AK_PARAM_GROUP_FAILSAFE,   /* what it does when the link goes */
    AK_PARAM_GROUP_NAVIGATION, /* return-to-home, mission, launch, fence */
    AK_PARAM_GROUP_SENSORS,    /* board alignment and per-sensor trim */
    AK_PARAM_GROUP_NETWORK,    /* the radio build's own settings */
    AK_PARAM_GROUP_TIMING,     /* how long a loop step may span */
    AK_PARAM_GROUP_COUNT
} ak_param_group_t;

/* The name of a group, or "" for an unknown number - never a guess and never a
 * neighbour's name, because a client showing the wrong group heading is worse
 * than one showing none. */
const char *ak_param_group_name(uint8_t group);

/* How many groups there are, so the protocol can serve the table and a test can
 * walk it without a second copy of the count. */
unsigned ak_param_group_count(void);

/* A secret parameter is one whose value is not printed: `params`, `get`, the
 * protocol's param get and the boot report all show "***" for it. It still
 * round-trips through save and load, because the aircraft has to be able to
 * remember its own Wi-Fi password - what it does not do is tell anybody who
 * asks. See 16-protocol.md for why that matters more than it sounds: the
 * protocol has no authentication of its own. */
#define AK_PARAM_SECRET 0x01u

/* Set by a load on every parameter the saved record carried. It is transient -
 * it says what the *last* load did rather than how a parameter is stored - and
 * it is what lets a load report which of this build's parameters the record
 * never mentioned, which is the "added since the record was written" half of a
 * firmware upgrade. */
#define AK_PARAM_SEEN 0x02u

typedef struct {
    const char       *name;
    const char       *help;
    uint8_t           type;
    uint8_t           group;    /* ak_param_group_t: what it belongs to */
    uint8_t           decimals; /* float display precision */
    void             *value;    /* points at the live float or uint32_t */
    ak_param_value_t  def;
    ak_param_value_t  min;
    ak_param_value_t  max;
    uint8_t           max_len;   /* text: characters it may hold */
    uint8_t           flags;     /* AK_PARAM_SECRET */
    const char       *def_text;  /* text: what `defaults` restores */
} ak_param_t;

typedef struct {
    ak_param_t *items;
    unsigned    count;
    unsigned    changed; /* set by set/reset, cleared by mark_saved */

    /* What the last load did, which is the question an upgrade asks: the
     * record was written by a build that may have had different parameters.
     * `applied` is what came from the record, `unknown` is what the record
     * carried and this build does not have, and `unmentioned` is this build's
     * and the record never saw. The first name of each, because a count says
     * something happened and a name says what. */
    unsigned    load_applied;
    unsigned    load_unknown;
    unsigned    load_unmentioned;
    char        load_unknown_name[AK_PARAM_NAME_MAX];
    char        load_unmentioned_name[AK_PARAM_NAME_MAX];
    int         load_have_report;
} ak_params_t;

/* The same thing as a value, so a caller does not reach into the table. */
typedef struct {
    unsigned    total;      /* parameters this build has */
    unsigned    applied;    /* of them, that the record set */
    unsigned    unknown;    /* in the record, not in this build */
    unsigned    unmentioned;/* in this build, never in the record */
    const char *unknown_name;      /* "" when there were none */
    const char *unmentioned_name;
} ak_params_load_report_t;

void ak_params_init(ak_params_t *params, ak_param_t *items, unsigned count);

/*
 * Table builders, so that a module can describe its own tunables without
 * repeating the plumbing. Each returns the next free index.
 *
 * **`group` is the last argument and it is not optional, deliberately.** It
 * would have been easy to give it a default and let the existing ninety-odd
 * registrations keep compiling; a default is a value nobody chose, and the
 * whole reason the field exists is that a screen was *deriving* a structure
 * from names. As the last argument of a function every call site names, a
 * registration that forgets its group does not compile - so the failure mode is
 * a build error, which somebody fixes in a minute, rather than a parameter
 * filed under the wrong heading, which nobody notices at all.
 */
unsigned ak_params_add_float(ak_param_t *items, unsigned n, const char *name,
                             const char *help, float *value, uint8_t decimals,
                             float low, float high, uint8_t group);
unsigned ak_params_add_u32(ak_param_t *items, unsigned n, const char *name,
                           const char *help, uint32_t *value, uint32_t low,
                           uint32_t high, uint8_t group);

/* A string parameter: a network name, a password, a callsign. The buffer and
 * its length are the caller's, and `default_text` is what `defaults` puts
 * back - a text parameter has no numeric range, so the length is its limit. */
unsigned ak_params_add_text(ak_param_t *items, unsigned n, const char *name,
                            const char *help, char *value, unsigned max_len,
                            uint8_t flags, const char *default_text,
                            uint8_t group);

/* How many registrations were refused because the table was full. It should
 * always be zero, and it is counted rather than assumed because the failure is
 * invisible otherwise: the parameter that was dropped is missing from `params`
 * and nothing else says so. The boot report and the preflight print it. */
unsigned ak_params_overflow(void);

/* How many registrations were refused because the name did not fit
 * AK_PARAM_NAME_MAX. It should always be zero, and it is a separate count from
 * the overflow above because the failure is a different one: an overflow loses
 * a parameter outright, while a long name registers, reads, sets and saves and
 * is then quietly not loaded - the aircraft flies the default and the record
 * gets the blame. See the comment on AK_PARAM_NAME_MAX. */
unsigned ak_params_long_name(void);

ak_param_t *ak_params_find(ak_params_t *params, const char *name);

/* Returns 0 on success; -1 for an unknown name, -2 for a value that does not
 * parse or is outside its range. `msg` is filled for the caller to print. */
int ak_params_set(ak_params_t *params, const char *name, const char *text,
                  char *msg, unsigned msg_len);

/* The current value as text, for showing: "0.250", "60000", or "***" for a
 * parameter whose value is a secret. */
void ak_params_get_text(const ak_param_t *item, char *buf, unsigned len);

/*
 * A bound or a default, printed exactly the way the value itself is printed:
 * the same decimal count, the same whole-number rule.
 *
 * It exists because a configurator shows a range next to a value, and those two
 * have to be the same number spelled the same way. A client that formatted a
 * bound for itself would be a second authority for what a parameter's text
 * means - and the two authorities would disagree first about a float, where
 * "0.1" and "0.100" are the same number and not the same string, and then about
 * a secret, which is the one value that must never be printed at all.
 *
 * `describe_range` below was already the only place that spelled a bound, and
 * this is that spelling lifted out of it so the wire and the range message
 * cannot drift apart.
 */
unsigned ak_params_format_value(const ak_param_t *item, ak_param_value_t value,
                                char *buf, unsigned len);

/*
 * What `defaults` would put back, as text - and "***" for a parameter whose
 * value is a secret. A default is a value, so it is hidden on the same terms as
 * one: a Wi-Fi password restored by `defaults` is the same string as a Wi-Fi
 * password, and a screen that could read it here could read it without ever
 * asking for the parameter itself.
 */
void ak_params_get_default_text(const ak_param_t *item, char *buf, unsigned len);

/* And the value itself, for writing down. The difference between the two is
 * the whole point of AK_PARAM_SECRET: a configuration file has to hold the
 * Wi-Fi password, and a console dump must not. */
unsigned ak_params_get_value(const ak_param_t *item, char *buf, unsigned len);

void ak_params_dump(ak_params_t *params, ak_printf_fn out);

/* Put one row back to the value it was registered with, by index.
 *
 * The wire's `default` needs the single-row form, and it is this function
 * rather than a second copy of the same three-way type switch: `reset` is now
 * this in a loop, so the two cannot come to mean different things for a
 * parameter type somebody adds later. Out-of-range is a no-op here and is
 * refused by the caller, which is where "no such parameter" belongs.
 *
 * Text is restored from `def_text` and not from `def`, and a secret is restored
 * as the secret - that is what a default is. `ak_params_get_default_text` is
 * what hides it on the way *out*. */
void ak_params_default_one(ak_params_t *params, unsigned index);
void ak_params_reset(ak_params_t *params);
void ak_params_mark_saved(ak_params_t *params);

/*
 * The configuration's identity: a 32-bit hash of the values the table holds
 * *now*, as opposed to `changed`, which is a count of differences from the last
 * save and therefore an answer to a different question.
 *
 * The distinction is the whole reason this exists. `changed = 0` says "nothing
 * has moved since something unnamed", which is not something an experiment
 * result can be filed under - two aircraft with different gains both report
 * zero. This says which configuration it was.
 *
 * **A secret does not go into it.** A parameter whose value is not printed
 * contributes its *name* and the marker the console already prints for it
 * (`***`), never its contents. So the hash does not change when only a Wi-Fi
 * password changes, and the password cannot be recovered from it - which
 * matters because a hash of a short, low-entropy, human-chosen string is a
 * string somebody can guess offline in a few minutes.
 *
 * FNV-1a, the same algorithm the F405 uses for its flash record's checksum, for
 * the reason the record gives: there is one of these here already and a second
 * would be a second answer. Over a configuration with no secrets set the two
 * numbers are equal by construction - `ak_params_hash` hashes exactly the bytes
 * `ak_params_serialize` writes, with the secret rule applied.
 */
uint32_t ak_params_hash(const ak_params_t *params);

/* "name=value\n" per line. Returns the number of bytes written, always
 * terminated when len > 0; a return of len - 1 means it was truncated. */
unsigned ak_params_serialize(ak_params_t *params, char *buf, unsigned len);

/*
 * The same text, with the one rule `save` needs: if the table does not fit the
 * buffer, nothing is written and the answer is -1. A truncated record loads
 * back as a mixture - the parameters that fit come from the file and the rest
 * keep whatever the aircraft happens to hold - which is worse than a save that
 * refuses and says so.
 *
 * It lives here, next to the table, because there are two ways to save one:
 * the console's `save` and the protocol's param save. Each of them checking
 * for itself is how the console came to have the check and the protocol not.
 */
int ak_params_store(ak_params_t *params, char *buf, unsigned len);

#define AK_PARAMS_SAVE_TOO_BIG  (-1)  /* the table does not fit the record */
#define AK_PARAMS_SAVE_REFUSED  (-2)  /* the board would not take it */
#define AK_PARAMS_SAVE_BLOCKED  (-3)  /* not while it is flying: see below */

/*
 * The one way a configuration reaches the board: serialise the table, refuse a
 * table that does not fit rather than saving a prefix, write it, and only then
 * mark it saved.
 *
 * `ak_params_store` above already carries the argument for the first half -
 * each route checking for itself is how the console came to have the check and
 * the protocol not. The point of B3 is that the same sentence was true of the
 * other three quarters: the console's `save` and main.c's save_parameters were
 * two copies of this sequence, agreeing today and free to disagree tomorrow,
 * and the second copy is the one both the protocol's param save and every
 * future caller goes through.
 *
 * `write` is the board's, and a null one answers REFUSED rather than being a
 * special case every caller has to remember: a device with no storage is a
 * device whose write always fails.
 *
 * `writable` is the armed-state policy's answer, and it is an argument rather
 * than something this function works out for itself because the table does not
 * know what an aircraft is. Passing it in is what makes the policy unskippable:
 * a route cannot call this without having asked ak_flight_config_writable()
 * first, so a third route added later inherits the rule by construction
 * instead of by remembering it.
 *
 * Returns the number of bytes written, or one of the three constants above.
 */
int ak_params_save(ak_params_t *params, char *buf, unsigned len,
                   int (*write)(const void *buf, uint32_t len), int writable);

/* Reads what serialize() wrote. Unknown names are skipped - a file from a newer
 * build should not stop an older one - while an invalid value for a known name
 * is an error. Returns 0 on success, -1 with `msg` set otherwise. */
int ak_params_deserialize(ak_params_t *params, const char *text, char *msg,
                          unsigned msg_len);

/* What the last successful deserialize did: see ak_params_load_report_t. Before
 * any load it reports the whole table as unmentioned. */
void ak_params_load_report(const ak_params_t *params,
                           ak_params_load_report_t *out);

#endif /* AK_CORE_AK_PARAMS_H */
