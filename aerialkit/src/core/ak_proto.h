#ifndef AK_CORE_AK_PROTO_H
#define AK_CORE_AK_PROTO_H

#include <stdint.h>

#include "ak_console.h"
#include "ak_params.h"

/*
 * The machine-readable side of the console.
 *
 * The console is for a person and the protocol is for a program: the NAS web
 * console, a companion computer on the aircraft, a test script. They share the
 * same UART, distinguished by the frame's first byte, and they share the same
 * parameter table - which is the point of having one table.
 *
 *   AA 55  version  command  length  payload...  crc16(lo)  crc16(hi)
 *
 * The crc covers the version, command, length and payload - everything except
 * the sync pair and the crc itself, which is the only arrangement where a
 * corrupted length cannot be hidden by the bytes that follow it.
 *
 * Commands are small and their replies carry the same command with the top bit
 * set, so a reply to command 1 is 0x81: one field to check instead of a
 * correlation table.
 *
 * Byte-fed, like every other parser here, because that is the shape a UART
 * gives you - and because it makes a truncated frame a nothing rather than a
 * wrong answer.
 */

#define AK_PROTO_SYNC1 0xAAu
#define AK_PROTO_SYNC2 0x55u
#define AK_PROTO_VERSION 1u
#define AK_PROTO_MAX_PAYLOAD 96u
#define AK_PROTO_FRAME_MAX (6u + AK_PROTO_MAX_PAYLOAD + 2u)
#define AK_PROTO_RESPONSE_BIT 0x80u
#define AK_PROTO_GAP_MS 50u

enum {
    AK_PROTO_CMD_HELLO = 0x01,
    AK_PROTO_CMD_PARAM_GET = 0x02,
    AK_PROTO_CMD_PARAM_SET = 0x03,
    AK_PROTO_CMD_PARAM_SAVE = 0x04,
    AK_PROTO_CMD_STATUS = 0x05,
    AK_PROTO_CMD_LOG_INFO = 0x06,
    AK_PROTO_CMD_LOG_GET = 0x07,
    /* Ask for a telemetry stream at this rate in Hz, 0 to stop. The reply
     * acks the rate that was accepted. */
    AK_PROTO_CMD_TELEMETRY = 0x08,
    /* Which log the next LOG_INFO and LOG_GET are about. A device has three
     * answers to "what happened": the fast ring in RAM, the long one that
     * survives a reset, and the one in flash that survives the battery. The
     * reply says which one is now selected and how many records it holds, so a
     * tool needs one round trip rather than two.
     *
     * LOG_GET's index is *within the selected log*, oldest first, which is
     * what makes the three of them the same interface from a client's side. */
    AK_PROTO_CMD_LOG_SOURCE = 0x09,
    /* What a parameter *is*, as opposed to what it holds: its type, its group,
     * its bounds and its default. PARAM_GET has always answered the value and
     * the console has always printed all of this beside it, but the wire
     * carried none of it - so every client either showed a value with no range
     * or invented one, and a screen that guessed a group from a name prefix is
     * what the `group` field was added to stop.
     *
     * Paged, because an entry is a name and three numbers spelled out and the
     * frame is 96 bytes. `carried` in the reply is what was actually written,
     * so a page that ran out of room says so rather than reporting a short
     * list as a complete one. */
    AK_PROTO_CMD_PARAM_INFO = 0x0A,
    /* The long prose beside a parameter, fetched per row and on demand. It is a
     * separate command because it is the one field nobody needs for all ninety
     * at once, and because it is the only one with no useful bound on its
     * length - so it is walked by offset rather than paged by count, and no
     * part of it is ever silently cut off. */
    AK_PROTO_CMD_PARAM_HELP = 0x0B,
    /* Put parameters back to the defaults this build was compiled with.
     *
     * `mode` selects what is reset: 1 is one parameter, named by the `index`
     * that follows; 2 is the whole table. **A bare request is refused** - an
     * empty frame meaning "wipe every parameter" is a booby trap, and the one
     * command on this wire that a truncated frame could turn into a factory
     * reset is not a command to give a default to. Mode 0 is not "all"; it is
     * a request that named nothing.
     *
     * Unlike the console's `defaults`, this refuses while the aircraft is
     * armed - see `ak_proto_io_t::writable`. The two disagree on purpose and
     * docs/16-protocol.md says why. */
    AK_PROTO_CMD_PARAM_DEFAULT = 0x0C,
    /* What the receiver is hearing, right now: the counts as they arrived, the
     * sticks those counts mean, and enough counters to tell "no receiver" from
     * "a receiver saying nonsense".
     *
     * Polled, never streamed, and that is a decision rather than an omission.
     * The console link carries no stream at all (`can_stream`), and the one tab
     * a person most wants while holding a transmitter is exactly the one that
     * should work on the cable. A client that wants it repeatedly asks
     * repeatedly; at the rates a handset moves, that costs nothing.
     *
     * **The firmware decodes the sticks.** `rc_min`, `rc_mid`, `rc_max` and
     * `rc_deadband` are parameters, and an app that recomputed roll and pitch
     * from them would be a second authority on the one number that decides
     * whether the aircraft is being commanded - two implementations of
     * `centred()` in two languages, disagreeing at the deadband edge, with the
     * screen and the airframe each believing its own. The counts are sent too,
     * because a count out of range is the first sign of a receiver on the wrong
     * baud rate, but they are sent as evidence and not as an input.
     *
     * A request has no arguments. There is nothing to name: this is what the
     * receiver is doing, not a query about a channel. */
    AK_PROTO_CMD_RC_CHANNELS = 0x0D,
    /* What the board is *hearing* rather than what it is doing: the IMU, the
     * barometer, the rangefinder, the flight pack and the GPS, each in the
     * terms its own driver reports it.
     *
     * Named by topic rather than sent as one frame, because the five have
     * nothing in common but the question. A combined reply would be either
     * larger than the frame or missing whichever sensor the board happens to
     * carry, and a client that has to ask about a board with no rangefinder
     * every time it wants the battery is a client that reads four fifths
     * nothing.
     *
     * **The absent case is the reason this opcode is shaped the way it is.**
     * The console has always answered "none fitted" in words, and a wire that
     * answered a struct of zeros instead would be saying "this board has a
     * barometer and it reads zero pressure" - which is not a missing sensor,
     * it is a sensor that has failed, and the two call for different afternoons
     * of work. So `present` is a byte on every topic and the body is *absent*,
     * not zeroed, when it is clear: a body that is not there cannot be mistaken
     * for a reading of zero.
     *
     * A board with no sensor reporting at all leaves `sensor_state` null, and
     * every topic then answers AK_PROTO_SENSOR_NO_SUCH - which is true of it,
     * and is why there is no separate "this board has no sensors" status to
     * get wrong. */
    AK_PROTO_CMD_SENSOR_INFO = 0x0E,
};

/* The status byte on RC_CHANNELS.
 *
 * The second value is the one worth having. A board with no receiver port could
 * answer a frame of zeros, and that frame would be indistinguishable from a
 * receiver that is plugged in and has never framed - which is a wiring fault at
 * the other end of the cable, a different afternoon's work, and the thing the
 * console already distinguishes by having no `rc_report` at all. */
enum {
    AK_PROTO_RC_OK = 0,   /* this board has a receiver */
    AK_PROTO_RC_NONE = 1, /* this board has no receiver input */
};

/* The most channels a reply can carry, and a bound on the firmware's own count
 * rather than a copy of it: AK_RC_CHANNELS is the flight core's number and this
 * is the wire's, so a firmware that grew its receiver could keep the same
 * client working by sending the first AK_PROTO_RC_MAX of them. */
#define AK_PROTO_RC_MAX 16u

/* Facts about the receiver, as bits. Facts, not warnings: which of them is
 * worth colouring red is the app's business, and a firmware that decided would
 * be pre-rendering a screen it cannot see.
 *
 * The four counters that follow the switches are per protocol, and the ones
 * that do not apply read zero - which is true rather than a placeholder, since
 * an SBUS frame has no CRC to fail and a CRSF frame cannot be flagged
 * failsafe by a receiver that has no such flag. */
#define AK_PROTO_RC_LINK        (1u << 0) /* a channel frame has arrived */
#define AK_PROTO_RC_FAILSAFE    (1u << 1) /* the receiver's own failsafe is set */
/* The four sticks below are this frame's, decoded by the firmware. Clear when
 * there is no frame, and clear when there is one it could not use - an
 * all-zero frame, or counts that cannot be a stick. The sticks are zeroed in
 * that case, so this bit is the only thing that tells "not decoded" from
 * "centred", and the two must never be shown the same way. */
#define AK_PROTO_RC_DECODED     (1u << 2)
/* This board's receiver pin has no inverter in front of it. Only meaningful on
 * SBUS, where it is the difference between a working link and an afternoon
 * spent on the baud rate - the same warning the console prints. */
#define AK_PROTO_RC_NO_INVERTER (1u << 3)
/* There is a return path to the handset. CRSF has one; SBUS does not, and a
 * client that offered telemetry settings on a board without one would be
 * offering a control that cannot act. */
#define AK_PROTO_RC_TELEMETRY   (1u << 4)

#define AK_PROTO_RC_ARM_ON  (1u << 0)
#define AK_PROTO_RC_ANGLE   (1u << 1)

/* The receiver, as a caller fills it in. Spelled out here for the same reason
 * ak_proto_status_t is: the protocol has no opinion about where the numbers
 * come from, and a board with a receiver on a different bus fills the same
 * struct. */
typedef struct {
    uint8_t  flags;
    uint8_t  protocol; /* 0 CRSF, 1 SBUS - the same numbers as ak_rc_protocol_t */
    uint8_t  count;    /* how many of `raw` are meaningful */
    uint8_t  switches; /* AK_PROTO_RC_ARM_ON | AK_PROTO_RC_ANGLE */
    uint16_t raw[AK_PROTO_RC_MAX];
    /* Per-mille, in the order the console prints them: roll, pitch, yaw,
     * throttle. Roll/pitch/yaw are -1000..1000 and throttle is 0..1000. */
    int16_t  sticks[4];
    /* Counters, named as the receiver names them so there is no translation to
     * get wrong. See the flag comment for which are zero on which protocol. */
    uint32_t bytes;
    uint32_t frames;
    uint32_t crc_errors;
    uint32_t rejected;
    uint32_t lost;
    uint32_t failsafe_frames;
    uint32_t dropped; /* the UART receive buffer's drops, which only the board knows */
} ak_proto_rc_t;

/*
 * The sensors, as SENSOR_INFO asks for them.
 *
 * Five topics, numbered rather than named, so a client added to before the
 * firmware is older than the firmware is asked for a topic it does not have
 * and gets a clean refusal rather than a misread frame. The numbers are the
 * order the console's own bring-up checklist prints them in, which is the order
 * of how much is lost without them.
 */
#define AK_PROTO_SENSOR_IMU     0u
#define AK_PROTO_SENSOR_BARO    1u
#define AK_PROTO_SENSOR_RANGE   2u
#define AK_PROTO_SENSOR_BATTERY 3u
#define AK_PROTO_SENSOR_GPS     4u
#define AK_PROTO_SENSOR_TOPICS  5u

/* How long a driver's name may be, including the terminating zero. Twelve
 * because the longest in this tree is "lsm6dso" - the field is fixed so the
 * body's length does not depend on which part answered, and a fixed field means
 * a bound that has to be stated. A name that does not fit is cut, and the
 * template that defines the drivers has a check that fails the build if one
 * ever would be. */
#define AK_PROTO_SENSOR_NAME 12u

/*
 * The status byte on SENSOR_INFO.
 *
 * Two values, and the split between them is not "found" and "not found". NO_SUCH
 * is a statement about the *build*: this firmware does not answer for that
 * topic, either because the topic does not exist or because the board wired no
 * sensor reporting at all. A topic that exists and has nothing fitted is
 * AK_PROTO_SENSOR_OK with `present` clear - the board knows the question and
 * its answer is that there is no part there, which is a different sentence from
 * "I do not know what you are asking", and a client that showed them the same
 * way would send somebody looking for a driver instead of a socket. */
enum {
    AK_PROTO_SENSOR_OK = 0,      /* this build knows the topic */
    AK_PROTO_SENSOR_NO_SUCH = 1, /* no such topic on this board */
};

/* Least significant of the four reasons a board can have no IMU, as bits, so a
 * client can show the console's sentence without a table of its own. The values
 * do not match ak_imu_result_t's and are not meant to: that enum is the
 * driver layer's and this is the wire's, and pinning one to the other would
 * make renumbering either a protocol change. */
#define AK_PROTO_IMU_ABSENT_NOBODY        1u /* nothing answered on the bus */
#define AK_PROTO_IMU_ABSENT_UNKNOWN_PART  2u /* something answered, and it is not known */
#define AK_PROTO_IMU_ABSENT_NO_CONFIG     3u /* the right part answered and would not configure */

/* Each body is a fixed layout, written field by field in the dispatch, so the
 * structs below are how a caller hands the numbers over rather than a
 * description of the bytes. Sizes are counted in append_* calls, not in
 * sizeof - padding is the compiler's business and the wire has none. */

typedef struct {
    char     driver[AK_PROTO_SENSOR_NAME]; /* empty when absent_reason is set */
    uint8_t  absent_reason;    /* 0, or one of AK_PROTO_IMU_ABSENT_* */
    uint8_t  whoami;           /* what the part answered to its identity register */
    int16_t  accel[3];         /* per-mille of g */
    int16_t  gyro[3];          /* milliradians per second */
    int16_t  align[3];         /* degrees, as the aircraft's axes were set */
    int16_t  gyro_bias[3];     /* milli-degrees per second, as `calibrate gyro` left it */
    uint32_t samples;
    uint32_t errors;
} ak_proto_imu_t;

typedef struct {
    char     driver[AK_PROTO_SENSOR_NAME];
    int32_t  pressure_pa;
    int16_t  temperature_cdeg; /* hundredths of a degree */
    uint8_t  have_reference;   /* the ground pressure was captured */
    int32_t  reference_pa;
    int32_t  height_cm;        /* above the reference, from the barometer alone */
    uint8_t  have_gps_reference;
    int32_t  fused_cm;         /* what the aircraft actually flies on */
    uint32_t samples;
    uint32_t errors;
    uint32_t fails;            /* reads in a row with no answer */
    uint32_t baro_samples;     /* into the altitude filter */
    uint32_t gps_samples;
} ak_proto_baro_t;

typedef struct {
    char     driver[AK_PROTO_SENSOR_NAME];
    uint8_t  address;          /* the I2C address the part answered at */
    uint16_t max_mm;
    int32_t  distance_mm;      /* negative means nothing in range */
    uint32_t age_ms;           /* since that measurement, or since the last try */
    uint32_t samples;
    uint32_t out_of_range;
    uint32_t rejected;         /* impossible readings thrown away */
    uint32_t faults;           /* reads the part did not answer */
    uint32_t fails;            /* in a row */
    uint32_t land_mm;          /* what counts as reached */
    uint16_t agree_cm;         /* how far baro and range may differ */
} ak_proto_range_t;

typedef struct {
    uint8_t  ready;            /* this board has a divider at all */
    uint8_t  have_reading;     /* as opposed to a divider with nothing on it yet */
    uint8_t  state;            /* ak_battery_state_t, as the flight code numbers it */
    uint8_t  cells;
    uint16_t volts_cv;         /* the pack, in centivolts */
    uint16_t per_cell_cv;
    int16_t  pin_mv;           /* millivolts at the ADC pin; negative if none */
    uint16_t ratio_milli;      /* vbat_ratio, thousandths */
    uint8_t  rth;              /* the aircraft comes home on a low cell */
    uint16_t warn_cell_mv;
    uint16_t critical_cell_mv;
    uint32_t samples;
    uint32_t rejected;
    uint32_t returns;          /* how many times it has been called home */
} ak_proto_battery_t;

typedef struct {
    uint8_t  have_fix;         /* the receiver has produced a position */
    uint8_t  fix_type;         /* u-blox's own numbering, untranslated */
    uint8_t  fix_ok;           /* the receiver's usable bit */
    uint8_t  satellites;
    uint8_t  valid_now;        /* and it is recent, by the same age rule the console uses */
    int32_t  lat_e7;
    int32_t  lon_e7;
    int32_t  alt_msl_mm;
    int32_t  speed_mm_s;       /* ground speed */
    int32_t  course_e5;        /* degrees * 1e5 */
    uint8_t  have_home;
    int32_t  home_lat_e7;
    int32_t  home_lon_e7;
    int32_t  home_distance_m;  /* negative when there is no home or no fix */
    int32_t  home_bearing_cdeg;
    uint8_t  returning;        /* the navigator is flying it home */
    uint8_t  rth_enabled;
    uint32_t fixes;
    uint32_t dropped;          /* bytes lost in the UART receive buffer */
    uint32_t config_sends;     /* attempts to configure the receiver */
} ak_proto_gps_t;

/* One topic's answer, as a caller fills it in.
 *
 * The five bodies are a union because exactly one of them is meaningful per
 * request, and the topic byte says which - so this is not a struct a client
 * reads whole, it is a way for the callback to hand over the right one without
 * five callbacks that would each need their own null check in io_t. */
typedef struct {
    uint8_t present; /* 1 when a part is fitted and this body was filled */
    uint8_t topic;
    union {
        ak_proto_imu_t     imu;
        ak_proto_baro_t    baro;
        ak_proto_range_t   range;
        ak_proto_battery_t battery;
        ak_proto_gps_t     gps;
    } as;
} ak_proto_sensor_t;

/* The status byte on PARAM_SET, PARAM_SAVE and PARAM_DEFAULT.
 *
 * They share one vocabulary because they are the same kind of answer - "did
 * this write happen, and if not, what stopped it" - and a client that has to
 * learn three of them will get one wrong. `AK_PROTO_WRITE_REFUSED_ARMED` is
 * the one added by the write gate: it means the board is not in a position to
 * be reconfigured, which is a different fact from "the value was out of range"
 * and from "the board has nowhere to save", and a screen that showed either of
 * those for it would be telling a person to try the wrong thing. */
enum {
    AK_PROTO_WRITE_OK = 0,
    AK_PROTO_WRITE_NO_SUCH = 1,      /* no such parameter */
    AK_PROTO_WRITE_REJECTED = 2,     /* the table refused the value */
    AK_PROTO_WRITE_NO_STORAGE = 3,   /* nowhere to save, or nothing to save to */
    AK_PROTO_WRITE_REFUSED_ARMED = 5,
    /* 4 is "the board refused the write" on PARAM_SAVE and is left where it
     * is - it is the storage's own error, not a policy. Renumbering it would
     * change what an existing client reads off an existing opcode, which is
     * the one thing AK_PROTO_VERSION exists to prevent. */
    AK_PROTO_WRITE_STORAGE_ERROR = 4,
};

/* The three sources, and what a device that has none of them says. */
#define AK_PROTO_LOG_FAST  0u
#define AK_PROTO_LOG_LONG  1u
#define AK_PROTO_LOG_FLASH 2u
#define AK_PROTO_LOG_MAX   2u

/* The most a telemetry stream can be asked for. A flight controller's own loop
 * runs at a kilohertz and none of this is worth that: the aircraft does not
 * change meaningfully in 20 ms, and a stream faster than the link is a stream
 * that fills a socket buffer and then delays the config that shares it. */
#define AK_PROTO_TELEMETRY_MAX_HZ 50u

/*
 * What a board can answer, as a word HELLO carries.
 *
 * A client's problem is not "which protocol version is this" - that is one
 * byte and it is in every frame - it is "does this build answer the command I
 * am about to send". Until this word existed the only way to find out was to
 * send it and wait out a timeout, once per command, on a link that carries a
 * person's configuration.
 *
 * A bit means the command exists in this build *and* does what
 * docs/16-protocol.md says it does. It is set when that is true and not
 * before: a board that set AK_PROTO_FEATURE_OUTPUT_TEST because this file
 * defines the constant, while its dispatch had no case for it, would have a
 * client send a frame and wait for an answer that never comes - which is the
 * exact failure the word exists to prevent. Defining a constant here is how
 * the vocabulary is written down; setting it in `ak_proto_io_t::features` is
 * how a board says it speaks it, and the two are deliberately different acts.
 *
 * The word is u32 and there are 12 bits spoken for, which leaves room to grow
 * without ever having to move AK_PROTO_VERSION: an old client reads the bits
 * it knows and ignores the rest, and an old board sends a HELLO that ends
 * before the field - so a client that reads no capability word must report
 * `absent`, never `no capabilities`. Those are different claims and only one
 * of them is true of a board that predates this.
 */
#define AK_PROTO_FEATURE_PARAM_INFO (1u << 0)
#define AK_PROTO_FEATURE_PARAM_DEFAULT (1u << 1)
/* A successful set makes the board re-apply its configuration, not merely
 * change the table. True of any build whose `on_change` is wired up. */
#define AK_PROTO_FEATURE_APPLIES_ON_WRITE (1u << 2)
/* Every write path - set, save, default, output test - refuses while the
 * aircraft is armed. Not the same claim as "save is gated": a board can gate
 * one route and not the others, and one that did would be worse than either,
 * because the guard would depend on which button was pressed. */
#define AK_PROTO_FEATURE_GATES_ON_ARMED (1u << 3)
#define AK_PROTO_FEATURE_RC_CHANNELS (1u << 4)
#define AK_PROTO_FEATURE_SENSOR_INFO (1u << 5)
#define AK_PROTO_FEATURE_OUTPUT_INFO (1u << 6)
#define AK_PROTO_FEATURE_OUTPUT_TEST (1u << 7)
#define AK_PROTO_FEATURE_LOG_STREAM (1u << 8)
#define AK_PROTO_FEATURE_PREFLIGHT (1u << 9)
#define AK_PROTO_FEATURE_CALIBRATE (1u << 10)
#define AK_PROTO_FEATURE_MISSION (1u << 11)

/* What a caller has to be able to say about itself. Kept as a struct filled by
 * the caller rather than a pile of getters, so the protocol has no opinion
 * about where the numbers come from. */
typedef struct {
    uint8_t  flight_state;
    uint8_t  link_live;
    uint8_t  gps_fix_type;
    uint8_t  gps_satellites;
    int16_t  roll_ddeg;
    int16_t  pitch_ddeg;
    int16_t  yaw_ddeg;
    int32_t  lat_e7;
    int32_t  lon_e7;
    uint8_t  motor[4];
} ak_proto_status_t;

typedef struct {
    ak_params_t *params;
    /* The AK_PROTO_FEATURE_* bits this build answers for, sent in HELLO.
     *
     * A field rather than a callback, and that is the whole design: the bit
     * word is a property of the compiled firmware, not a reading of the
     * hardware, so there is nothing to ask at run time and nothing that could
     * answer differently between two calls. A callback here would invite
     * exactly the mistake this word exists to remove - a board that reports a
     * capability it is not currently in a position to exercise.
     *
     * Zero is a real value - a board with no optional command on it - and it
     * is not the same as the field being absent, which is what an old board's
     * HELLO says by ending early. */
    uint32_t features;
    /* Fills `out` with the current flight state. Required. */
    void (*status)(void *ctx, ak_proto_status_t *out);
    /* A parameter has moved: let the board act on it.
     *
     * The console has had this since it had a `set` - ak_cli.h's on_change -
     * and the protocol did not, so `set` over the wire changed the table and
     * stopped there. The table is not the aircraft: `parameters_changed()` in
     * main.c is what turns a parameter into behaviour (the airframe, the whole
     * table, the DShot timer's rate, and the receiver's protocol on both the
     * parser and the board's line), and a client that set `dshot_khz` over the
     * wire got a `params` reply reporting the new value while the timer ran the
     * old one. A reported value that the hardware does not have is the worst
     * pair available on an aircraft.
     *
     * The same signature as the console's, and for the same reason: it is the
     * same act, and the two links must not be able to disagree about when it
     * happens.
     *
     * Optional, like `save` - a device with nothing to re-apply leaves it
     * null, and the protocol changes the table exactly as it used to. */
    void (*on_change)(void);
    /* Persists the parameters. Optional - a board with no storage returns
     * negative and the protocol says so rather than pretending. */
    int (*save)(void *ctx);
    /* Whether this board will accept a configuration write *now*: nonzero for
     * yes. Optional, and null means "asked no such question" - a device with no
     * aircraft to be armed is not a device that is permanently disarmed, and
     * the two must not be confused.
     *
     * Every write route consults this - set, save, default, and the output test
     * when it lands - at the moment of the request rather than once at connect,
     * because a link that stays up across an arming is the normal case and a
     * gate checked at connect is a gate that is wrong a second later.
     *
     * It returns a *policy* answer, not a reason. The board decides what makes
     * a configuration write unsafe - main.c passes ak_flight_config_writable(),
     * the same predicate the console's `save` and the airborne routes use - and
     * the protocol makes no attempt to infer it from `status`, which reports
     * the flight state for display and is not a gate.
     *
     * This is what AK_PROTO_FEATURE_GATES_ON_ARMED names. A board that sets
     * that bit while leaving this null would be advertising a guard it does not
     * have, which is why main.c sets the two together or not at all. */
    int (*writable)(void *ctx);
    /* The receiver, as RC_CHANNELS asks for it. Optional in the same way
     * `log_count` is, and null means "this board has no receiver input" - a
     * fact about the board that the reply says outright rather than expressing
     * as a frame of zeros, because a frame of zeros is also what a receiver
     * that is plugged in and silent looks like.
     *
     * A callback and not a field, unlike `features`, for the reason the two are
     * different: the capability word is a property of the compiled firmware and
     * cannot change while it runs, and this changes every time a handset moves.
     *
     * Filled per request, with the counters included, so that a client polling
     * this opcode needs no second command to find out whether the receiver is
     * healthy - the whole point of polling something at the rate a person can
     * move a stick. */
    void (*rc_state)(void *ctx, ak_proto_rc_t *out);
    /* One sensor, as SENSOR_INFO asks for it.
     *
     * Optional in the same way `rc_state` is, and null means "this board
     * reports no sensors" - which every topic answers AK_PROTO_SENSOR_NO_SUCH
     * for. That is deliberately the same answer as an out-of-range topic, and
     * deliberately not the same as a topic that exists with nothing fitted:
     * the first is a fact about the build and the second is a fact about the
     * aircraft, and only the second is a reason to go and look at a socket.
     *
     * Called only for topics this header defines, so the callback's own default
     * case is unreachable from the wire and a topic added here without an arm
     * there is a build-time gap rather than a client's misread frame. It is
     * still asked to leave `present` clear rather than to trust that, because
     * the cost of being wrong is a client believing a zeroed struct.
     *
     * A callback and not a field, for `rc_state`'s reason: half of these
     * change while the board runs. Filled per request rather than cached,
     * because a cache here would be a second copy of the driver's state and
     * the whole value of this opcode is that it is the driver's own numbers. */
    void (*sensor_state)(void *ctx, uint8_t topic, ak_proto_sensor_t *out);
    /* The logs, if there are any: how many records `source` holds, and the
     * i-th oldest of that source as the firmware's own record bytes. Optional
     * in the same way - and a device with one log answers for one source and
     * says "no" for the others, which is what the reply's status is for.
     *
     * `log_count` returns -1 for a source this device does not have, which is
     * how a client tells "no such log" from "an empty one": the second answer
     * is zero, and both are worth being able to say. */
    int32_t (*log_count)(void *ctx, uint8_t source);
    unsigned (*log_record)(void *ctx, uint8_t source, uint16_t index,
                           uint8_t *out, unsigned capacity);
    void *ctx;
} ak_proto_io_t;

typedef struct {
    uint8_t  state;
    uint8_t  buffer[AK_PROTO_FRAME_MAX];
    uint16_t held;
    uint16_t expected;
    uint16_t crc_received;
    uint32_t last_byte_ms;

    /* Per link, because the protocol parser is: one of these is the console
     * and one is the network, and they subscribe separately. */
    uint8_t  telemetry_hz;
    /* Whether this link can push frames at a client without being asked. The
     * console cannot and should not: it is a shared, human-facing wire, and a
     * stream on it would arrive between a person's keystrokes. A network link
     * can. The subscribe command answers 0 on a link that cannot - a rate a
     * client will never receive is worse than being told no, and "the answer is
     * the rate that will actually be sent" is what that command has always
     * promised. */
    uint8_t  can_stream;
    /* And per link for the same reason: two clients asking about different
     * logs should not move each other's pointer. */
    uint8_t  log_source;

    uint32_t frames;
    uint32_t responses;
    uint32_t bad_crc;
    uint32_t bad_length;
    uint32_t unknown_commands;
    uint32_t bytes;
} ak_proto_t;

void ak_proto_init(ak_proto_t *proto);

/* A telemetry frame, built to be pushed rather than sent in reply to anything:
 * the command byte has no response bit, which is how a client tells a stream
 * from its own answers on the same connection. Returns the frame length, or 0
 * if the buffer is too small. */
unsigned ak_proto_telemetry_frame(const ak_proto_io_t *io, uint32_t now_ms,
                                  uint8_t *out, unsigned capacity);

/* One byte. Returns the length of a response written into `response` when this
 * byte completed a frame that has one, and 0 otherwise. */
unsigned ak_proto_feed(ak_proto_t *proto, const ak_proto_io_t *io, uint8_t byte,
                       uint32_t now_ms, uint8_t *response, unsigned capacity);

uint16_t ak_proto_crc16(const uint8_t *data, unsigned length);

/* True when the parser would read the *next* byte as the start of a frame:
 * between frames, and not part way through one that has gone quiet.
 *
 * `state == STATE_SYNC1` alone does not answer that, and taking it for the
 * answer is a bug with two halves. A frame that stops part way is abandoned
 * only when the next byte arrives and the gap since the last one is longer than
 * AK_PROTO_GAP_MS - so in between, the parser reports non-idle while being, in
 * every way that matters, finished with the frame. A caller that trusts the raw
 * state sends that next byte down the protocol's path, where it is discarded:
 * the first character of a command typed after a half-frame disappears, and
 * "version" arrives as "ersion".
 *
 * The gap is the same test ak_proto_feed makes before it abandons a frame, so
 * the two cannot disagree about whose byte this is. That matters more than it
 * sounds: if the caller decides the byte is the console's, ak_proto_feed is
 * never called for it, so the parser's own idea of the gap does not advance -
 * the two would drift apart on exactly the byte they disagree about. */
int ak_proto_idle_at(const ak_proto_t *proto, uint32_t now_ms);

void ak_proto_report(const ak_proto_t *proto, ak_printf_fn out);

#endif /* AK_CORE_AK_PROTO_H */
