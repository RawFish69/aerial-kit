/*
 * The control core as a library - see ak_control.h for what this is for and
 * why it is shaped like this.
 *
 * The whole file is glue. There is no control law in it, no filter and no
 * mixer: those are `ak_flight_step` and the objects it is linked against, which
 * are the same objects the MCU image is built from. What is here is the four
 * things a caller outside the firmware needs that the firmware does not have -
 * a clock it owns, a configuration handed in as text, a receiver it can
 * synthesise, and a description of its own structs.
 */

#include "ak_control.h"

#include <stddef.h>
#include <string.h>

#include "ak_estimator.h"
#include "ak_math.h"
#include "ak_mixer.h"
#include "ak_output.h"
#include "ak_text.h"
#include "ak_version.h"

static ak_flight_t     flight;
static ak_param_t      items[AK_PARAMS_MAX];
static ak_params_t     params;
static ak_servo_trim_t trims[AK_MAX_SERVOS];

static int      configured;
static uint32_t output_sequence;

/*
 * The last frame the core produced, so that a refused step can still answer
 * "what is it doing" with the truth - the outputs it is still holding - rather
 * than leaving the caller's buffer as it found it. A caller that ignores the
 * result of a step must not be handed uninitialised memory it cannot tell from
 * a fresh frame; that is the failure mode `akc_step`'s "written even when the
 * result is not OK" sentence exists to rule out, and neither zero-filling the
 * buffer nor leaving a stale stack one achieves it.
 */
static akc_outputs_t last_outputs;

/* --- the layout, so a binding can check its own idea of it ---------------- */

#define LAYOUT_ROW(which, type, field) \
    { #field, (uint32_t)offsetof(type, field), (uint32_t)sizeof(((type *)0)->field) }

/*
 * `fields` is written as 0 here and filled in by akc_layout() by counting rows
 * to the terminator. A hand-written count is the one number in this table the
 * compiler does not check, and it fails in the direction that hides: too low
 * and a binding compares the first N fields, agrees with them, and never looks
 * at the rest. Counting the rows it can see is the only version of this number
 * that cannot disagree with the rows.
 */
static const akc_layout_t layouts[] = {
    { "imu", sizeof(akc_imu_t), 0u, {
        LAYOUT_ROW(imu, akc_imu_t, t_ms),
        LAYOUT_ROW(imu, akc_imu_t, sequence),
        LAYOUT_ROW(imu, akc_imu_t, gyro),
        LAYOUT_ROW(imu, akc_imu_t, accel),
        LAYOUT_ROW(imu, akc_imu_t, valid),
        { 0, 0u, 0u } } },
    { "baro", sizeof(akc_baro_t), 0u, {
        LAYOUT_ROW(baro, akc_baro_t, t_ms),
        LAYOUT_ROW(baro, akc_baro_t, sequence),
        LAYOUT_ROW(baro, akc_baro_t, pressure_pa),
        LAYOUT_ROW(baro, akc_baro_t, temperature_c),
        LAYOUT_ROW(baro, akc_baro_t, valid),
        { 0, 0u, 0u } } },
    { "gps", sizeof(akc_gps_t), 0u, {
        LAYOUT_ROW(gps, akc_gps_t, t_ms),
        LAYOUT_ROW(gps, akc_gps_t, sequence),
        LAYOUT_ROW(gps, akc_gps_t, lat_e7),
        LAYOUT_ROW(gps, akc_gps_t, lon_e7),
        LAYOUT_ROW(gps, akc_gps_t, alt_msl_mm),
        LAYOUT_ROW(gps, akc_gps_t, speed_mm_s),
        LAYOUT_ROW(gps, akc_gps_t, course_e5),
        LAYOUT_ROW(gps, akc_gps_t, fix_type),
        LAYOUT_ROW(gps, akc_gps_t, satellites),
        LAYOUT_ROW(gps, akc_gps_t, valid),
        { 0, 0u, 0u } } },
    { "samples", sizeof(akc_samples_t), 0u, {
        LAYOUT_ROW(samples, akc_samples_t, t_ms),
        LAYOUT_ROW(samples, akc_samples_t, sequence),
        LAYOUT_ROW(samples, akc_samples_t, imu),
        LAYOUT_ROW(samples, akc_samples_t, baro),
        LAYOUT_ROW(samples, akc_samples_t, gps),
        { 0, 0u, 0u } } },
    { "command", sizeof(akc_command_t), 0u, {
        LAYOUT_ROW(command, akc_command_t, t_ms),
        LAYOUT_ROW(command, akc_command_t, roll),
        LAYOUT_ROW(command, akc_command_t, pitch),
        LAYOUT_ROW(command, akc_command_t, yaw),
        LAYOUT_ROW(command, akc_command_t, throttle),
        LAYOUT_ROW(command, akc_command_t, angle_mode),
        LAYOUT_ROW(command, akc_command_t, arm_request),
        { 0, 0u, 0u } } },
    { "outputs", sizeof(akc_outputs_t), 0u, {
        LAYOUT_ROW(outputs, akc_outputs_t, t_ms),
        LAYOUT_ROW(outputs, akc_outputs_t, sequence),
        LAYOUT_ROW(outputs, akc_outputs_t, motor),
        LAYOUT_ROW(outputs, akc_outputs_t, servo),
        LAYOUT_ROW(outputs, akc_outputs_t, dshot),
        LAYOUT_ROW(outputs, akc_outputs_t, servo_us),
        { 0, 0u, 0u } } },
    /* Every field of akc_state_t, in declaration order. Complete on purpose:
     * a table that stopped early would let a field move in the part it never
     * described, and the check would call that agreement. */
    { "state", sizeof(akc_state_t), 0u, {
        LAYOUT_ROW(state, akc_state_t, now_ms),
        LAYOUT_ROW(state, akc_state_t, steps),
        LAYOUT_ROW(state, akc_state_t, state),
        LAYOUT_ROW(state, akc_state_t, link_live),
        LAYOUT_ROW(state, akc_state_t, converged),
        LAYOUT_ROW(state, akc_state_t, arm_block),
        LAYOUT_ROW(state, akc_state_t, arm_detail),
        LAYOUT_ROW(state, akc_state_t, q_wxyz),
        LAYOUT_ROW(state, akc_state_t, roll),
        LAYOUT_ROW(state, akc_state_t, pitch),
        LAYOUT_ROW(state, akc_state_t, yaw),
        LAYOUT_ROW(state, akc_state_t, gyro),
        LAYOUT_ROW(state, akc_state_t, rate_setpoint),
        LAYOUT_ROW(state, akc_state_t, torque),
        LAYOUT_ROW(state, akc_state_t, cmd_roll),
        LAYOUT_ROW(state, akc_state_t, cmd_pitch),
        LAYOUT_ROW(state, akc_state_t, cmd_yaw),
        LAYOUT_ROW(state, akc_state_t, cmd_throttle),
        LAYOUT_ROW(state, akc_state_t, cmd_angle_mode),
        LAYOUT_ROW(state, akc_state_t, cmd_arm_request),
        LAYOUT_ROW(state, akc_state_t, imu_age_ms),
        LAYOUT_ROW(state, akc_state_t, baro_age_ms),
        LAYOUT_ROW(state, akc_state_t, gps_age_ms),
        LAYOUT_ROW(state, akc_state_t, imu_stale),
        LAYOUT_ROW(state, akc_state_t, baro_stale),
        LAYOUT_ROW(state, akc_state_t, gps_stale),
        LAYOUT_ROW(state, akc_state_t, command_expired),
        LAYOUT_ROW(state, akc_state_t, timing_gap_steps),
        LAYOUT_ROW(state, akc_state_t, timing_catchup_steps),
        LAYOUT_ROW(state, akc_state_t, timing_dropped_ms),
        LAYOUT_ROW(state, akc_state_t, timing_duplicates),
        LAYOUT_ROW(state, akc_state_t, timing_long_loops),
        LAYOUT_ROW(state, akc_state_t, timing_max_loop_ms),
        LAYOUT_ROW(state, akc_state_t, config_hash),
        LAYOUT_ROW(state, akc_state_t, mix_needed_motors),
        LAYOUT_ROW(state, akc_state_t, mix_needed_servos),
        LAYOUT_ROW(state, akc_state_t, board_motors),
        LAYOUT_ROW(state, akc_state_t, board_servos),
        { 0, 0u, 0u } } },
};

#define LAYOUT_COUNT (sizeof layouts / sizeof layouts[0])

int akc_layout(const char *which, akc_layout_t *out)
{
    if (which == 0 || out == 0) {
        return -1;
    }
    for (unsigned i = 0; i < LAYOUT_COUNT; i++) {
        if (strcmp(layouts[i].name, which) != 0) {
            continue;
        }
        *out = layouts[i];
        out->fields = 0u;
        while (out->fields < AKC_LAYOUT_MAX && out->field[out->fields].name != 0) {
            out->fields++;
        }
        /* The terminator is the last row the array can hold only when the
         * description ran out of room, in which case what came back is a
         * prefix and must not be read as the whole struct. */
        if (out->fields == AKC_LAYOUT_MAX) {
            return -1;
        }
        return 0;
    }
    return -1;
}

/* --- identity ------------------------------------------------------------ */

uint32_t akc_abi_version(void) { return AKC_ABI_VERSION; }
const char *akc_product(void) { return AK_PRODUCT_STR; }
const char *akc_board(void) { return AK_BOARD_STR; }
const char *akc_revision(void) { return AK_REV_STR; }
const char *akc_built(void) { return AK_STAMP_STR; }

/* --- configuration ------------------------------------------------------- */

uint32_t akc_config_hash(void)
{
    return configured ? ak_params_hash(&params) : 0u;
}

unsigned akc_config_text(char *buf, unsigned len)
{
    if (buf == 0 || len == 0u || !configured) {
        return 0u;
    }
    return ak_params_serialize(&params, buf, len);
}

int akc_set_board_outputs(unsigned motors, unsigned servos)
{
    /* A board cannot drive more than the arrays hold, and saying it does would
     * move the failure from here - where it is a returned error - to the
     * mixer, where it is a write past the end of `out`. */
    if (motors > AK_MAX_MOTORS || servos > AK_MAX_SERVOS) {
        return -1;
    }
    ak_flight_set_board_outputs(&flight, motors, servos);
    return 0;
}

/*
 * The effective table as text, kept so that a reset can put it back.
 *
 * `ak_flight_init` calls `ak_flight_default_config`, so re-initialising the
 * core throws the configuration away - and a reset that did that while
 * `configured` stayed 1 would hand back an aircraft flying defaults under the
 * name of the one that was asked for. That is the failure C8 exists to make
 * visible, and the identity is what makes it visible: `config_hash` would
 * change across a reset that nothing else reported.
 *
 * What is stored is the *serialised table* rather than the text the caller
 * passed. They load back to the same table by definition - that is what
 * serialise and deserialise are for - and this one is bounded by
 * `AK_PARAMS_TEXT_MAX`, which the caller's is not.
 */
static char     saved_text[AK_PARAMS_TEXT_MAX];
static int      have_saved;

static int configure_from(const char *text, char *msg, unsigned msg_len)
{
    /* Init first, and with the quad's mix: `ak_flight_param_table` reads the
     * fields as they are *at this moment* to record what `defaults` restores,
     * so anything that writes them before this line becomes the default. The
     * mixer chosen here is replaced two calls later by whichever airframe the
     * text asks for; it is a starting point, not an answer. */
    ak_flight_init(&flight, &ak_mixer_quad_x);
    ak_servo_trim_defaults(trims, AK_MAX_SERVOS);

    unsigned count = ak_flight_param_table(&flight, items, AK_PARAMS_MAX);

    if (count == 0u) {
        if (msg != 0 && msg_len > 0u) {
            (void)ak_strlcpy(msg, "the parameter table did not fit", msg_len);
        }
        configured = 0;
        return -1;
    }

    ak_params_init(&params, items, count);

    if (ak_params_deserialize(&params, text, msg, msg_len) != 0) {
        configured = 0;
        return -1;
    }

    /* The airframe is a parameter, so the mixer follows the text rather than
     * an argument: one way to say which aircraft this is. So are the rate
     * loop's gains, which is why this is `apply_config` and not
     * `apply_airframe`: the same call a board makes after a `set`. */
    ak_flight_apply_config(&flight);

    /* A table too big for the record leaves the configuration standing and
     * unrestorable. Saying so here means the reset below refuses rather than
     * flying defaults under the name of the aircraft that was asked for. */
    have_saved = ak_params_store(&params, saved_text, sizeof saved_text) >= 0;

    configured = 1;
    output_sequence = 0u;
    last_outputs = (akc_outputs_t){0};
    return 0;
}

int akc_config(const char *text, char *msg, unsigned msg_len)
{
    if (text == 0) {
        return -1;
    }
    if (msg != 0 && msg_len > 0u) {
        msg[0] = '\0';
    }
    return configure_from(text, msg, msg_len);
}

void akc_reset(void)
{
    if (have_saved) {
        (void)configure_from(saved_text, 0, 0u);
        return;
    }
    /* No restorable configuration: the core goes back to a just-initialised
     * state and stops claiming to be configured, because it no longer is. */
    ak_flight_init(&flight, &ak_mixer_quad_x);
    ak_servo_trim_defaults(trims, AK_MAX_SERVOS);
    configured = 0;
    output_sequence = 0u;
    last_outputs = (akc_outputs_t){0};
}

/* --- the receiver the caller does not have ------------------------------- */

/*
 * C6 fixes the command as the stick shape, and the firmware's own front door
 * takes receiver *counts* and decodes them. Rather than add a second way into
 * the flight core - which would be a second decoder, and a second decoder is a
 * second answer to what a stick means - this synthesises counts that the
 * firmware's own `ak_rc_decode` turns back into the sticks that were asked for.
 *
 * The inversion is exact for the three centred axes and for the throttle, and
 * it round-trips through `centred()`'s deadband: a stick inside the deadband
 * decodes as zero, which is the firmware's rule and not this file's to change.
 * What keeps that honest is `akc_step` comparing what came back out of the
 * decoder with what went in, and reporting the difference in the state.
 */
static uint16_t counts_for(float value, const ak_rc_config_t *cfg)
{
    float raw;

    if (value >= 0.0f) {
        raw = (float)cfg->mid + value * (float)(cfg->max - cfg->mid);
    } else {
        raw = (float)cfg->mid + value * (float)(cfg->mid - cfg->min);
    }
    if (raw < (float)cfg->min) {
        raw = (float)cfg->min;
    }
    if (raw > (float)cfg->max) {
        raw = (float)cfg->max;
    }
    return (uint16_t)(raw + 0.5f);
}

static uint16_t switch_for(int on, uint32_t threshold)
{
    /* One count either side, rather than the threshold itself: the decoder's
     * test is a strict `>`, so the threshold's own value is "off". */
    return (uint16_t)(on ? threshold + 1u : threshold - 1u);
}

static void receiver_from(const akc_command_t *command, ak_rc_input_t *rc)
{
    const ak_rc_config_t *cfg = &flight.rc_cfg;
    float span = (float)(cfg->max - cfg->min);

    rc->channel[AK_RC_ROLL] = counts_for(command->roll, cfg);
    rc->channel[AK_RC_PITCH] = counts_for(command->pitch, cfg);
    rc->channel[AK_RC_YAW] = counts_for(command->yaw, cfg);
    rc->channel[AK_RC_THROTTLE] =
        (uint16_t)((float)cfg->min + ak_clampf(command->throttle, 0.0f, 1.0f) * span);
    rc->channel[AK_RC_MODE] = switch_for(command->angle_mode, cfg->mode_threshold);
    rc->channel[AK_RC_ARM] = switch_for(command->arm_request, cfg->arm_threshold);
    rc->last_update_ms = command->t_ms;
    rc->valid = 1;
}

/* --- one step ------------------------------------------------------------ */

/*
 * The last step's inputs, kept so that `akc_state` answers about the step that
 * happened rather than about a default: an age of zero out of a function that
 * was never given a sample is the kind of number that reads as "fresh".
 */
static akc_samples_t last_samples;
static uint32_t      last_now_ms;
static int           last_expired;
static int           have_last;

static void fill_state(akc_state_t *state)
{
    const ak_estimator_t *est = &flight.est;
    const akc_samples_t *samples = &last_samples;
    uint32_t now_ms = last_now_ms;

    *state = (akc_state_t){0};

    state->now_ms = now_ms;
    state->steps = flight.steps;
    state->state = (int)flight.state;
    state->link_live = flight.link_live;
    state->converged = est->converged;
    state->arm_block = (int)ak_flight_arm_check(&flight, &flight.cmd, &state->arm_detail);

    for (int i = 0; i < 4; i++) {
        state->q_wxyz[i] = est->q[i];
    }
    state->roll = est->roll;
    state->pitch = est->pitch;
    state->yaw = est->yaw;

    for (int i = 0; i < 3; i++) {
        state->gyro[i] = flight.gyro[i];
        state->rate_setpoint[i] = flight.rate_setpoint[i];
        state->torque[i] = flight.torque[i];
    }

    /* What the receiver decoded to, which is the round trip a caller can check
     * against the sticks it asked for: the command went in as counts the
     * firmware's own decoder turned back into sticks, and these are its
     * answer, not this file's. */
    state->cmd_roll = flight.cmd.roll;
    state->cmd_pitch = flight.cmd.pitch;
    state->cmd_yaw = flight.cmd.yaw;
    state->cmd_throttle = flight.cmd.throttle;
    state->cmd_angle_mode = flight.cmd.angle_mode;
    state->cmd_arm_request = flight.cmd.arm_request;

    if (have_last) {
        state->imu_age_ms = now_ms - samples->imu.t_ms;
        state->baro_age_ms = now_ms - samples->baro.t_ms;
        state->gps_age_ms = now_ms - samples->gps.t_ms;
        /* A sensor that was never present has no age: reporting one would make
         * "not fitted" and "not answering" the same number. */
        state->imu_stale = samples->imu.t_ms != 0u &&
                           (now_ms - samples->imu.t_ms) > AKC_SAMPLE_MAX_AGE_MS;
        state->baro_stale = samples->baro.t_ms != 0u &&
                            (now_ms - samples->baro.t_ms) > AKC_SAMPLE_MAX_AGE_MS;
        state->gps_stale = samples->gps.t_ms != 0u &&
                           (now_ms - samples->gps.t_ms) > AKC_SAMPLE_MAX_AGE_MS;
    }
    state->command_expired = last_expired;

    state->timing_gap_steps = flight.timing.gap_steps;
    state->timing_catchup_steps = flight.timing.catchup_steps;
    state->timing_dropped_ms = flight.timing.dropped_ms;
    state->timing_duplicates = flight.timing.duplicates;
    state->timing_long_loops = flight.timing.long_loops;
    state->timing_max_loop_ms = flight.timing.max_loop_ms;

    state->config_hash = akc_config_hash();
    state->mix_needed_motors = flight.needed_motors;
    state->mix_needed_servos = flight.needed_servos;
    state->board_motors = flight.board_motors;
    state->board_servos = flight.board_servos;
}

int akc_step(const akc_samples_t *samples, const akc_command_t *command,
             uint32_t now_ms, akc_outputs_t *outputs)
{
    ak_imu_sample_t imu;
    ak_rc_input_t   rc;
    akc_command_t   used;
    ak_output_frame_t frame;
    int result = AKC_OK;

    if (samples == 0 || command == 0) {
        if (outputs != 0) {
            *outputs = last_outputs;
        }
        return AKC_BAD_ARGUMENT;
    }
    if (!configured) {
        if (outputs != 0) {
            *outputs = last_outputs;
        }
        return AKC_NOT_CONFIGURED;
    }

    /*
     * The age bound, before anything is flown. A frame the caller took too long
     * to deliver is refused rather than integrated: the alternative is a core
     * that flies on a number whose timestamp it can see and chose to ignore,
     * which is the failure C5 exists to make impossible.
     */
    if ((now_ms - samples->t_ms) > AKC_SAMPLE_MAX_AGE_MS) {
        if (outputs != 0) {
            *outputs = last_outputs;
        }
        return AKC_SAMPLE_STALE;
    }
    if (!samples->imu.valid) {
        /* Not refused: the firmware has a whole failsafe path for an inertial
         * sensor that has stopped answering, and this is how a caller reaches
         * it. Reported, and flown. */
        result = AKC_NO_IMU;
    }

    /*
     * C6's expiry. The firmware has no TTL on a command value - a consumed
     * command is consumed - because on a board the *link* is what expires, and
     * `rc_timeout_ms` is the rule. Here there is no link: the caller is the
     * link, so the caller's staleness is the link's, and the boundary is the
     * right place for the rule rather than a second one inside the core.
     */
    used = *command;
    last_expired = 0;
    /* Set before the expiry test so that a refused command is the one the state
     * reports on rather than the one before it. */
    if ((now_ms - command->t_ms) > AKC_COMMAND_MAX_AGE_MS) {
        used.roll = 0.0f;
        used.pitch = 0.0f;
        used.yaw = 0.0f;
        used.throttle = 0.0f;
        last_expired = 1;
        result = AKC_COMMAND_EXPIRED;
    }

    last_samples = *samples;
    last_now_ms = now_ms;
    have_last = 1;

    receiver_from(&used, &rc);

    imu = (ak_imu_sample_t){ .time_ms = samples->imu.t_ms,
                             .valid = samples->imu.valid };
    for (int i = 0; i < 3; i++) {
        imu.gyro[i] = samples->imu.gyro[i];
        imu.accel[i] = samples->imu.accel[i];
    }

    ak_flight_step(&flight, &imu, &rc, now_ms);

    /* Recorded whether or not the caller asked for it: the next refused step
     * has to be able to hand back the frame the core is still holding, and a
     * caller that never asks for outputs would otherwise have a core whose
     * "last outputs" were whatever the step before its first one left. */
    {
        const ak_outputs_t *out = ak_flight_outputs(&flight);

        last_outputs = (akc_outputs_t){0};
        last_outputs.t_ms = now_ms;
        /* Only a step that flew gets a new sequence number. A refused step
         * reports the frame it is still holding, and a number that advanced
         * there would say a new frame had been produced. */
        last_outputs.sequence = output_sequence++;
        for (int i = 0; i < AK_MAX_MOTORS; i++) {
            last_outputs.motor[i] = out->motor[i];
        }
        for (int i = 0; i < AK_MAX_SERVOS; i++) {
            last_outputs.servo[i] = out->servo[i];
        }
        ak_output_encode(out, trims, 0, &frame);
        for (int i = 0; i < AK_MAX_MOTORS; i++) {
            last_outputs.dshot[i] = frame.dshot[i];
        }
        for (int i = 0; i < AK_MAX_SERVOS; i++) {
            last_outputs.servo_us[i] = frame.servo_us[i];
        }
    }

    if (outputs != 0) {
        *outputs = last_outputs;
    }

    return result;
}

int akc_state(akc_state_t *state)
{
    if (state == 0) {
        return -1;
    }
    if (!configured) {
        return -1;
    }
    fill_state(state);
    return 0;
}
