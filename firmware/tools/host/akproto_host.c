#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ak_params.h"
#include "ak_proto.h"
#include "ak_log.h"

static ak_param_t items[8];
static ak_params_t params;
static float rate_gain = 0.25f;
static uint32_t waypoint_count = 2;
static char callsign[32] = "bench";
static int writable = 1;

static void status_cb(void *ctx, ak_proto_status_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->flight_state = 1;
    out->link_live = 1;
    out->gps_fix_type = 3;
    out->gps_satellites = 14;
    out->roll_ddeg = 125;
    out->pitch_ddeg = -250;
    out->yaw_ddeg = 900;
    out->lat_e7 = 473977000;
    out->lon_e7 = -1227654321;
    out->motor[0] = 20;
    out->motor[1] = 30;
    out->motor[2] = 40;
    out->motor[3] = 50;
}

static int save_cb(void *ctx)
{
    (void)ctx;
    return 0;
}

static int writable_cb(void *ctx)
{
    (void)ctx;
    return writable;
}

static void rc_cb(void *ctx, ak_proto_rc_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->flags = AK_PROTO_RC_LINK | AK_PROTO_RC_DECODED | AK_PROTO_RC_TELEMETRY;
    out->protocol = 0;
    out->count = 8;
    out->switches = AK_PROTO_RC_ARM_ON | AK_PROTO_RC_ANGLE;
    for (unsigned i = 0; i < out->count; i++) {
        out->raw[i] = (uint16_t)(992u + i);
    }
    out->sticks[0] = 100;
    out->sticks[1] = -200;
    out->sticks[2] = 0;
    out->sticks[3] = 500;
    out->frames = 123;
}

static void sensor_cb(void *ctx, uint8_t topic, ak_proto_sensor_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->topic = topic;
    if (topic == AK_PROTO_SENSOR_BATTERY) {
        out->present = 1;
        out->as.battery.state = 1;
        out->as.battery.cells = 3;
        out->as.battery.volts_cv = 1155;
        out->as.battery.per_cell_cv = 385;
    }
}

static int32_t log_count_cb(void *ctx, uint8_t source)
{
    (void)ctx;
    return source == 0 ? 2 : -1;
}

static unsigned log_record_cb(void *ctx, uint8_t source, uint16_t index,
                              uint8_t *out, unsigned capacity)
{
    (void)ctx;
    if (source != 0 || index >= 2 || capacity < AK_LOG_WIRE_BYTES) {
        return 0;
    }
    memset(out, 0, AK_LOG_WIRE_BYTES);
    out[0] = (uint8_t)index;
    return AK_LOG_WIRE_BYTES;
}

static unsigned outputs_cb(void *ctx, ak_proto_output_t *out, unsigned capacity)
{
    (void)ctx;
    if (capacity < 2) {
        return 2;
    }
    memset(out, 0, sizeof(ak_proto_output_t) * 2u);
    out[0].kind = AK_PROTO_OUTPUT_MOTOR;
    out[0].index = 0;
    out[1].kind = AK_PROTO_OUTPUT_SERVO;
    out[1].index = 0;
    out[1].travel_us = 400;
    return 2;
}

static int output_test_cb(void *ctx, uint8_t op, uint8_t kind, uint8_t index,
                          uint8_t level_pct, ak_proto_output_test_t *out)
{
    (void)ctx;
    if (!writable) {
        return AK_PROTO_OUTPUT_TEST_ARMED;
    }
    if (kind > AK_PROTO_OUTPUT_SERVO || index > 0) {
        return AK_PROTO_OUTPUT_TEST_NO_OUTPUT;
    }
    out->level_pct = level_pct > AK_PROTO_OUTPUT_TEST_MAX_PCT
                         ? AK_PROTO_OUTPUT_TEST_MAX_PCT
                         : level_pct;
    out->remaining_ms = op == AK_PROTO_OUTPUT_TEST_STOP ? 0 : 500;
    return op == AK_PROTO_OUTPUT_TEST_STOP ? AK_PROTO_OUTPUT_TEST_STOPPED
                                           : AK_PROTO_OUTPUT_TEST_OK;
}

static const ak_preflight_line_t checklist[] = {
    {"receiver", AK_PROTO_PREFLIGHT_VERDICT_PASS, "frames are arriving"},
};

static int preflight_cb(void *ctx, unsigned first,
                        const ak_preflight_line_t **lines, unsigned *count)
{
    (void)ctx;
    (void)first;
    *lines = checklist;
    *count = 1;
    return 1;
}

static int mission_cb(void *ctx, uint8_t op, ak_proto_mission_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->count = 2;
    out->index = AK_PROTO_MISSION_NO_INDEX;
    out->hold_alt_mm = 10000;
    if (op == AK_PROTO_MISSION_START) {
        out->requested = 1;
        out->started = 1;
    }
    return AK_PROTO_MISSION_OK;
}

static int calibrate_cb(void *ctx, uint8_t verb, uint8_t face, uint32_t mv,
                        ak_proto_calibration_t *out)
{
    (void)ctx;
    (void)face;
    (void)mv;
    memset(out, 0, sizeof *out);
    out->verb = verb;
    out->step = AK_PROTO_CALIBRATE_NO_STEP;
    return AK_PROTO_CALIBRATE_OK;
}

static int perf_cb(void *ctx, ak_proto_perf_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->loops = 1000;
    out->samples = 990;
    out->nominal_us = 1000;
    out->period_last_us = 1000;
    out->period_min_us = 990;
    out->period_max_us = 1100;
    out->late = 4;
    out->jitter_p50_us = 5;
    out->jitter_p99_us = 25;
    out->jitter_max_us = 80;
    out->load_permille = 125;
    return 1;
}

static int motor_cb(void *ctx, ak_proto_motor_telemetry_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->count = 1;
    out->poles = 14;
    out->motor[0].flags = AK_PROTO_MOTOR_FLAG_MEASURED |
                          AK_PROTO_MOTOR_FLAG_RPM |
                          AK_PROTO_MOTOR_FLAG_VOLTAGE |
                          AK_PROTO_MOTOR_FLAG_CURRENT;
    out->motor[0].erpm = 10000;
    out->motor[0].rpm = 714;
    out->motor[0].millivolts = 12600;
    out->motor[0].milliamps = 500;
    out->motor[0].packets = 100;
    return 1;
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--locked") == 0) {
        writable = 0;
    }

    ak_params_init(&params, items, sizeof(items) / sizeof(items[0]));
    unsigned n = 0;
    n = ak_params_add_float(items, n, "rate_kp", "rate loop gain",
                            &rate_gain, 3, 0.0f, 3.0f, AK_PARAM_GROUP_RATES);
    n = ak_params_add_u32(items, n, "wp_count", "waypoints to fly",
                          &waypoint_count, 0u, 4u, AK_PARAM_GROUP_NAVIGATION);
    n = ak_params_add_text(items, n, "callsign", "aircraft callsign",
                           callsign, sizeof(callsign), 0u, "bench",
                           AK_PARAM_GROUP_NETWORK);
    params.count = n;

    ak_proto_io_t io;
    memset(&io, 0, sizeof io);
    io.params = &params;
    io.features = (1u << 14) - 1u;
    io.status = status_cb;
    io.save = save_cb;
    io.writable = writable_cb;
    io.rc_state = rc_cb;
    io.sensor_state = sensor_cb;
    io.log_count = log_count_cb;
    io.log_record = log_record_cb;
    io.outputs = outputs_cb;
    io.output_test = output_test_cb;
    io.preflight = preflight_cb;
    io.mission = mission_cb;
    io.calibrate = calibrate_cb;
    io.perf = perf_cb;
    io.motor_telemetry = motor_cb;

    ak_proto_t proto;
    ak_proto_init(&proto);
    proto.can_stream = 1;
    setvbuf(stdout, 0, _IONBF, 0);

    uint8_t byte;
    uint8_t response[AK_PROTO_FRAME_MAX];
    uint32_t now_ms = 0;
    while (read(STDIN_FILENO, &byte, 1) == 1) {
        unsigned length = ak_proto_feed(&proto, &io, byte, now_ms++,
                                        response, sizeof response);
        if (length > 0) {
            if (fwrite(response, 1, length, stdout) != length) {
                return 1;
            }
        }
    }
    return 0;
}
