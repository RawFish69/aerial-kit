/*
 * replay - run the flight core over a recorded input file.
 *
 *   build-host/aerialkit-replay quad-x|elevon-wing < trace.csv > outputs.csv
 *
 * Input, one row per sample, header optional:
 *
 *   t_ms,gyro_x,gyro_y,gyro_z,ax,ay,az,roll,pitch,yaw,throttle,arm,mode
 *
 * which is deliberately the shape a blackbox log will have, so that when the
 * blackbox exists, "replay the flight" is this program and not a new one.
 * Output is one row per sample:
 *
 *   t_ms,m0,m1,m2,m3,s0,s1,state
 *
 * Nothing here is on the aircraft: it is the flight core in a process, which is
 * the point of keeping that core free of I/O.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ak_flight.h"
#include "ak_mixer.h"

static int parse_row(char *line, float fields[13], uint32_t *time_ms)
{
    char *cursor = line;
    for (int i = 0; i < 13; i++) {
        char *end = NULL;
        float value = strtof(cursor, &end);
        if (end == cursor) {
            return 0;
        }
        fields[i] = value;
        cursor = end;
        while (*cursor == ',' || *cursor == ' ' || *cursor == '\t') {
            cursor++;
        }
    }
    if (fields[0] < 0.0f) {
        return 0;
    }
    *time_ms = (uint32_t)fields[0];
    return 1;
}

/* Sticks are -1..1 in the file and receiver counts on the wire. */
static uint16_t stick_to_counts(const ak_rc_config_t *cfg, float stick)
{
    float span = (float)(stick >= 0.0f ? cfg->max - cfg->mid : cfg->mid - cfg->min);
    return (uint16_t)((float)cfg->mid + stick * span);
}

int main(int argc, char **argv)
{
    const char *shape = argc > 1 ? argv[1] : "quad-x";
    const ak_mixer_t *mixer;

    if (strcmp(shape, "quad-x") == 0) {
        mixer = &ak_mixer_quad_x;
    } else if (strcmp(shape, "elevon-wing") == 0) {
        mixer = &ak_mixer_elevon_wing;
    } else {
        fprintf(stderr, "usage: %s quad-x|elevon-wing < trace.csv\n", argv[0]);
        return 2;
    }

    ak_flight_t flight;
    ak_flight_init(&flight, mixer);

    printf("t_ms,m0,m1,m2,m3,s0,s1,state\n");

    char line[512];
    while (fgets(line, sizeof line, stdin) != NULL) {
        float f[13];
        uint32_t time_ms = 0;
        if (!parse_row(line, f, &time_ms)) {
            continue; /* header or a line the parser cannot use */
        }

        ak_imu_sample_t imu;
        imu.gyro[0] = f[1];
        imu.gyro[1] = f[2];
        imu.gyro[2] = f[3];
        imu.accel[0] = f[4];
        imu.accel[1] = f[5];
        imu.accel[2] = f[6];
        imu.time_ms = time_ms;
        imu.valid = 1;

        ak_rc_input_t rc;
        ak_rc_config_t cfg;
        ak_rc_default_config(&cfg);
        for (int i = 0; i < AK_RC_CHANNELS; i++) {
            rc.channel[i] = (uint16_t)cfg.mid;
        }
        /* Sticks arrive as -1..1 and 0..1 in the file; the flight core wants
         * receiver counts only on the far side of ak_rc_decode. */
        rc.channel[AK_RC_ROLL] = stick_to_counts(&cfg, f[7]);
        rc.channel[AK_RC_PITCH] = stick_to_counts(&cfg, f[8]);
        rc.channel[AK_RC_YAW] = stick_to_counts(&cfg, f[9]);
        rc.channel[AK_RC_THROTTLE] =
            (uint16_t)((float)cfg.min + f[10] * (float)(cfg.max - cfg.min));
        rc.channel[AK_RC_ARM] = (uint16_t)(f[11] > 0.5f ? cfg.max : cfg.min);
        rc.channel[AK_RC_MODE] = (uint16_t)(f[12] > 0.5f ? cfg.max : cfg.min);
        rc.last_update_ms = time_ms;
        rc.valid = 1;

        ak_flight_step(&flight, &imu, &rc, time_ms);
        const ak_outputs_t *o = ak_flight_outputs(&flight);
        printf("%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n", time_ms,
               (double)o->motor[0], (double)o->motor[1], (double)o->motor[2],
               (double)o->motor[3], (double)o->servo[0], (double)o->servo[1],
               (int)ak_flight_state(&flight));
    }

    return 0;
}
