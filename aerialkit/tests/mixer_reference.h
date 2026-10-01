#ifndef AK_TESTS_MIXER_REFERENCE_H
#define AK_TESTS_MIXER_REFERENCE_H

/*
 * Two reference mixers, transcribed from the upstreams this repository is built
 * alongside. They are data, like tests/ubx_reference.h: the coefficients and
 * the row order are the reference's, with no code taken from it.
 *
 * The quad-X table is Betaflight's `mixerQuadX` - the one table in either
 * project that still spells the mix out in the firmware rather than loading it
 * from configuration:
 *
 *   betaflight-2026.6.1/src/main/flight/mixer_init.c @ 6dbc4218
 *     { 1.0f, -1.0f,  1.0f, -1.0f },   // REAR_R
 *     { 1.0f, -1.0f, -1.0f,  1.0f },   // FRONT_R
 *     { 1.0f,  1.0f,  1.0f,  1.0f },   // REAR_L
 *     { 1.0f,  1.0f, -1.0f, -1.0f },   // FRONT_L
 *
 * Columns are throttle, roll, pitch, yaw, which is the same four numbers in the
 * same order as this firmware's ak_mixer_t - and the same as INAV's
 * motorMixer_t, which is where the shape came from.
 *
 * INAV 9.1.0 loads its motor mixer from configuration rather than tabling it,
 * but the values it ships for its own targets are the same mix:
 *
 *   inav-9.1.0/src/main/target/ALIENFLIGHTF4/config.c @ e519b69
 *     { 1.0f, -0.414178f,  1.0f, -1.0f };   // REAR_R
 *
 * -0.414178 instead of -1 because that airframe is a hexacopter in a
 * different frame; the signs, which are what a parity check is for, are
 * identical. Both are recorded in docs/03-attribution.md.
 */

typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} mixer_reference_row_t;

/*
 * The reference's pitch axis points the other way.
 *
 * Betaflight's pitch is positive when the nose is *down* - its attitude
 * estimate is computed that way, and the comment "negative is backwards" in
 * `flight/imu.c` is what says so - and its QUADX table speeds the rear motors
 * up for it, which is what puts the nose down. This firmware's pitch is
 * positive when the nose is *up*: its estimator computes it from the
 * accelerometer that way and its elevon mix uses it that way.
 *
 * Both are self-consistent; they disagree about which direction the number
 * points. So the comparison multiplies the reference's pitch by this, and the
 * physical check - same stick, same motors - is what makes the sign convention
 * something a test can catch rather than something a reader has to notice.
 */
#define MIXER_REFERENCE_PITCH_SIGN (-1.0f)

#define MIXER_REFERENCE_ROWS 4

/* Row order: rear right, front right, rear left, front left. */
static const mixer_reference_row_t mixer_reference_quad_x[MIXER_REFERENCE_ROWS] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },
    { 1.0f, -1.0f, -1.0f,  1.0f },
    { 1.0f,  1.0f,  1.0f,  1.0f },
    { 1.0f,  1.0f, -1.0f, -1.0f },
};

/*
 * The other frames' tables, from the same file and revision - `mixerQuadX1234`,
 * `mixerQuadP`, `mixerY4`, `mixerVtail4` and `mixerTricopter`. They are here
 * because a mixer is the one place where "close" is not a word: a frame with a
 * row in the wrong place takes off and pirouettes, and the row order is the
 * part a person cannot check by reading their own aircraft.
 *
 * The tricopter's table is three motors and no yaw column, because the
 * reference carries the tail rotor's tilt in its *servo* mixer - a different
 * mechanism for the same thing, and the reason ours has a fourth row (a servo)
 * that is compared separately.
 */

/* Row order: front left, front right, rear right, rear left. */
static const mixer_reference_row_t mixer_reference_quad_x_1234[MIXER_REFERENCE_ROWS] = {
    { 1.0f,  1.0f, -1.0f, -1.0f },  /* FRONT_L */
    { 1.0f, -1.0f, -1.0f,  1.0f },  /* FRONT_R */
    { 1.0f, -1.0f,  1.0f, -1.0f },  /* REAR_R  */
    { 1.0f,  1.0f,  1.0f,  1.0f },  /* REAR_L  */
};

/* Row order: rear, right, left, front. */
static const mixer_reference_row_t mixer_reference_quad_p[MIXER_REFERENCE_ROWS] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },  /* REAR  */
    { 1.0f, -1.0f,  0.0f,  1.0f },  /* RIGHT */
    { 1.0f,  1.0f,  0.0f,  1.0f },  /* LEFT  */
    { 1.0f,  0.0f, -1.0f, -1.0f },  /* FRONT */
};

/* Row order: rear top, front right, rear bottom, front left. */
static const mixer_reference_row_t mixer_reference_y4[MIXER_REFERENCE_ROWS] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },  /* REAR_TOP CW    */
    { 1.0f, -1.0f, -1.0f,  0.0f },  /* FRONT_R CCW    */
    { 1.0f,  0.0f,  1.0f,  1.0f },  /* REAR_BOTTOM CCW */
    { 1.0f,  1.0f, -1.0f,  0.0f },  /* FRONT_L CW     */
};

/* Row order: rear right, front right, rear left, front left. */
static const mixer_reference_row_t mixer_reference_vtail4[MIXER_REFERENCE_ROWS] = {
    { 1.0f, -0.58f,  0.58f,  1.0f },  /* REAR_R  */
    { 1.0f, -0.46f, -0.39f, -0.5f },  /* FRONT_R */
    { 1.0f,  0.58f,  0.58f, -1.0f },  /* REAR_L  */
    { 1.0f,  0.46f, -0.39f,  0.5f },  /* FRONT_L */
};

/* Row order: rear, right, left - three motors, and the tail's tilt is the
 * reference's servo mixer rather than a row here. */
static const mixer_reference_row_t mixer_reference_tri[3] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },  /* REAR  */
    { 1.0f, -1.0f, -0.666667f,  0.0f },  /* RIGHT */
    { 1.0f,  1.0f, -0.666667f,  0.0f },  /* LEFT  */
};

/* The reference's own arithmetic, which is one line: every output is the sum
 * of its coefficients times the four inputs. There is no authority limit and no
 * idle floor here because neither is part of the mix - both are things this
 * firmware added, and the parity check is about the mix underneath them. */
static float mixer_reference_apply(const mixer_reference_row_t *row,
                                   float throttle, float roll, float pitch,
                                   float yaw)
{
    return row->throttle * throttle + row->roll * roll + row->pitch * pitch +
           row->yaw * yaw;
}

#endif /* AK_TESTS_MIXER_REFERENCE_H */
