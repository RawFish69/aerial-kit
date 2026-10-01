#ifndef AK_FLIGHT_MATH_H
#define AK_FLIGHT_MATH_H

#include <stdint.h>

/*
 * The math the flight core needs, written here rather than taken from libm.
 *
 * The image links -nostdlib, and that is not only about code size: a flight
 * controller wants its transcendentals to be the same function on every build
 * and every host, with no libm version quietly changing a filter's behaviour.
 * Both implementations below have a stated accuracy, and the host tests check
 * them against libm.
 */

#define AK_PI 3.14159265358979f

static inline float ak_clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static inline float ak_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static inline float ak_deg2rad(float degrees)
{
    return degrees * (AK_PI / 180.0f);
}

static inline float ak_rad2deg(float radians)
{
    return radians * (180.0f / AK_PI);
}

/* The largest angle the wrap below will reduce, in radians: about sixteen
 * thousand turns. Past that the estimate is not an attitude any more, and this
 * says so by returning zero rather than by answering a question about an angle
 * nobody has. */
#define AK_WRAP_LIMIT 1.0e5f

/*
 * An angle in radians, wrapped to -pi..pi.
 *
 * Angles are carried *continuously* through the flight core on purpose: the
 * quadrotor's hover reconstructs the course it is making by taking the
 * difference between two yaw readings, and an angle that wrapped underneath
 * that difference would turn a quarter turn of travel into three quarters of
 * one the other way. Everything that carries an angle in a *fixed width* is
 * periodic instead - the blackbox stores a tenth of a degree in an int16, the
 * CRSF attitude frame stores 1e-4 radians in one - and a continuous angle
 * overflows both of them after a few turns: nine turns for the log, less than
 * one for the frame. Wrapping on the way into those is exact, a heading being
 * the same heading a whole turn later.
 *
 * This is not hypothetical. The sanitizer caught the unwrapped conversion as
 * `32768.5 is outside the range of representable values of type 'short int'`
 * in the loop that writes the log, mid-flight in the three-hundred-second hold
 * session - a yaw that had reached 3276.85 degrees. `make test` passed through
 * all of it, because an out-of-range float-to-integer conversion on this
 * compiler does exactly what the cast was written to do and produces a number.
 *
 * The reduction takes the whole turns out in one step and then corrects by at
 * most one turn, so it is O(1) and cannot run away: subtracting one turn at a
 * time would be a loop that spins for minutes if a sensor ever handed the
 * estimate a number like 1e30, and a hang in the flight loop is worse than a
 * stale heading. Measured against the true remainder over +/- 159 turns, the
 * worst error is 0.0016 degrees, twenty times finer than the field it feeds;
 * at the guard it is 0.16 degrees, which is 1.6 of those fields' own bits, and
 * an angle sixteen thousand turns from zero is a fault rather than a heading.
 */
static inline float ak_wrap_pi(float radians)
{
    const float turn = 6.28318531f;
    int whole;

    /* NaN and the infinities are not angles, and a float-to-integer conversion
     * of either is undefined - which is the conversion every caller of this is
     * about to do. The comparisons are written this way round because both of
     * them are false for a NaN, which is what catches it. */
    if (!(radians <= AK_WRAP_LIMIT && radians >= -AK_WRAP_LIMIT)) {
        return 0.0f;
    }

    /* Whole turns out at once. Truncation is toward zero, so this is the same
     * step in both directions, and it is defined only because of the guard
     * above: the float-to-int here is the very thing this function exists to
     * keep away from a narrowed field. */
    whole = (int)(radians / turn);
    radians -= (float)whole * turn;

    /* What is left is within one turn of zero, so this runs at most once. */
    if (radians > AK_PI) {
        radians -= turn;
    } else if (radians < -AK_PI) {
        radians += turn;
    }
    return radians;
}

/* An attitude in the unit the blackbox record and the status telemetry both
 * carry it in: tenths of a degree, in an int16. The wrap is what makes that
 * conversion total - it lands inside 1800 either way - and it belongs here
 * rather than at either caller, because the two must not be able to disagree
 * about how a heading is written down. */
static inline int16_t ak_attitude_ddeg(float radians)
{
    return (int16_t)(ak_rad2deg(ak_wrap_pi(radians)) * 10.0f);
}

/* Square root by bit-hack seed and Newton-Raphson. Relative error < 1e-6. */
float ak_sqrtf(float x);

/* atan2 by CORDIC in vectoring mode, 18 iterations. Absolute error < 2e-5 rad
 * over the full circle, including the fix-ups at x = 0 and y = 0. */
float ak_atan2f(float y, float x);

/* Sine and cosine by range reduction and a polynomial. Absolute error < 1e-6
 * for |x| up to a few hundred radians, which covers a rotation matrix, a filter
 * coefficient, and - since the estimator carries its attitude as a quaternion -
 * the half-angles of a quaternion product. That last caller is the one to keep
 * in mind if the accuracy claim ever moves: it passes at most |x| = pi/2, so it
 * is far inside this range rather than anywhere near the edge of it. */
float ak_sinf(float x);
float ak_cosf(float x);

/*
 * First-order low pass, and **the cutoff is where it says it is**.
 *
 * The loop is `y += alpha * (x - y)`, whose response is
 * `H = alpha / (1 - (1 - alpha) z^-1)`. Solving that for -3 dB gives a closed
 * form for alpha in terms of `theta = 2*pi*f*dt`:
 *
 *     u     = 1 - cos(theta)
 *     alpha = sqrt(u*u + 2u) - u
 *
 * The first version of this was `w*dt / (1 + w*dt)`, which is cheap and
 * *wrong about its own number*: the same arithmetic below measures the two
 * against each other, and at a 1 kHz loop the old one put a "250 Hz" filter's
 * -3 dB at 163 Hz and an "80 Hz" D filter's at 66 Hz. That is a filter that
 * filters more than anybody asked for, which sounds harmless until the number
 * is borrowed from a reference configuration: INAV flies this airframe with
 * `gyro_anti_aliasing_lpf_hz = 250`, and a firmware that thinks it is doing
 * that while doing 163 is not doing what its page says. So the coefficient is
 * solved, not approximated, and `tests/test_aerialkit.c` holds both filters to
 * their own cutoff.
 *
 * Above Nyquist the solve has no answer and alpha saturates: a one-pole in this
 * form cannot attenuate past f*dt = 0.5 and its response folds back up. What it
 * buys is the band a gyro's noise lives in (100-400 Hz at a kilohertz loop),
 * and the page says so rather than implying a brick wall.
 */
static inline float ak_lpf_alpha(float cutoff_hz, float dt)
{
    if (cutoff_hz <= 0.0f || dt <= 0.0f) {
        return 1.0f; /* no filtering */
    }
    float theta = 6.283185307f * cutoff_hz * dt;
    if (theta >= 3.14159265f) {
        return 1.0f; /* at or past Nyquist: the fastest this form can be */
    }
    float u = 1.0f - ak_cosf(theta);
    return ak_sqrtf(u * u + 2.0f * u) - u;
}

#endif /* AK_FLIGHT_MATH_H */
