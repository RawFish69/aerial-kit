#ifndef AK_FLIGHT_ESTIMATOR_H
#define AK_FLIGHT_ESTIMATOR_H

#include "ak_types.h"

/*
 * Attitude estimate: complementary filter.
 *
 * The accelerometer gives a drift-free but noisy absolute roll and pitch; the
 * gyro gives a smooth but drifting rate. The filter trusts the gyro over short
 * windows and the accelerometer over long ones, with a single time constant
 * saying where the crossover is.
 *
 * Yaw is gyro integration and drifts, and there is no magnetometer on any
 * board here - the selftest prints that, so it cannot be forgotten. What there
 * is, once the aircraft is moving, is the GPS's ground course:
 * ak_estimator_aid_heading() pulls yaw toward it, slowly and only while the
 * aircraft is actually moving, which is what makes yaw good enough for the
 * quadrotor's position-controlled return (ak_nav.c). Still deliberately not
 * here: a magnetometer, and anything resembling an EKF. AerialKit has one
 * magnetometer *driver* and no board with one fitted, and an EKF whose
 * measurement models have never seen a flight is a file that looks like
 * rigour and tests like arithmetic.
 */

typedef struct {
    /*
     * The attitude, carried as a quaternion (w, x, y, z), because body rates
     * are not Euler derivatives and integrating them as if they were is wrong
     * by tens of degrees in any coupled rotation - see the block comment in
     * ak_estimator.c. roll / pitch / yaw below are derived from this once per
     * update, not integrated.
     *
     * They are still writable, and a write is honoured: the next update folds
     * whatever it finds there back into the quaternion before the gyro moves
     * it. That is deliberate, not vestigial - ak_estimator_aid_heading() adds
     * to yaw and callers set an attitude before a step, and both have to keep
     * working.
     */
    float q[4];
    float roll;      /* rad, derived from q */
    float pitch;     /* rad, derived from q */
    float yaw;       /* rad, derived from q's yaw plus however many turns it has made */
    float tau;       /* filter time constant, s */
    int   converged; /* set once the accelerometer has been used enough times */
    int   accel_used;
    /* How long the ground track has been agreeing with the yaw estimate, and
     * whether that is long enough to steer by. The position controller asks:
     * a world-frame error turned into a body-frame tilt with a yaw that is 90
     * degrees out flies the aircraft confidently in the wrong direction. */
    float track_aligned_s;
    int   track_aligned;
} ak_estimator_t;

void ak_estimator_init(ak_estimator_t *est, float tau);

/* dt in seconds. An accelerometer vector far from 1 g is ignored, so a
 * manoeuvring airframe does not drag the attitude estimate with it. */
void ak_estimator_update(ak_estimator_t *est, const ak_imu_sample_t *imu, float dt);

/*
 * The filter's own error, in degrees: the angle between the up the estimate
 * predicts and the up the accelerometer reports. It is the quantity
 * est_gravity_correct() acts on, so it is zero exactly when the estimate and the
 * sensor agree - and it is the only signal in the core that says how far the
 * estimate still has to travel rather than how many samples it has seen.
 *
 * `converged` is a count of accelerometer updates and says nothing about this.
 * A caller that needs the estimate to be *right* wants this number, not that
 * flag. Measured in FINDING 9 of the 2026-09-21 estimator record: a 20 degree
 * aircraft is cleared to arm on an estimate 6.7 degrees out.
 *
 * Returns 180 when the sample has no direction to compare against, so a dead or
 * saturated accelerometer reads as disagreement rather than as agreement.
 */
float ak_estimator_innovation_deg(const ak_estimator_t *est, const float accel[3]);

/*
 * Yaw from the track the aircraft is making *through the air*.
 *
 * A GPS with a fix reports the direction the aircraft is travelling *over the
 * ground*, and in wind that is not the direction it is pointing - which is the
 * whole reason a position controller in the body frame needs a heading at all.
 * The first version of this aligned yaw to the raw ground track, and the
 * difference is not a detail:
 *
 * - **A drifting aircraft's "track" is the wind.** A quadrotor holding station
 *   in five metres a second of wind moves at five metres a second - in the
 *   wind's direction - with its nose anywhere it likes, so the old rule pulled
 *   the yaw estimate onto the wind and then steered with it. Measured in the
 *   simulator: a mission's flight had the estimate 50 to 136 degrees away from
 *   the nose, and the aircraft ended up chasing its own tail with the motors
 *   on their stops.
 * - **A downwind leg's ground track is not the nose either.** An aircraft
 *   flying at six metres a second through the air with five metres a second of
 *   tailwind has a ground track 50 degrees off its nose, and every one of
 *   those degrees went into the tilt the navigator computed.
 *
 * So the caller converts the ground vector the module reports into the air
 * vector the airframe actually made, by subtracting the wind - and the wind is
 * a number this firmware already has, because the navigator learns exactly
 * what it must fly at to hold station in it (ak_nav.c's standing term). Then
 * this aligns the yaw to *that*: a hover in wind has no air track and earns
 * nothing, which is the correct answer, and a turn at speed earns the nose
 * direction, which is the frame the tilts are computed in.
 *
 * `speed_m_s` is the aircraft's speed *through the air*: below a walking pace
 * the course is noise that changes every fix, and nothing here will touch the
 * estimate. The
 * correction is slow (a time constant of seconds), so a stale or wrong course
 * adds a small error rather than turning the estimate into noise, and the
 * shortest way round the circle is taken, so a yaw of 359 degrees is one
 * degree from a course of 0.
 */
void ak_estimator_aid_heading(ak_estimator_t *est, float course_rad,
                              float speed_m_s, float dt);

/*
 * The ground vector the module reports, minus the wind, is the track the
 * aircraft made through the air. `speed_mm_s` and `course_e5` are the fix's
 * own units, `wind_n_m_s`/`wind_e_m_s` are the velocity the navigator flies at
 * to hold station, and the outputs are the air speed in metres a second and
 * the course in radians, both of which are what ak_estimator_aid_heading()
 * wants. Split out as a function so the arithmetic - and in particular the
 * case where the subtraction leaves nothing at all - is testable without a
 * flight.
 */
void ak_estimator_air_track(int32_t speed_mm_s, int32_t course_e5,
                            float wind_n_m_s, float wind_e_m_s,
                            float *speed_m_s, float *course_rad);

#endif /* AK_FLIGHT_ESTIMATOR_H */
