#include "ak_estimator.h"

#include "ak_math.h"

/* Outside this band the accelerometer is measuring manoeuvre, not gravity. */
#define AK_ACCEL_TRUST_LOW  0.7f
#define AK_ACCEL_TRUST_HIGH 1.3f
#define AK_CONVERGE_UPDATES 50

/*
 * How far the published angles may sit from what the quaternion implies before
 * the quaternion is rebuilt from them.
 *
 * They should agree exactly: both come out of the same conversion applied to
 * the same quaternion, and nothing in this file writes one without the other.
 * A disagreement therefore means a *caller* wrote `est->roll` or `est->yaw`
 * directly - which is something this struct has always allowed and two things
 * still rely on: main.c's heading aid adds to `est->yaw`, and the tests set an
 * attitude before a step. A write like that has to be folded back into the
 * quaternion or it would be silently discarded by the next update, which is a
 * worse failure than the one F1 describes because it breaks working callers
 * rather than wrong ones.
 *
 * The threshold is well above the float noise of that round trip and far below
 * any attitude a caller could mean.
 */
#define AK_ATT_SYNC_EPS 1.0e-5f

/*
 * Below this the gyro is reporting nothing worth calling a rotation, and the
 * exact step below divides by its magnitude. A gyro at rest is a real sample,
 * not a fault.
 */
#define AK_GYRO_DEADBAND 1.0e-3f

/*
 * Below this the course a module reports is noise: a hovering quad has no
 * direction of travel, and the module still reports one.
 *
 * Three metres a second, and a *lower* gate was tried and put back: the wind
 * correction above only helps once the navigator has learned the wind, and a
 * return spends its first seconds with the correction still at zero - so a
 * gate low enough to be cleared by the drift alone is a gate that aligns the
 * yaw to the weather, which is the failure this file exists to avoid.
 * Measured: at two metres a second the quadrotor's return held its station
 * 2.0 m from home against 0.5 m at three.
 */
#define AK_TRACK_MIN_SPEED 3.0f
/* The time constant of the pull toward the track, and how long the pull has to
 * have been possible before the yaw counts as aligned. */
#define AK_TRACK_TAU 3.0f
#define AK_TRACK_ALIGNED_S 2.0f

/*
 * ---------------------------------------------------------------------------
 * Attitude is a quaternion here, and that is the whole of F1's repair.
 *
 * The previous version added `gyro[0]*dt` to roll, `gyro[1]*dt` to pitch and
 * `gyro[2]*dt` to yaw. Those are body rates being used as if they were Euler
 * derivatives, and they are not: at a bank of phi with pitch theta the ZYX
 * relations are
 *
 *     roll'  = p + q sin(phi) tan(theta) + r cos(phi) tan(theta)
 *     pitch' = q cos(phi) - r sin(phi)
 *     yaw'   = (q sin(phi) + r cos(phi)) / cos(theta)
 *
 * so the three rates are coupled, and the factors run away as the pitch
 * approaches ninety degrees. At forty-five degrees of bank with a yaw rate of
 * one radian a second the old code moved yaw alone, and left pitch exactly
 * where it was, while the truth is zero roll, minus 0.707 pitch and plus 0.707
 * yaw. Measured, before this change: 36.5 degrees of error on a case the
 * existing 2,047 checks all passed, because every one of them held either a
 * decoupled attitude or a small angle.
 *
 * The repair is not to write those three lines out more carefully. It is to
 * stop carrying an attitude in the coordinates that have the singularity: the
 * quaternion has no gimbal lock, composes rotations exactly, and its
 * kinematics are one linear equation, q' = 0.5 q (x) (0, omega), which is what
 * makes the body-rate-to-attitude map correct by construction rather than by
 * three hand-derived coefficients.
 *
 * The Euler angles remain the published interface - the blackbox record, the
 * status telemetry and every controller in ak_flight.c read them - so they are
 * derived from the quaternion once per update instead of being integrated.
 * ---------------------------------------------------------------------------
 */

/* q <- q (x) exp(0.5 * omega * dt).
 *
 * The exact solution for a constant body rate over the step, which is the best
 * model of a gyro sample there is. RK4 was not used: for this equation the
 * closed form is both cheaper and exact, so the error it leaves is the gyro's
 * rather than the integrator's, and there is no step size to argue about. */
static void est_propagate(float *q, const float *gyro, float dt)
{
    float wx = gyro[0], wy = gyro[1], wz = gyro[2];
    float mag = ak_sqrtf(wx * wx + wy * wy + wz * wz);
    float c, s;
    float w, x, y, z;

    if (mag > AK_GYRO_DEADBAND) {
        c = ak_cosf(0.5f * mag * dt);
        s = ak_sinf(0.5f * mag * dt) / mag;
    } else {
        /* sin(h)/mag -> dt/2 as mag -> 0. */
        c = 1.0f;
        s = 0.5f * dt;
    }

    w = q[0]; x = q[1]; y = q[2]; z = q[3];
    q[0] = w * c       - x * (s * wx) - y * (s * wy) - z * (s * wz);
    q[1] = w * (s * wx) + x * c       + y * (s * wz) - z * (s * wy);
    q[2] = w * (s * wy) - x * (s * wz) + y * c       + z * (s * wx);
    q[3] = w * (s * wz) + x * (s * wy) - y * (s * wx) + z * c;
}

static void est_normalize(float *q)
{
    float n = ak_sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    if (n > 1.0e-6f) {
        float inv = 1.0f / n;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    } else {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    }
}

/* The direction "up" points in the body frame, which is what a level
 * accelerometer measures. The third row of R = Rz(yaw) Ry(pitch) Rx(roll),
 * written out from q: the leading minus on the first component is what makes
 * atan2(-ax, hypot(ay,az)) come back as *plus* the pitch. */
static void est_body_up(const float *q, float *ux, float *uy, float *uz)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    *ux = 2.0f * (x * z - w * y);
    *uy = 2.0f * (y * z + w * x);
    *uz = 1.0f - 2.0f * (x * x + y * y);
}

/* Roll and pitch out of the quaternion. The same two atan2 calls the old
 * accelerometer correction made, now asked of the estimate instead of the
 * sensor, so the published angles keep the convention every reader expects. */
static void est_euler_from_q(const float *q, float *roll, float *pitch)
{
    float ux, uy, uz;

    est_body_up(q, &ux, &uy, &uz);
    *roll  = ak_atan2f(uy, uz);
    *pitch = ak_atan2f(-ux, ak_sqrtf(uy * uy + uz * uz));
}

/* Yaw out of the quaternion, always inside (-pi, pi]. */
static float est_yaw_from_q(const float *q)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    return ak_atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z));
}

/*
 * The angle between the two unit vectors, written as atan2(|a x b|, a . b)
 * rather than acos(a . b). There is no acos in ak_math.h, and that is not the
 * only reason: acos loses most of its precision exactly where this is read,
 * near zero, where the difference between half a degree and two degrees is the
 * whole question. atan2 keeps its resolution all the way down.
 */
float ak_estimator_innovation_deg(const ak_estimator_t *est, const float accel[3])
{
    float ux, uy, uz, n, d, cx, cy, cz, m;

    est_body_up(est->q, &ux, &uy, &uz);
    n = ak_sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                 accel[2] * accel[2]);
    if (n < 1.0e-6f) {
        return 180.0f;
    }
    d = (ux * accel[0] + uy * accel[1] + uz * accel[2]) / n;
    cx = uy * accel[2] - uz * accel[1];
    cy = uz * accel[0] - ux * accel[2];
    cz = ux * accel[1] - uy * accel[0];
    m = ak_sqrtf(cx * cx + cy * cy + cz * cz) / n;
    return ak_rad2deg(ak_atan2f(m, d));
}

static void est_q_from_euler(float *q, float roll, float pitch, float yaw)
{
    float cr = ak_cosf(roll * 0.5f),  sr = ak_sinf(roll * 0.5f);
    float cp = ak_cosf(pitch * 0.5f), sp = ak_sinf(pitch * 0.5f);
    /* The quaternion has no room for the turns est->yaw may have accumulated.
     * The sine and cosine of a heading do not care how many there were. */
    float hy = ak_wrap_pi(yaw) * 0.5f;
    float cy = ak_cosf(hy), sy = ak_sinf(hy);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

/* Fold any write a caller made to the published angles back into the
 * quaternion. See AK_ATT_SYNC_EPS. */
static void est_resync(ak_estimator_t *est)
{
    float roll, pitch;
    float psi = est_yaw_from_q(est->q);

    est_euler_from_q(est->q, &roll, &pitch);

    if (ak_absf(roll - est->roll) < AK_ATT_SYNC_EPS &&
        ak_absf(pitch - est->pitch) < AK_ATT_SYNC_EPS &&
        ak_absf(ak_wrap_pi(psi - est->yaw)) < AK_ATT_SYNC_EPS) {
        return;
    }

    est_q_from_euler(est->q, est->roll, est->pitch, est->yaw);
}

/* Rotate the estimate a little way toward the direction gravity is actually
 * coming from. `ux,uy,uz` is the measured specific force as a unit vector and
 * `gain` is the fraction of the remaining error to take out this update.
 *
 * The rotation that would bring the predicted up onto the measured one, to
 * first order, is `measured x predicted` - its magnitude is the sine of the
 * angle between them and its direction is the axis to turn about. That it
 * cannot touch yaw is not an accident of the formula: two vectors that are
 * both "up-like" span a plane whose normal is horizontal in the body frame, so
 * the correction is a levelling and nothing else. Gravity has never been able
 * to say which way the nose is pointing and this does not pretend otherwise.
 *
 * The order of that product is the whole of the sign, and this file had it
 * backwards first time. A body-frame rotation of delta changes the up vector
 * the body sees to `up - delta x up`, so levelling to a measured `m` wants
 * `delta x up = up - m`, which `m x up` satisfies and `up x m` does not. The
 * check that settles it is one line: level, with the true roll at twenty
 * degrees, `m x up` has a positive body-x component and rolling further over
 * is what has to happen. `up x m` gives minus that, so the "correction" drove
 * the estimate away from gravity - invisible over one second and one hundred
 * and fifty degrees wrong over five. */
static void est_gravity_correct(float *q, float ux, float uy, float uz,
                                float gain)
{
    float px, py, pz;
    float dx, dy, dz;
    float w, x, y, z;

    est_body_up(q, &px, &py, &pz);

    dx = (uy * pz - uz * py) * gain;
    dy = (uz * px - ux * pz) * gain;
    dz = (ux * py - uy * px) * gain;

    /* q <- q (x) (1, d/2): a rotation of |d| about d, in body axes, which is
     * the side of the product a body-frame increment belongs on. */
    w = q[0]; x = q[1]; y = q[2]; z = q[3];
    q[0] = w - 0.5f * (x * dx + y * dy + z * dz);
    q[1] = x + 0.5f * (w * dx + y * dz - z * dy);
    q[2] = y + 0.5f * (w * dy - x * dz + z * dx);
    q[3] = z + 0.5f * (w * dz + x * dy - y * dx);
}

static void est_publish(ak_estimator_t *est)
{
    float psi = est_yaw_from_q(est->q);

    est_euler_from_q(est->q, &est->roll, &est->pitch);
    /* est->yaw is carried unwrapped - see ak_estimator_aid_heading() - so the
     * quaternion's answer, which is always inside (-pi, pi], is moved to the
     * turn nearest the heading already published rather than replacing it. A
     * yaw of 365 degrees stays 365 and a yaw that has wound round three times
     * keeps its three turns. */
    est->yaw += ak_wrap_pi(psi - est->yaw);
}

void ak_estimator_init(ak_estimator_t *est, float tau)
{
    est->roll = 0.0f;
    est->pitch = 0.0f;
    est->yaw = 0.0f;
    est->q[0] = 1.0f;
    est->q[1] = 0.0f;
    est->q[2] = 0.0f;
    est->q[3] = 0.0f;
    est->tau = tau;
    est->converged = 0;
    est->accel_used = 0;
    est->track_aligned_s = 0.0f;
    est->track_aligned = 0;
}

void ak_estimator_aid_heading(ak_estimator_t *est, float course_rad,
                              float speed_m_s, float dt)
{
    float error;
    float alpha;

    if (dt <= 0.0f) {
        return;
    }
    if (speed_m_s < AK_TRACK_MIN_SPEED) {
        /* Not moving in a direction worth believing. The alignment that was
         * earned while moving is kept - it does not rot in a hover - but a
         * hover cannot earn more of it. */
        return;
    }

    /* The shortest way round, which is what this is for: a course of five
     * degrees against a yaw of three hundred and fifty-five are ten degrees
     * apart, not three hundred and fifty. The yaw itself is carried unwrapped -
     * see ak_wrap_pi() - so the error can be any number of turns out and the
     * wrap has to cope with that. */
    error = ak_wrap_pi(course_rad - est->yaw);

    alpha = dt / (AK_TRACK_TAU + dt);
    est->yaw += error * alpha;

    /* The quaternion is not touched here. This adjusts the published heading,
     * and ak_estimator_update() folds that back into the attitude on its next
     * call (est_resync) - which is also what lets a caller set est->yaw by
     * hand and have it mean something. */
    est->track_aligned_s += dt;
    if (est->track_aligned_s >= AK_TRACK_ALIGNED_S) {
        est->track_aligned = 1;
    }
}

void ak_estimator_air_track(int32_t speed_mm_s, int32_t course_e5,
                            float wind_n_m_s, float wind_e_m_s,
                            float *speed_m_s, float *course_rad)
{
    float course = ak_deg2rad((float)course_e5 / 100000.0f);
    float ground = (float)speed_mm_s / 1000.0f;
    float vn = ground * ak_cosf(course) + wind_n_m_s;
    float ve = ground * ak_sinf(course) + wind_e_m_s;

    *speed_m_s = ak_sqrtf(vn * vn + ve * ve);
    *course_rad = ak_atan2f(ve, vn);
}

void ak_estimator_update(ak_estimator_t *est, const ak_imu_sample_t *imu, float dt)
{
    float ax, ay, az, magnitude;

    if (!imu->valid || dt <= 0.0f) {
        return;
    }

    /* Anything a caller wrote to est->roll / est->pitch / est->yaw since the
     * last update is taken as the new attitude, before the gyro moves it. */
    est_resync(est);

    /* The gyro carries the estimate between accelerometer updates, exactly. */
    est_propagate(est->q, imu->gyro, dt);

    ax = imu->accel[0];
    ay = imu->accel[1];
    az = imu->accel[2];
    magnitude = ak_sqrtf(ax * ax + ay * ay + az * az);

    if (magnitude > AK_ACCEL_TRUST_LOW && magnitude < AK_ACCEL_TRUST_HIGH) {
        float inv = 1.0f / magnitude;

        /* The accel weight of the complementary filter, unchanged: the old
         * code used (1 - tau/(tau+dt)) on the angle error and this takes the
         * same fraction of it, just as a rotation about a cross product
         * instead of two independent scalar pulls. */
        est_gravity_correct(est->q, ax * inv, ay * inv, az * inv,
                            dt / (est->tau + dt));

        if (est->accel_used < AK_CONVERGE_UPDATES) {
            est->accel_used++;
            if (est->accel_used >= AK_CONVERGE_UPDATES) {
                est->converged = 1;
            }
        }
    }

    est_normalize(est->q);
    est_publish(est);
}
