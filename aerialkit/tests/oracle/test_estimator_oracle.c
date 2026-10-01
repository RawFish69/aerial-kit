/*
 * An independent oracle for ak_estimator_update() - B2, the measuring half.
 *
 * The rule this file follows: nothing here may be derived from the estimator's
 * own equations. Truth is a quaternion integrated from body rates by
 * q' = 0.5 * q (x) (0, omega), and the accelerometer the estimator is fed is
 * the true body-frame gravity vector taken from that quaternion. If the
 * estimator and the truth agree, they agree because both are right, not
 * because they share an approximation - which is exactly what F2 says the
 * existing simulator cannot establish.
 *
 * Convention, taken from the firmware and checked rather than assumed: ZYX
 * (yaw, then pitch, then roll), so the body-frame "up" vector that a level
 * accelerometer reports is
 *
 *     body_up = (-sin(pitch), sin(roll)cos(pitch), cos(roll)cos(pitch))
 *
 * The leading minus is not decoration and this comment had it wrong first
 * time round: body_up is the third row of R = Rz(yaw) Ry(pitch) Rx(roll),
 * which is [-sin(pitch), cos(pitch)sin(roll), cos(pitch)cos(roll)]. Written
 * without it the identity below does not hold - atan2(-ax, hypot(ay,az)) would
 * return minus the pitch - so the self-check would have failed and this file
 * would have been measuring a frame nothing uses. The code was right and the
 * prose was wrong, which is the direction that gets copied.
 *
 * The estimator's own corrections invert that vector:
 * atan2(ay, az) = roll and atan2(-ax, hypot(ay,az)) = pitch. Those two
 * identities are asserted below before anything else is measured, so that a
 * disagreement later is a disagreement about propagation and not about frames.
 */
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <string.h>

#include "ak_estimator.h"

/* ---------------------------------------------------------------- oracle */

typedef struct { double w, x, y, z; } quat_t;

static quat_t qmul(quat_t a, quat_t b)
{
    quat_t r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

static quat_t qnorm(quat_t q)
{
    double n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    quat_t r = { q.w/n, q.x/n, q.y/n, q.z/n };
    return r;
}

static quat_t q_from_euler_zyx(double roll, double pitch, double yaw)
{
    double cr = cos(roll*0.5), sr = sin(roll*0.5);
    double cp = cos(pitch*0.5), sp = sin(pitch*0.5);
    double cy = cos(yaw*0.5), sy = sin(yaw*0.5);
    quat_t q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

/* Euler angles out of the quaternion, ZYX - derived from the rotation matrix
 * written out from q, not from any integration the estimator performs. */
static void euler_from_q(quat_t q, double *roll, double *pitch, double *yaw)
{
    double w=q.w, x=q.x, y=q.y, z=q.z;
    double r00 = 1-2*(y*y+z*z), r10 = 2*(x*y+w*z), r20 = 2*(x*z-w*y);
    double r21 = 2*(y*z+w*x), r22 = 1-2*(x*x+y*y);
    *roll  = atan2(r21, r22);
    *pitch = atan2(-r20, sqrt(r00*r00 + r10*r10));
    *yaw   = atan2(r10, r00);
}

/* The third row of the rotation matrix, i.e. R^T z_world: the direction of
 * "up" as the body sees it. Written straight from the quaternion. */
static void body_up(quat_t q, double *ax, double *ay, double *az)
{
    double w=q.w, x=q.x, y=q.y, z=q.z;
    *ax = 2*(x*z - w*y);
    *ay = 2*(y*z + w*x);
    *az = 1 - 2*(x*x + y*y);
}

/* q' = 0.5 * q (x) (0, omega), integrated by RK4 at the estimator's own dt. */
static quat_t q_deriv(quat_t q, double p, double r_, double r2)
{
    quat_t omega = { 0.0, p, r_, r2 };
    quat_t d = qmul(q, omega);
    quat_t out = { 0.5*d.w, 0.5*d.x, 0.5*d.y, 0.5*d.z };
    return out;
}

static quat_t q_step_rk4(quat_t q, double p, double qq, double r, double dt)
{
    quat_t k1 = q_deriv(q, p, qq, r);
    quat_t a = { q.w+0.5*dt*k1.w, q.x+0.5*dt*k1.x, q.y+0.5*dt*k1.y, q.z+0.5*dt*k1.z };
    quat_t k2 = q_deriv(a, p, qq, r);
    quat_t b = { q.w+0.5*dt*k2.w, q.x+0.5*dt*k2.x, q.y+0.5*dt*k2.y, q.z+0.5*dt*k2.z };
    quat_t k3 = q_deriv(b, p, qq, r);
    quat_t c = { q.w+dt*k3.w, q.x+dt*k3.x, q.y+dt*k3.y, q.z+dt*k3.z };
    quat_t k4 = q_deriv(c, p, qq, r);
    quat_t out = {
        q.w + dt/6.0*(k1.w + 2*k2.w + 2*k3.w + k4.w),
        q.x + dt/6.0*(k1.x + 2*k2.x + 2*k3.x + k4.x),
        q.y + dt/6.0*(k1.y + 2*k2.y + 2*k3.y + k4.y),
        q.z + dt/6.0*(k1.z + 2*k2.z + 2*k3.z + k4.z),
    };
    return qnorm(out);
}

/* ------------------------------------------------------------------ cases */

static double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

typedef struct {
    const char *name;
    double roll0, pitch0, yaw0;   /* initial true attitude, rad */
    double p, q, r;               /* constant body rates, rad/s */
    double seconds;
    int    accel_valid;           /* feed true specific force, or none */
    int    zero_accel;            /* feed an all-zero accelerometer (sensor loss) */
} case_t;

/* Returns 1 if the estimate stayed within `tol_deg` of the truth over the whole
 * case, 0 otherwise - so this file is a test and can fail, not a report. */
static int run(const case_t *c, double dt, double tol_deg)
{
    ak_estimator_t est;
    ak_estimator_init(&est, 0.5f);
    est.roll  = (float)c->roll0;
    est.pitch = (float)c->pitch0;
    est.yaw   = (float)c->yaw0;

    quat_t q = q_from_euler_zyx(c->roll0, c->pitch0, c->yaw0);
    int steps = (int)(c->seconds / dt);

    /* The analytic instantaneous Euler rates for ZYX at this attitude, for
     * comparison against what the estimator's first step does. */
    double sr = sin(c->roll0), cr = cos(c->roll0);
    double tp = tan(c->pitch0), cp = cos(c->pitch0);
    double roll_dot_true  = c->p + c->q*sr*tp + c->r*cr*tp;
    double pitch_dot_true = c->q*cr - c->r*sr;
    double yaw_dot_true   = (c->q*sr + c->r*cr) / cp;

    for (int i = 1; i <= steps; i++) {
        ak_imu_sample_t imu;
        memset(&imu, 0, sizeof imu);
        imu.valid = 1;
        imu.time_ms = (uint32_t)i;
        imu.gyro[0] = (float)c->p;
        imu.gyro[1] = (float)c->q;
        imu.gyro[2] = (float)c->r;

        double ux, uy, uz;
        body_up(q, &ux, &uy, &uz);
        if (c->zero_accel) {
            imu.accel[0] = imu.accel[1] = imu.accel[2] = 0.0f;
        } else if (c->accel_valid) {
            imu.accel[0] = (float)ux;
            imu.accel[1] = (float)uy;
            imu.accel[2] = (float)uz;
        } else {
            /* Out of the 0.7-1.3 g trust band: 2 g of manoeuvre. */
            imu.accel[0] = (float)(2.0*ux);
            imu.accel[1] = (float)(2.0*uy);
            imu.accel[2] = (float)(2.0*uz);
        }

        ak_estimator_update(&est, &imu, (float)dt);
        q = q_step_rk4(q, c->p, c->q, c->r, dt);

        if (i == 1) {
            /* What one step of each did, in rad/s. */
            printf("    after 1 step: estimator d(roll,pitch,yaw) = %9.6f %9.6f %9.6f\n",
                   (double)est.roll  - c->roll0,
                   (double)est.pitch - c->pitch0,
                   (double)est.yaw   - c->yaw0);
            printf("    true rates       (roll,pitch,yaw) = %9.6f %9.6f %9.6f  (rad/s)\n",
                   roll_dot_true, pitch_dot_true, yaw_dot_true);
        }
    }

    double tr, tpi, ty;
    euler_from_q(q, &tr, &tpi, &ty);

    printf("  %-46s\n", c->name);
    printf("    truth      roll %9.4f  pitch %9.4f  yaw %9.4f  (deg)\n",
           tr*180/M_PI, tpi*180/M_PI, wrap_pi(ty)*180/M_PI);
    printf("    estimate   roll %9.4f  pitch %9.4f  yaw %9.4f  (deg)\n",
           (double)est.roll*180/M_PI, (double)est.pitch*180/M_PI,
           wrap_pi((double)est.yaw)*180/M_PI);
    double er = ((double)est.roll  - tr )*180/M_PI;
    double ep = ((double)est.pitch - tpi)*180/M_PI;
    double ey = wrap_pi((double)est.yaw - ty)*180/M_PI;
    double worst = fabs(er);
    if (fabs(ep) > worst) worst = fabs(ep);
    if (fabs(ey) > worst) worst = fabs(ey);

    printf("    error      roll %9.4f  pitch %9.4f  yaw %9.4f  (deg)\n", er, ep, ey);
    printf("    %s: worst %.4f deg against a %.2f deg tolerance\n\n",
           worst <= tol_deg ? "PASS" : "FAIL", worst, tol_deg);
    return worst <= tol_deg;
}

int main(void)
{
    /* If either self-check fails, nothing below it means anything, and the
     * exit status has to say so rather than let a broken oracle be read as an
     * estimator defect. */
    int oracle_bad = 0;

    /* Frames first: if these two identities do not hold, a disagreement later
     * is a disagreement about convention, not about propagation. */
    printf("== frame check: the estimator's own corrections against the oracle ==\n");
    {
        double worst_r = 0.0, worst_p = 0.0;
        for (int ir = -80; ir <= 80; ir += 5) {
            for (int ip = -80; ip <= 80; ip += 5) {
                double roll = ir*M_PI/180.0, pitch = ip*M_PI/180.0;
                quat_t q = q_from_euler_zyx(roll, pitch, 0.0);
                double ax, ay, az;
                body_up(q, &ax, &ay, &az);
                double est_roll  = atan2(ay, az);
                double est_pitch = atan2(-ax, sqrt(ay*ay + az*az));
                double dr = fabs(est_roll - roll);
                double dp = fabs(est_pitch - pitch);
                if (dr > worst_r) worst_r = dr;
                if (dp > worst_p) worst_p = dp;
            }
        }
        printf("   worst |atan2(ay,az) - roll|  over a 33x33 grid: %.6f rad (%.4f deg)\n",
               worst_r, worst_r*180/M_PI);
        printf("   worst |atan2(-ax,..) - pitch|                   : %.6f rad (%.4f deg)\n",
               worst_p, worst_p*180/M_PI);
        if (!(worst_r < 1e-9 && worst_p < 1e-9)) {
            oracle_bad = 1;
        }
        printf("   -> %s\n\n", (worst_r < 1e-9 && worst_p < 1e-9)
               ? "the estimator's convention IS the oracle's: ZYX, body_up as written"
               : "CONVENTIONS DIFFER - stop and fix the oracle before measuring");
    }

    /* The oracle checked against itself, so that a later disagreement with the
     * estimator is not an oracle bug. Integrating the quaternion and then
     * extracting Euler angles must reproduce the analytic ZYX rate relation:
     *   roll_dot  = p + q sin(roll) tan(pitch) + r cos(roll) tan(pitch)
     *   pitch_dot = q cos(roll) - r sin(roll)
     *   yaw_dot   = (q sin(roll) + r cos(roll)) / cos(pitch)
     * If those two routes agree, the oracle's kinematics are right and any
     * error it reports is the estimator's. */
    printf("== oracle self-check: RK4 of q, then Euler, against the analytic rates ==\n");
    {
        double worst = 0.0;
        const double dt = 1e-5;
        for (int ir = -60; ir <= 60; ir += 20) {
            for (int ip = -60; ip <= 60; ip += 20) {
                double r0 = ir*M_PI/180.0, p0 = ip*M_PI/180.0;
                double rates[3][3] = { {0.0, 0.0, 1.0}, {0.4, -0.3, 0.7}, {0.0, 0.9, 0.0} };
                for (int k = 0; k < 3; k++) {
                    double p = rates[k][0], qq = rates[k][1], r = rates[k][2];
                    quat_t q0 = q_from_euler_zyx(r0, p0, 0.0);
                    /* Central difference, not forward. A forward difference
                     * carries a truncation error of about half a step times the
                     * angular curvature, which at dt=1e-5 with the 1/cos(pitch)
                     * factors of a ZYX rate is ~1e-5 - the same size as the
                     * agreement being claimed, so a forward difference cannot
                     * tell an oracle bug from its own error. */
                    quat_t qp = q_step_rk4(q0, p, qq, r, dt);
                    quat_t qm = q_step_rk4(q0, p, qq, r, -dt);
                    double apr, app, apy, amr, amp, amy;
                    euler_from_q(qp, &apr, &app, &apy);
                    euler_from_q(qm, &amr, &amp, &amy);
                    double nr = (apr-amr)/(2*dt), np = (app-amp)/(2*dt);
                    double ny = wrap_pi(apy-amy)/(2*dt);
                    double sr = sin(r0), cr = cos(r0);
                    double tp = tan(p0), cp = cos(p0);
                    double er = p + qq*sr*tp + r*cr*tp;
                    double ep = qq*cr - r*sr;
                    double ey = (qq*sr + r*cr)/cp;
                    double d = fabs(nr-er);
                    if (fabs(np-ep) > d) d = fabs(np-ep);
                    if (fabs(ny-ey) > d) d = fabs(ny-ey);
                    if (d > worst) worst = d;
                }
            }
        }
        printf("   worst |numeric - analytic| over 7x7 attitudes x 3 rate sets: %.9f rad/s\n", worst);
        if (!(worst < 1e-6)) {
            oracle_bad = 1;
        }
        printf("   -> %s\n\n", worst < 1e-6
               ? "the oracle's integration and its Euler extraction are consistent"
               : "ORACLE INCONSISTENT - its numbers mean nothing yet");
    }

    /*
     * The acceptance tolerance for B2. Two degrees is not a tight bound on a
     * complementary filter - it is the bound at which the aircraft is still
     * being told which way is up. The estimator misses it by an order of
     * magnitude today, which is the defect, not the tolerance.
     */
    const double TOL_DEG = 2.0;
    int failures = 0;

    printf("== the assessment's counterexample, executed rather than asserted ==\n");
    printf("   roll 45 deg, pitch 0, body rates p=0 q=0 r=1 rad/s\n");
    printf("   the assessment predicts true (roll_dot, pitch_dot, yaw_dot) = 0, -0.7071, +0.7071\n\n");
    {
        case_t c = { "45 deg bank, r = 1 rad/s, 1 s, accelerometer ignored",
                     45*M_PI/180.0, 0, 0, 0, 0, 1, 1.0, 0, 0 };
        failures += !run(&c, 0.001, TOL_DEG);
    }
    {
        case_t c = { "45 deg bank, r = 1 rad/s, 1 s, true accelerometer fed",
                     45*M_PI/180.0, 0, 0, 0, 0, 1, 1.0, 1, 0 };
        failures += !run(&c, 0.001, TOL_DEG);
    }

    printf("== sustained coupled motion, true accelerometer fed ==\n\n");
    {
        case_t c = { "45 deg bank, sustained turn (p=0 q=0.3 r=0.6, 5 s)",
                     45*M_PI/180.0, 0, 0, 0, 0.3, 0.6, 5.0, 1, 0 };
        failures += !run(&c, 0.001, TOL_DEG);
    }
    {
        case_t c = { "60 deg bank and 20 deg pitch, p=0.2 q=-0.4 r=0.9, 5 s",
                     60*M_PI/180.0, 20*M_PI/180.0, 0, 0.2, -0.4, 0.9, 5.0, 1, 0 };
        failures += !run(&c, 0.001, TOL_DEG);
    }

    printf("== sensor loss and normalisation ==\n\n");
    {
        case_t c = { "30 deg bank, accelerometer reads zero (loss), 3 s",
                     30*M_PI/180.0, 0, 0, 0, 0, 0.8, 3.0, 0, 1 };
        failures += !run(&c, 0.001, TOL_DEG);
    }
    {
        case_t c = { "level, r = 1 rad/s, 10 s, accelerometer ignored",
                     0, 0, 0, 0, 0, 1, 10.0, 0, 0 };
        failures += !run(&c, 0.001, TOL_DEG);
    }
    printf("== verdict ==\n");
    if (oracle_bad) {
        printf("   THE ORACLE ITSELF FAILED its frame check or its self-check.\n");
        printf("   Nothing above it is evidence about the estimator. Fix the\n");
        printf("   oracle first (exit 2, distinct from a real failure).\n");
        return 2;
    }
    if (failures == 0) {
        printf("   the estimator stayed within %.2f deg of the quaternion oracle\n",
               TOL_DEG);
        return 0;
    }
    printf("   %d of the cases above exceeded %.2f deg. This is F1 of the\n",
           failures, TOL_DEG);
    printf("   2026-09-18 assessment: ak_estimator_update() adds gyro[0]*dt to\n");
    printf("   roll, gyro[1]*dt to pitch and gyro[2]*dt to yaw, but body rates\n");
    printf("   are not Euler derivatives. B2's second half is the repair; this\n");
    printf("   file is its acceptance test.\n");
    return 1;
}
