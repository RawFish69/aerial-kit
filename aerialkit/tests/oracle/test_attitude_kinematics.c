/*
 * An independent oracle for ak_zyx_step() - the plant's attitude kinematics.
 *
 * The rule this file follows is the same one test_estimator_oracle.c follows:
 * nothing here may be derived from the equations under test. Truth is a
 * quaternion integrated from body rates by q' = 0.5 * q (x) (0, omega), with
 * libm's sin/cos in double and RK4; the attitude under test is produced by
 * ak_zyx_step() with the firmware's own float polynomial trigonometry. If the
 * two agree, they agree because both are right, not because they share an
 * approximation.
 *
 * Why this file exists at all. The plant's step is what every other measurement
 * in this suite is taken against, and until now the only thing asserting it was
 * the kinematics was a scratch harness in a job's temporary directory, reached
 * through an AK_ATTITUDE_TRACE hook. That is a bad property for exactly one
 * piece of the system: if the plant's attitude step were wrong, the sessions
 * that pass would be passing against a wrong world, and there would have been
 * nothing in the repository to re-run and notice.
 *
 * **Attitudes are compared as rotations, not as Euler triples.** The two
 * attitudes are turned into rotation matrices and the reported number is the
 * angle of the rotation between them, in degrees. Comparing roll, pitch and
 * heading componentwise is not a distance: near pitch = 180 degrees two triples
 * that differ by 180 in the numbers are the same attitude, which is what the
 * inverted case below measured the first time this file was run. The matrix
 * construction is a definition, not an integration - nothing in it comes from
 * the kinematics under test.
 *
 * **The residual is identified rather than tolerated.** ak_zyx_step() is a
 * first-order step and the oracle is a converged one, so the residual is
 * C*dt + B/dt: the first-order truncation error, falling with dt, plus the
 * per-step bias of doing the arithmetic in float with a polynomial sine and
 * cosine, which grows as there are more steps to pay it on. Measured, that
 * turnover is between dt = 5e-4 and dt = 2e-4 - which is why the second run
 * below halves dt rather than dividing it by ten. Halved, the residual has to
 * halve, and it does so to two figures on every case in this file. That is
 * what separates "converging to the right answer" from "converging to a
 * slightly wrong one": an error in the equation holds the residual flat as dt
 * shrinks, and would otherwise hide under a loose tolerance. Dividing dt by
 * ten measures the bias term instead of the truncation term and shows the
 * error *growing*, which is a fact about float, not about the kinematics.
 *
 * **The floor near vertical is not part of the kinematics, and is asserted
 * separately.** ak_zyx_step() clamps cos(pitch) away from zero, so within about
 * 3 degrees of vertical it deliberately departs from the relation this file
 * oracles. The cases below stay outside that band, and `near_vertical` asserts
 * the clamp's actual behaviour rather than letting it hide inside a tolerance.
 *
 * **Level flight cannot tell the right arithmetic from the wrong one, and that
 * is asserted, not skipped.** With roll and pitch both zero the ZYX relation
 * collapses to roll_dot = p, pitch_dot = q, yaw_dot = r - exactly the defect
 * that lived in both fw_sim.c and ak_estimator.c. The `level` case below
 * asserts that the two agree there, because that equivalence is why the defect
 * survived every session that stayed near level. Every other case asserts the
 * opposite: the wrong arithmetic has to FAIL.
 */
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <string.h>

#include "attitude_step.h"

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
    double cr = cos(roll*0.5),  sr = sin(roll*0.5);
    double cp = cos(pitch*0.5), sp = sin(pitch*0.5);
    double cy = cos(yaw*0.5),   sy = sin(yaw*0.5);
    quat_t q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

/* q' = 0.5 * q (x) (0, omega), RK4, renormalised. The reason it is RK4 and not
 * one first-order step is above: the angles under test come from a first-order
 * step, so the oracle has to be the converged answer for the difference between
 * them to mean anything. */
static quat_t q_deriv(quat_t q, double p, double qq, double r)
{
    quat_t omega = { 0.0, p, qq, r };
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

/* ------------------------------------------------------------- rotations */

typedef double mat3_t[3][3];

/* R = Rz(yaw) Ry(pitch) Rx(roll): ZYX, the order ak_zyx_step() integrates.
 * Written out from that definition, which is not the relation under test. */
static void rot_from_euler_zyx(double roll, double pitch, double yaw, mat3_t R)
{
    double cr = cos(roll),  sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);

    R[0][0] = cy*cp;  R[0][1] = cy*sp*sr - sy*cr;  R[0][2] = cy*sp*cr + sy*sr;
    R[1][0] = sy*cp;  R[1][1] = sy*sp*sr + cy*cr;  R[1][2] = sy*sp*cr - cy*sr;
    R[2][0] = -sp;    R[2][1] = cp*sr;             R[2][2] = cp*cr;
}

static void rot_from_quat(quat_t q, mat3_t R)
{
    double w=q.w, x=q.x, y=q.y, z=q.z;
    R[0][0] = 1-2*(y*y+z*z);  R[0][1] = 2*(x*y-w*z);    R[0][2] = 2*(x*z+w*y);
    R[1][0] = 2*(x*y+w*z);    R[1][1] = 1-2*(x*x+z*z);  R[1][2] = 2*(y*z-w*x);
    R[2][0] = 2*(x*z-w*y);    R[2][1] = 2*(y*z+w*x);    R[2][2] = 1-2*(x*x+y*y);
}

/* How far apart two attitudes are, in degrees: the angle of the rotation that
 * takes one to the other. This is a distance on the rotation group, so it is
 * the same number whatever the Euler triples happen to read - which is what
 * makes the inverted case below mean something.
 *
 * From sin and cos of that angle rather than from acos of the cosine alone.
 * acos is ill-conditioned near 1, so an acos-only version cannot resolve a
 * difference below about 1e-6 deg however exactly the two matrices agree - a
 * threshold no correct implementation could reach, which is a trap this file
 * fell into on its first run. atan2 of the pair is well conditioned at both
 * ends. */
static double attitude_error_deg(const mat3_t A, const mat3_t B)
{
    double R[3][3], tr = 0.0, s, c;
    int i, k;
    for (i = 0; i < 3; i++)
        for (k = 0; k < 3; k++) {
            int m;
            R[i][k] = 0.0;
            for (m = 0; m < 3; m++) R[i][k] += A[m][i] * B[m][k];  /* A^T B */
        }
    for (i = 0; i < 3; i++) tr += R[i][i];
    s = 0.5 * sqrt((R[2][1]-R[1][2])*(R[2][1]-R[1][2]) +
                   (R[0][2]-R[2][0])*(R[0][2]-R[2][0]) +
                   (R[1][0]-R[0][1])*(R[1][0]-R[0][1]));
    c = (tr - 1.0) * 0.5;
    return atan2(s, c) * 180.0 / M_PI;
}

/* ------------------------------------------------------------------ cases */

typedef struct {
    const char *name;
    double roll0_deg, pitch0_deg, heading0_deg;
    double p, q, r_dps;   /* constant body rates: rad/s, rad/s, deg/s */
    double seconds;
    /* 1: the Euler-as-body arithmetic - the defect that lived in both fw_sim.c
     * and ak_estimator.c - must be far from the truth in this case, or the case
     * discriminates nothing. The level case, where the two are required to
     * agree, is not a case: see level_equivalence() below. */
} case_t;

/* The oracle's answer at this dt: the converged attitude, as a rotation. */
static void oracle_run(const case_t *c, double dt, mat3_t R)
{
    double r_rad = c->r_dps * M_PI / 180.0;
    quat_t q = q_from_euler_zyx(c->roll0_deg * M_PI / 180.0,
                                c->pitch0_deg * M_PI / 180.0,
                                c->heading0_deg * M_PI / 180.0);
    long n = (long)(c->seconds / dt);
    long i;
    for (i = 0; i < n; i++) {
        q = q_step_rk4(q, c->p, c->q, r_rad, dt);
    }
    rot_from_quat(q, R);
}

/* The step under test at the same dt - and, with `wrong` set, the arithmetic
 * both the plant and the estimator used to have: each body rate added to the
 * like-named angle. */
static void under_test(const case_t *c, double dt, int wrong, mat3_t R)
{
    ak_euler_t att;
    long n = (long)(c->seconds / dt);
    long i;

    att.roll = (float)(c->roll0_deg * M_PI / 180.0);
    att.pitch = (float)(c->pitch0_deg * M_PI / 180.0);
    att.heading_deg = (float)c->heading0_deg;

    for (i = 0; i < n; i++) {
        if (wrong) {
            att.roll += (float)(c->p * dt);
            att.pitch += (float)(c->q * dt);
            att.heading_deg += (float)(c->r_dps * dt);
            while (att.heading_deg >= 360.0f) att.heading_deg -= 360.0f;
            while (att.heading_deg < 0.0f)    att.heading_deg += 360.0f;
        } else {
            ak_zyx_step(&att, (float)c->p, (float)c->q, (float)c->r_dps, (float)dt);
        }
    }
    rot_from_euler_zyx((double)att.roll, (double)att.pitch,
                       (double)att.heading_deg * M_PI / 180.0, R);
}

static double err_deg(const case_t *c, double dt, int wrong)
{
    mat3_t truth, got;
    oracle_run(c, dt, truth);
    under_test(c, dt, wrong, got);
    return attitude_error_deg(truth, got);
}

/* Converging to the right answer, or converging to a slightly wrong one? A
 * first-order step's truncation error is proportional to dt, so halving dt has
 * to halve the residual; an error in the equation does not fall at all.
 *
 * Measured at 4e-3 -> 2e-3, not at the 1e-3 the accuracy cases use. The
 * residual is C*dt + B/dt and the crossover moves with the size of the rates:
 * for the 3 rad/s roll case it sits at about 1e-3, so halving dt *there*
 * measures the bias term and reports the residual flat. At 4e-3 every case in
 * this file is truncation-dominated and every one of them halves to two
 * figures. See the header. */
#define ORDER_DT    0.004
#define ORDER_FACTOR 0.6

static int integration_order(double dt)
{
    size_t i;
    int ok = 1;
    static const struct { const char *name; case_t c; } cs[] = {
        { "45 deg bank, pure yaw rate",         { "", 45, 0,   0,   0,   0,   60, 5.0 } },
        { "45 deg bank with pitch rate",        { "", 45, 0,   0,   0,   0.3, 30, 5.0 } },
        { "60 deg bank, 20 deg pitch, mixed",   { "", 60, 20,  0,   0.2, -0.4, 50, 5.0 } },
        { "heading across the wrap",            { "", 10, 5,   350, 0,   0.1, 60, 5.0 } },
        { "inverted at 170 deg pitch",          { "", 30, 170, 0,   0,   0.2, 40, 5.0 } },
        { "steep at 80 deg pitch",              { "", 15, 80,  0,   0,   0.15, 30, 5.0 } },
        { "rolling hard at 3 rad/s",            { "", 60, 10,  0,   3.0, 0,   20, 5.0 } },
    };

    printf("== the integration order, where the truncation term is visible ==\n");
    printf("   residual at dt = %.0e against dt = %.0e; halving dt has to halve\n",
           dt, dt / 2.0);
    printf("   it, or the equation is wrong in a way the tolerance would hide\n\n");

    for (i = 0; i < sizeof(cs)/sizeof(cs[0]); i++) {
        double e1 = err_deg(&cs[i].c, dt, 0);
        double e2 = err_deg(&cs[i].c, dt / 2.0, 0);
        double ratio = e1 > 0.0 ? e2 / e1 : 1.0;
        if (ratio > ORDER_FACTOR) {
            printf("   FAIL  %-36s %8.4f -> %.4f  (ratio %.2f)\n",
                   cs[i].name, e1, e2, ratio);
            ok = 0;
        } else {
            printf("   ok    %-36s %8.4f -> %.4f  (ratio %.2f)\n",
                   cs[i].name, e1, e2, ratio);
        }
    }
    return ok;
}

/* --------------------------------------------------- level is not a case */

/* With roll and pitch both zero the ZYX relation collapses to roll_dot = p,
 * pitch_dot = q, yaw_dot = r - the three body rates *are* the three Euler
 * rates, which is precisely the arithmetic both fw_sim.c and ak_estimator.c
 * used to have. Two consequences, and both are asserted here rather than left
 * as a hole:
 *
 * 1. The wrong arithmetic is not wrong at level. This case cannot tell the two
 *    apart, so it is not asked to; it asserts that they agree, and the reason
 *    that matters is that it is why the defect survived. Every session that
 *    stayed near level agreed with the oracle.
 * 2. All three rates are constant, so the first-order step is *exact* and the
 *    truncation error is identically zero. The residual that remains is float
 *    bias alone, which grows as dt falls. Asking this case to converge would
 *    be asking it to measure the one term that is not there. */
static int level_equivalence(void)
{
    case_t c = { "level, pure yaw rate", 0, 0, 0, 0, 0, 60, 5.0 };
    double dt = 0.001;
    double right = err_deg(&c, dt, 0);
    double wrong = err_deg(&c, dt, 1);
    double finer = err_deg(&c, dt / 2.0, 0);
    int ok = 1;

    printf("== level flight, where the two arithmetics are the same one ==\n\n");
    printf("   the kinematics says  %.6f deg from the oracle\n", right);
    printf("   the defect says      %.6f deg from the oracle\n", wrong);

    if (wrong > 0.5) {
        printf("   FAIL the Euler-as-body arithmetic was expected to agree at\n");
        printf("   level and it is %.4f deg out. If it disagrees here, the two\n", wrong);
        printf("   arithmetics differ somewhere they should not.\n");
        ok = 0;
    } else {
        printf("   and: they agree. Level flight cannot distinguish them, which\n");
        printf("   is why a defect in this step survived every session that\n");
        printf("   stayed near level.\n");
    }

    if (finer > right * 1.5) {
        printf("   FAIL the residual grew more than half again when dt halved\n");
        printf("   (%.6f -> %.6f). It is expected to grow slightly: the\n", right, finer);
        printf("   truncation error is identically zero here, so what is left is\n");
        printf("   float bias, paid once per step.\n");
        ok = 0;
    } else {
        printf("   and the residual %.6f -> %.6f as dt halved: rising, because\n",
               right, finer);
        printf("   the truncation error is exactly zero here and only the\n");
        printf("   per-step float bias is left.\n");
    }
    return ok;
}

/* The accuracy of the step at the dt the simulator actually uses, and whether
 * the case can tell the kinematics from the defect at all. */
static int run(const case_t *c, double dt, double tol_deg)
{
    double err = err_deg(c, dt, 0);
    double bad = err_deg(c, dt, 1);
    int ok = 1;

    printf("   %-52s err %8.4f deg\n", c->name, err);

    if (err > tol_deg) {
        printf("         FAIL over the %.2f deg tolerance - this is not the\n",
               tol_deg);
        printf("         ZYX kinematics, and every session measured against this\n");
        printf("         plant was measured against a wrong world.\n");
        ok = 0;
    }
    if (bad <= tol_deg) {
        printf("         FAIL not discriminating: the Euler-as-body control is\n");
        printf("         within %.2f deg (%.4f) here, so this case cannot tell\n",
               tol_deg, bad);
        printf("         the two arithmetics apart and proves nothing.\n");
        ok = 0;
    } else {
        printf("         the Euler-as-body arithmetic is %.2f deg out here, so\n", bad);
        printf("         this case does tell them apart\n");
    }
    return ok;
}

/* ------------------------------------------------------------- the floor */

/* ak_zyx_step() clamps cos(pitch) to +/-0.05, a departure from the kinematics
 * asserted above - deliberate, and documented in attitude_step.h. Near vertical
 * the two disagree by design, so this asserts what the clamp actually does
 * rather than pretending it is kinematics. */
static int near_vertical(void)
{
    ak_euler_t att;
    int ok = 1;

    /* Nose vertical: cos(pitch) is 0, the clamp makes it 0.05, so the heading
     * rate is finite rather than a division by zero. */
    att.roll = 0.0f;
    att.pitch = (float)(90.0 * M_PI / 180.0);
    att.heading_deg = 0.0f;
    ak_zyx_step(&att, 0.0f, 0.0f, 90.0f, 0.001f);
    if (!(att.heading_deg == att.heading_deg)) {
        printf("   FAIL  vertical: heading is NaN\n");
        ok = 0;
    } else if (att.heading_deg <= 0.0f) {
        printf("   FAIL  vertical: heading did not advance (%.4f)\n",
               (double)att.heading_deg);
        ok = 0;
    } else {
        printf("   ok    vertical: clamped, finite, heading %.4f deg in one step\n",
               (double)att.heading_deg);
    }

    /* And the wrap: a negative heading comes back into [0, 360). */
    att.roll = 0.0f;
    att.pitch = 0.0f;
    att.heading_deg = 0.5f;
    ak_zyx_step(&att, 0.0f, 0.0f, -90.0f, 0.01f);
    if (att.heading_deg < 0.0f || att.heading_deg >= 360.0f) {
        printf("   FAIL  wrap: heading %.4f is outside [0, 360)\n",
               (double)att.heading_deg);
        ok = 0;
    } else {
        printf("   ok    wrap: heading %.4f deg is inside [0, 360)\n",
               (double)att.heading_deg);
    }
    return ok;
}

/* ------------------------------------------------------------------ main */

static int oracle_bad;

/* The oracle's own frames, before anything is measured. Two independent things
 * have to hold, or a disagreement later is a disagreement about frames rather
 * than about propagation: the quaternion round-trips through the Euler triple,
 * and the quaternion's matrix agrees with the matrix built straight from the
 * triple. The second is what the comparisons below actually use. */
static void oracle_frame_check(void)
{
    const double angles[][3] = {
        { 0, 0, 0 }, { 30, 0, 0 }, { 0, 20, 0 }, { 0, 0, 45 },
        { 45, -10, 170 }, { -60, 25, -100 }, { 10, 5, 359 },
        { 30, 170, 0 }, { 0, 90, 0 }, { 180, -45, 90 },
    };
    size_t i;
    for (i = 0; i < sizeof(angles)/sizeof(angles[0]); i++) {
        double r = angles[i][0]*M_PI/180.0, p = angles[i][1]*M_PI/180.0;
        double y = angles[i][2]*M_PI/180.0;
        quat_t q = q_from_euler_zyx(r, p, y);
        mat3_t from_triple, from_quat;
        double d;

        rot_from_euler_zyx(r, p, y, from_triple);
        rot_from_quat(q, from_quat);
        d = attitude_error_deg(from_triple, from_quat);
        if (d > 1e-9) {            printf("   ORACLE FRAME CHECK FAILED at (%.0f, %.0f, %.0f): "
                   "the quaternion and the triple differ by %.3e deg\n",
                   angles[i][0], angles[i][1], angles[i][2], d);
            oracle_bad = 1;
        }

    }
    printf("   oracle frame check: %zu attitudes, quaternion matrix against\n",
           sizeof(angles)/sizeof(angles[0]));
    printf("   the matrix built from the triple, all within 1e-9 deg\n");
}

int main(void)
{
    /* The plant is first-order and the oracle is converged, so this is the
     * measured size of that difference at dt, with headroom. The halving
     * assertion in run() is what keeps it from being a hiding place: the
     * Euler-as-body control is 44 to 179 degrees out on these cases, so the
     * margin between a pass and a wrong equation is two to three orders of
     * magnitude. */
    const double TOL_DEG = 0.5;
    const double DT = 0.001;
    int failures = 0;

    printf("== the oracle's own frames, before anything is measured ==\n");
    oracle_frame_check();

    printf("\n== the plant's attitude step against the quaternion oracle ==\n");
    printf("   dt = %.4f s; libm doubles and RK4 against the firmware's float\n",
           DT);
    printf("   polynomials and one first-order step\n\n");
    {
        case_t c = { "45 deg bank, pure yaw rate (r = 60 deg/s, 5 s)",
                     45, 0, 0, 0, 0, 60, 5.0 };
        failures += !run(&c, DT, TOL_DEG);
    }
    {
        case_t c = { "45 deg bank, pitch rate too (q = 0.3, r = 30 deg/s, 5 s)",
                     45, 0, 0, 0, 0.3, 30, 5.0 };
        failures += !run(&c, DT, TOL_DEG);
    }
    {
        case_t c = { "60 deg bank and 20 deg pitch, p=0.2 q=-0.4 r=50 deg/s, 5 s",
                     60, 20, 0, 0.2, -0.4, 50, 5.0 };
        failures += !run(&c, DT, TOL_DEG);
    }
    {
        case_t c = { "heading wrap: 350 deg start, r = 60 deg/s, 5 s",
                     10, 5, 350, 0, 0.1, 60, 5.0 };
        failures += !run(&c, DT, TOL_DEG);
    }
    {
        case_t c = { "inverted: 30 deg bank, 170 deg pitch, r = 40 deg/s, 5 s",
                     30, 170, 0, 0, 0.2, 40, 5.0 };
        failures += !run(&c, DT, TOL_DEG);
    }
    {
        case_t c = { "steep but outside the clamp: 80 deg pitch, r = 30 deg/s, 5 s",
                     15, 80, 0, 0, 0.15, 30, 5.0 };
        failures += !run(&c, DT, TOL_DEG);
    }
    {
        case_t c = { "rolling hard: p = 3.0 rad/s, 60 deg bank, 5 s",
                     60, 10, 0, 3.0, 0, 20, 5.0 };
        failures += !run(&c, DT, TOL_DEG);
    }

    printf("\n");
    failures += !integration_order(ORDER_DT);

    printf("\n");
    failures += !level_equivalence();

    printf("\n== the clamp near vertical, asserted rather than hidden ==\n\n");
    failures += !near_vertical();

    printf("\n== verdict ==\n");
    if (oracle_bad) {
        printf("   THE ORACLE ITSELF FAILED its frame check.\n");
        printf("   Nothing above it is evidence about the plant. Fix the oracle\n");
        printf("   first (exit 2, distinct from a real failure).\n");
        return 2;
    }
    if (failures == 0) {
        printf("   ak_zyx_step() agreed with the quaternion oracle on every case at\n");
        printf("   the dt the simulator uses; the residual halved every time dt\n");
        printf("   halved, so what is left of it is the integrator's order and not\n");
        printf("   a wrong equation; and every case told the Euler-as-body defect\n");
        printf("   apart from the kinematics - except at level, where they are the\n");
        printf("   same arithmetic, which is asserted rather than papered over.\n");
        printf("   The plant's attitude step is the ZYX kinematics.\n");
        return 0;
    }
    printf("   %d check(s) above did not hold.\n", failures);
    return 1;
}
