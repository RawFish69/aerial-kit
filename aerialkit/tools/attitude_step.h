#ifndef AK_TOOLS_ATTITUDE_STEP_H
#define AK_TOOLS_ATTITUDE_STEP_H

/*
 * Where the aircraft ends up pointing, from how fast it is turning.
 *
 * This is lifted out of fw_sim.c so that something other than fw_sim.c can
 * check it. It stayed there for as long as it did because it was three lines of
 * arithmetic in the middle of a 6,000-line simulator, and the cost of that was
 * that the only thing asserting it was the kinematics was a scratch harness in
 * a job's temporary directory - see the ledger's item 6. The kinematics of the
 * plant is the thing every other measurement in the suite is taken against, so
 * it is not a good candidate for being the one part nobody can re-run.
 *
 * It lives in tools/ rather than in src/core/flight/ on purpose, and the reason
 * is not tidiness: this arithmetic belongs to the *simulator*. The firmware
 * never integrates Euler angles from body rates - its estimator carries a
 * quaternion - so putting this in the flight core would compile simulator code
 * into the image the boards run, on a target where B5 measured the flash
 * budget. Every .c under tools/ is already globbed into TOOL_OBJS, so a file
 * here is built by the existing rules and linked only by the tools and tests
 * that ask for it.
 *
 * The rates are *body* rates: rotation about the aircraft's own three axes. A
 * moment produces them, a gyro reports them, and they are not the rates at
 * which the roll, pitch and yaw angles change. Level, the two triples are the
 * same three numbers; banked or pitched they are not, and fw_sim.c treated them
 * as the same for as long as it has existed - as did ak_estimator.c, which
 * added gyro[0]*dt to roll. Both being wrong the same way is exactly why no
 * session ever showed it.
 *
 * Attitude is Euler angles in the ZYX order the rest of the repository uses -
 * roll about the body x axis, then pitch about the new y, then yaw about the
 * new z - so the angles move as
 *
 *     roll_dot  = p + q sin(roll) tan(pitch) + r cos(roll) tan(pitch)
 *     pitch_dot = q cos(roll) - r sin(roll)
 *     yaw_dot   = (q sin(roll) + r cos(roll)) / cos(pitch)
 *
 * and ak_zyx_step() is that relation, integrated one step from the rates the
 * two airframes produce.
 *
 * That relation is singular with the nose vertical, and the step floors
 * cos(pitch) away from zero rather than dividing by it. Neither airframe flies
 * with its nose vertical, and a scenario that has gone wrong should produce a
 * wrong number rather than a division by zero that spreads a NaN through every
 * session after it. **The floor is a deliberate departure from the kinematics,
 * so a test that asserts the kinematics has to stay outside the region where it
 * applies** - `test_attitude_kinematics.c` does, and asserts the floor
 * separately rather than letting it hide inside a tolerance.
 */

typedef struct {
    float roll;        /* radians */
    float pitch;       /* radians */
    float heading_deg; /* degrees, wrapped to [0, 360) */
} ak_euler_t;

/* Advance `att` by dt seconds under body rates p and q in rad/s and r in
 * degrees per second, in place. */
void ak_zyx_step(ak_euler_t *att, float p, float q, float r_dps, float dt);

#endif
