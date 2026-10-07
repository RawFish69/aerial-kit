#include "ak_selftest.h"

#include "ak_flight.h"
#include "ak_math.h"
#include "ak_mixer.h"

typedef struct {
    ak_printf_fn out;
    int          failures;
} test_ctx_t;

static void check(test_ctx_t *ctx, const char *name, int passed,
                  const char *detail)
{
    if (passed) {
        ctx->out("  ok       %s%s%s\n", name, detail[0] != '\0' ? " - " : "", detail);
    } else {
        ctx->out("  FAILED   %s%s%s\n", name, detail[0] != '\0' ? " - " : "", detail);
        ctx->failures++;
    }
}

static int near(float a, float b, float tolerance)
{
    return ak_absf(a - b) <= tolerance;
}

/* Synthetic samples, so the same checks run with no hardware at all. */
static void synthetic_imu(ak_imu_sample_t *imu, uint32_t time_us, float roll,
                          float pitch)
{
    imu->gyro[0] = 0.0f;
    imu->gyro[1] = 0.0f;
    imu->gyro[2] = 0.0f;
    imu->accel[0] = -pitch;
    imu->accel[1] = roll;
    imu->accel[2] = 1.0f;
    imu->time_us = time_us;
    imu->valid = 1;
}

static void rc_frame(ak_rc_input_t *rc, uint32_t now_ms, float roll,
                     float throttle, int arm)
{
    ak_rc_config_t cfg;
    ak_rc_default_config(&cfg);

    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        rc->channel[i] = (uint16_t)cfg.mid;
    }
    rc->channel[AK_RC_ROLL] = roll >= 0.0f
        ? (uint16_t)((float)cfg.mid + roll * (float)(cfg.max - cfg.mid))
        : (uint16_t)((float)cfg.mid + roll * (float)(cfg.mid - cfg.min));
    rc->channel[AK_RC_THROTTLE] =
        (uint16_t)((float)cfg.min + throttle * (float)(cfg.max - cfg.min));
    rc->channel[AK_RC_ARM] = (uint16_t)(arm ? cfg.max : cfg.min);
    rc->last_update_ms = now_ms;
    rc->valid = 1;
}

int ak_flight_selftest(ak_printf_fn out)
{
    test_ctx_t ctx = { .out = out, .failures = 0 };
    ak_outputs_t out_buf;
    ak_imu_sample_t imu;
    ak_rc_input_t rc;
    /* Milliseconds: this self-test drives the receiver timeout, which is a
     * millisecond rule, so the counter it counts in is the millisecond one.
     * The flight core below is handed microseconds - see the conversions. */
    uint32_t t_ms = 0;

    /* Math: two results with a known right answer. */
    check(&ctx, "squareroot of 9 is 3", near(ak_sqrtf(9.0f), 3.0f, 0.001f), "");
    check(&ctx, "atan2(1,1) is pi/4",
          near(ak_atan2f(1.0f, 1.0f), AK_PI / 4.0f, 0.0005f), "");

    /* Mixer: hover is symmetric, and each torque moves the right side. */
    ak_mixer_apply(&ak_mixer_quad_x, 0.5f, 0.0f, 0.0f, 0.0f, &out_buf);
    check(&ctx, "quad hover is symmetric",
          near(out_buf.motor[0], 0.5f, 0.001f) && near(out_buf.motor[1], 0.5f, 0.001f) &&
          near(out_buf.motor[2], 0.5f, 0.001f) && near(out_buf.motor[3], 0.5f, 0.001f),
          "");

    ak_mixer_apply(&ak_mixer_quad_x, 0.5f, 0.1f, 0.0f, 0.0f, &out_buf);
    check(&ctx, "quad roll right lifts the left pair",
          out_buf.motor[2] > out_buf.motor[0] && out_buf.motor[3] > out_buf.motor[1], "");

    ak_mixer_apply(&ak_mixer_quad_x, 0.5f, 0.0f, 0.1f, 0.0f, &out_buf);
    /* A positive pitch is nose-up in this firmware's convention (see
     * ak_mixer.c), and lifting the front pair is what pitches a nose up. */
    check(&ctx, "quad nose-up lifts the front pair",
          out_buf.motor[1] > out_buf.motor[0] && out_buf.motor[3] > out_buf.motor[2], "");

    ak_mixer_apply(&ak_mixer_elevon_wing, 0.5f, 0.0f, 0.1f, 0.0f, &out_buf);
    check(&ctx, "wing pitch moves both elevons together",
          out_buf.servo[0] > 0.0f && out_buf.servo[1] > 0.0f, "");

    ak_mixer_apply(&ak_mixer_elevon_wing, 0.5f, 0.1f, 0.0f, 0.0f, &out_buf);
    /* Roll right raises the right elevon's trailing edge and lowers the left
     * (ak_mixer.h's convention): the right wing loses lift and drops. */
    check(&ctx, "wing roll right raises the right elevon and lowers the left",
          out_buf.servo[0] < 0.0f && out_buf.servo[1] > 0.0f, "");

    ak_mixer_apply(&ak_mixer_elevon_wing, 0.5f, 0.0f, 0.0f, 0.1f, &out_buf);
    check(&ctx, "wing yaw is differential thrust",
          out_buf.motor[0] > out_buf.motor[1], "");

    /* Estimator: level in, level out, with the accelerometer in charge. */
    ak_estimator_t est;
    ak_estimator_init(&est, 0.5f);
    for (int i = 0; i < 200; i++) {
        synthetic_imu(&imu, (uint32_t)(i + 1) * 1000u, 0.0f, 0.0f);
        ak_estimator_update(&est, &imu, 0.001f);
    }
    check(&ctx, "estimator holds level when level",
          near(est.roll, 0.0f, 0.01f) && near(est.pitch, 0.0f, 0.01f), "");

    /*
     * Flight loop: disarmed means zero, and arming takes the whole sequence.
     *
     * `static`, and that is a decision rather than a habit. This function runs
     * at boot on the main stack, and an `ak_flight_t` is nearly three kilobytes
     * - it carries the whole gyro chain, the dynamic notch's window and its
     * biquad bank (ak_dyn_notch.h). Two of them as locals was a six-kilobyte
     * stack frame, which `make stack-check` caught the moment roadmap 2.3 made
     * the struct that big: the image's largest frame went 1760 bytes to 6096,
     * and the pessimistic reading over the graph went 7184 to 19160 against
     * the 12288 the project allows. Moving the two objects out of the frame is
     * the fix that keeps the check's meaning; raising the limit would not have
     * been. The price is the same bytes in .bss, where they are visible in the
     * image report's own "largest things in RAM" list instead of hidden in a
     * stack requirement nobody reads.
     *
     * One object per phase rather than one reused across both, so that a later
     * edit reading the quadrotor's state after the wing has been initialised
     * cannot silently read the wing's.
     */
    static ak_flight_t flight;
    ak_flight_init(&flight, &ak_mixer_quad_x);
    /* The board's outputs, as the firmware hands them over: the self-test arms
     * an aircraft, and arming compares this against the mix. Four motors and two
     * servos is what this aircraft's mixer needs and what the board this
     * self-test stands for drives - the self-test has no board, so it states the
     * pair rather than reading it, and a board that drives fewer (the Feather
     * F405 drives two motors) is refused for the quad_x by the same gate. */
    ak_flight_set_board_outputs(&flight, 4u, 2u);

    for (int i = 0; i < 100; i++) {
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        rc_frame(&rc, t_ms, 0.0f, 0.0f, 0);
        ak_flight_step(&flight, &imu, &rc, t_ms * 1000u, t_ms);
    }
    const ak_outputs_t *o = ak_flight_outputs(&flight);
    check(&ctx, "disarmed means motors at zero and servos centred",
          o->motor[0] == 0.0f && o->motor[1] == 0.0f && o->motor[2] == 0.0f &&
          o->motor[3] == 0.0f && o->servo[0] == 0.0f && o->servo[1] == 0.0f, "");

    /* Arm: switch on, throttle low, for longer than the hold. */
    for (int i = 0; i < 700; i++) {
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        rc_frame(&rc, t_ms, 0.0f, 0.0f, 1);
        ak_flight_step(&flight, &imu, &rc, t_ms * 1000u, t_ms);
    }
    check(&ctx, "arm switch plus low throttle arms",
          flight.state == AK_FLIGHT_ARMED, "");

    /* Throttle up and roll right: motors spin, and the mix leans right. */
    for (int i = 0; i < 50; i++) {
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        rc_frame(&rc, t_ms, 0.5f, 0.5f, 1);
        ak_flight_step(&flight, &imu, &rc, t_ms * 1000u, t_ms);
    }
    o = ak_flight_outputs(&flight);
    check(&ctx, "armed, roll right lifts the left pair",
          o->motor[2] > o->motor[0] && o->motor[3] > o->motor[1], "");

    /* Loss of RC is a failsafe, and it stays latched. */
    for (int i = 0; i < 300; i++) {
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        ak_flight_step(&flight, &imu, &rc, (t_ms + 500u) * 1000u, t_ms + 500u); /* receiver has gone quiet */
    }
    o = ak_flight_outputs(&flight);
    check(&ctx, "RC loss stops the motors",
          o->motor[0] == 0.0f && o->motor[1] == 0.0f && o->motor[2] == 0.0f &&
          o->motor[3] == 0.0f, "");
    check(&ctx, "failsafe is latched, not just this frame",
          flight.state == AK_FLIGHT_FAILSAFE, "");

    for (int i = 0; i < 100; i++) {
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        rc_frame(&rc, t_ms, 0.0f, 0.0f, 1); /* arm switch still on */
        ak_flight_step(&flight, &imu, &rc, t_ms * 1000u, t_ms);
    }
    check(&ctx, "failsafe holds until the arm switch is cycled",
          flight.state == AK_FLIGHT_FAILSAFE, "");

    /*
     * And the airframe whose answer to the same event is different.
     *
     * A quadrotor that loses its link stops, because thrust is what holds it
     * up; a fixed wing that stops flying is a brick with a battery in it, so
     * it circles down instead - see the descend state in ak_flight.c. Checked
     * *here* rather than only in the host suite because this is the function a
     * board runs at boot: the aircraft with the wing on it verifies the wing's
     * failsafe every time it is switched on.
     */
    static ak_flight_t wing;  /* out of the frame for the reason above */
    ak_flight_init(&wing, &ak_mixer_elevon_wing);
    /* The airframe parameter is what picks the mix - and what the failsafe
     * reads to decide whether this aircraft can stop - so setting the pointer
     * alone would be undone by init's own resolve. */
    wing.airframe = 1u;
    ak_flight_apply_airframe(&wing);
    ak_flight_set_board_outputs(&wing, 4u, 2u);
    /* Powered up with the switch already on: arming wants it seen off first
     * (AK_ARM_SWITCH_HELD), so a board that boots armed-in-waiting stays
     * disarmed however long the switch is held. */
    for (int i = 0; i < 800; i++) {
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        rc_frame(&rc, t_ms, 0.0f, 0.0f, 1);
        ak_flight_step(&wing, &imu, &rc, t_ms * 1000u, t_ms);
    }
    check(&ctx, "a switch already on at power-up does not arm",
          wing.state == AK_FLIGHT_DISARMED, "");
    for (int i = 0; i < 20; i++) { /* the switch off ... */
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        rc_frame(&rc, t_ms, 0.0f, 0.0f, 0);
        ak_flight_step(&wing, &imu, &rc, t_ms * 1000u, t_ms);
    }
    for (int i = 0; i < 800; i++) { /* ... then on: throttle low, hold */
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        rc_frame(&rc, t_ms, 0.0f, 0.0f, 1);
        ak_flight_step(&wing, &imu, &rc, t_ms * 1000u, t_ms);
    }
    check(&ctx, "a wing arms the same way", wing.state == AK_FLIGHT_ARMED, "");

    for (int i = 0; i < 300; i++) { /* the receiver goes quiet */
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        ak_flight_step(&wing, &imu, &rc, (t_ms + 500u) * 1000u, t_ms + 500u);
    }
    const ak_outputs_t *w = ak_flight_outputs(&wing);
    check(&ctx, "a wing that loses its link circles down instead of stopping",
          wing.state == AK_FLIGHT_DESCEND && w->motor[0] > 0.0f &&
              w->motor[1] > 0.0f &&
              (w->servo[0] != 0.0f || w->servo[1] != 0.0f), "");

    /* The receiver coming back does not take it back: the switch has to be
     * cycled, which is the same rule the quadrotor's failsafe follows. A link
     * that reappeared for a frame must not hand an aircraft back in the middle
     * of a descent. */
    for (int i = 0; i < 100; i++) {
        t_ms += 1;
        synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
        rc_frame(&rc, t_ms, 0.0f, 0.0f, 1); /* receiver back, switch still on */
        ak_flight_step(&wing, &imu, &rc, t_ms * 1000u, t_ms);
    }
    check(&ctx, "and a link that comes back does not take it back mid-descent",
          wing.state == AK_FLIGHT_DESCEND, "");

    /* The ground does. The landing detector is the caller's answer, and it
     * disarms rather than latching - the same ending a return has. */
    wing.landed = 1;
    t_ms += 1;
    synthetic_imu(&imu, t_ms * 1000u, 0.0f, 0.0f);
    rc_frame(&rc, t_ms, 0.0f, 0.0f, 1);
    ak_flight_step(&wing, &imu, &rc, t_ms * 1000u, t_ms);
    check(&ctx, "and the ground disarms it",
          wing.state == AK_FLIGHT_DISARMED, "");

    return ctx.failures == 0 ? 0 : 1;
}
