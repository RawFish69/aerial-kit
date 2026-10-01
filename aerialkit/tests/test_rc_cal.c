/*
 * Receiver calibration and the stick offsets it removes.
 *
 * The failure this is about is quiet: a receiver whose centre is a few counts
 * off makes the flight core believe the pilot is holding a small amount of
 * stick, so the aircraft drifts on the bench - and a drift on the bench looks
 * exactly like an accelerometer problem. Measuring the centre takes a second
 * and removes the ambiguity.
 */

#include <math.h>
#include <stdint.h>

#include "ak_rc.h"
#include "tests.h"

static ak_rc_input_t frame_with_mid(uint16_t mid, uint16_t throttle)
{
    ak_rc_input_t input;
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        input.channel[i] = mid;
    }
    input.channel[AK_RC_THROTTLE] = throttle;
    input.last_update_ms = 0;
    input.valid = 1;
    return input;
}

static void test_offset_shows_up(void)
{
    ak_rc_config_t cfg;
    ak_rc_command_t cmd;
    ak_rc_default_config(&cfg);

    /* A receiver centred 20 counts high: the sticks are held still, and the
     * decoder reports a stick input anyway. */
    ak_rc_input_t input =
        frame_with_mid((uint16_t)(cfg.mid + 20u), (uint16_t)cfg.min);
    expect("the decoder is fooled by a receiver that is off centre",
           ak_rc_decode(&input, &cfg, &cmd) == 1 && cmd.roll > 0.01f);
}

static void test_calibration_removes_it(void)
{
    ak_rc_config_t cfg;
    ak_rc_command_t cmd;
    ak_rc_cal_t cal;
    ak_rc_default_config(&cfg);

    uint16_t receiver_mid = (uint16_t)(cfg.mid + 20u);
    ak_rc_input_t input = frame_with_mid(receiver_mid, (uint16_t)cfg.min);

    ak_rc_cal_init(&cal, 50u);
    expect("nothing is applied before a calibration", cal.done == 0);

    ak_rc_cal_start(&cal);
    for (int i = 0; i < 50; i++) {
        input.channel[AK_RC_ROLL] = (uint16_t)(receiver_mid + (i % 3) - 1);
        ak_rc_cal_feed(&cal, &input);
    }
    expect("the calibration finishes after the frames it asked for",
           cal.done == 1 && cal.samples == 50);

    int32_t offset[AK_RC_CHANNELS];
    expect("applying it reports how far each channel was off",
           ak_rc_cal_apply(&cal, &cfg, offset) == 1);
    expect("the centre moved to where the receiver actually is",
           fabsf((float)(int32_t)cfg.mid - (float)receiver_mid) <= 1.0f);
    expect("and the offsets say so", offset[AK_RC_ROLL] > 15 &&
           offset[AK_RC_ROLL] < 25);

    /* The same frame that produced a roll command before now reads zero. */
    input = frame_with_mid(receiver_mid, (uint16_t)cfg.min);
    expect("the sticks now read centred",
           ak_rc_decode(&input, &cfg, &cmd) == 1 && fabsf(cmd.roll) < 0.001f &&
           fabsf(cmd.yaw) < 0.001f);

    /* Throttle is at the bottom, not the middle: it must not drag the centre
     * downwards just because the pilot is holding it down. */
    expect("throttle does not vote on where the middle is",
           cfg.mid > cfg.mid - 60 && cfg.mid >= 900u);
}

static void test_switches_are_not_centres(void)
{
    ak_rc_config_t cfg;
    ak_rc_cal_t cal;
    ak_rc_default_config(&cfg);

    ak_rc_input_t input = frame_with_mid((uint16_t)cfg.mid, (uint16_t)cfg.min);
    input.channel[AK_RC_ARM] = 1811; /* the switch is on */

    ak_rc_cal_init(&cal, 10u);
    ak_rc_cal_start(&cal);
    for (int i = 0; i < 10; i++) {
        ak_rc_cal_feed(&cal, &input);
    }

    int32_t offset[AK_RC_CHANNELS];
    (void)ak_rc_cal_apply(&cal, &cfg, offset);
    expect("a switch held on is reported, not averaged into the centre",
           offset[AK_RC_ARM] > 800 && cfg.mid > 900u && cfg.mid < 1100u);
}

static void test_invalid_frames_are_ignored(void)
{
    ak_rc_cal_t cal;
    ak_rc_input_t input = frame_with_mid(992, 172);

    ak_rc_cal_init(&cal, 10u);
    ak_rc_cal_start(&cal);
    input.valid = 0;
    for (int i = 0; i < 20; i++) {
        ak_rc_cal_feed(&cal, &input);
    }
    expect("frames the receiver did not stamp are not counted",
           cal.samples == 0 && cal.done == 0);
}

void test_rc_calibration(void)
{
    test_offset_shows_up();
    test_calibration_removes_it();
    test_switches_are_not_centres();
    test_invalid_frames_are_ignored();
}
