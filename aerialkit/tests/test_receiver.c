/*
 * The receive path: the interrupt's ring buffer and the receiver in front of
 * the flight core.
 *
 * Both are places where the hardware can fail in ways that are invisible from
 * a distance - a dropped byte is one bad frame, a wrong baud rate is a stream
 * of bytes that fail the crc - so the counters are part of what is tested.
 *
 * The console's `rc` report is tested here too, because it is the same path
 * seen from the other end: it prints the frames this receiver assembled. It is
 * also the only place on a bench where a person can see what the sticks are
 * doing without arming anything.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crsf_fixture.h"
#include "ak_crsf.h"
#include "ak_ring.h"
#include "ak_rc_receiver.h"
#include "tests.h"

static void test_ring(void)
{
    ak_ring_t ring;
    ak_ring_init(&ring);

    uint8_t byte = 0;
    expect("an empty ring yields nothing", !ak_ring_pop(&ring, &byte));

    for (uint16_t i = 0; i < 10; i++) {
        ak_ring_push(&ring, (uint8_t)(0x40 + i));
    }
    expect("the ring counts what it holds", ak_ring_count(&ring) == 10);

    int ordered = 1;
    for (uint16_t i = 0; i < 10; i++) {
        ordered = ordered && ak_ring_pop(&ring, &byte) && byte == 0x40 + i;
    }
    expect("bytes come back in the order they went in", ordered);
    expect("and the ring is empty again", ak_ring_count(&ring) == 0);

    /* Wrap past the end of the buffer and check the sequence survives. */
    ak_ring_init(&ring);
    for (uint32_t i = 0; i < 300; i++) {
        ak_ring_push(&ring, (uint8_t)i);
    }
    /* The ring holds 255 and drops the newest 45, so what comes back starts at
     * the first byte pushed - the reader's data is never overwritten. */
    uint8_t expected = 0;
    ordered = 1;
    uint32_t popped = 0;
    while (ak_ring_pop(&ring, &byte)) {
        ordered = ordered && byte == expected;
        expected = (uint8_t)(expected + 1u);
        popped++;
    }
    expect("a full ring drops the newest bytes and keeps the sequence",
           ordered && popped == 255);
    expect("and it counts what it dropped", ring.dropped == 45);

    /* A reader that empties the ring immediately must never lose a byte. */
    ak_ring_init(&ring);
    uint32_t sent = 0;
    uint32_t got = 0;
    int intact = 1;
    for (uint32_t i = 0; i < 5000; i++) {
        ak_ring_push(&ring, (uint8_t)(i * 7u));
        sent++;
        if (ak_ring_pop(&ring, &byte)) {
            intact = intact && byte == (uint8_t)(got * 7u);
            got++;
        }
    }
    expect("push and pop interleaved lose nothing",
           intact && got == sent && ring.dropped == 0);
}

static void test_rc_receiver(void)
{
    ak_rc_receiver_t rx;
    ak_rc_receiver_init(&rx);

    uint16_t channels[AK_CRSF_CHANNELS];
    for (int i = 0; i < AK_CRSF_CHANNELS; i++) {
        channels[i] = (uint16_t)(992 + i);
    }
    channels[AK_RC_THROTTLE] = 172;  /* throttle down */
    channels[AK_RC_ROLL] = 1400;     /* stick right */
    channels[AK_RC_ARM] = 1811;      /* arm switch on */

    uint8_t frame[AK_CRSF_MAX_FRAME];
    uint8_t length = crsf_build_frame(frame, channels, AK_CRSF_TYPE_RC_CHANNELS);

    int decoded = 0;
    for (uint8_t i = 0; i < length; i++) {
        decoded = ak_rc_receiver_feed(&rx, frame[i], 1000) || decoded;
    }
    expect("a frame from the receiver decodes", decoded == 1 && rx.frames == 1);
    expect("the channels arrive in order",
           rx.channels.channel[0] == 1400 && rx.channels.channel[1] == 993 &&
           rx.channels.channel[2] == 172 && rx.channels.channel[5] == 1811);
    expect("and the frame is stamped with the time it arrived",
           rx.channels.valid == 1 && rx.channels.last_update_ms == 1000);
    expect("every byte is counted", rx.bytes == length);

    /* The sticks and switches, as the flight core will see them. */
    ak_rc_command_t cmd;
    ak_rc_config_t cfg;
    ak_rc_default_config(&cfg);
    expect("the decoder reads the frame the receiver assembled",
           ak_rc_decode(&rx.channels, &cfg, &cmd) == 1 && cmd.arm_request == 1 &&
           cmd.throttle < 0.01f && cmd.roll > 0.4f);

    /* Noise: a receiver on the wrong baud rate, or nothing plugged in. */
    for (int i = 0; i < 200; i++) {
        ak_rc_receiver_feed(&rx, (uint8_t)(0x37 + i * 11), 1100);
    }
    expect("a stream of noise decodes nothing",
           rx.frames == 1 &&
               rx.crsf.crc_errors + rx.crsf.rejected > 0);
    expect("and the last good frame is still there", rx.channels.valid == 1);
}

/* --- the console's own report --------------------------------------------- */

static char   said[2048];
static size_t said_used;

static int capture(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(said + said_used, sizeof said - said_used, fmt, ap);
    va_end(ap);
    if (written > 0 && said_used + (size_t)written < sizeof said) {
        said_used += (size_t)written;
    }
    return written;
}

/* The two lines ak_rc_receiver_report owes for this configuration.
 *
 * Built by calling ak_rc_decode rather than by repeating its arithmetic here:
 * the claim under test is that the report and the decoder agree, and a third
 * copy of the mapping written in this file would only ever agree with itself.
 * A frame the decoder refuses becomes a line that cannot be found, so the
 * comparison fails on it rather than passing against an empty string. */
static void expected_lines(const ak_rc_input_t *input, const ak_rc_config_t *cfg,
                           char *sticks, size_t sticks_len, char *switches,
                           size_t switches_len)
{
    ak_rc_command_t cmd;
    if (!ak_rc_decode(input, cfg, &cmd)) {
        snprintf(sticks, sticks_len, "sticks:    this frame does not decode");
        snprintf(switches, switches_len, "switches:  this frame does not decode");
        return;
    }
    snprintf(sticks, sticks_len,
             "sticks:    roll %d, pitch %d, yaw %d, throttle %d per-mille",
             (int)(cmd.roll * 1000.0f), (int)(cmd.pitch * 1000.0f),
             (int)(cmd.yaw * 1000.0f), (int)(cmd.throttle * 1000.0f));
    snprintf(switches, switches_len, "switches:  arm %s, mode %s",
             cmd.arm_request ? "on" : "off", cmd.angle_mode ? "angle" : "rate");
}

/*
 * The console's `rc` prints the sticks. Which configuration it printed them
 * against is the whole question, and it is not answerable by looking at the
 * numbers: a receiver and a calibration together produce them, and the report
 * has to be handed the calibration the flight core is using.
 *
 * It was not, at first - the report called ak_rc_default_config() for itself.
 * A board that had been through `calibrate rc` therefore printed one set of
 * sticks on the console and flew another, and nothing failed, because no test
 * in this tree called the report at all. That is what this test is for, and it
 * is why the second half drives a configuration that differs from the built-in
 * one in a way this frame can see.
 */
static void test_rc_report(void)
{
    ak_rc_receiver_t rx;
    ak_rc_receiver_init(&rx);

    uint16_t channels[AK_CRSF_CHANNELS];
    for (int i = 0; i < AK_CRSF_CHANNELS; i++) {
        channels[i] = (uint16_t)(992 + i);
    }
    channels[AK_RC_ROLL] = 1400;  /* stick right, far enough to matter */
    channels[AK_RC_THROTTLE] = 172;
    channels[AK_RC_ARM] = 1811;   /* arm on, against the built-in threshold */

    uint8_t frame[AK_CRSF_MAX_FRAME];
    uint8_t length = crsf_build_frame(frame, channels, AK_CRSF_TYPE_RC_CHANNELS);
    for (uint8_t i = 0; i < length; i++) {
        ak_rc_receiver_feed(&rx, frame[i], 1000);
    }
    expect("the report has a framed receiver to describe", rx.channels.valid == 1);

    ak_rc_config_t built_in, measured;
    ak_rc_default_config(&built_in);
    measured = built_in;
    /* What `calibrate rc` writes when the handset centres 50 counts off spec,
     * and an arm threshold above this frame's switch. Neither is exotic: the
     * first is the reason the calibration exists, and the second is a
     * parameter anyone can set. */
    measured.mid += 50;
    measured.arm_threshold = 1900;

    ak_rc_command_t as_built_in, as_measured;
    expect("the two configurations this test compares really do decode this "
           "frame differently, so neither half can pass by agreeing",
           ak_rc_decode(&rx.channels, &built_in, &as_built_in) == 1 &&
               ak_rc_decode(&rx.channels, &measured, &as_measured) == 1 &&
               (as_built_in.roll != as_measured.roll ||
                as_built_in.arm_request != as_measured.arm_request));

    char want_sticks[128], want_switches[64];

    said_used = 0;
    said[0] = '\0';
    ak_rc_receiver_report(&rx, &built_in, capture);
    expected_lines(&rx.channels, &built_in, want_sticks, sizeof want_sticks,
                   want_switches, sizeof want_switches);
    expect("the console's report prints the sticks the configuration it was "
           "handed decodes",
           strstr(said, want_sticks) != NULL &&
               strstr(said, want_switches) != NULL);

    said_used = 0;
    said[0] = '\0';
    ak_rc_receiver_report(&rx, &measured, capture);
    expected_lines(&rx.channels, &measured, want_sticks, sizeof want_sticks,
                   want_switches, sizeof want_switches);
    expect("and prints the measured ones when that is what it was handed, "
           "rather than falling back to the built-in defaults",
           strstr(said, want_sticks) != NULL &&
               strstr(said, want_switches) != NULL);

    /* The counters are the other half of what a person reads here, and they
     * come from the receiver rather than from any configuration. */
    expect("and counts the frame it described",
           strstr(said, "receiver:  crsf, ") != NULL &&
               strstr(said, "link:      framing") != NULL);
}

void test_receive_path(void)
{
    test_ring();
    test_rc_receiver();
    test_rc_report();
}
