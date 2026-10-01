/*
 * SBUS, against frames built the way the protocol says they are built.
 *
 * The reference this is checked against is Betaflight's sbus.c and
 * sbus_channels.h, and the facts taken from it are the ones that are easy to
 * get wrong: the header and the footer, eleven bits per channel little-endian
 * from bit 11*n, the channel data being twenty-two bytes and the frame
 * twenty-five, and which bits of the flags byte mean what.
 *
 * The interesting half is not the unpacking, though. It is what a frame with
 * the receiver's failsafe bit set does: it must not look like a live link,
 * because a receiver that has lost its transmitter keeps sending frames. A
 * timeout cannot see that, and the flight core relies on this parser to.
 */

#include <stdio.h>

#include "ak_sbus.h"
#include "tests.h"

/* A frame with every channel at a value we choose, packed the way a receiver
 * packs them: channel n's eleven bits start at bit 11*n of the 22 data bytes,
 * least significant bit first. */
static void build_frame(uint8_t frame[AK_SBUS_FRAME],
                        const uint16_t channels[16], uint8_t flags)
{
    for (unsigned i = 0; i < AK_SBUS_FRAME; i++) {
        frame[i] = 0u;
    }
    frame[0] = AK_SBUS_HEADER;
    frame[AK_SBUS_FRAME - 2u] = flags;
    frame[AK_SBUS_FRAME - 1u] = AK_SBUS_FOOTER;

    for (unsigned n = 0; n < 16; n++) {
        unsigned bit = 11u * n;
        unsigned index = 1u + bit / 8u; /* the data starts after the header */
        unsigned shift = bit % 8u;
        uint32_t value = (uint32_t)(channels[n] & 0x7FFu) << shift;

        frame[index] |= (uint8_t)(value & 0xFFu);
        frame[index + 1u] |= (uint8_t)((value >> 8) & 0xFFu);
        frame[index + 2u] |= (uint8_t)((value >> 16) & 0xFFu);
    }
}

static int feed_frame(ak_sbus_t *sbus, const uint8_t frame[AK_SBUS_FRAME],
                      ak_rc_input_t *out, uint32_t now_ms)
{
    int decoded = 0;
    for (unsigned i = 0; i < AK_SBUS_FRAME; i++) {
        decoded = ak_sbus_feed(sbus, frame[i], out, now_ms);
    }
    return decoded;
}

static void test_the_frame(void)
{
    uint16_t channels[16];
    uint8_t frame[AK_SBUS_FRAME];
    ak_rc_input_t out;
    ak_sbus_t sbus;

    /* Sixteen channels that are all different, so a packing bug in either
     * direction shows up as a wrong channel rather than as a coincidence. */
    for (unsigned n = 0; n < 16; n++) {
        channels[n] = (uint16_t)(200u + n * 97u);
    }
    build_frame(frame, channels, 0u);

    expect("a frame is twenty-five bytes", AK_SBUS_FRAME == 25u);
    expect("and starts with 0x0F and ends with 0x00",
           frame[0] == 0x0F && frame[24] == 0x00);

    uint16_t unpacked[AK_SBUS_CHANNELS];
    ak_sbus_unpack(frame, unpacked);
    int same = 1;
    for (unsigned n = 0; n < 16; n++) {
        same = same && unpacked[n] == channels[n];
    }
    expect("sixteen channels survive the packing", same);

    /* And the round trip is not hiding a shift: the eleventh bit of the last
     * channel is the last bit of the data, so a frame that keeps it is a frame
     * that read the window right. */
    for (unsigned n = 0; n < 16; n++) {
        channels[n] = 0x7FFu;
    }
    build_frame(frame, channels, 0u);
    ak_sbus_unpack(frame, unpacked);
    same = 1;
    for (unsigned n = 0; n < 16; n++) {
        same = same && unpacked[n] == 0x7FFu;
    }
    expect("all eleven bits of every channel, at the top of the range", same);

    /* The counts a real receiver sends, at the two ends and the middle. */
    channels[0] = 172u;  /* stick fully one way */
    channels[1] = 992u;  /* centred */
    channels[2] = 1811u; /* fully the other way */
    for (unsigned n = 3; n < 16; n++) {
        channels[n] = 992u;
    }
    build_frame(frame, channels, 0u);

    ak_sbus_init(&sbus);
    expect("a good frame is a frame the parser takes",
           feed_frame(&sbus, frame, &out, 1000u) == 1);
    expect("and it has the channels that were packed into it",
           out.channel[0] == 172u && out.channel[1] == 992u &&
               out.channel[2] == 1811u);
    expect("stamped with the time it arrived",
           out.last_update_ms == 1000u && out.valid == 1);
    expect("and counted", sbus.frames == 1u && sbus.rejected == 0u);
}

static void test_the_flags(void)
{
    uint16_t channels[16];
    uint8_t frame[AK_SBUS_FRAME];
    ak_rc_input_t out;
    ak_sbus_t sbus;

    for (unsigned n = 0; n < 16; n++) {
        channels[n] = 992u;
    }

    /* The receiver's own failsafe. The channels in the frame are whatever the
     * receiver was configured to send when it lost the transmitter - often
     * throttle down, sometimes the last position, and there is no telling
     * which - so they are not a command from anybody, and this is the whole
     * reason the flags byte is read at all. */
    ak_sbus_init(&sbus);
    out.last_update_ms = 500u;
    out.valid = 1;
    build_frame(frame, channels, AK_SBUS_FLAG_FAILSAFE);
    expect("a failsafe frame is not a command",
           feed_frame(&sbus, frame, &out, 1000u) == 0);
    expect("and it does not refresh the link's timestamp",
           out.last_update_ms == 500u);
    expect("but it is counted, and it is visible",
           sbus.failsafe_frames == 1u && sbus.failsafe == 1 && sbus.frames == 0u);

    /* A normal frame clears it again, which is what a receiver coming back to
     * life looks like. */
    build_frame(frame, channels, 0u);
    expect("and a frame with no flags is live again",
           feed_frame(&sbus, frame, &out, 1014u) == 1);
    expect("with the timestamp moving", out.last_update_ms == 1014u);
    expect("and the failsafe flag cleared", sbus.failsafe == 0);

    /* Lost frames are not a failsafe: the channels are still the pilot's, and
     * a receiver that is dropping frames is worth counting rather than
     * acting on. */
    ak_sbus_init(&sbus);
    build_frame(frame, channels, AK_SBUS_FLAG_SIGNAL_LOSS);
    expect("a frame with the signal-loss bit is still a command",
           feed_frame(&sbus, frame, &out, 2000u) == 1);
    expect("and it is counted separately", sbus.lost_frames == 1u);

    /* Both bits at once: the failsafe wins, because that is the one that says
     * the pilot is not there. */
    ak_sbus_init(&sbus);
    build_frame(frame, channels,
                AK_SBUS_FLAG_SIGNAL_LOSS | AK_SBUS_FLAG_FAILSAFE);
    expect("with both bits set the failsafe is the one that counts",
           feed_frame(&sbus, frame, &out, 3000u) == 0 &&
               sbus.failsafe_frames == 1u && sbus.lost_frames == 1u);
}

static void test_framing(void)
{
    uint16_t channels[16];
    uint8_t frame[AK_SBUS_FRAME];
    uint8_t bad[AK_SBUS_FRAME];
    ak_rc_input_t out;
    ak_sbus_t sbus;

    for (unsigned n = 0; n < 16; n++) {
        channels[n] = 992u;
    }
    build_frame(frame, channels, 0u);

    /* Noise, which is what a receiver on the wrong baud rate looks like. The
     * run is chosen so that none of it is 0x0F - a header byte starts a frame
     * whether or not it was meant as one, and that is the correct behaviour to
     * have but a different check. */
    ak_sbus_init(&sbus);
    for (int i = 0; i < 200; i++) {
        ak_sbus_feed(&sbus, (uint8_t)(0x40 + i), &out, 1000u);
    }
    expect("two hundred bytes of noise are two hundred rejected",
           sbus.frames == 0u && sbus.rejected == 200u);

    /* A frame whose last byte is wrong. The header and footer are the only
     * thing SBUS gives a parser to check, so this is the check that has to
     * work - there is no crc behind it. */
    for (unsigned i = 0; i < AK_SBUS_FRAME; i++) {
        bad[i] = frame[i];
    }
    bad[AK_SBUS_FRAME - 1u] = 0x5Au;
    ak_sbus_init(&sbus);
    expect("a frame with the wrong last byte is refused",
           feed_frame(&sbus, bad, &out, 1000u) == 0 && sbus.frames == 0u &&
               sbus.rejected == 1u);

    /* And the parser recovers: the next good frame is a good frame. */
    expect("and the next good frame is taken",
           feed_frame(&sbus, frame, &out, 1014u) == 1 && sbus.frames == 1u);

    /* A frame that stops half way, with a long silence after it. The next
     * frame is not stitched onto the end of the one that died. */
    ak_sbus_init(&sbus);
    for (unsigned i = 0; i < 12u; i++) {
        ak_sbus_feed(&sbus, frame[i], &out, 1000u);
    }
    expect("half a frame is not a frame",
           feed_frame(&sbus, frame, &out, 1000u + AK_SBUS_GAP_MS + 1u) == 1);
    expect("and the half that died was counted",
           sbus.rejected == 1u && sbus.frames == 1u);

    /* Bytes that arrive between frames are not a frame either - but 0x0F
     * arriving alone is a frame that has not finished yet, not a rejection. */
    ak_sbus_init(&sbus);
    ak_sbus_feed(&sbus, AK_SBUS_HEADER, &out, 1000u);
    expect("a header with nothing behind it is not yet anything",
           sbus.rejected == 0u && sbus.frames == 0u && sbus.held == 1u);
}

void test_sbus(void)
{
    test_the_frame();
    test_the_flags();
    test_framing();
}
