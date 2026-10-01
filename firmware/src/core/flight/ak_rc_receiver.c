#include "ak_rc_receiver.h"

void ak_rc_receiver_init(ak_rc_receiver_t *rx)
{
    rx->protocol = AK_RC_PROTOCOL_CRSF;
    ak_crsf_init(&rx->crsf);
    ak_sbus_init(&rx->sbus);
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        rx->channels.channel[i] = 0;
    }
    rx->channels.last_update_ms = 0;
    rx->channels.valid = 0;
    rx->bytes = 0;
    rx->frames = 0;
}

void ak_rc_receiver_set_protocol(ak_rc_receiver_t *rx, uint32_t protocol)
{
    rx->protocol = protocol;
}

uint32_t ak_rc_receiver_protocol(const ak_rc_receiver_t *rx)
{
    return rx->protocol;
}

const char *ak_rc_protocol_name(uint32_t protocol)
{
    return protocol == AK_RC_PROTOCOL_SBUS ? "sbus" : "crsf";
}

int ak_rc_receiver_feed(ak_rc_receiver_t *rx, uint8_t byte, uint32_t now_ms)
{
    rx->bytes++;
    int decoded = rx->protocol == AK_RC_PROTOCOL_SBUS
                      ? ak_sbus_feed(&rx->sbus, byte, &rx->channels, now_ms)
                      : ak_crsf_feed(&rx->crsf, byte, &rx->channels, now_ms);
    if (decoded) {
        rx->frames++;
    }
    return decoded;
}

void ak_rc_receiver_report(const ak_rc_receiver_t *rx, const ak_rc_config_t *cfg,
                           ak_printf_fn out)
{
    out("receiver:  %s, %u bytes, %u frames\n",
        ak_rc_protocol_name(rx->protocol), rx->bytes, rx->frames);
    if (rx->protocol == AK_RC_PROTOCOL_SBUS) {
        out("sbus:      %u good, %u failsafe, %u with lost frames, %u rejected\n",
            rx->sbus.frames, rx->sbus.failsafe_frames, rx->sbus.lost_frames,
            rx->sbus.rejected);
        if (rx->sbus.failsafe) {
            out("           the receiver's own failsafe is active: it has lost\n"
                "           its transmitter, and the link is treated as gone\n");
        }
    } else {
        out("crsf:      %u crc errors, %u rejected\n", rx->crsf.crc_errors,
            rx->crsf.rejected);
    }
    out("link:      %s\n", rx->channels.valid ? "framing" : "no frames yet");

    if (!rx->channels.valid) {
        return;
    }

    /* Counts first, because a raw count out of range is the first sign of a
     * receiver on the wrong baud rate or the wrong protocol. */
    out("channels:  ");
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        out("%u ", rx->channels.channel[i]);
    }
    out("\n");

    ak_rc_command_t cmd;
    if (ak_rc_decode(&rx->channels, cfg, &cmd)) {
        out("sticks:    roll %d, pitch %d, yaw %d, throttle %d per-mille\n",
            (int)(cmd.roll * 1000.0f), (int)(cmd.pitch * 1000.0f),
            (int)(cmd.yaw * 1000.0f), (int)(cmd.throttle * 1000.0f));
        out("switches:  arm %s, mode %s\n", cmd.arm_request ? "on" : "off",
            cmd.angle_mode ? "angle" : "rate");
    }
}
