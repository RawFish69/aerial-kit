#include "ak_crsf.h"
#include "ak_crsf_tunnel.h"
#include "ak_params.h"
#include "ak_proto.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t response[512];
static unsigned response_length;

static void receive_frame(void *ctx, const uint8_t *frame, unsigned length)
{
    (void)ak_crsf_tunnel_feed_frame(
        (ak_crsf_tunnel_t *)ctx, frame, length, 100u);
}

static void send_frame(void *ctx, const uint8_t *frame, unsigned length)
{
    (void)ctx;
    if (response_length + length > sizeof response) {
        return;
    }
    memcpy(&response[response_length], frame, length);
    response_length += length;
}

static void make_request(const uint8_t *segment, unsigned segment_length,
                         uint8_t *frame, unsigned *frame_length)
{
    unsigned total = 13u + segment_length;
    frame[0] = 0xC8u;
    frame[1] = (uint8_t)(total - 2u);
    frame[2] = AK_CRSF_TYPE_MSP_REQUEST;
    frame[3] = 0xC8u;
    frame[4] = 0xEAu;
    frame[5] = 0x50u;
    frame[6] = 0u;
    frame[7] = AK_CRSF_TUNNEL_MSP_FUNCTION;
    frame[8] = 0u;
    frame[9] = (uint8_t)segment_length;
    frame[10] = 0u;
    memcpy(&frame[11], segment, segment_length);
    frame[11 + segment_length] = ak_crsf_crc8(
        &frame[6], (uint8_t)(segment_length + 5u));
    frame[12 + segment_length] = ak_crsf_crc8(
        &frame[2], (uint8_t)(total - 3u));
    *frame_length = total;
}

static int parse_response(const uint8_t *frame, unsigned length,
                          uint8_t *akp)
{
    if (length < 16u || frame[2] != AK_CRSF_TYPE_MSP_RESPONSE ||
        frame[5] != 0x30u || frame[7] != AK_CRSF_TUNNEL_MSP_FUNCTION) {
        return 0;
    }
    unsigned msp_length = frame[6];
    if (length != msp_length + 9u ||
        ak_crsf_crc8(&frame[6], (uint8_t)(msp_length + 1u)) !=
            frame[7 + msp_length]) {
        return 0;
    }
    unsigned segment_length = msp_length - 1u;
    if (segment_length < AK_CRSF_TUNNEL_HEADER ||
        frame[8] != AK_CRSF_TUNNEL_MAGIC) {
        return 0;
    }
    unsigned total = ((unsigned)frame[11] << 8) | frame[12];
    unsigned sequence = frame[10] & 0x3Fu;
    if (sequence * AK_CRSF_TUNNEL_DOWNLINK_CHUNK >= total) {
        return 0;
    }
    unsigned chunk = segment_length - AK_CRSF_TUNNEL_HEADER;
    unsigned offset = sequence * AK_CRSF_TUNNEL_DOWNLINK_CHUNK;
    if (offset + chunk > total) {
        return 0;
    }
    memcpy(&akp[offset], &frame[8 + AK_CRSF_TUNNEL_HEADER], chunk);
    return (int)total;
}

int main(void)
{
    ak_params_t params;
    ak_param_t items[1];
    static float value;
    ak_proto_io_t io = {0};
    ak_proto_t proto;
    ak_crsf_tunnel_t tunnel;
    ak_crsf_t crsf;
    uint8_t akp_request[7] = { 0xAAu, 0x55u, 1u, 1u, 0u, 0u, 0u };
    uint16_t request_crc = ak_proto_crc16(&akp_request[2], 3u);
    akp_request[5] = (uint8_t)request_crc;
    akp_request[6] = (uint8_t)(request_crc >> 8);

    ak_params_init(&params, items, 1u);
    (void)ak_params_add_float(
        items, 0u, "test", "test value", &value, 0, 0.0f, 1.0f,
        AK_PARAM_GROUP_RATES);
    params.count = 1u;
    io.params = &params;
    io.features = AK_PROTO_FEATURE_RC_CHANNELS;
    ak_proto_init(&proto);
    {
        ak_proto_t direct;
        uint8_t direct_response[AK_PROTO_FRAME_MAX];
        unsigned direct_length = 0;
        ak_proto_init(&direct);
        for (unsigned i = 0; i < sizeof akp_request; i++) {
            unsigned written = ak_proto_feed(
                &direct, &io, akp_request[i], 100u,
                direct_response, sizeof direct_response);
            if (written > 0u) {
                direct_length = written;
            }
        }
        if (direct.frames != 1u || direct.responses != 1u ||
            direct_length == 0u) {
            return 3;
        }
    }
    ak_crsf_tunnel_init(&tunnel, &proto, &io, send_frame, 0);
    ak_crsf_init(&crsf);
    ak_crsf_set_msp_frame_callback(&crsf, receive_frame, &tunnel);

    for (unsigned sequence = 0; sequence < 3u; sequence++) {
        uint8_t segment[AK_CRSF_TUNNEL_HEADER + AK_CRSF_TUNNEL_UPLINK_CHUNK];
        unsigned offset = sequence * AK_CRSF_TUNNEL_UPLINK_CHUNK;
        unsigned chunk = 7u - offset;
        if (chunk > AK_CRSF_TUNNEL_UPLINK_CHUNK) {
            chunk = AK_CRSF_TUNNEL_UPLINK_CHUNK;
        }
        uint8_t flags = (uint8_t)sequence;
        if (sequence == 0u) {
            flags |= 0x40u;
        }
        if (sequence == 2u) {
            flags |= 0x80u;
        }
        segment[0] = AK_CRSF_TUNNEL_MAGIC;
        segment[1] = 1u;
        segment[2] = flags;
        segment[3] = 0u;
        segment[4] = 7u;
        memcpy(&segment[5], &akp_request[offset], chunk);
        uint8_t frame[64];
        unsigned frame_length;
        make_request(segment, 5u + chunk, frame, &frame_length);
        for (unsigned i = 0; i < frame_length; i++) {
            ak_rc_input_t ignored;
            (void)ak_crsf_feed(&crsf, frame[i], &ignored, 100u + i);
        }
    }

    uint8_t assembled[AK_PROTO_FRAME_MAX] = {0};
    unsigned total = 0;
    for (unsigned at = 0; at < response_length;) {
        unsigned length = response[at + 1] + 2u;
        int parsed = parse_response(&response[at], length, assembled);
        if (parsed <= 0) {
            printf("bad response frame at %u: ", at);
            for (unsigned j = 0; j < length; j++) {
                printf("%02x", response[at + j]);
            }
            printf("\n");
            return 1;
        }
        total = (unsigned)parsed;
        at += length;
    }
    if (total < 5u ||
        assembled[0] != AK_PROTO_SYNC1 ||
        assembled[1] != AK_PROTO_SYNC2 ||
        assembled[2] != 1u ||
        assembled[3] != (AK_PROTO_CMD_HELLO | AK_PROTO_RESPONSE_BIT)) {
        return 2;
    }
    printf("akp-radio: ok (%u response bytes)\n", total);
    return 0;
}
