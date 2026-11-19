#include "ak_crsf_tunnel.h"

#include <string.h>

void ak_crsf_tunnel_init(ak_crsf_tunnel_t *tunnel, ak_proto_t *proto,
                        const ak_proto_io_t *io, ak_crsf_tunnel_send_fn send,
                        void *send_ctx)
{
    memset(tunnel, 0, sizeof *tunnel);
    tunnel->proto = proto;
    tunnel->io = io;
    tunnel->send = send;
    tunnel->send_ctx = send_ctx;
}

static uint8_t segment_sequence(uint8_t flags)
{
    return flags & 0x3Fu;
}

static int bytes_equal(const uint8_t *a, const uint8_t *b, unsigned length)
{
    for (unsigned i = 0; i < length; i++) {
        if (a[i] != b[i]) {
            return 0;
        }
    }
    return 1;
}

static int msp_request_data(const uint8_t *frame, unsigned length,
                            const uint8_t **data, unsigned *data_length)
{
    uint8_t type = frame[2];

    if (type == AK_CRSF_TYPE_MSP_REQUEST) {
        if (length < 13u || frame[5] != 0x50u) {
            return 0;
        }
        uint16_t function = (uint16_t)frame[7] |
                            ((uint16_t)frame[8] << 8);
        uint16_t msp_length = (uint16_t)frame[9] |
                              ((uint16_t)frame[10] << 8);
        if (function != AK_CRSF_TUNNEL_MSP_FUNCTION ||
            length != (unsigned)msp_length + 13u) {
            return 0;
        }
        if (ak_crsf_crc8(&frame[6], (uint8_t)(msp_length + 5u)) !=
            frame[11u + msp_length]) {
            return 0;
        }
        *data = &frame[11];
        *data_length = msp_length;
        return 1;
    }

    if (type == AK_CRSF_TYPE_MSP_WRITE) {
        if (length < 10u || frame[5] != 0x30u ||
            frame[7] != AK_CRSF_TUNNEL_MSP_FUNCTION) {
            return 0;
        }
        unsigned msp_length = frame[6];
        if (msp_length < 1u || length != msp_length + 9u) {
            return 0;
        }
        if (ak_crsf_crc8(&frame[6], (uint8_t)(msp_length + 1u)) !=
            frame[7u + msp_length]) {
            return 0;
        }
        *data = &frame[8];
        *data_length = msp_length - 1u;
        return 1;
    }
    return 0;
}

static void send_segments(ak_crsf_tunnel_t *tunnel, uint8_t session,
                          const uint8_t *data, unsigned length)
{
    unsigned chunk_size = AK_CRSF_TUNNEL_DOWNLINK_CHUNK;
    unsigned count = length == 0u ? 1u : (length + chunk_size - 1u) / chunk_size;

    for (unsigned sequence = 0; sequence < count; sequence++) {
        unsigned offset = sequence * chunk_size;
        unsigned chunk = length - offset;
        if (chunk > chunk_size) {
            chunk = chunk_size;
        }

        uint8_t frame[AK_CRSF_MAX_FRAME];
        unsigned total = 10u + AK_CRSF_TUNNEL_HEADER + chunk;
        if (total > sizeof frame) {
            tunnel->errors++;
            return;
        }
        uint8_t flags = (uint8_t)sequence;
        if (sequence == 0u) {
            flags |= 0x40u;
        }
        if (sequence + 1u == count) {
            flags |= 0x80u;
        }

        unsigned at = 0;
        frame[at++] = 0xC8u;
        frame[at++] = (uint8_t)(total - 2u);
        frame[at++] = AK_CRSF_TYPE_MSP_RESPONSE;
        frame[at++] = 0xEAu;
        frame[at++] = 0xC8u;
        frame[at++] = 0x30u;
        frame[at++] = (uint8_t)(1u + AK_CRSF_TUNNEL_HEADER + chunk);
        frame[at++] = AK_CRSF_TUNNEL_MSP_FUNCTION;
        frame[at++] = AK_CRSF_TUNNEL_MAGIC;
        frame[at++] = session;
        frame[at++] = flags;
        frame[at++] = (uint8_t)(length >> 8);
        frame[at++] = (uint8_t)length;
        if (chunk > 0u) {
            memcpy(&frame[at], &data[offset], chunk);
            at += chunk;
        }
        frame[at++] = ak_crsf_crc8(
            &frame[6], (uint8_t)(2u + AK_CRSF_TUNNEL_HEADER + chunk));
        uint8_t frame_crc = ak_crsf_crc8(&frame[2], (uint8_t)(at - 2u));
        frame[at++] = frame_crc;
        tunnel->send(tunnel->send_ctx, frame, at);
    }
}

static void send_akp_frame(ak_crsf_tunnel_t *tunnel, uint8_t session,
                           const uint8_t *data, unsigned length)
{
    if (length > 0u) {
        send_segments(tunnel, session, data, length);
    }
}

static void deliver(ak_crsf_tunnel_t *tunnel, uint32_t now_ms)
{
    uint8_t response[AK_PROTO_FRAME_MAX];
    unsigned response_length = 0;

    for (unsigned i = 0; i < tunnel->total; i++) {
        unsigned written = ak_proto_feed(
            tunnel->proto, tunnel->io, tunnel->buffer[i], now_ms,
            response, sizeof response);
        if (written > 0u) {
            response_length = written;
        }
    }
    tunnel->requests++;
    tunnel->active = 0u;
    tunnel->received = 0u;
    tunnel->total = 0u;
    if (response_length > 0u) {
        tunnel->responses++;
        send_akp_frame(tunnel, tunnel->session,
                       response, response_length);
    }
}

int ak_crsf_tunnel_feed_frame(ak_crsf_tunnel_t *tunnel,
                             const uint8_t *frame, unsigned length,
                             uint32_t now_ms)
{
    const uint8_t *data = 0;
    unsigned data_length = 0;

    if (tunnel == 0 || frame == 0 || length < 4u ||
        !msp_request_data(frame, length, &data, &data_length)) {
        return 0;
    }
    if (data_length < AK_CRSF_TUNNEL_HEADER || data[0] != AK_CRSF_TUNNEL_MAGIC) {
        tunnel->errors++;
        return 1;
    }

    uint8_t session = data[1];
    uint8_t flags = data[2];
    uint8_t sequence = segment_sequence(flags);
    uint16_t total = (uint16_t)(((uint16_t)data[3] << 8) | data[4]);
    const uint8_t *chunk = &data[AK_CRSF_TUNNEL_HEADER];
    unsigned chunk_length = data_length - AK_CRSF_TUNNEL_HEADER;

    if (sequence * AK_CRSF_TUNNEL_UPLINK_CHUNK >= total ||
        total > sizeof tunnel->buffer) {
        tunnel->errors++;
        return 1;
    }
    unsigned offset = sequence * AK_CRSF_TUNNEL_UPLINK_CHUNK;
    unsigned expected = total - offset;
    if (expected > AK_CRSF_TUNNEL_UPLINK_CHUNK) {
        expected = AK_CRSF_TUNNEL_UPLINK_CHUNK;
    }
    if (chunk_length != expected) {
        tunnel->errors++;
        return 1;
    }

    if ((flags & 0x40u) != 0u) {
        if (sequence != 0u) {
            tunnel->errors++;
            return 1;
        }
        tunnel->active = 1u;
        tunnel->session = session;
        tunnel->total = total;
        tunnel->received = 0u;
        tunnel->saw_last = 0u;
        tunnel->last_sequence = (uint8_t)((total == 0u ? 0u :
                                          (total - 1u) /
                                          AK_CRSF_TUNNEL_UPLINK_CHUNK));
    } else if (tunnel->active == 0u || tunnel->session != session ||
               tunnel->total != total) {
        tunnel->errors++;
        return 1;
    }

    uint64_t bit = (uint64_t)1u << sequence;
    if ((tunnel->received & bit) != 0u) {
        if (!bytes_equal(&tunnel->buffer[offset], chunk, chunk_length)) {
            tunnel->errors++;
            tunnel->active = 0u;
            return 1;
        }
    } else {
        memcpy(&tunnel->buffer[offset], chunk, chunk_length);
        tunnel->received |= bit;
    }
    if ((flags & 0x80u) != 0u && sequence != tunnel->last_sequence) {
        tunnel->errors++;
        tunnel->active = 0u;
        return 1;
    }
    if ((flags & 0x80u) != 0u) {
        tunnel->saw_last = 1u;
    }

    uint64_t complete = tunnel->last_sequence == 63u
                            ? ~UINT64_C(0)
                            : (((uint64_t)1u << (tunnel->last_sequence + 1u)) - 1u);
    if (tunnel->saw_last != 0u &&
        (tunnel->received & complete) == complete) {
        deliver(tunnel, now_ms);
    }
    return 1;
}

void ak_crsf_tunnel_service(ak_crsf_tunnel_t *tunnel, uint32_t now_ms)
{
    if (tunnel == 0 || tunnel->proto->telemetry_hz == 0u ||
        (int32_t)(now_ms - tunnel->telemetry_next_ms) < 0) {
        return;
    }

    uint8_t frame[AK_PROTO_FRAME_MAX];
    unsigned length = ak_proto_telemetry_frame(
        tunnel->io, now_ms, frame, sizeof frame);
    if (length > 0u) {
        send_akp_frame(tunnel, tunnel->session, frame, length);
        tunnel->telemetry_frames++;
    }
    uint32_t period = 1000u / tunnel->proto->telemetry_hz;
    if (period < 100u) {
        period = 100u;
    }
    tunnel->telemetry_next_ms = now_ms + period;
}
