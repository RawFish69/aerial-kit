#ifndef AK_FLIGHT_CRSF_TUNNEL_H
#define AK_FLIGHT_CRSF_TUNNEL_H

#include "ak_crsf.h"
#include "ak_proto.h"

#include <stdint.h>

#define AK_CRSF_TUNNEL_MAGIC 0xA1u
#define AK_CRSF_TUNNEL_HEADER 5u
#define AK_CRSF_TUNNEL_UPLINK_CHUNK 3u
#define AK_CRSF_TUNNEL_DOWNLINK_CHUNK 49u
#define AK_CRSF_TUNNEL_MAX_FRAME AK_PROTO_FRAME_MAX
#define AK_CRSF_TUNNEL_MSP_FUNCTION 0x4Bu

typedef void (*ak_crsf_tunnel_send_fn)(
    void *ctx, const uint8_t *frame, unsigned length);

typedef struct {
    ak_proto_t *proto;
    const ak_proto_io_t *io;
    ak_crsf_tunnel_send_fn send;
    void *send_ctx;
    uint8_t buffer[AK_CRSF_TUNNEL_MAX_FRAME];
    uint16_t total;
    uint64_t received;
    uint8_t session;
    uint8_t last_sequence;
    uint8_t active;
    uint8_t saw_last;
    uint32_t telemetry_next_ms;
    uint32_t requests;
    uint32_t responses;
    uint32_t telemetry_frames;
    uint32_t errors;
} ak_crsf_tunnel_t;

void ak_crsf_tunnel_init(ak_crsf_tunnel_t *tunnel, ak_proto_t *proto,
                        const ak_proto_io_t *io, ak_crsf_tunnel_send_fn send,
                        void *send_ctx);

/* Complete CRSF MSP request/write frame from ak_crsf_feed. */
int ak_crsf_tunnel_feed_frame(ak_crsf_tunnel_t *tunnel,
                             const uint8_t *frame, unsigned length,
                             uint32_t now_ms);

/* Sends at most one requested telemetry frame when it is due. */
void ak_crsf_tunnel_service(ak_crsf_tunnel_t *tunnel, uint32_t now_ms);

#endif /* AK_FLIGHT_CRSF_TUNNEL_H */
