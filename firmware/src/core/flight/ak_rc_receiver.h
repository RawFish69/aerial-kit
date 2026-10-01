#ifndef AK_FLIGHT_RC_RECEIVER_H
#define AK_FLIGHT_RC_RECEIVER_H

#include "ak_console.h"
#include "ak_crsf.h"
#include "ak_rc.h"
#include "ak_sbus.h"
#include "ak_types.h"

/*
 * The receiver, as the flight core sees it: bytes in, sticks out.
 *
 * It owns one parser per protocol and the last good channel frame. Bytes
 * arrive from whatever the board's UART collected; nothing here knows what a
 * UART is, so the whole path - framing, crc or the lack of one, channel
 * unpacking, and what happens when the stream is garbage - is testable on a
 * host.
 *
 * Two protocols rather than one, because which one a receiver speaks is a
 * property of the receiver somebody plugged in. CRSF is what the wing flies
 * and what an ELRS link speaks; SBUS is what most receivers a person buys
 * already speak. Downstream of the channels the flight core cannot tell them
 * apart, and that is the point - the counts are the same 172..1811 with 992 in
 * the middle in both.
 *
 * A link that stops producing frames is not this module's problem: the flight
 * core already treats a stale `last_update_ms` as a failsafe. This one just
 * keeps that timestamp honest - including for a receiver that is still sending
 * frames while its own failsafe is active, which must not look like a link.
 */

typedef enum {
    AK_RC_PROTOCOL_CRSF = 0,
    AK_RC_PROTOCOL_SBUS = 1,
} ak_rc_protocol_t;

typedef struct {
    uint32_t      protocol;
    ak_crsf_t     crsf;
    ak_sbus_t     sbus;
    ak_rc_input_t channels;
    uint32_t      bytes;
    uint32_t      frames;
} ak_rc_receiver_t;

void ak_rc_receiver_init(ak_rc_receiver_t *rx);

/* Which protocol the bytes are. Both parsers are kept, and switching does not
 * lose what the other one had made of the stream. The board has to be told
 * separately, because the two do not even run at the same line settings. */
void ak_rc_receiver_set_protocol(ak_rc_receiver_t *rx, uint32_t protocol);
uint32_t ak_rc_receiver_protocol(const ak_rc_receiver_t *rx);
const char *ak_rc_protocol_name(uint32_t protocol);

/* One byte from the port. Returns 1 when that byte completed a channel frame,
 * in which case `channels` is fresh and stamped with now_ms. */
int ak_rc_receiver_feed(ak_rc_receiver_t *rx, uint8_t byte, uint32_t now_ms);

/* What the console prints for `rc`: the last frame's channels, and enough
 * counters to tell "no receiver" from "a receiver saying nonsense".
 *
 * The configuration is an argument rather than a `ak_rc_default_config()` call
 * inside, which is what it used to be. The sticks are only meaningful against
 * the calibration they were decoded with, and `rc_min`, `rc_mid`, `rc_max` and
 * `rc_deadband` are parameters - so a board that had been through `calibrate rc`
 * printed one set of sticks here and flew another, and the console is the only
 * place on a bench where a person can see them. Passing the config in makes
 * this report and the flight core read the same numbers by construction. */
void ak_rc_receiver_report(const ak_rc_receiver_t *rx, const ak_rc_config_t *cfg,
                           ak_printf_fn out);

#endif /* AK_FLIGHT_RC_RECEIVER_H */
