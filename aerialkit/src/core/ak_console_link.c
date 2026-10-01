#include "ak_console_link.h"

unsigned ak_console_link_feed(ak_proto_t *proto, const ak_proto_io_t *io,
                              ak_cli_t *cli, uint8_t byte, uint32_t now_ms,
                              uint8_t *response, unsigned capacity)
{
    if (ak_proto_idle_at(proto, now_ms) && byte != (uint8_t)AK_PROTO_SYNC1) {
        ak_cli_feed(cli, (char)byte);
        return 0u;
    }

    /* The wire is the protocol's from here, so a line half-typed on it is not a
     * line: whatever put those bytes in the buffer was not a person pressing
     * keys, and the next command typed must not arrive glued to them. */
    ak_cli_forget(cli);
    return ak_proto_feed(proto, io, byte, now_ms, response, capacity);
}
