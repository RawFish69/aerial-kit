#ifndef AK_CORE_AK_CONSOLE_LINK_H
#define AK_CORE_AK_CONSOLE_LINK_H

#include <stdint.h>

#include "ak_cli.h"
#include "ak_proto.h"

/*
 * The console link: one wire carrying two things that cannot be told apart by
 * looking at it.
 *
 * The human console and the binary config protocol share the port the console
 * is on, distinguished only by a frame's first byte. That is a good trade - one
 * connector, one adapter, one port on the host, and the person and the script
 * use the same one - but the arbitration has to be right, because neither
 * reader can recognise the other's bytes: ak_cli_feed sees a printable payload
 * byte as a character somebody typed, and ak_proto_feed reads a typed "U" as
 * the byte after a sync that never came.
 *
 * This is a file of its own rather than a function in ak_proto.c for a
 * mechanical reason that looks like a preference and is not one. The arbitration
 * needs both readers, so whatever holds it depends on both - and the fuzzer and
 * the contract vectors link ak_proto.o on purpose without the console
 * (tools/fuzz_parsers.c fuzzes parsers, and the console is not one). Putting
 * this in ak_proto.c drags the console into both of them, or breaks their link.
 * A translation unit of its own is linked by the firmware and the tests, and by
 * nothing that does not want it.
 */

/* One byte arriving on the shared port. Returns the length of a response
 * written into `response` when this byte completed a protocol frame that has
 * one, and 0 otherwise - the same contract as ak_proto_feed, so the caller
 * writes the answer out the same way whatever it turns out to be.
 *
 * The rule: the byte is the console's when the parser would read it as the
 * start of a frame and it is not a frame sync; otherwise the protocol owns the
 * wire and any half-typed line is forgotten. See ak_proto_idle_at for why that
 * first half is not just `state == STATE_SYNC1`, and ak_cli_forget for what
 * forgetting is for. */
unsigned ak_console_link_feed(ak_proto_t *proto, const ak_proto_io_t *io,
                              ak_cli_t *cli, uint8_t byte, uint32_t now_ms,
                              uint8_t *response, unsigned capacity);

#endif /* AK_CORE_AK_CONSOLE_LINK_H */
