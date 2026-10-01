#ifndef AK_TESTS_CRSF_FIXTURE_H
#define AK_TESTS_CRSF_FIXTURE_H

#include <stdint.h>

/* Builds a CRSF frame the way a receiver does, from the wire description
 * rather than from the decoder: sixteen 11-bit channels packed little-endian,
 * an 8-bit crc over the type and payload. Used by the decoder tests and by the
 * receiver tests, so both are checked against the same independent encoder. */
uint8_t crsf_build_frame(uint8_t *frame, const uint16_t *channels, uint8_t type);

#endif /* AK_TESTS_CRSF_FIXTURE_H */
