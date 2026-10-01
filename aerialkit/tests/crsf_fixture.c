#include "crsf_fixture.h"

#include "ak_crsf.h"

uint8_t crsf_build_frame(uint8_t *frame, const uint16_t *channels, uint8_t type)
{
    uint8_t payload[22];
    for (int i = 0; i < 22; i++) {
        payload[i] = 0;
    }
    /* Sixteen eleven-bit channels are twenty-two bytes to the bit, so the last
     * channel ends on the last bit of the last byte and a byte-wise unpack
     * writes one byte past the array for it (it wrote a zero, which is why the
     * frames were still right and nothing ever noticed). Filled bit by bit
     * instead, which cannot leave the payload: bit + 10 is 175 for the last
     * channel - see the same fix in tests/test_aerialkit.c, found by the
     * sanitizer run. */
    for (int n = 0; n < AK_CRSF_CHANNELS; n++) {
        unsigned bit = (unsigned)n * 11u;
        uint32_t value = (uint32_t)channels[n] & 0x7FFu;
        for (unsigned b = 0; b < 11u; b++) {
            if ((value >> b) & 1u) {
                payload[(bit + b) / 8u] |= (uint8_t)(1u << ((bit + b) % 8u));
            }
        }
    }

    frame[0] = 0xC8; /* address: the flight controller */
    frame[1] = 2 + 22;
    frame[2] = type;
    for (int i = 0; i < 22; i++) {
        frame[3 + i] = payload[i];
    }
    frame[25] = ak_crsf_crc8(&frame[2], 23);
    return 26;
}
