#include "ak_logenc.h"

#include <stddef.h>
#include <string.h>

/*
 * The field table: every scalar of `ak_log_record_t`, in the order
 * `ak_log_encode_record` writes them (ak_logenc.h, decision 1).
 *
 * The offsets are `offsetof` rather than a hand-counted byte number, and that
 * is the whole reason this table can be trusted: the struct has padding between
 * `motor`, `state`, `flags` and the two positions, and every hand-counted
 * version of this table would have been right until the day a member was added
 * before one of them. The index-and-size form is for the arrays, where the
 * elements are contiguous by definition.
 *
 * The names are in the table because a refusal has to be able to say *which*
 * field it refused, and a reader that says "frame 4193 is malformed" has told
 * nobody anything.
 */
#define AK_LOGF(name_, kind_, member, index, elem)                         \
    {                                                                      \
        name_, AK_LF_##kind_,                                              \
            (uint16_t)(offsetof(ak_log_record_t, member) +                 \
                       (index) * (elem))                                   \
    }

static const ak_logenc_field_t k_fields[AK_LOGENC_FIELDS] = {
    AK_LOGF("time_ms", U32, time_ms, 0, 4),
    AK_LOGF("gyro0", I16, gyro, 0, 2),
    AK_LOGF("gyro1", I16, gyro, 1, 2),
    AK_LOGF("gyro2", I16, gyro, 2, 2),
    AK_LOGF("accel0", I16, accel, 0, 2),
    AK_LOGF("accel1", I16, accel, 1, 2),
    AK_LOGF("accel2", I16, accel, 2, 2),
    AK_LOGF("attitude0", I16, attitude, 0, 2),
    AK_LOGF("attitude1", I16, attitude, 1, 2),
    AK_LOGF("yaw", I16, yaw, 0, 2),
    AK_LOGF("alt_mm", I32, alt_mm, 0, 4),
    AK_LOGF("stick0", I16, stick, 0, 2),
    AK_LOGF("stick1", I16, stick, 1, 2),
    AK_LOGF("stick2", I16, stick, 2, 2),
    AK_LOGF("stick3", I16, stick, 3, 2),
    AK_LOGF("torque0", I8, torque, 0, 1),
    AK_LOGF("torque1", I8, torque, 1, 1),
    AK_LOGF("torque2", I8, torque, 2, 1),
    AK_LOGF("motor0", U8, motor, 0, 1),
    AK_LOGF("motor1", U8, motor, 1, 1),
    AK_LOGF("motor2", U8, motor, 2, 1),
    AK_LOGF("motor3", U8, motor, 3, 1),
    AK_LOGF("state", U8, state, 0, 1),
    AK_LOGF("flags", U8, flags, 0, 1),
    AK_LOGF("lat_e7", I32, lat_e7, 0, 4),
    AK_LOGF("lon_e7", I32, lon_e7, 0, 4),
    AK_LOGF("gyro_filtered0", I16, gyro_filtered, 0, 2),
    AK_LOGF("gyro_filtered1", I16, gyro_filtered, 1, 2),
    AK_LOGF("gyro_filtered2", I16, gyro_filtered, 2, 2),
    AK_LOGF("notch_hz0", U16, notch_hz, 0, 2),
    AK_LOGF("notch_hz1", U16, notch_hz, 1, 2),
    AK_LOGF("notch_hz2", U16, notch_hz, 2, 2),
    AK_LOGF("notch_engaged0", U8, notch_engaged, 0, 1),
    AK_LOGF("notch_engaged1", U8, notch_engaged, 1, 1),
    AK_LOGF("notch_engaged2", U8, notch_engaged, 2, 1),
    /* Log version 4 (roadmap 4.1's fields), appended: field numbers are the
     * presence map's bit positions, and an existing one never moves. */
    AK_LOGF("time_us", U32, time_us, 0, 4),
    AK_LOGF("rate_setpoint0", I16, rate_setpoint, 0, 2),
    AK_LOGF("rate_setpoint1", I16, rate_setpoint, 1, 2),
    AK_LOGF("rate_setpoint2", I16, rate_setpoint, 2, 2),
    AK_LOGF("pid_p0", I8, pid_p, 0, 1),
    AK_LOGF("pid_p1", I8, pid_p, 1, 1),
    AK_LOGF("pid_p2", I8, pid_p, 2, 1),
    AK_LOGF("pid_i0", I8, pid_i, 0, 1),
    AK_LOGF("pid_i1", I8, pid_i, 1, 1),
    AK_LOGF("pid_i2", I8, pid_i, 2, 1),
    AK_LOGF("pid_d0", I8, pid_d, 0, 1),
    AK_LOGF("pid_d1", I8, pid_d, 1, 1),
    AK_LOGF("pid_d2", I8, pid_d, 2, 1),
    AK_LOGF("vbat_mv", U16, vbat_mv, 0, 2),
};

const char *ak_logenc_field_name(unsigned index)
{
    return index < AK_LOGENC_FIELDS ? k_fields[index].name : 0;
}

unsigned ak_logenc_mask_count(uint64_t mask)
{
    unsigned n = 0;

    for (unsigned i = 0; i < AK_LOGENC_FIELDS; i++) {
        if (mask & ((uint64_t)1 << i)) {
            n++;
        }
    }
    return n;
}

static unsigned width_of(ak_logenc_kind_t kind)
{
    switch (kind) {
    case AK_LF_I8:
    case AK_LF_U8:
        return 1u;
    case AK_LF_I16:
    case AK_LF_U16:
        return 2u;
    case AK_LF_I32:
    case AK_LF_U32:
    default:
        return 4u;
    }
}

static uint64_t mask_all(void)
{
    return AK_LOGENC_MASK_ALL;
}

/* Sign-extend a `bits`-wide two's complement value. Written as arithmetic on
 * the unsigned value rather than as a cast to a narrow signed type, because
 * converting an out-of-range unsigned to a signed type is implementation-
 * defined and this module's whole job is to be the same on two machines. */
static int64_t sign_extend(uint64_t v, unsigned bits)
{
    const uint64_t m = (uint64_t)1 << (bits - 1u);

    return (int64_t)((v ^ m) - m);
}

static int64_t field_read(const ak_log_record_t *record,
                          const ak_logenc_field_t *field)
{
    const uint8_t *p = (const uint8_t *)record + field->offset;

    switch (field->kind) {
    case AK_LF_I8:
        return sign_extend(p[0], 8u);
    case AK_LF_U8:
        return (int64_t)p[0];
    case AK_LF_I16: {
        uint64_t v = (uint64_t)p[0] | ((uint64_t)p[1] << 8);

        return sign_extend(v, 16u);
    }
    case AK_LF_U16:
        return (int64_t)((uint64_t)p[0] | ((uint64_t)p[1] << 8));
    case AK_LF_I32:
    case AK_LF_U32:
    default: {
        uint64_t v = (uint64_t)p[0] | ((uint64_t)p[1] << 8) |
                     ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24);

        return field->kind == AK_LF_I32 ? sign_extend(v, 32u) : (int64_t)v;
    }
    }
}

static void field_write(ak_log_record_t *record,
                        const ak_logenc_field_t *field, int64_t value)
{
    uint8_t *p = (uint8_t *)record + field->offset;
    const uint64_t u = (uint64_t)value;
    const unsigned width = width_of(field->kind);

    for (unsigned i = 0; i < width; i++) {
        p[i] = (uint8_t)((u >> (8u * i)) & 0xFFu);
    }
}

/* Zigzag, so that a delta of -1 costs one byte rather than ten.
 *
 * Written without a right shift of a negative value: `v >> 63` is an
 * arithmetic shift on every compiler in this tree and is implementation-defined
 * in the language, and `~u` for a negative v is the same number without the
 * question. */
static uint64_t zigzag(int64_t v)
{
    const uint64_t u = (uint64_t)v << 1;

    return v < 0 ? ~u : u;
}

static int64_t unzigzag(uint64_t u)
{
    const int64_t half = (int64_t)(u >> 1);

    /* Not `-(half + 1)`: for the one input that decodes to INT64_MIN that
     * expression overflows before it negates. */
    return (u & 1u) ? -half - 1 : half;
}

static unsigned varint_len(uint64_t v)
{
    unsigned n = 1u;

    while (v >= 0x80u) {
        v >>= 7;
        n++;
    }
    return n;
}

static unsigned put_varint(uint8_t *out, unsigned at, uint64_t v)
{
    for (;;) {
        uint8_t b = (uint8_t)(v & 0x7Fu);

        v >>= 7;
        if (v != 0u) {
            b |= 0x80u;
        }
        out[at++] = b;
        if (v == 0u) {
            return at;
        }
    }
}

/* Returns the bytes consumed, or 0 when the varint runs off the end of the
 * buffer or is longer than any 64-bit value can be. A truncated varint is a
 * refusal and not a partial value: the byte it stopped on is the first byte of
 * the *next* field if the frame was truncated and the last byte of this one if
 * it was not, and there is no way to tell those apart from here. */
static unsigned get_varint(const uint8_t *in, unsigned len, uint64_t *out)
{
    uint64_t v = 0;
    unsigned shift = 0;

    for (unsigned n = 0; n < len; n++) {
        const uint8_t b = in[n];

        if (shift == 63u && (b & 0x7Fu) > 1u) {
            return 0; /* more than 64 bits of value */
        }
        v |= (uint64_t)(b & 0x7Fu) << shift;
        if ((b & 0x80u) == 0u) {
            *out = v;
            return n + 1u;
        }
        if (shift >= 63u) {
            return 0;
        }
        shift += 7u;
    }
    return 0;
}

static unsigned put_u16(uint8_t *out, unsigned at, uint16_t v)
{
    out[at++] = (uint8_t)(v & 0xFFu);
    out[at++] = (uint8_t)((v >> 8) & 0xFFu);
    return at;
}

static uint16_t get_u16(const uint8_t *in, unsigned at)
{
    return (uint16_t)((uint16_t)in[at] | ((uint16_t)in[at + 1u] << 8));
}

static unsigned put_u32(uint8_t *out, unsigned at, uint32_t v)
{
    for (unsigned i = 0; i < 4u; i++) {
        out[at++] = (uint8_t)((v >> (8u * i)) & 0xFFu);
    }
    return at;
}

static uint32_t get_u32(const uint8_t *in, unsigned at)
{
    return (uint32_t)in[at] | ((uint32_t)in[at + 1u] << 8) |
           ((uint32_t)in[at + 2u] << 16) | ((uint32_t)in[at + 3u] << 24);
}

static unsigned put_u64(uint8_t *out, unsigned at, uint64_t v)
{
    for (unsigned i = 0; i < 8u; i++) {
        out[at++] = (uint8_t)((v >> (8u * i)) & 0xFFu);
    }
    return at;
}

static uint64_t get_u64(const uint8_t *in, unsigned at)
{
    uint64_t v = 0;

    for (unsigned i = 0; i < 8u; i++) {
        v |= (uint64_t)in[at + i] << (8u * i);
    }
    return v;
}

unsigned ak_logenc_write_header(uint64_t field_mask, uint16_t rate_hz,
                                uint16_t decimation, uint32_t config_hash,
                                uint8_t *out, unsigned capacity)
{
    if (out == 0 || capacity < AK_LOGENC_HEADER_BYTES) {
        return 0;
    }
    if ((field_mask & ~mask_all()) != 0u) {
        return 0; /* names a field this build does not have */
    }

    unsigned at = put_u32(out, 0u, AK_LOGENC_MAGIC);

    out[at++] = (uint8_t)AK_LOGENC_VERSION;
    out[at++] = (uint8_t)AK_LOGENC_FIELDS;
    at = put_u16(out, at, rate_hz);
    at = put_u16(out, at, decimation);
    at = put_u32(out, at, config_hash);
    at = put_u64(out, at, field_mask);
    return at;
}

int ak_logenc_read_header(const uint8_t *in, unsigned len,
                          ak_logenc_header_t *out)
{
    if (in == 0 || out == 0 || len < AK_LOGENC_HEADER_BYTES) {
        return 0;
    }
    if (get_u32(in, 0u) != AK_LOGENC_MAGIC) {
        return 0;
    }
    if (in[4] != (uint8_t)AK_LOGENC_VERSION) {
        return 0;
    }

    uint8_t count = in[5];
    uint64_t mask = get_u64(in, 14u);

    /* A log written by a build with more fields than this one is a log this one
     * cannot walk: the frames carry values for fields whose widths it does not
     * know, so every byte after the first unknown field is at an offset it
     * guessed. Refusing is the only honest answer - a reader that skipped them
     * would print numbers from the wrong columns. */
    if (count > AK_LOGENC_FIELDS) {
        return 0;
    }
    /* And a mask naming a field beyond the count the header declares is a
     * header that disagrees with itself. */
    if (count < 64u && (mask & ~(((uint64_t)1 << count) - 1u)) != 0u) {
        return 0;
    }

    out->magic = AK_LOGENC_MAGIC;
    out->version = in[4];
    out->field_count = count;
    out->rate_hz = get_u16(in, 6u);
    out->decimation = get_u16(in, 8u);
    out->config_hash = get_u32(in, 10u);
    out->field_mask = mask;
    return 1;
}

void ak_logenc_encoder_init(ak_logenc_enc_t *enc, uint64_t field_mask,
                            uint32_t keyframe_every)
{
    if (enc == 0) {
        return;
    }
    memset(enc, 0, sizeof *enc);
    enc->field_mask = field_mask & mask_all();
    enc->keyframe_every = keyframe_every;
    enc->have_previous = 0;
}

void ak_logenc_decoder_init(ak_logenc_dec_t *dec, uint64_t field_mask)
{
    if (dec == 0) {
        return;
    }
    memset(dec, 0, sizeof *dec);
    dec->field_mask = field_mask & mask_all();
    dec->have_previous = 0;
}

unsigned ak_logenc_encode(ak_logenc_enc_t *enc, const ak_log_record_t *record,
                          uint8_t *out, unsigned capacity)
{
    if (enc == 0 || record == 0 || out == 0 || capacity < 2u) {
        return 0;
    }

    const uint64_t mask = enc->field_mask;
    const int forced = enc->keyframe_every != 0u &&
                       enc->since_key >= enc->keyframe_every;

    unsigned key_bytes = 1u;
    unsigned delta_bytes = 1u + AK_LOGENC_MAP_BYTES;

    for (unsigned i = 0; i < AK_LOGENC_FIELDS; i++) {
        if ((mask & ((uint64_t)1 << i)) == 0u) {
            continue;
        }
        key_bytes += width_of(k_fields[i].kind);
        if (enc->have_previous) {
            const int64_t d = field_read(record, &k_fields[i]) - enc->previous[i];

            if (d != 0) {
                delta_bytes += varint_len(zigzag(d));
            }
        } else {
            /* No previous record: a delta frame is not available at all, and
             * saying so with a number larger than any frame is clearer than a
             * separate branch in the choice below. */
            delta_bytes = 0xFFFFFFFFu;
            break;
        }
    }

    /* The choice, and the bound it buys: a delta that is not strictly smaller
     * than the keyframe is not written, so no frame is ever larger than the
     * fixed-width record plus its type byte (ak_logenc.h, decision 2). */
    const int use_key = !enc->have_previous || forced ||
                        delta_bytes >= key_bytes;
    const unsigned total = use_key ? key_bytes : delta_bytes;

    if (total > capacity || total > AK_LOGENC_MAX_FRAME_BYTES) {
        return 0;
    }

    unsigned at = 0;

    if (use_key) {
        out[at++] = (uint8_t)AK_LOGENC_I;
        for (unsigned i = 0; i < AK_LOGENC_FIELDS; i++) {
            if ((mask & ((uint64_t)1 << i)) == 0u) {
                continue;
            }
            const int64_t v = field_read(record, &k_fields[i]);

            /* Written a byte at a time rather than through a helper, because
             * this is the path that has to reproduce `ak_log_encode_record`
             * byte for byte and a helper with a width argument is one more
             * place for the two to drift. */
            for (unsigned b = 0; b < width_of(k_fields[i].kind); b++) {
                out[at++] = (uint8_t)(((uint64_t)v >> (8u * b)) & 0xFFu);
            }
        }
    } else {
        out[at++] = (uint8_t)AK_LOGENC_P;
        const unsigned map_at = at;

        for (unsigned b = 0; b < AK_LOGENC_MAP_BYTES; b++) {
            out[at++] = 0u;
        }
        for (unsigned i = 0; i < AK_LOGENC_FIELDS; i++) {
            if ((mask & ((uint64_t)1 << i)) == 0u) {
                continue;
            }
            const int64_t d = field_read(record, &k_fields[i]) - enc->previous[i];

            if (d == 0) {
                continue;
            }
            out[map_at + (i / 8u)] |= (uint8_t)(1u << (i % 8u));
            at = put_varint(out, at, zigzag(d));
        }
    }

    for (unsigned i = 0; i < AK_LOGENC_FIELDS; i++) {
        if (mask & ((uint64_t)1 << i)) {
            enc->previous[i] = field_read(record, &k_fields[i]);
        }
    }
    enc->have_previous = 1;
    enc->since_key = use_key ? 0u : enc->since_key + 1u;
    return at;
}

unsigned ak_logenc_decode(ak_logenc_dec_t *dec, const uint8_t *in, unsigned len,
                          ak_log_record_t *out)
{
    if (dec == 0 || in == 0 || out == 0 || len < 1u) {
        return 0;
    }

    const uint64_t mask = dec->field_mask;
    const uint8_t type = in[0];
    unsigned at = 1u;

    /* Everything is decoded into a local and copied out at the end, so that a
     * refusal leaves the caller's record as it was. A half-decoded record that
     * looks like a reading is the failure this module exists to avoid. */
    int64_t value[AK_LOGENC_FIELDS];
    uint8_t map[AK_LOGENC_MAP_BYTES];

    for (unsigned b = 0; b < AK_LOGENC_MAP_BYTES; b++) {
        map[b] = 0u;
    }

    if (type == (uint8_t)AK_LOGENC_I) {
        for (unsigned i = 0; i < AK_LOGENC_FIELDS; i++) {
            if ((mask & ((uint64_t)1 << i)) == 0u) {
                value[i] = 0;
                continue;
            }
            const unsigned w = width_of(k_fields[i].kind);

            if (len - at < w) {
                return 0;
            }
            uint64_t raw = 0;

            for (unsigned b = 0; b < w; b++) {
                raw |= (uint64_t)in[at + b] << (8u * b);
            }
            at += w;
            value[i] = (k_fields[i].kind == AK_LF_I8 ||
                        k_fields[i].kind == AK_LF_I16 ||
                        k_fields[i].kind == AK_LF_I32)
                           ? sign_extend(raw, w * 8u)
                           : (int64_t)raw;
        }
    } else if (type == (uint8_t)AK_LOGENC_P) {
        /* A delta frame with nothing to be a delta from is refused rather than
         * decoded against a zeroed previous record. The zeroed record is a
         * plausible flight - level, stationary, at the origin - and decoding
         * into it produces a log that reads as real data (ak_logenc.h,
         * decision 3's neighbourhood: a refusal, not a guess). */
        if (!dec->have_previous) {
            return 0;
        }
        if (len - at < AK_LOGENC_MAP_BYTES) {
            return 0;
        }
        uint64_t present = 0;

        for (unsigned b = 0; b < AK_LOGENC_MAP_BYTES; b++) {
            map[b] = in[at + b];
            present |= (uint64_t)map[b] << (8u * b);
        }
        at += AK_LOGENC_MAP_BYTES;

        /* The map is seven bytes for forty-nine fields, so seven of its bits
         * name no field at all. Setting one is claiming a field this build has
         * never heard of - the same fact the header's field_count carries,
         * arriving by a route the header cannot check - and the bytes it
         * implies are not in the frame. Refusing is the answer the field count
         * gets, and it costs this module nothing: the loop below sets a bit
         * only for a field in the table, so nothing this file writes ever
         * sets one. */
        if ((present & ~AK_LOGENC_MASK_ALL) != 0u) {
            return 0;
        }

        for (unsigned i = 0; i < AK_LOGENC_FIELDS; i++) {
            const int changed = (map[i / 8u] >> (i % 8u)) & 1u;

            if ((mask & ((uint64_t)1 << i)) == 0u) {
                /* A field the log does not carry has no value here, and 0 is
                 * the absence of one rather than a reading of zero. The
                 * header's mask is how a reader tells those apart. */
                value[i] = 0;
                if (changed) {
                    return 0; /* a map bit for a field that is not carried */
                }
                continue;
            }
            if (changed) {
                uint64_t z = 0;
                const unsigned n = get_varint(in + at, len - at, &z);

                if (n == 0) {
                    return 0;
                }
                at += n;
                /* In unsigned arithmetic: a frame this codec wrote never
                 * leaves the field's range, but a crafted or corrupted one
                 * can carry a delta that overflows int64, which is undefined
                 * (UBSan: 1000 + INT64_MAX). Wrapping is defined, and the
                 * field's own width is what is written out anyway. */
                value[i] = (int64_t)((uint64_t)dec->previous[i] +
                                     (uint64_t)unzigzag(z));
            } else {
                value[i] = dec->previous[i];
            }
        }
    } else {
        return 0; /* neither I nor P: not a frame this codec wrote */
    }

    for (unsigned i = 0; i < AK_LOGENC_FIELDS; i++) {
        if (mask & ((uint64_t)1 << i)) {
            field_write(out, &k_fields[i], value[i]);
            dec->previous[i] = value[i];
        } else {
            field_write(out, &k_fields[i], 0);
        }
    }
    dec->have_previous = 1;
    return at;
}
