#ifndef AK_FLIGHT_AK_LOGENC_H
#define AK_FLIGHT_AK_LOGENC_H

#include "ak_log.h"
#include "ak_types.h"

/*
 * The blackbox's encoding (roadmap 4.2): delta and variable length, so a log
 * at 1 kHz fits in the flash the fixed-width record cannot.
 *
 * The record in ak_log.h is 87 bytes (version 4, roadmap 4.1), and in flash it
 * occupies a 96-byte slot (ak_flashlog.h: 1365 slots to a 128 KB sector, and the
 * F405 gives the blackbox four of them, so 512 KB and 5460 slots). At 1 kHz that
 * region holds **5.5 seconds** of flight, and the 60-second log phase 4 asks for
 * is 5.2 MB - ten times the region, and five times the whole 1 MB part. Phase 4's
 * acceptance asks for that log to download intact, so the bytes have to come
 * from somewhere, and the only place they can come from is the fact that
 * consecutive records are mostly the same.
 *
 * What that buys, measured rather than estimated (tests/test_logenc.c, on
 * records carrying version 4's controller columns): 33.0% of the fixed width on
 * a clean flight and 59.3% with the gyro jittered across its range. At 33.0% the
 * 512 KB region holds 18 seconds instead of 5.5 - three times the flight in the
 * same part - and a 60-second 1 kHz log is still 1.7 MB.
 * That is what roadmap 4.3's SPI flash and SD-card backends are for. The
 * internal region was never going to hold sixty seconds; what this milestone
 * does is make adding the external one worth it, because the difference is
 * between a log of the interesting 18 seconds and a log of the first 5.
 *
 * One consequence belongs to 4.3 and is written here so that it is not a
 * surprise there: a frame is not a fixed width any more, and the slot geometry
 * above is what lets a sector be read without decoding everything before it. A
 * stream of variable-length frames has to be walked from its start, so either a
 * sector gains a header saying where its keyframes are, or a frame pads to the
 * slot and 4.3 picks the slot size to make the padding pay. Nothing in this
 * file decides that; this file only makes the frames small enough that the
 * question is worth asking.
 *
 * The design is the general one - an independent frame every so often, and
 * between them only what changed - and the two decisions worth reading are
 * about what "changed" costs and when the codec gives up on being clever.
 *
 * DECISION 1: the field table IS the record layout, scalar by scalar.
 *
 * A field here is one scalar of `ak_log_record_t`, in the order
 * `ak_log_encode_record` writes them, and there are 49 of them. The order is
 * not decoration and not an accident of how this file was typed: it is the wire
 * order, so an I-frame carrying every field is *byte for byte* the fixed-width
 * record that `ak_log_encode_record` produces. A reader that predates this
 * module can still read the keyframes out of a log, and the round-trip test
 * asserts the equality rather than describing it.
 *
 * DECISION 2: a P-frame that would be bigger than an I-frame is not written.
 *
 * The encoder builds the delta frame first and compares it against what a
 * keyframe would cost; if the delta is not smaller, the keyframe is what goes
 * out. The consequence is a bound that holds for every input rather than for
 * the inputs someone thought of: **no frame this module writes is larger than
 * the fixed-width record plus one byte**, which is a statement about the worst
 * case and therefore one that a log's size can be planned with.
 *
 * The worst case is reachable and it is worth saying what it costs, because it
 * is *not* free: a record whose every scalar is a fresh random number carries
 * the same number of bits as a delta as it does full width, so every frame is a
 * keyframe and the log is the fixed format plus its type byte - 101.1% of what
 * the fixed format costs, measured over 256 such records in tests/test_logenc.c.
 * One byte per record and no more is the whole of the downside, and no flight
 * is that input: a real one repeats itself in most of its forty-nine fields
 * even when the gyro is noisy, and the same test measures 59.3% there and 33.0%
 * on a clean flight.
 *
 * What "changed" costs: one bit per scalar, so the presence map is seven bytes,
 * and then a zigzag LEB128 delta for each changed scalar. Seven bytes of map is
 * not free at 1 kHz - it is 7 kB/s on its own - and it is still the right
 * trade, because the alternative is a fixed mask of "the fields that usually
 * change", which is a guess about the data that is wrong on exactly the flight
 * that matters. A field that did not change costs one bit and nothing else;
 * a field that changed by a little costs one bit and one byte.
 *
 * DECISION 3: the header carries the field set and the parameter hash, not a
 * hash of this table.
 *
 * The header has the mask of fields the log carries and `config_hash`, the
 * caller's hash of the parameter table the flight was flown with - which is
 * what makes a log explainable: the same aircraft with different gains is a
 * different aircraft, and a log that does not say which one it was is a log
 * whose numbers cannot be compared with anything.
 *
 * What the header deliberately does *not* carry is a hash of the field table
 * itself. It would be easy to add and it would be wrong: fields are appended
 * to this table, never reordered or resized, so a reader built before a field
 * existed reads every log that does not use it - which is the same
 * compatibility story the record layout already tells, and hashing the table
 * would refuse those logs instead. What the header carries instead is
 * `field_count`, and the decoder refuses a log that used a field it has never
 * heard of rather than skipping bytes it does not understand and reading
 * everything after them at the wrong offset.
 *
 * The rule that keeps that safe is the record's own rule, and it is worth
 * restating where the table is: **a field may be appended, and a field's kind
 * may never change** - changing one is a new `AK_LOGENC_VERSION`, because the
 * bytes are the same length and the numbers are not the same numbers.
 */

#define AK_LOGENC_MAGIC   0x324C4B41u /* "AKL2" */
#define AK_LOGENC_VERSION 1u

/* The scalars of `ak_log_record_t`, in the order the record is written. */
#define AK_LOGENC_FIELDS 49u

/* One bit per field, rounded up: seven bytes today, and the test asserts the
 * arithmetic rather than the number so that appending a field cannot silently
 * leave the map too small. */
#define AK_LOGENC_MAP_BYTES ((AK_LOGENC_FIELDS + 7u) / 8u)

/* magic 4, version 1, field_count 1, rate 2, decimation 2, config_hash 4,
 * field_mask 8. */
#define AK_LOGENC_HEADER_BYTES 22u

/* A frame is one type byte plus at most a keyframe's payload. The encoder
 * chooses, so this is the worst case and not an estimate. */
#define AK_LOGENC_MAX_FRAME_BYTES (1u + AK_LOG_WIRE_BYTES)

typedef enum {
    AK_LOGENC_I = 0, /* every carried field, at its full width */
    AK_LOGENC_P = 1  /* the presence map and the deltas */
} ak_logenc_type_t;

/* A field's storage class. The widths are the record's own, and they are the
 * reason a field's kind may never change: `state` and `motor0` are both one
 * byte, so a kind that changed from unsigned to signed would keep every frame
 * the same length and read back different numbers. */
typedef enum {
    AK_LF_I8,
    AK_LF_U8,
    AK_LF_I16,
    AK_LF_U16,
    AK_LF_I32,
    AK_LF_U32
} ak_logenc_kind_t;

typedef struct {
    const char *name;
    ak_logenc_kind_t kind;
    uint16_t offset; /* into ak_log_record_t */
} ak_logenc_field_t;

typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  field_count;
    uint16_t rate_hz;
    uint16_t decimation;
    uint32_t config_hash;
    uint64_t field_mask;
} ak_logenc_header_t;

/*
 * Writes the header. Returns the bytes written, or 0 when the capacity is too
 * small or the mask names a field this build does not have - a log whose
 * header cannot be written is not a log, and half a header is worse than none.
 */
unsigned ak_logenc_write_header(uint64_t field_mask, uint16_t rate_hz,
                                uint16_t decimation, uint32_t config_hash,
                                uint8_t *out, unsigned capacity);

/*
 * Reads one back. Returns 1 when the header is one this build can read, 0 when
 * it is not - and every way of not being readable is a refusal rather than a
 * guess: a wrong magic, a version this build does not know, a field count
 * larger than the table, or a mask that names a field beyond that count.
 */
int ak_logenc_read_header(const uint8_t *in, unsigned len,
                          ak_logenc_header_t *out);

/* The mask of every field, which is what a log with no parameter-driven field
 * selection carries. */
#define AK_LOGENC_MASK_ALL ((AK_LOGENC_FIELDS >= 64u) \
                                ? ~(uint64_t)0 \
                                : (((uint64_t)1 << AK_LOGENC_FIELDS) - 1u))

typedef struct {
    int64_t  previous[AK_LOGENC_FIELDS];
    uint64_t field_mask;
    int      have_previous;
    uint32_t since_key;
    /* How often a keyframe is forced, in frames, or 0 for "only when the delta
     * is not smaller". A forced keyframe is what bounds how far a reader has to
     * rewind to start reading a log in the middle, and it is the caller's
     * decision because the caller knows what the log is for. */
    uint32_t keyframe_every;
} ak_logenc_enc_t;

void ak_logenc_encoder_init(ak_logenc_enc_t *enc, uint64_t field_mask,
                            uint32_t keyframe_every);

/*
 * Encodes one record. Returns the bytes written, which is never more than
 * AK_LOGENC_MAX_FRAME_BYTES, or 0 when the capacity is too small.
 *
 * The first record after init is always a keyframe: a delta needs something to
 * be a delta from, and inventing a zeroed previous record is how a silently
 * wrong log gets written.
 */
unsigned ak_logenc_encode(ak_logenc_enc_t *enc, const ak_log_record_t *record,
                          uint8_t *out, unsigned capacity);

typedef struct {
    int64_t  previous[AK_LOGENC_FIELDS];
    uint64_t field_mask;
    int      have_previous;
} ak_logenc_dec_t;

void ak_logenc_decoder_init(ak_logenc_dec_t *dec, uint64_t field_mask);

/*
 * Decodes one frame. Returns the bytes it consumed, or 0 when the frame is not
 * one this decoder can read - a type byte that is neither I nor P, a map that
 * runs past the buffer, a varint that does not end, or a P-frame with no
 * keyframe before it. A refusal leaves `out` untouched, so a caller that keeps
 * its last good record keeps a record and not a mixture.
 *
 * The returned length is what makes a stream walkable: decode in a loop,
 * advancing by what each call consumed, and a frame the codec cannot read stops
 * the walk instead of desynchronising it.
 */
unsigned ak_logenc_decode(ak_logenc_dec_t *dec, const uint8_t *in, unsigned len,
                          ak_log_record_t *out);

/* The name of a field, for a reader that wants to say which one it refused.
 * Returns 0 for an index past the table. */
const char *ak_logenc_field_name(unsigned index);

/* How many fields a mask names. */
unsigned ak_logenc_mask_count(uint64_t mask);

#endif /* AK_FLIGHT_AK_LOGENC_H */
