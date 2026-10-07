#ifndef AK_CORE_AK_FLASHLOG_H
#define AK_CORE_AK_FLASHLOG_H

#include "ak_console.h"
#include "ak_log.h"

/*
 * The blackbox that outlives the battery.
 *
 * The two RAM rings answer "what did the loop do just now" and "what was it
 * doing when it stopped", and both lose everything when the power goes - which
 * is exactly what a crashed aircraft does. This is the third log: the same
 * records, written into flash, at a rate slow enough that a word-at-a-time
 * programming stall is affordable, and slow enough that the region lasts.
 *
 * The region is a ring **over sectors**. Records are written into a sector
 * until it is full, then into the next one, and a sector that is about to be
 * reused has to be erased first. That erase is the whole design problem: on
 * this part a 128 KB sector takes about a second to erase and the CPU waits
 * for it. So the log never erases anything by itself.
 *
 *   - It writes into sectors that are already erased.
 *   - When it runs out of those, it stops, and says so.
 *   - `ak_flashlog_service(log, may_erase)` erases the next sector, and the
 *     caller passes "yes" only when the aircraft is not flying.
 *
 * What that buys is a log that never costs the aircraft a second of control,
 * at the price of: the logging stops if nobody gives it a safe moment. On the
 * bench that is every boot; in the air it is the moment between disarming and
 * the next flight. `ak_flashlog_service` is cheap when there is nothing
 * pending, so the loop can call it every pass.
 *
 * Layout, and why each piece is where it is:
 *
 *   sector:  [32-byte header][record slots, 96 bytes each]
 *   header:  magic, version|slot bytes, sector index, sequence, first ms,
 *            reserved, reserved, checksum        (8 words)
 *   slot:    magic, payload checksum, 87-byte record, 1 byte of padding
 *
 * The sequence in the header is what makes "newest" a fact rather than a
 * guess: the reader takes the sector with the highest sequence as the one
 * being written, and reads the others in sequence order behind it. The
 * checksums are what make a half-written header or a slot the power cut in
 * half *skippable*: a record that does not check out is reported as torn
 * rather than decoded into a plausible-looking lie. And the slot size is 96
 * because a 128 KB sector holds exactly 1365 of them after the header - the
 * arithmetic is a property of the part, and it is better for it to divide
 * evenly than to leave a tail nobody accounts for. (It was 48 until the record
 * grew a yaw and an altitude, which is the pair a bad landing is judged by,
 * then 52, then 60; 80 is the size roadmap 2.4's debug fields reach, and it is
 * the *smallest* one that works rather than the next one: a slot is programmed
 * a word at a time, so its size has to be a multiple of four, and of the sizes
 * above the record's 74 bytes that divide a sector exactly, 80 is the first -
 * so a sector held 1638 records until 4.1 rather than the 2184 a 60-byte slot gave it,
 * which is 546 fewer: at five records a second a sector is five and a half
 * minutes of flight where it was seven. The six bytes between the record and
 * the end of its slot are padding rather than a field - and closing them by
 * growing the record would spend 6,912 bytes of the part on six bytes nobody
 * reads, because the record is held *three* times in RAM: the fast ring, the
 * long ring's fallback and the retained one, 384 records each. See ak_log.h.)
 *
 * **96 since 2026-10-05** (roadmap 4.1's fields, log version 4): the record is
 * 87 bytes and its slot 95, and of the multiples of four from there that
 * divide 131,040 exactly, 96 is the first - one byte of padding. A sector holds
 * 1365 records where it held 1638, and the F405's four-sector region 5460 where
 * it held 6552: eighteen minutes at five records a second where it was
 * twenty-two. The RAM cost is the record's growth times 384 times *two* now,
 * the fallback ring having gone in 4.1's first half.
 *
 * An 80-byte ring a previous build left in flash is not read by this one -
 * `version|slot bytes` is checked together, so a log written under the old
 * geometry is refused rather than parsed at the new stride.
 *
 * The record itself is stored in the explicit little-endian layout
 * `ak_log_encode_record` writes, not as a C struct: a log that only the
 * compiler that wrote it can read is not a log you can trust after the build
 * changes.
 */

#define AK_FLASHLOG_MAGIC      0x414B464Cu /* "AKFL" */
#define AK_FLASHLOG_SLOT_MAGIC 0x414B4653u /* "AKFS" */
#define AK_FLASHLOG_VERSION    4u
#define AK_FLASHLOG_HEADER_BYTES 32u
#define AK_FLASHLOG_SLOT_BYTES   96u
/* The most regions a store may offer. Eight was the F405's six with room; the
 * ESP32's partition is eleven 64 KB regions, so it is sixteen - and an arch
 * that offers more than this gets a compile-time answer rather than a log that
 * quietly reports "no storage" at boot, which is exactly how this number was
 * found. */
#define AK_FLASHLOG_MAX_SECTORS  16u
#define AK_FLASHLOG_ERASED       0xFFFFFFFFu

/* What a push or a service call reports. */
#define AK_FLASHLOG_OK          0
#define AK_FLASHLOG_NO_STORAGE (-1) /* the board has no flash for this */
#define AK_FLASHLOG_NEEDS_ERASE (-2) /* stopped: the next sector is not erased */
#define AK_FLASHLOG_WRITE_FAILED (-3) /* the flash refused a write */

/* One sector of the region a board offers. Variable-sized on purpose: the
 * part's sectors are not all the same size, and the code that walks them
 * should not have to know which is which. */
typedef struct {
    uint32_t base;
    uint32_t bytes;
} ak_flashlog_region_t;

typedef struct {
    const ak_flashlog_region_t *regions;
    unsigned count;
    /* Read the word at `address`. Reading needs no driver on either board this
     * firmware targets, but it does go through here, so that the log code
     * never contains an address it was not handed. */
    uint32_t (*read)(uint32_t address);
    /*
     * Optional: the same read over a whole span, in one call down to the
     * transport. Null means "read it a word at a time", which is exactly right
     * for memory-mapped flash - the F405 pays one load per word and nothing
     * else.
     *
     * It exists because the F405's assumption is not true on every chip. The
     * ESP32 reads flash through a vendor call that disables the cache and
     * starts a transaction, so a word-at-a-time scan is one of those per four
     * bytes: a resume that touches about two thousand words is about two
     * thousand transactions, and an emulator charges about a second for each.
     * A store that has this fills it in and the scan costs one call per record
     * instead. A failed block read is reported the way an erased word is - the
     * caller's magic check decides what it means - so a store that leaves this
     * null and a store that provides one cannot disagree about the log's
     * contents, only about how long the answer took.
     */
    int (*read_block)(uint32_t address, uint8_t *out, unsigned len);
    /* Erase sector `index` of the region. 0 on success. */
    int (*erase)(unsigned index);
    /* Program `len` bytes at `address`, word aligned, into erased words. */
    int (*write)(uint32_t address, const void *bytes, unsigned len);
} ak_flashlog_store_t;

/* The board's flash region for this log, or 0 for a board that has none - in
 * which case the RAM rings are all there is and the console says so. */
const ak_flashlog_store_t *ak_board_log_store(void);

typedef struct {
    const ak_flashlog_store_t *store;
    unsigned sector;         /* sector being written */
    uint32_t sequence;       /* its header's sequence */
    unsigned slot;           /* next slot to write in it */
    unsigned slots;          /* slots in that sector */
    unsigned pending_sector; /* the one that needs an erase */
    uint32_t records;        /* records the whole log holds */
    unsigned torn;           /* slots that failed their checksum */
    int      active;         /* a header has been written */
    int      stopped;        /* waiting for a safe moment to erase */
} ak_flashlog_t;

/* Look at what is in the flash and get ready to write. 0, or a negative
 * result from the list above. An empty or never-written part is a fresh log,
 * not an error.
 *
 * The store is an argument rather than a lookup: this module does not call the
 * board itself, so a test can hand it a region of ordinary memory and run the
 * whole ring - which is what tests/test_log.c does - while the firmware hands
 * it the one the board offers. */
int ak_flashlog_resume(ak_flashlog_t *log, const ak_flashlog_store_t *store);

/* One record into the flash. Returns AK_FLASHLOG_OK, or NEEDS_ERASE when the
 * log has stopped and wants `ak_flashlog_service` to make room. The caller
 * decides how often to call this: the log does not know the loop rate, and the
 * rate is what decides how long the region lasts.
 */
int ak_flashlog_push(ak_flashlog_t *log, const ak_log_record_t *record);

/* Erase the sector the log is waiting for, if `may_erase` - which the caller
 * sets from the aircraft's state, because this stops the CPU for about a
 * second. Cheap when there is nothing to do. */
int ak_flashlog_service(ak_flashlog_t *log, int may_erase);

/* How many records the whole log holds - its logical length, which is every
 * slot the log has written. A slot whose checksum does not match (a write the
 * power cut in half) is counted here and refused by `record_at`, because it
 * occupies a place in the order: a reader that skipped it would silently shift
 * every record after it. */
uint32_t ak_flashlog_count(const ak_flashlog_t *log);

/* The i-th record, oldest first, without reading the ones before it: 1 and the
 * record, 0 when the index is past the end, -1 for a slot that is there but
 * does not check out. The mapping from an index to a sector and a slot is
 * arithmetic - every sector behind the newest one is full - so this is a
 * handful of reads rather than a scan, which is what lets a tool pull a
 * sixteen-thousand-record log one record at a time. */
int ak_flashlog_record_at(const ak_flashlog_t *log, uint32_t index,
                          ak_log_record_t *record);

/* The whole log as CSV, oldest first, in the same columns as the RAM rings.
 * Scans the flash rather than using the writer's state, so it works on a
 * board that has only just booted. Returns the number of records written to
 * `out`. */
uint32_t ak_flashlog_dump(const ak_flashlog_t *log, ak_printf_fn out);

/* Erase every sector of the log and start again. The records are gone; this is
 * the command that has to be typed deliberately. */
int ak_flashlog_clear(ak_flashlog_t *log);

#endif /* AK_CORE_AK_FLASHLOG_H */
