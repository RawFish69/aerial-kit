/*
 * The log in flash. See ak_flashlog.h for the layout and for why the erase is
 * the caller's decision.
 *
 * Two properties are worth keeping in mind while reading this. Nothing writes
 * to a word twice: flash can only clear bits, so a slot is programmed once and
 * a sector is erased before it is reused - which is why the write pointer is
 * *found*, by looking for the first slot that is still all ones, rather than
 * kept in a counter that would itself have to be rewritten. And nothing here
 * believes a header or a record just because it is there: both carry a
 * checksum, and one that fails is skipped and counted rather than decoded into
 * a plausible-looking lie.
 */

#include "ak_flashlog.h"

#include <string.h>

static uint32_t fnv1a(const uint8_t *bytes, unsigned len)
{
    uint32_t hash = 2166136261u;
    for (unsigned i = 0; i < len; i++) {
        hash ^= bytes[i];
        hash *= 16777619u;
    }
    return hash;
}

static void put_u32(uint8_t *out, unsigned at, uint32_t value)
{
    out[at + 0u] = (uint8_t)(value & 0xFFu);
    out[at + 1u] = (uint8_t)((value >> 8) & 0xFFu);
    out[at + 2u] = (uint8_t)((value >> 16) & 0xFFu);
    out[at + 3u] = (uint8_t)((value >> 24) & 0xFFu);
}

static uint32_t get_u32(const uint8_t *in, unsigned at)
{
    return (uint32_t)in[at + 0u] | ((uint32_t)in[at + 1u] << 8) |
           ((uint32_t)in[at + 2u] << 16) | ((uint32_t)in[at + 3u] << 24);
}

static uint32_t word_at(const ak_flashlog_t *log, uint32_t address)
{
    return log->store->read(address);
}

static void read_bytes(const ak_flashlog_t *log, uint32_t address, uint8_t *out,
                       unsigned len)
{
    /* One call for the whole span where the transport has one: on a chip whose
     * flash is behind a driver, the word loop below is one transaction per
     * four bytes, and a scan reads a few thousand words. */
    if (log->store->read_block != 0) {
        if (log->store->read_block(address, out, len) == 0) {
            return;
        }
        /* A read the transport would not do is read as an erased part: every
         * check above this treats one as "there is nothing of ours here",
         * which is the honest answer and not a made-up record. */
        for (unsigned i = 0u; i < len; i++) {
            out[i] = (uint8_t)AK_FLASHLOG_ERASED;
        }
        return;
    }

    for (unsigned at = 0u; at < len; at += 4u) {
        uint32_t word = word_at(log, address + at);
        for (unsigned byte = 0u; byte < 4u && at + byte < len; byte++) {
            out[at + byte] = (uint8_t)((word >> (8u * byte)) & 0xFFu);
        }
    }
}

static uint32_t slot_address(const ak_flashlog_t *log, unsigned sector,
                             unsigned slot)
{
    return log->store->regions[sector].base + AK_FLASHLOG_HEADER_BYTES +
           slot * AK_FLASHLOG_SLOT_BYTES;
}

static unsigned slots_in(const ak_flashlog_t *log, unsigned sector)
{
    uint32_t bytes = log->store->regions[sector].bytes;

    if (bytes <= AK_FLASHLOG_HEADER_BYTES) {
        return 0u;
    }
    return (bytes - AK_FLASHLOG_HEADER_BYTES) / AK_FLASHLOG_SLOT_BYTES;
}

/* A sector with nothing in it starts with an erased word. Anything else has
 * been written by somebody - this log before a wrap, or another firmware
 * entirely - and has to be erased before it can be used. */
static int sector_blank(const ak_flashlog_t *log, unsigned sector)
{
    return word_at(log, log->store->regions[sector].base) == AK_FLASHLOG_ERASED;
}

static void header_build(unsigned sector, uint32_t sequence, uint32_t first_ms,
                         uint8_t *out)
{
    put_u32(out, 0u, AK_FLASHLOG_MAGIC);
    put_u32(out, 4u, (uint32_t)AK_FLASHLOG_VERSION |
                         ((uint32_t)AK_FLASHLOG_SLOT_BYTES << 16));
    put_u32(out, 8u, sector);
    put_u32(out, 12u, sequence);
    put_u32(out, 16u, first_ms);
    put_u32(out, 20u, 0u);
    put_u32(out, 24u, 0u);
    put_u32(out, 28u, fnv1a(out, 28u));
}

/* Reads a sector header. Returns 1 when it is one of ours and says what it
 * says; 0 for an erased, foreign or damaged one - which are all the same thing
 * to a reader: not a sector to take records from. */
static int header_read(const ak_flashlog_t *log, unsigned sector,
                       uint32_t *sequence, uint32_t *first_ms)
{
    uint8_t header[AK_FLASHLOG_HEADER_BYTES];

    read_bytes(log, log->store->regions[sector].base, header, sizeof header);

    if (get_u32(header, 0u) != AK_FLASHLOG_MAGIC ||
        (get_u32(header, 4u) & 0xFFFFu) != AK_FLASHLOG_VERSION ||
        (get_u32(header, 4u) >> 16) != AK_FLASHLOG_SLOT_BYTES ||
        get_u32(header, 8u) != sector ||
        get_u32(header, 28u) != fnv1a(header, 28u)) {
        return 0;
    }
    if (sequence != 0) {
        *sequence = get_u32(header, 12u);
    }
    if (first_ms != 0) {
        *first_ms = get_u32(header, 16u);
    }
    return 1;
}

/* One slot: 1 and the record, 0 for a slot that was never written, -1 for one
 * whose checksum does not match - a slot the power cut in half, which is
 * skipped rather than decoded. */
static int slot_read(const ak_flashlog_t *log, unsigned sector, unsigned slot,
                     ak_log_record_t *record)
{
    uint32_t address = slot_address(log, sector, slot);
    uint32_t magic = word_at(log, address);
    uint8_t payload[AK_LOG_WIRE_BYTES];

    if (magic == AK_FLASHLOG_ERASED) {
        return 0;
    }
    if (magic != AK_FLASHLOG_SLOT_MAGIC) {
        return -1;
    }
    read_bytes(log, address + 8u, payload, sizeof payload);
    if (word_at(log, address + 4u) != fnv1a(payload, sizeof payload)) {
        return -1;
    }
    return ak_log_decode_record(payload, sizeof payload, record) ? 1 : -1;
}

static int slot_write(const ak_flashlog_t *log, unsigned sector, unsigned slot,
                      const ak_log_record_t *record)
{
    uint8_t bytes[AK_FLASHLOG_SLOT_BYTES];
    uint8_t payload[AK_LOG_WIRE_BYTES];

    if (ak_log_encode_record(record, payload, sizeof payload) !=
        AK_LOG_WIRE_BYTES) {
        return -1;
    }

    put_u32(bytes, 0u, AK_FLASHLOG_SLOT_MAGIC);
    put_u32(bytes, 4u, fnv1a(payload, sizeof payload));
    memcpy(&bytes[8], payload, sizeof payload);
    /* Whatever is left over: the padding that makes a slot a whole number of
     * words and a sector divide evenly into them. Written out rather than
     * assumed, because the two numbers change together when the record does -
     * they just did, and a hard-coded 3 would have written past the buffer. */
    if (8u + sizeof payload < sizeof bytes) {
        memset(&bytes[8 + sizeof payload], 0,
               sizeof bytes - 8u - sizeof payload);
    }

    return log->store->write(slot_address(log, sector, slot), bytes,
                             sizeof bytes) == 0
               ? 0
               : -1;
}

/* What a sector holds: how many slots have been touched, how many of those
 * decode, and how many do not. Slots are written in order, so the first slot
 * still erased is where writing stopped. */
typedef struct {
    unsigned used;
    unsigned valid;
    unsigned torn;
} sector_scan_t;

static void scan_sector(const ak_flashlog_t *log, unsigned sector,
                        sector_scan_t *scan)
{
    unsigned slots = slots_in(log, sector);

    memset(scan, 0, sizeof *scan);
    for (unsigned slot = 0u; slot < slots; slot++) {
        ak_log_record_t record;
        int state = slot_read(log, sector, slot, &record);

        if (state == 0) {
            break;
        }
        scan->used++;
        if (state > 0) {
            scan->valid++;
        } else {
            scan->torn++;
        }
    }
}

static void forget(ak_flashlog_t *log)
{
    log->sector = 0u;
    log->sequence = 0u;
    log->slot = 0u;
    log->slots = log->store != 0 ? slots_in(log, 0u) : 0u;
    log->pending_sector = 0u;
    log->records = 0u;
    log->torn = 0u;
    log->active = 0;
    log->stopped = 0;
}

int ak_flashlog_resume(ak_flashlog_t *log, const ak_flashlog_store_t *store)
{
    memset(log, 0, sizeof *log);
    if (store == 0 || store->count == 0u ||
        store->count > AK_FLASHLOG_MAX_SECTORS || store->regions == 0 ||
        store->read == 0 || store->write == 0 || store->erase == 0) {
        /* Left inert rather than half-set-up: every other call checks the
         * store, and a log whose store is a struct full of zeroes would
         * dereference a region table that is not there. */
        return AK_FLASHLOG_NO_STORAGE;
    }
    log->store = store;

    /* Which sector is being written is the one with the highest sequence: it
     * is the only piece of order this log keeps, and it is what makes a wrap
     * readable - the sectors behind it are older, whatever their position in
     * the region. */
    unsigned newest = 0u;
    uint32_t newest_sequence = 0u;
    unsigned found = 0u;

    for (unsigned sector = 0u; sector < log->store->count; sector++) {
        uint32_t sequence = 0u;

        if (!header_read(log, sector, &sequence, 0)) {
            continue;
        }
        if (!found || sequence > newest_sequence) {
            newest = sector;
            newest_sequence = sequence;
            found = 1u;
        }
    }

    if (!found) {
        forget(log); /* a part with nothing in it, or nothing of ours */
        return AK_FLASHLOG_OK;
    }

    log->sector = newest;
    log->sequence = newest_sequence;
    log->slots = slots_in(log, newest);
    log->active = 1;

    /* Everything the log holds counts towards the total the console reports,
     * and the newest sector is also where writing carries on: its first erased
     * slot is the write pointer. */
    for (unsigned sector = 0u; sector < log->store->count; sector++) {
        sector_scan_t scan;

        if (!header_read(log, sector, 0, 0)) {
            continue;
        }
        scan_sector(log, sector, &scan);
        log->records += scan.used;
        log->torn += scan.torn;
        if (sector == newest) {
            log->slot = scan.used;
        }
    }
    return AK_FLASHLOG_OK;
}

/* Writes a sector header, or reports that the sector is not ready. The caller
 * has already decided that this sector is the one to use. */
static int start_sector(ak_flashlog_t *log, uint32_t first_ms)
{
    uint8_t header[AK_FLASHLOG_HEADER_BYTES];

    log->slots = slots_in(log, log->sector);
    if (log->slots == 0u) {
        return AK_FLASHLOG_NO_STORAGE;
    }
    if (!sector_blank(log, log->sector)) {
        log->pending_sector = log->sector;
        log->stopped = 1;
        return AK_FLASHLOG_NEEDS_ERASE;
    }

    header_build(log->sector, log->sequence, first_ms, header);
    if (log->store->write(log->store->regions[log->sector].base, header,
                          sizeof header) != 0) {
        return AK_FLASHLOG_WRITE_FAILED;
    }
    log->slot = 0u;
    log->active = 1;
    return AK_FLASHLOG_OK;
}

int ak_flashlog_push(ak_flashlog_t *log, const ak_log_record_t *record)
{
    if (log->store == 0) {
        return AK_FLASHLOG_NO_STORAGE;
    }
    if (log->stopped) {
        return AK_FLASHLOG_NEEDS_ERASE;
    }

    if (!log->active) {
        int started;

        if (log->sequence == 0u) {
            log->sequence = 1u; /* the first sector this part has seen */
        }
        started = start_sector(log, record->time_ms);
        if (started != AK_FLASHLOG_OK) {
            return started;
        }
    }

    if (log->slot >= log->slots) {
        /* This sector is full. The next one has to be erased before anything
         * more can be written - and that is the moment the log stops, because
         * erasing is not a decision this code is allowed to make by itself. */
        int started;

        log->sector = (log->sector + 1u) % log->store->count;
        log->sequence++;
        log->active = 0;

        started = start_sector(log, record->time_ms);
        if (started != AK_FLASHLOG_OK) {
            return started;
        }
    }

    if (slot_write(log, log->sector, log->slot, record) != 0) {
        return AK_FLASHLOG_WRITE_FAILED;
    }
    log->slot++;
    log->records++;
    return AK_FLASHLOG_OK;
}

int ak_flashlog_service(ak_flashlog_t *log, int may_erase)
{
    if (log->store == 0) {
        return AK_FLASHLOG_NO_STORAGE;
    }
    if (!log->stopped) {
        return AK_FLASHLOG_OK;
    }
    if (!may_erase) {
        return AK_FLASHLOG_NEEDS_ERASE;
    }

    /* What is in the sector that is about to go is no longer in the log, and
     * the count the console reports has to say so: an erase is the only thing
     * here that ever removes records. */
    sector_scan_t scan;

    scan_sector(log, log->pending_sector, &scan);
    if (log->store->erase(log->pending_sector) != 0) {
        return AK_FLASHLOG_WRITE_FAILED;
    }
    log->records = log->records >= scan.used ? log->records - scan.used : 0u;
    log->torn = log->torn >= scan.torn ? log->torn - scan.torn : 0u;
    log->stopped = 0;
    return AK_FLASHLOG_OK;
}

uint32_t ak_flashlog_count(const ak_flashlog_t *log)
{
    return log->store != 0 ? log->records : 0u;
}

/* The sectors in the log, in the order they were written: the sequence in each
 * header is the only order there is, because the ring wraps. Returns how many
 * were found. */
static unsigned sector_order(const ak_flashlog_t *log, unsigned *order)
{
    uint32_t sequence[AK_FLASHLOG_MAX_SECTORS];
    unsigned found = 0u;

    for (unsigned sector = 0u; sector < log->store->count; sector++) {
        uint32_t value = 0u;

        if (!header_read(log, sector, &value, 0)) {
            continue;
        }
        order[found] = sector;
        sequence[found] = value;
        found++;
    }
    for (unsigned i = 1u; i < found; i++) {
        for (unsigned j = i; j > 0u && sequence[j - 1u] > sequence[j]; j--) {
            unsigned sector = order[j - 1u];
            uint32_t value = sequence[j - 1u];

            order[j - 1u] = order[j];
            sequence[j - 1u] = sequence[j];
            order[j] = sector;
            sequence[j] = value;
        }
    }
    return found;
}

int ak_flashlog_record_at(const ak_flashlog_t *log, uint32_t index,
                          ak_log_record_t *record)
{
    unsigned order[AK_FLASHLOG_MAX_SECTORS];
    unsigned found;

    if (log->store == 0 || record == 0) {
        return 0;
    }
    found = sector_order(log, order);
    if (found == 0u) {
        return 0;
    }

    /* Every sector behind the newest one is full - the log only moves on when
     * the one it is in has no slots left - so the index is a subtraction per
     * sector and then a slot number, with no scanning at all. */
    unsigned newest = order[found - 1u];

    for (unsigned i = 0u; i < found; i++) {
        unsigned sector = order[i];
        unsigned in_sector = slots_in(log, sector);
        unsigned used = sector == newest ? log->slot : in_sector;

        if (index < used) {
            return slot_read(log, sector, index, record);
        }
        index -= used;
    }
    return 0;
}

uint32_t ak_flashlog_dump(const ak_flashlog_t *log, ak_printf_fn out)
{
    /* The sectors in the order they were written, which is what sector_order()
     * is for - not their order in the region, because the ring wraps. */
    unsigned order[AK_FLASHLOG_MAX_SECTORS];
    sector_scan_t scan[AK_FLASHLOG_MAX_SECTORS];
    unsigned found = 0u;
    uint32_t records = 0u;
    unsigned torn = 0u;
    uint32_t written = 0u;

    if (log->store == 0) {
        out("# no flash log on this board\n");
        return 0u;
    }

    found = sector_order(log, order);
    for (unsigned i = 0u; i < found; i++) {
        scan_sector(log, order[i], &scan[i]);
        records += scan[i].valid;
        torn += scan[i].torn;
    }

    out("# aerialkit blackbox (flash), %u records in %u sector%s", records,
        found, found == 1u ? "" : "s");
    if (torn > 0u) {
        out(", %u slot%s skipped - torn or damaged", torn,
            torn == 1u ? "" : "s");
    }
    out("\n");
    out("# oldest first; this one outlives the battery\n");
    out("# units: gyro 0.1 dps, accel 0.001 g, attitude 0.1 deg, "
        "alt mm above the take-off reference, sticks per-mille, torque "
        "percent, motor 0..254\n");
    out("time_ms,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,roll,pitch,yaw,"
        "alt_mm,stick_roll,stick_pitch,stick_yaw,stick_throttle,torque_roll,"
        "torque_pitch,torque_yaw,motor1,motor2,motor3,motor4,state,flags\n");

    for (unsigned i = 0u; i < found; i++) {
        for (unsigned slot = 0u; slot < scan[i].used; slot++) {
            ak_log_record_t record;

            if (slot_read(log, order[i], slot, &record) == 1) {
                ak_log_write_record(&record, out);
                written++;
            }
        }
    }
    return written;
}

int ak_flashlog_clear(ak_flashlog_t *log)
{
    if (log->store == 0) {
        return AK_FLASHLOG_NO_STORAGE;
    }
    for (unsigned sector = 0u; sector < log->store->count; sector++) {
        if (log->store->erase(sector) != 0) {
            return AK_FLASHLOG_WRITE_FAILED;
        }
    }
    forget(log);
    return AK_FLASHLOG_OK;
}
