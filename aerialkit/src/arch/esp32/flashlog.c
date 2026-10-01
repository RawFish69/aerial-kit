#include "esp.h"

#include "esp_partition.h"

#include "ak_flashlog.h"

/*
 * The blackbox in flash, on the ESP32.
 *
 * The log itself is the F405's: src/core/ak_flashlog.c, the same ring over
 * regions, the same 48-byte records, the same refusal to erase anything without
 * being asked. What is different is underneath it. The STM32's flash is a
 * register block the core can read and program; the ESP32's is behind a cache
 * and a vendor API, so this file is the adapter - and it is also where the
 * partition has to exist at all, because a chip with a partition table has no
 * free flash to write to, only flash that nobody has claimed yet.
 *
 * That is why there is a partitions.csv beside this port: a 704 KB `aerialkit`
 * partition, eleven 64 KB regions, about fifty minutes of log. A build whose
 * table does not have it gets a board with no flash log and a line on the
 * console saying so, rather than one that writes over whatever was there.
 *
 * Reads go through esp_partition_read, and that is where the interesting
 * number is. The core's log reads its records a *word* at a time, which costs
 * nothing on the F405 - memory-mapped flash, so a word is a load - and costs a
 * whole transaction here, because the vendor call disables the cache, talks to
 * the chip and re-enables it. A resume touches about two thousand words, so
 * the word path is about two thousand transactions before the aircraft has
 * done anything at all.
 *
 * That is why the store fills in read_block (ak_flashlog.h): one call per
 * record instead of one per four bytes, which is the same flash traffic to the
 * same addresses and 12 times fewer calls. Measured both ways in QEMU, where
 * the difference is impossible to miss - a boot with a used log went from 142
 * seconds to 12 - and measured on the host in tests/test_log.c, which runs the
 * same log over both read paths and counts the calls.
 *
 * It is an mmap-free version of the same idea: an mmap would remove the
 * per-call cost too, and it is a second view of flash that has to be kept in
 * step with every write. The flight loop is not the place to find out that it
 * was not.
 */

#define AK_ESP_LOG_LABEL  "aerialkit"
#define AK_ESP_LOG_SUBTYPE 0x40u /* a data partition of our own, not IDF's */
#define AK_ESP_LOG_REGION_BYTES (64u * 1024u)
#define AK_ESP_LOG_REGIONS 11u

_Static_assert(AK_ESP_LOG_REGION_BYTES * AK_ESP_LOG_REGIONS == 0x0B0000u,
               "the regions do not add up to the partition in partitions.csv");
_Static_assert(AK_ESP_LOG_REGIONS <= AK_FLASHLOG_MAX_SECTORS,
               "the core's log holds fewer regions than this partition has");

static const esp_partition_t *log_partition;
static ak_flashlog_region_t log_regions[AK_ESP_LOG_REGIONS];
static ak_flashlog_store_t log_store;
static int log_probed;

static uint32_t esp_log_read(uint32_t address)
{
    uint8_t bytes[4];
    uint32_t offset = address - log_partition->address;
    uint32_t word = 0u;

    if (esp_partition_read(log_partition, (size_t)offset, bytes,
                           sizeof bytes) != ESP_OK) {
        /* An erased word, which is what a failed read of flash looks like from
         * the log's side: it ends the scan rather than inventing a record. */
        return AK_FLASHLOG_ERASED;
    }
    for (unsigned i = 0u; i < 4u; i++) {
        word |= (uint32_t)bytes[i] << (8u * i);
    }
    return word;
}

/* A whole header or record in one call. The vendor API is happy with any
 * length here - the four-byte alignment the write path insists on is a
 * property of programming, not of reading - and the core only asks for the
 * header and the slots, so the buffer is always large enough. */
static int esp_log_read_block(uint32_t address, uint8_t *out, unsigned len)
{
    uint32_t offset;

    if (log_partition == 0 || len == 0u) {
        return -1;
    }
    offset = address - log_partition->address;
    if (offset + len > log_partition->size) {
        return -1;
    }
    return esp_partition_read(log_partition, (size_t)offset, out, (size_t)len) ==
                   ESP_OK
               ? 0
               : -1;
}

static int esp_log_erase(unsigned index)
{
    if (log_partition == 0 || index >= AK_ESP_LOG_REGIONS) {
        return -1;
    }
    /* Sixty-four kilobytes at a time is sixteen flash sectors: about half a
     * second of blocked CPU, which is why the core only asks for this when the
     * aircraft is disarmed. */
    return esp_partition_erase_range(
               log_partition, (size_t)index * AK_ESP_LOG_REGION_BYTES,
               AK_ESP_LOG_REGION_BYTES) == ESP_OK
               ? 0
               : -1;
}

static int esp_log_write(uint32_t address, const void *bytes, unsigned len)
{
    uint32_t offset = address - log_partition->address;

    if (log_partition == 0 || (len & 3u) != 0u || (offset & 3u) != 0u) {
        return -1; /* the vendor API wants whole words, and so does the part */
    }
    return esp_partition_write(log_partition, (size_t)offset, bytes,
                               (size_t)len) == ESP_OK
               ? 0
               : -1;
}

const ak_flashlog_store_t *ak_esp_log_store(void)
{
    if (log_probed) {
        return log_partition != 0 ? &log_store : 0;
    }
    log_probed = 1;

    log_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                             AK_ESP_LOG_SUBTYPE,
                                             AK_ESP_LOG_LABEL);
    if (log_partition == 0 ||
        log_partition->size < AK_ESP_LOG_REGION_BYTES * AK_ESP_LOG_REGIONS) {
        ak_console_write("flash log: no '" AK_ESP_LOG_LABEL "' partition in "
                         "this build's partition table\r\n");
        log_partition = 0;
        return 0;
    }

    for (unsigned i = 0u; i < AK_ESP_LOG_REGIONS; i++) {
        log_regions[i].base = log_partition->address + i * AK_ESP_LOG_REGION_BYTES;
        log_regions[i].bytes = AK_ESP_LOG_REGION_BYTES;
    }
    log_store.regions = log_regions;
    log_store.count = AK_ESP_LOG_REGIONS;
    log_store.read = esp_log_read;
    log_store.read_block = esp_log_read_block;
    log_store.erase = esp_log_erase;
    log_store.write = esp_log_write;
    return &log_store;
}
