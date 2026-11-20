/*
 * Aerial Kit Rev A3 boot-time ELRS role selector.
 *
 * GPIO4 is sampled only during the second-stage boot. High selects the TX
 * role; low selects RX. The bootloader chooses the active application slot
 * for that role from the role_state partition. If the preferred slot is
 * invalid, it falls back only to the other slot for the same role.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AERIALKIT_GPIO_IN_REG (*(volatile uint32_t *)0x6000403cU)
#define AERIALKIT_IO_MUX_GPIO4_REG (*(volatile uint32_t *)0x60009014U)
#define AERIALKIT_GPIO4_FUNCTION 1U
#define AERIALKIT_MCU_SEL_SHIFT 12U
#define AERIALKIT_FUN_IE (1U << 9)

#define PARTITION_TABLE_OFFSET 0x8000u
#define PARTITION_TABLE_MAX_LEN 0xc00u
#define PARTITION_MAGIC 0x50aau
#define PARTITION_TYPE_END 0xffu

#define ROLE_TX 0u
#define ROLE_RX 1u
#define TX_SLOT_0 0u
#define TX_SLOT_1 1u
#define RX_SLOT_0 2u
#define RX_SLOT_1 3u
#define INVALID_PARTITION_INDEX (-99)

#define ROLE_RECORD_MAGIC 0x414b5253u

extern void *bootloader_mmap(uint32_t src_addr, uint32_t size);
extern void bootloader_munmap(const void *mapping);

typedef struct
{
    uint32_t offset;
    uint32_t size;
} partition_pos_t;

typedef struct
{
    uint16_t magic;
    uint8_t type;
    uint8_t subtype;
    partition_pos_t pos;
    uint8_t label[16];
    uint32_t flags;
} partition_info_t;

typedef struct
{
    partition_pos_t ota_info;
    partition_pos_t factory;
    partition_pos_t test;
    partition_pos_t ota[16];
    uint32_t app_count;
} bootloader_state_t;

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint8_t tx_slot;
    uint8_t rx_slot;
    uint16_t reserved;
    uint32_t crc;
} role_record_t;

static uint32_t fnv1a(const uint8_t *data, uint32_t length)
{
    uint32_t value = 2166136261u;
    for (uint32_t i = 0; i < length; ++i)
    {
        value ^= data[i];
        value *= 16777619u;
    }
    return value;
}

static bool record_valid(const role_record_t *record)
{
    return record->magic == ROLE_RECORD_MAGIC &&
           record->tx_slot <= TX_SLOT_1 &&
           record->rx_slot >= RX_SLOT_0 &&
           record->rx_slot <= RX_SLOT_1 &&
           record->crc == fnv1a((const uint8_t *)record,
                                offsetof(role_record_t, crc));
}

static uint32_t role_state_offset(void)
{
    const partition_info_t *table = bootloader_mmap(
        PARTITION_TABLE_OFFSET, PARTITION_TABLE_MAX_LEN);
    if (table == NULL)
    {
        return 0;
    }

    uint32_t offset = 0;
    const uint32_t count = PARTITION_TABLE_MAX_LEN / sizeof(partition_info_t);
    for (uint32_t i = 0; i < count; ++i)
    {
        if (table[i].type == PARTITION_TYPE_END)
        {
            break;
        }
        if (table[i].magic != PARTITION_MAGIC)
        {
            break;
        }
        if (table[i].label[0] == 'r' && table[i].label[1] == 'o' &&
            table[i].label[2] == 'l' && table[i].label[3] == 'e' &&
            table[i].label[4] == '_' && table[i].label[5] == 's' &&
            table[i].label[6] == 't' && table[i].label[7] == 'a' &&
            table[i].label[8] == 't' && table[i].label[9] == 'e' &&
            table[i].label[10] == 0)
        {
            offset = table[i].pos.offset;
            break;
        }
    }

    bootloader_munmap(table);
    return offset;
}

static const role_record_t *newest_record(const role_record_t records[2])
{
    const bool first_valid = record_valid(&records[0]);
    const bool second_valid = record_valid(&records[1]);
    if (first_valid && second_valid)
    {
        return records[1].sequence > records[0].sequence
                   ? &records[1]
                   : &records[0];
    }
    if (first_valid)
    {
        return &records[0];
    }
    if (second_valid)
    {
        return &records[1];
    }
    return NULL;
}

static bool image_header_valid(const bootloader_state_t *state, uint32_t slot)
{
    if (slot >= 16 || slot >= state->app_count ||
        state->ota[slot].offset == 0 || state->ota[slot].size == 0)
    {
        return false;
    }
    const uint8_t *header = bootloader_mmap(state->ota[slot].offset, 1);
    const bool valid = header != NULL && header[0] == 0xe9;
    if (header != NULL)
    {
        bootloader_munmap(header);
    }
    return valid;
}

static int choose_slot(const bootloader_state_t *state,
                       const role_record_t *record,
                       uint32_t role)
{
    const uint32_t primary = role == ROLE_TX ? TX_SLOT_0 : RX_SLOT_0;
    const uint32_t fallback = role == ROLE_TX ? TX_SLOT_1 : RX_SLOT_1;
    const uint32_t configured = record
                                    ? (role == ROLE_TX ? record->tx_slot
                                                       : record->rx_slot)
                                    : primary;

    if (image_header_valid(state, configured))
    {
        return configured;
    }
    if (image_header_valid(state, fallback))
    {
        return fallback;
    }
    if (image_header_valid(state, primary))
    {
        return primary;
    }
    return INVALID_PARTITION_INDEX;
}

int aerialkit_select_boot_partition(const bootloader_state_t *state)
{
    AERIALKIT_IO_MUX_GPIO4_REG =
        (AERIALKIT_IO_MUX_GPIO4_REG &
         ~(7U << AERIALKIT_MCU_SEL_SHIFT)) |
        (AERIALKIT_GPIO4_FUNCTION << AERIALKIT_MCU_SEL_SHIFT) |
        AERIALKIT_FUN_IE;

    const uint32_t role = (AERIALKIT_GPIO_IN_REG & (1U << 4)) ? ROLE_TX : ROLE_RX;
    const uint32_t state_offset = role_state_offset();
    const role_record_t *record = NULL;

    if (state_offset != 0)
    {
        const role_record_t *records = bootloader_mmap(state_offset, 0x2000);
        if (records != NULL)
        {
            record = newest_record(records);
            bootloader_munmap(records);
        }
    }

    return choose_slot(state, record, role);
}
