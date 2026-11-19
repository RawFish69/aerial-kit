#pragma once

#include <stdbool.h>
#include <stdint.h>

#define AERIALKIT_ROLE_RECORD_MAGIC 0x414b5253u
#define AERIALKIT_ROLE_STATE_SUBTYPE 0x40
#define AERIALKIT_ROLE_STATE_SECTOR_SIZE 0x1000u

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint8_t tx_slot;
    uint8_t rx_slot;
    uint16_t reserved;
    uint32_t crc;
} aerialkit_role_record_t;

static inline uint32_t aerialkit_role_record_crc(
    const aerialkit_role_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t value = 2166136261u;
    for (uint32_t i = 0; i < __builtin_offsetof(aerialkit_role_record_t, crc);
         ++i)
    {
        value ^= bytes[i];
        value *= 16777619u;
    }
    return value;
}

static inline bool aerialkit_role_record_valid(
    const aerialkit_role_record_t *record)
{
    return record->magic == AERIALKIT_ROLE_RECORD_MAGIC &&
           record->tx_slot <= 1 &&
           record->rx_slot >= 2 &&
           record->rx_slot <= 3 &&
           record->crc == aerialkit_role_record_crc(record);
}
