#include "aerialkit_dual_role_ota.h"
#include "aerialkit_role_state.h"

#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <stdio.h>
#include <string.h>

namespace
{
const esp_partition_t *pending_target;

const esp_partition_t *find_role_partition(const char *role, uint8_t slot)
{
    char label[8];
    snprintf(label, sizeof(label), "%s_%u", role, slot);
    return esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        (esp_partition_subtype_t)(ESP_PARTITION_SUBTYPE_APP_OTA_MIN + slot),
        label);
}

const esp_partition_t *role_state_partition()
{
    return esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        (esp_partition_subtype_t)AERIALKIT_ROLE_STATE_SUBTYPE,
        "role_state");
}

int newest_record(const aerialkit_role_record_t records[2])
{
    const bool first_valid = aerialkit_role_record_valid(&records[0]);
    const bool second_valid = aerialkit_role_record_valid(&records[1]);
    if (first_valid && second_valid)
    {
        return records[1].sequence > records[0].sequence ? 1 : 0;
    }
    if (first_valid)
    {
        return 0;
    }
    return second_valid ? 1 : -1;
}
} // namespace

extern "C" const esp_partition_t *aerialkit_dual_role_next_update_partition(
    const esp_partition_t *start_from)
{
    (void)start_from;
    pending_target = nullptr;

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == nullptr)
    {
        return nullptr;
    }

    const char *role = nullptr;
    uint8_t active_slot = 0;
    if (strcmp(running->label, "tx_0") == 0)
    {
        role = "tx";
        active_slot = 0;
    }
    else if (strcmp(running->label, "tx_1") == 0)
    {
        role = "tx";
        active_slot = 1;
    }
    else if (strcmp(running->label, "rx_0") == 0)
    {
        role = "rx";
        active_slot = 0;
    }
    else if (strcmp(running->label, "rx_1") == 0)
    {
        role = "rx";
        active_slot = 1;
    }
    else
    {
        return nullptr;
    }

    pending_target = find_role_partition(role, active_slot ^ 1u);
    return pending_target;
}

extern "C" bool aerialkit_dual_role_commit_ota(void)
{
    if (pending_target == nullptr)
    {
        return false;
    }

    const esp_partition_t *state = role_state_partition();
    if (state == nullptr ||
        state->size < 2 * AERIALKIT_ROLE_STATE_SECTOR_SIZE)
    {
        return false;
    }

    aerialkit_role_record_t records[2] = {};
    for (uint32_t sector = 0; sector < 2; ++sector)
    {
        if (esp_partition_read(state,
                               sector * AERIALKIT_ROLE_STATE_SECTOR_SIZE,
                               &records[sector],
                               sizeof(records[sector])) != ESP_OK)
        {
            return false;
        }
    }

    const int active_index = newest_record(records);
    const aerialkit_role_record_t active =
        active_index >= 0
            ? records[active_index]
            : (aerialkit_role_record_t){
                  .magic = AERIALKIT_ROLE_RECORD_MAGIC,
                  .sequence = 0,
                  .tx_slot = 0,
                  .rx_slot = 2,
              };
    const uint32_t target_sector = active_index == 0 ? 1u : 0u;
    const uint32_t sequence =
        active.sequence == UINT32_MAX ? 1u : active.sequence + 1u;
    const uint8_t target_local_slot =
        (uint8_t)(pending_target->subtype - ESP_PARTITION_SUBTYPE_APP_OTA_MIN);

    aerialkit_role_record_t next = {
        .magic = AERIALKIT_ROLE_RECORD_MAGIC,
        .sequence = sequence,
        .tx_slot = active.tx_slot,
        .rx_slot = active.rx_slot,
    };
    if (strncmp(pending_target->label, "tx_", 3) == 0)
    {
        next.tx_slot = target_local_slot;
    }
    else if (strncmp(pending_target->label, "rx_", 3) == 0)
    {
        next.rx_slot = (uint8_t)(2 + target_local_slot);
    }
    else
    {
        return false;
    }
    next.crc = aerialkit_role_record_crc(&next);

    const uint32_t offset = target_sector * AERIALKIT_ROLE_STATE_SECTOR_SIZE;
    if (esp_partition_erase_range(state, offset,
                                  AERIALKIT_ROLE_STATE_SECTOR_SIZE) != ESP_OK ||
        esp_partition_write(state, offset, &next, sizeof(next)) != ESP_OK)
    {
        return false;
    }

    aerialkit_role_record_t verify = {};
    return esp_partition_read(state, offset, &verify, sizeof(verify)) == ESP_OK &&
           memcmp(&verify, &next, sizeof(next)) == 0 &&
           aerialkit_role_record_valid(&verify);
}
