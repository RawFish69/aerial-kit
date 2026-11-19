#pragma once

#include <esp_partition.h>
#include <esp_ota_ops.h>

#ifdef __cplusplus
extern "C"
{
#endif

const esp_partition_t *aerialkit_dual_role_next_update_partition(
    const esp_partition_t *start_from);
bool aerialkit_dual_role_commit_ota(void);

#ifdef __cplusplus
}
#endif

/*
 * ExpressLRS' Update library includes this header through the target build
 * flags. Updates stay inside the active role's slot pair, and devWIFI commits
 * the selected slot after Update.end() succeeds.
 */
#define esp_ota_get_next_update_partition(start_from) \
    aerialkit_dual_role_next_update_partition(start_from)
