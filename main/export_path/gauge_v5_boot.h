#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Separate gauge_v5/boot key; never changes the legacy default_page field.
 * 0..6: TEMP, INFO, CHART, NEEDLE, GEAR, RPM, SPEED.
 * 7..13: the seven native presets, including the new turbo at 13.
 * 14: Dragon Radar; 15: board IMU. Existing values keep their meaning. */
#define GV5_BOOT_PAGE_COUNT 16u
#define GV5_BOOT_DEFAULT 7u
#define GV5_BOOT_RADAR 14u
#define GV5_BOOT_IMU 15u

uint8_t gv5_boot_get(void);
/* False leaves the current in-memory selection unchanged, including when the
 * NVS commit fails. No namespace erase or migration of legacy settings. */
bool gv5_boot_set(uint8_t page);

/* Once per preview.6 installation, move a legacy startup choice to preset 7.
 * A committed gauge_v5/start6=1 preserves all subsequent user choices.
 * On a storage error return a native page for this boot, retain the cached
 * persisted choice, and retry on the next call. Never erase any keys.
 * Two NVS u8 keys are ordered writes, not a multi-key power-loss transaction. */
uint8_t gv5_boot_prepare_preview6_start(void);

#ifdef __cplusplus
}
#endif
