#pragma once
#include "lvgl.h"
#include <stdbool.h>
/* Native first-release UI. Funny screens are simulations, never vehicle data. */
void ui_phase1_brand_create(lv_obj_t *parent);
bool ui_phase1_boot_finished(void);
void ui_phase1_funny_open(void);
/* Synchronous boot entry; Logo may be released immediately after this call. */
void ui_phase1_funny_open_boot(void);

/* Shared approved brand imagery, lazily decoded once in PSRAM. */
lv_obj_t *ui_phase1_x_create(lv_obj_t *parent, int size);
lv_obj_t *ui_phase1_asset_create(lv_obj_t *parent, const void *asset);

bool ui_phase1_funny_active(void);
