#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct { uint16_t width,height; const uint8_t *rle; size_t size; } phase1_asset_t;
extern const phase1_asset_t phase1_brand_top;
extern const phase1_asset_t phase1_brand_x;
extern const phase1_asset_t phase1_digi_bg;
extern const phase1_asset_t phase1_digi_ring;
extern const phase1_asset_t phase1_radar_bg;
extern const phase1_asset_t phase1_agumon_0;
extern const phase1_asset_t phase1_agumon_1;
extern const phase1_asset_t phase1_greymon_0;
extern const phase1_asset_t phase1_greymon_1;
