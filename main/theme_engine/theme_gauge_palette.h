#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"

#define THEME_GAUGE_ABI "ino-gauge-v5/1"
#define THEME_GAUGE_PALETTE_COUNT 8

typedef struct {
    uint32_t colors[THEME_GAUGE_PALETTE_COUNT];
    bool valid;
} theme_gauge_palette_t;

// Parse once at load. Missing extension succeeds with valid=false. Any invalid
// extension fails with a cleared output; never retain a previous palette.
bool theme_gauge_palette_parse(const cJSON *manifest, theme_gauge_palette_t *out);
