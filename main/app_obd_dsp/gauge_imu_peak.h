#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Pure, fixed-storage trip envelope in calibrated screen-plane G units.
 * Reset only on explicit clear/accepted calibration, never on page changes. */
#define GM_PEAK_DIRECTIONS 64u
#define GM_PEAK_MAX_VERTICES 68u
#define GM_PEAK_INPUT_LIMIT_G 32.0f

typedef struct { float x, y; } gm_peak_point_t;
typedef struct {
    float support_g[GM_PEAK_DIRECTIONS];
    float radius_g;
    uint32_t sample_count;
} gm_peak_t;
typedef struct {
    gm_peak_point_t vertices[GM_PEAK_MAX_VERTICES];
    float radius_g;
    uint32_t sample_count;
    uint8_t vertex_count;
} gm_peak_snapshot_t;

void gm_peak_init(gm_peak_t *peak);
void gm_peak_clear(gm_peak_t *peak);
/* Rejects nonfinite inputs or either component outside +/-32G atomically.
 * Accepted points and origin are enclosed; support maxima never decrease. */
bool gm_peak_add(gm_peak_t *peak, float x_g, float y_g);
/* Returns 0 for empty history or insufficient capacity; no partial output.
 * Conservative convex outer envelope, not radial-bin joining. */
size_t gm_peak_polygon(const gm_peak_t *peak, gm_peak_point_t *out, size_t capacity);
void gm_peak_snapshot(const gm_peak_t *peak, gm_peak_snapshot_t *out);
bool gm_peak_contains(const gm_peak_t *peak, float x_g, float y_g, float tolerance_g);
float gm_peak_radius(const gm_peak_t *peak);
