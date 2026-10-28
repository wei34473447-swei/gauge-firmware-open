#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Display-only: never feed this ratio to the vehicle cache or history. */
typedef struct {
    uint32_t anchor_ms;
    bool armed, active;
    float ratio;
} gv5_demo_t;

static inline void gv5_demo_reset(gv5_demo_t *d, uint32_t now) {
    *d = (gv5_demo_t){.anchor_ms = now};
}
static inline void gv5_demo_update(gv5_demo_t *d, uint32_t now, bool allowed) {
    if (!allowed) { gv5_demo_reset(d, now); return; }
    if (!d->armed) { d->anchor_ms = now; d->armed = true; }
    uint32_t elapsed = now - d->anchor_ms;
    d->active = false; d->ratio = 0;
    if (elapsed < 5000) return;
    uint32_t phase = elapsed % 5000;
    if (phase >= 1600) return;
    d->active = true;
    float t = phase < 600 ? phase / 600.0f : phase < 1000 ? 1.0f : (1600 - phase) / 600.0f;
    d->ratio = .5f - .5f * cosf(3.14159265358979323846f * t);
}
static inline uint32_t gv5_turbo_scale_color(unsigned is_boost, float value) {
    return is_boost && value <= 1.0f ? 0x21232Au : 0xFF0188u;
}
