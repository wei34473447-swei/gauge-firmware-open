#ifndef GAUGE_LVGL_MEMORY_H
#define GAUGE_LVGL_MEMORY_H
#include <stddef.h>
#include "esp_heap_caps.h"
/* LVGL-only allocations. DMA draw buffers and all non-LVGL malloc policy
 * remain unchanged. No internal fallback: do not consume the DMA reserve.
 * ESP-IDF capability realloc preserves the old allocation on nonzero failure.
 * LVGL handles zero-size sentinel semantics before calling these hooks. */
static inline void *gauge_lvgl_alloc(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
static inline void gauge_lvgl_free(void *ptr)
{
    heap_caps_free(ptr);
}
static inline void *gauge_lvgl_realloc(void *ptr, size_t size)
{
    return heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
#endif
