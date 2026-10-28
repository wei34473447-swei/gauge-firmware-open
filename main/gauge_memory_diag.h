#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Observed touched-stack minima by kind since this boot, across instances.
 * UINT32_MAX means never sampled; zero is a real sampled value. No handle,
 * liveness, age, per-instance, whole-system or worst-peak claim is implied. */
typedef enum {
    GAUGE_STACK_MAIN = 0,
    GAUGE_STACK_LVGL,
    GAUGE_STACK_IMU,
    GAUGE_STACK_OBD,
    GAUGE_STACK_HTTPD,
    GAUGE_STACK_OTA_TIMEOUT,
    GAUGE_STACK_BM_WRITER,
    GAUGE_STACK_OTA_VALID,
    GAUGE_STACK_OTA_RESTART,
    GAUGE_STACK_KIND_COUNT,
} gauge_stack_kind_t;

/* Owner task only, gated by the caller's existing low-frequency loop/event.
 * Native NULL scan runs before the bounded shared statistics lock. */
void gauge_stack_diag_record_self(gauge_stack_kind_t kind);
void gauge_stack_diag_get(uint32_t out[GAUGE_STACK_KIND_COUNT]);
/* Strong app-side endpoint for the local HTTPD's weak owner-only hook. */
void gauge_httpd_stack_record_self(void);

/* Heap measurements are byte precision, sampled in the existing task only.
 * region_lower_bound sums independent region historical lows, not true peak.
 * observed_sampled_* is a sampled low and can miss brief allocation pressure.
 * attempted failures include successful fallbacks, not critical feature errors. */
typedef enum {
    GAUGE_MEMORY_ENTRY = 0,
    GAUGE_MEMORY_AFTER_DISPLAY,
    GAUGE_MEMORY_SINGLE_BUFFER_RETRY,
    GAUGE_MEMORY_BEFORE_BLE,
    GAUGE_MEMORY_AFTER_BLE,
    GAUGE_MEMORY_BEFORE_ESPNOW,
    GAUGE_MEMORY_AFTER_ESPNOW,
    GAUGE_MEMORY_RUNTIME,
} gauge_memory_phase_t;

typedef struct {
    uint32_t free_bytes;
    uint32_t largest_bytes;
    uint32_t region_lower_bound;
    uint32_t observed_sampled_free_low;
    uint32_t observed_sampled_largest_low;
} gauge_memory_cap_t;

typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    gauge_memory_cap_t cap[3]; /* internal8bit, internalDMA, PSRAM8bit */
    uint32_t attempted_failures; /* modulo 2^32; not a critical-error count */
    uint32_t last_request_bytes;
    uint32_t last_request_caps;
    uint32_t task_stack_free_low_bytes;
    esp_err_t hook_registration;
} gauge_memory_snapshot_t;

/* Call once early in app_main, never from an allocation callback. */
esp_err_t gauge_memory_diag_init(void);
/* NULL task measures the current task. No allocation/logging; caller must be
 * outside LVGL/cache/GATT callbacks. Sequential cap queries are not one atomic
 * all-heaps instant. A non-NULL task must remain alive during this call. */
esp_err_t gauge_memory_diag_get(gauge_memory_snapshot_t *out, TaskHandle_t task);
/* Owner-task-only historical wall-time maxima. UINT32_MAX is unsampled;
 * finite durations saturate at UINT32_MAX - 1 and a measured zero is valid.
 * work/diagnostic maxima cover completed prior iterations/groups; start gaps
 * include the requested delay and scheduling. These are not CPU time or LCD
 * frame latency. The logger borrows this pointer synchronously, never stores it. */
typedef struct {
    uint32_t lock_wait_max_us;
    uint32_t work_max_us;
    uint32_t diagnostic_max_us;
    uint32_t start_gap_max_us;
} gauge_render_timing_t;

/* Explicit owner kind; reuses the current-task snapshot HWM once.
 * Existing startup / ten-second consumer only; no new timer or task. */
void gauge_memory_diag_log(gauge_memory_phase_t phase, gauge_stack_kind_t kind,
                           uint32_t handler_max_us,
                           const gauge_render_timing_t *render_timing);
