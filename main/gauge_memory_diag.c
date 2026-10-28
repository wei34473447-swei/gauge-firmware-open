#include "gauge_memory_diag.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "mem";
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_attempted_failures;
static uint32_t s_last_request_bytes;
static uint32_t s_last_request_caps;
static uint32_t s_sequence;
static uint32_t s_free_low[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
static uint32_t s_largest_low[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
static esp_err_t s_hook_registration = ESP_ERR_INVALID_STATE;
static uint32_t s_logged_attempts;

/* Internal static storage is nominally 36 bytes; initialized minima may
 * live in .data. Final map/ELF cost has not been measured by this source gate. */
static uint32_t s_stack_min[GAUGE_STACK_KIND_COUNT] = {
    UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX,
    UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX
};
_Static_assert(sizeof(s_stack_min) == 36, "nine 32-bit kind minima required");

static void stack_diag_record_measured(gauge_stack_kind_t kind, uint32_t bytes)
{
    if ((unsigned)kind >= GAUGE_STACK_KIND_COUNT) return;
    portENTER_CRITICAL_SAFE(&s_stats_mux);
    if (bytes < s_stack_min[kind]) s_stack_min[kind] = bytes;
    portEXIT_CRITICAL_SAFE(&s_stats_mux);
}

void gauge_stack_diag_record_self(gauge_stack_kind_t kind)
{
    if ((unsigned)kind >= GAUGE_STACK_KIND_COUNT) return;
    /* This API never dereferences another task's possibly deleted handle. */
    const uint32_t bytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    stack_diag_record_measured(kind, bytes);
}

void gauge_stack_diag_get(uint32_t out[GAUGE_STACK_KIND_COUNT])
{
    if (!out) return;
    portENTER_CRITICAL_SAFE(&s_stats_mux);
    for (unsigned i = 0; i < GAUGE_STACK_KIND_COUNT; ++i) out[i] = s_stack_min[i];
    portEXIT_CRITICAL_SAFE(&s_stats_mux);
}

void gauge_httpd_stack_record_self(void)
{
    gauge_stack_diag_record_self(GAUGE_STACK_HTTPD);
}


static void IRAM_ATTR allocation_attempt_failed(size_t bytes, uint32_t caps,
                                               const char *function_name)
{
    (void)function_name;
    /* No heap scan/allocation/formatting, no retry or feature policy here.
     * The critical section contains four bounded 32-bit writes only. */
    portENTER_CRITICAL_SAFE(&s_stats_mux);
    s_last_request_bytes = bytes > UINT32_MAX ? UINT32_MAX : (uint32_t)bytes;
    s_last_request_caps = caps;
    ++s_attempted_failures;
    portEXIT_CRITICAL_SAFE(&s_stats_mux);
}

esp_err_t gauge_memory_diag_init(void)
{
    /* Registration itself does not allocate in the pinned ESP-IDF heap API. */
    s_hook_registration = heap_caps_register_failed_alloc_callback(allocation_attempt_failed);
    return s_hook_registration;
}

esp_err_t gauge_memory_diag_get(gauge_memory_snapshot_t *out, TaskHandle_t task)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    const uint32_t caps[3] = {
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
    };
    gauge_memory_snapshot_t snapshot = {0};
    const int64_t now = esp_timer_get_time();
    snapshot.timestamp_us = now > 0 ? (uint64_t)now : 0;
    for (unsigned i = 0; i < 3; ++i) {
        multi_heap_info_t info;
        heap_caps_get_info(&info, caps[i]);
        snapshot.cap[i].free_bytes = (uint32_t)info.total_free_bytes;
        snapshot.cap[i].largest_bytes = (uint32_t)info.largest_free_block;
        snapshot.cap[i].region_lower_bound = (uint32_t)info.minimum_free_bytes;
    }
    /* ESP-IDF's stack high-water mark is bytes, not upstream word units. */
    snapshot.task_stack_free_low_bytes = (uint32_t)uxTaskGetStackHighWaterMark(task);
    portENTER_CRITICAL_SAFE(&s_stats_mux);
    snapshot.sequence = ++s_sequence;
    for (unsigned i = 0; i < 3; ++i) {
        if (snapshot.cap[i].free_bytes < s_free_low[i]) s_free_low[i] = snapshot.cap[i].free_bytes;
        if (snapshot.cap[i].largest_bytes < s_largest_low[i]) s_largest_low[i] = snapshot.cap[i].largest_bytes;
        snapshot.cap[i].observed_sampled_free_low = s_free_low[i];
        snapshot.cap[i].observed_sampled_largest_low = s_largest_low[i];
    }
    snapshot.attempted_failures = s_attempted_failures;
    snapshot.last_request_bytes = s_last_request_bytes;
    snapshot.last_request_caps = s_last_request_caps;
    snapshot.hook_registration = s_hook_registration;
    portEXIT_CRITICAL_SAFE(&s_stats_mux);
    *out = snapshot;
    return ESP_OK;
}

void gauge_memory_diag_log(gauge_memory_phase_t phase, gauge_stack_kind_t kind,
                           uint32_t handler_max_us,
                           const gauge_render_timing_t *render_timing)
{
    gauge_memory_snapshot_t snapshot;
    if (gauge_memory_diag_get(&snapshot, NULL) != ESP_OK) return;
    stack_diag_record_measured(kind, snapshot.task_stack_free_low_bytes);
    unsigned safe_phase = (unsigned)phase <= GAUGE_MEMORY_RUNTIME ? (unsigned)phase : GAUGE_MEMORY_RUNTIME;
    if (render_timing) {
        /* Same existing row/frequency. Current group's own console cost is
         * observed after it completes, then appears in the next report. */
        ESP_LOGI(TAG, "t_us=%" PRIu64 " phase=%u seq=%u stackB=%u Tms=%u lockMaxUs=%u workPrevMaxUs=%u diagPrevMaxUs=%u startGapMaxUs=%u",
                 snapshot.timestamp_us, safe_phase, (unsigned)snapshot.sequence,
                 (unsigned)snapshot.task_stack_free_low_bytes, (unsigned)(handler_max_us / 1000u),
                 (unsigned)render_timing->lock_wait_max_us,
                 (unsigned)render_timing->work_max_us,
                 (unsigned)render_timing->diagnostic_max_us,
                 (unsigned)render_timing->start_gap_max_us);
    } else {
        ESP_LOGI(TAG, "t_us=%" PRIu64 " phase=%u seq=%u stackB=%u Tms=%u",
                 snapshot.timestamp_us, safe_phase, (unsigned)snapshot.sequence,
                 (unsigned)snapshot.task_stack_free_low_bytes, (unsigned)(handler_max_us / 1000u));
    }
    /* f/max in bytes; regionLB is independent region historical lower bound;
     * sampled is observed free/largest lows, never a true all-time peak. */
    for (unsigned i = 0; i < 3; ++i) {
        ESP_LOGI(TAG, "cap=%u f=%u max=%u regionLB=%u sampled=%u/%u",
                 i, (unsigned)snapshot.cap[i].free_bytes, (unsigned)snapshot.cap[i].largest_bytes,
                 (unsigned)snapshot.cap[i].region_lower_bound,
                 (unsigned)snapshot.cap[i].observed_sampled_free_low,
                 (unsigned)snapshot.cap[i].observed_sampled_largest_low);
    }
    bool report_attempts;
    portENTER_CRITICAL_SAFE(&s_stats_mux);
    report_attempts = snapshot.attempted_failures != s_logged_attempts;
    if (report_attempts) s_logged_attempts = snapshot.attempted_failures;
    portEXIT_CRITICAL_SAFE(&s_stats_mux);
    if (report_attempts || snapshot.hook_registration != ESP_OK) {
        ESP_LOGI(TAG, "attempt_fail=%u reqB=%u caps=%x hook=%d",
                 (unsigned)snapshot.attempted_failures, (unsigned)snapshot.last_request_bytes,
                 (unsigned)snapshot.last_request_caps, (int)snapshot.hook_registration);
    }
    if (phase == GAUGE_MEMORY_RUNTIME) {
        uint32_t low[GAUGE_STACK_KIND_COUNT];
        gauge_stack_diag_get(low);
        /* One bounded numeric line; historical kind minima, never active tasks. */
        ESP_LOGI(TAG, "kindMinB m=%u lv=%u imu=%u obd=%u http=%u ot=%u bm=%u val=%u rst=%u unset=4294967295",
                 (unsigned)low[GAUGE_STACK_MAIN], (unsigned)low[GAUGE_STACK_LVGL],
                 (unsigned)low[GAUGE_STACK_IMU], (unsigned)low[GAUGE_STACK_OBD],
                 (unsigned)low[GAUGE_STACK_HTTPD], (unsigned)low[GAUGE_STACK_OTA_TIMEOUT],
                 (unsigned)low[GAUGE_STACK_BM_WRITER], (unsigned)low[GAUGE_STACK_OTA_VALID],
                 (unsigned)low[GAUGE_STACK_OTA_RESTART]);
    }
}
