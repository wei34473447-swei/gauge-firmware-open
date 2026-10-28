#include "device_identity.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

#include "bsp_obd_dsp/lcd_driver/ST77916.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "theme_engine/theme_interface.h"

static const char *TAG = "device_identity";

#ifndef OBD_GAUGE_GIT_BRANCH
#define OBD_GAUGE_GIT_BRANCH "unknown"
#endif

#ifndef OBD_GAUGE_GIT_COUNT
#define OBD_GAUGE_GIT_COUNT 0
#endif

#ifndef OBD_GAUGE_BUILD_TAG
#define OBD_GAUGE_BUILD_TAG "unknown-0-unknown"
#endif

#define OBD_GAUGE_BOARD_NAME        "Waveshare ESP32-S3-Touch-LCD-1.85"
#define OBD_GAUGE_BOARD_VARIANT     "obd_brz_gauge"
#define OBD_GAUGE_LCD_NAME          "ST77916"
#define OBD_GAUGE_FLASH_MB          16u
#define OBD_GAUGE_PSRAM_MB          8u
#define OBD_GAUGE_OTA_SLOTS         2u
#define OBD_GAUGE_BOOTMEDIA_SLOTS   1u
#define OBD_GAUGE_BOOTMEDIA_FORMAT   1u

static const device_identity_t s_identity = {
    .board_name = OBD_GAUGE_BOARD_NAME,
    .board_variant = OBD_GAUGE_BOARD_VARIANT,
    .lcd_name = OBD_GAUGE_LCD_NAME,
    .screen_width = EXAMPLE_LCD_WIDTH,
    .screen_height = EXAMPLE_LCD_HEIGHT,
    .color_bits = EXAMPLE_LCD_COLOR_BITS,
    .flash_mb = OBD_GAUGE_FLASH_MB,
    .psram_mb = OBD_GAUGE_PSRAM_MB,
    .ota_slots = OBD_GAUGE_OTA_SLOTS,
    .bootmedia_slots = OBD_GAUGE_BOOTMEDIA_SLOTS,
    .bootmedia_format_version = OBD_GAUGE_BOOTMEDIA_FORMAT,
};

static char s_manifest_json[DEVICE_IDENTITY_MANIFEST_MAX_SIZE];
static char s_http_manifest_json[DEVICE_IDENTITY_HTTP_MANIFEST_MAX_SIZE];
static char s_image_sha256[65];
/* 0=not started, 1=in progress, 2=verified, 3=failed. A concurrent initial read
 * fails closed rather than spinning while flash is being hashed. No new task. */
static atomic_int s_image_hash_state = ATOMIC_VAR_INIT(0);

typedef struct {
    char *buf;
    size_t cap;
    size_t used;
    bool ok;
} identity_json_t;

static void json_append(identity_json_t *json, const char *fmt, ...)
{
    if (!json->ok) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(json->buf + json->used, json->cap - json->used, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= json->cap - json->used) {
        json->ok = false;
        return;
    }
    json->used += (size_t)n;
}

/* Fixed-size metadata may be non-ASCII. Bound bytes, preserve UTF-8, escape
 * JSON punctuation/control characters, and reject a missing terminating NUL. */
static void json_string(identity_json_t *json, const char *str, size_t bound)
{
    if (!json->ok) return;
    if (!str || strnlen(str, bound) == bound) {
        json->ok = false;
        return;
    }
    json_append(json, "\"");
    for (const unsigned char *p = (const unsigned char *)str; *p && json->ok; ++p) {
        if (*p == '"' || *p == '\\') json_append(json, "\\%c", *p);
        else if (*p < 0x20) json_append(json, "\\u%04x", (unsigned)*p);
        else json_append(json, "%c", *p);
    }
    json_append(json, "\"");
}

static bool image_hash_ready(const esp_partition_t *running)
{
    int state = atomic_load_explicit(&s_image_hash_state, memory_order_acquire);
    if (state == 2) return true;
    if (state != 0 || !running) return false;
    int expected = 0;
    if (!atomic_compare_exchange_strong_explicit(&s_image_hash_state, &expected, 1,
                                                 memory_order_acq_rel, memory_order_acquire)) return false;
    uint8_t digest[32];
    esp_err_t err = esp_partition_get_sha256(running, digest);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Running image identity unavailable: %s", esp_err_to_name(err));
        atomic_store_explicit(&s_image_hash_state, 3, memory_order_release);
        return false;
    }
    static const char hex[] = "0123456789abcdef";
    for (size_t i = 0; i < sizeof(digest); ++i) {
        s_image_sha256[2 * i] = hex[digest[i] >> 4];
        s_image_sha256[2 * i + 1] = hex[digest[i] & 0xf];
    }
    s_image_sha256[64] = '\0';
    atomic_store_explicit(&s_image_hash_state, 2, memory_order_release);
    return true;
}

static const char *running_ota_state(const esp_partition_t *running)
{
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) return "unknown";
    switch (state) {
        case ESP_OTA_IMG_NEW: return "new";
        case ESP_OTA_IMG_PENDING_VERIFY: return "pending_verify";
        case ESP_OTA_IMG_VALID: return "valid";
        case ESP_OTA_IMG_INVALID: return "invalid";
        case ESP_OTA_IMG_ABORTED: return "aborted";
        case ESP_OTA_IMG_UNDEFINED: return "undefined";
        default: return "unknown";
    }
}

const device_identity_t *device_identity_get(void)
{
    return &s_identity;
}

static const char *make_manifest(char *buf, size_t cap, bool extended)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_app_desc_t *app = esp_app_get_description();
    if (!running || !app || !image_hash_ready(running)) return "{\"error\":\"identity_unavailable\"}";
    identity_json_t json = {.buf = buf, .cap = cap, .ok = true};
    json_append(&json,
             "{"
             "\"device\":{"
             "\"board\":\"%s\","
             "\"variant\":\"%s\","
             "\"lcd\":\"%s\","
             "\"screen\":{\"w\":%u,\"h\":%u,\"bpp\":%u},"
             "\"flash_mb\":%u,"
             "\"psram_mb\":%u,"
             "\"ota_slots\":%u,"
             "\"bootmedia_slots\":%u,"
             "\"bootmedia_format\":%u"
             "},"
             "\"firmware\":{"
             "\"build_tag\":",
             s_identity.board_name,
             s_identity.board_variant,
             s_identity.lcd_name,
             s_identity.screen_width,
             s_identity.screen_height,
             s_identity.color_bits,
             s_identity.flash_mb,
             s_identity.psram_mb,
             s_identity.ota_slots,
             s_identity.bootmedia_slots,
             s_identity.bootmedia_format_version);
    json_string(&json, OBD_GAUGE_BUILD_TAG, sizeof(OBD_GAUGE_BUILD_TAG));
    json_append(&json, ",\"branch\":");
    json_string(&json, OBD_GAUGE_GIT_BRANCH, sizeof(OBD_GAUGE_GIT_BRANCH));
    json_append(&json, ",\"count\":%u,\"version\":", (unsigned)OBD_GAUGE_GIT_COUNT);
    json_string(&json, app->version, sizeof(app->version));
    json_append(&json, ",\"image_sha256\":\"%s\",\"running_slot\":", s_image_sha256);
    json_string(&json, running->label, sizeof(running->label));
    json_append(&json, ",\"ota_state\":\"%s\"}", running_ota_state(running));
    if (extended) {
        theme_info_t info = {0};
        bool loaded = theme_get_info(&info) == ESP_OK;
        const char *active_abi = loaded ? theme_get_gauge_abi() : NULL;
        json_append(&json, ",\"capabilities\":{\"theme_schema\":\"1.0\",\"gauge_ui_abi\":\"ino-gauge-v5/1\"},\"theme\":{\"id\":");
        json_string(&json, loaded ? info.id : "default", loaded ? sizeof(info.id) : sizeof("default"));
        json_append(&json, ",\"version\":");
        json_string(&json, loaded ? info.version : "builtin", loaded ? sizeof(info.version) : sizeof("builtin"));
        json_append(&json, ",\"gauge_ui_abi\":");
        if (active_abi) json_string(&json, active_abi, 32);
        else json_append(&json, "null");
        json_append(&json, "}");
    }
    json_append(&json, "}");
    if (!json.ok) {
        ESP_LOGE(TAG, "Identity metadata invalid or response exceeds %u bytes", (unsigned)cap);
        return "{\"error\":\"identity_metadata_invalid_or_too_large\"}";
    }
    return buf;
}

const char *device_identity_manifest_json(void)
{
    return make_manifest(s_manifest_json, sizeof(s_manifest_json), false);
}

const char *device_identity_manifest_http_json(void)
{
    return make_manifest(s_http_manifest_json, sizeof(s_http_manifest_json), true);
}
