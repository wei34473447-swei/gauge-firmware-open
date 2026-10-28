#pragma once

#include <stdint.h>

/* GATT values are limited to 512 bytes. JSON payload excludes its trailing NUL. */
#define DEVICE_IDENTITY_MANIFEST_MAX_SIZE 512u
#define DEVICE_IDENTITY_HTTP_MANIFEST_MAX_SIZE 768u

typedef struct {
    const char *board_name;
    const char *board_variant;
    const char *lcd_name;
    uint16_t screen_width;
    uint16_t screen_height;
    uint8_t color_bits;
    uint8_t flash_mb;
    uint8_t psram_mb;
    uint8_t ota_slots;
    uint8_t bootmedia_slots;
    uint8_t bootmedia_format_version;
} device_identity_t;

const device_identity_t *device_identity_get(void);
/* Compact INFO: immutable image hash is verified once; OTA state is read afresh.
 * image_sha256 is esp_partition_get_sha256(APP), NOT SHA256 of the whole download.
 * Copy each returned JSON immediately; each transport serializes its own reads.
 * Failure returns a complete {"error":...} object, never a truncated identity.
 */
const char *device_identity_manifest_json(void);
/* HTTP adds supported theme capabilities and currently loaded theme metadata. */
const char *device_identity_manifest_http_json(void);
