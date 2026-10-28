#include "theme_gauge_palette.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static const cJSON *unique(const cJSON *obj, const char *key, bool *ok)
{
    const cJSON *found = NULL;
    for (const cJSON *item = obj->child; item; item = item->next) {
        if (item->string && strcmp(item->string, key) == 0) {
            if (found) {
                *ok = false;
                return NULL;
            }
            found = item;
        }
    }
    return found;
}

static bool rgb(const cJSON *value, uint32_t *out)
{
    if (!cJSON_IsString(value) || !value->valuestring ||
        strlen(value->valuestring) != 7 || value->valuestring[0] != '#') {
        return false;
    }
    uint32_t color = 0;
    for (int i = 1; i < 7; ++i) {
        unsigned char c = (unsigned char)value->valuestring[i];
        unsigned digit;
        if (c >= '0' && c <= '9') digit = c - '0';
        else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
        else return false;
        color = (color << 4) | digit;
    }
    *out = color;
    return true;
}

static double linear(unsigned component)
{
    double c = component / 255.0;
    return c <= 0.04045 ? c / 12.92 : pow((c + 0.055) / 1.055, 2.4);
}

static double luminance(uint32_t color)
{
    return 0.2126 * linear((color >> 16) & 255) +
           0.7152 * linear((color >> 8) & 255) +
           0.0722 * linear(color & 255);
}

static double contrast(uint32_t first, uint32_t second)
{
    double a = luminance(first), b = luminance(second);
    return a > b ? (a + 0.05) / (b + 0.05) : (b + 0.05) / (a + 0.05);
}

bool theme_gauge_palette_parse(const cJSON *manifest, theme_gauge_palette_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    if (!cJSON_IsObject(manifest)) return false;
    bool ok = true;
    const cJSON *extension = unique(manifest, "gauge_ui", &ok);
    if (!ok) return false;
    if (!extension) return true;
    if (!cJSON_IsObject(extension)) return false;
    const cJSON *abi = unique(extension, "abi", &ok);
    const cJSON *palette = unique(extension, "palette", &ok);
    if (!ok || !cJSON_IsString(abi) || !abi->valuestring ||
        strcmp(abi->valuestring, THEME_GAUGE_ABI) || !cJSON_IsObject(palette)) {
        return false;
    }
    static const char *const keys[THEME_GAUGE_PALETTE_COUNT] = {
        "accent", "secondary", "multi_background", "analog_face",
        "analog_ink", "analog_muted", "needle", "trend"
    };
    uint32_t colors[THEME_GAUGE_PALETTE_COUNT];
    for (unsigned i = 0; i < THEME_GAUGE_PALETTE_COUNT; ++i) {
        const cJSON *value = unique(palette, keys[i], &ok);
        if (!ok || !rgb(value, &colors[i])) return false;
    }
    // Gauge text is intentionally protected by the core UI. Reject a palette
    // that makes its fixed white labels or analog text unreadable.
    if (contrast(0xFFFFFF, colors[2]) < 4.5 ||
        contrast(colors[4], colors[3]) < 4.5 ||
        contrast(colors[5], colors[3]) < 3.0) return false;
    memcpy(out->colors, colors, sizeof(colors));
    out->valid = true;
    return true;
}
