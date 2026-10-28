#include "gauge_v5_boot.h"
#include "nvs.h"

static bool loaded;
static uint8_t current_page = GV5_BOOT_DEFAULT;
static bool start6_retry;

uint8_t gv5_boot_get(void)
{
    if (loaded)
        return current_page;
    loaded = true;
    nvs_handle_t handle;
    if (nvs_open("gauge_v5", NVS_READONLY, &handle) != ESP_OK)
        return current_page;
    uint8_t page;
    if (nvs_get_u8(handle, "boot", &page) == ESP_OK && page < GV5_BOOT_PAGE_COUNT)
        current_page = page;
    nvs_close(handle);
    return current_page;
}

bool gv5_boot_set(uint8_t page)
{
    if (page >= GV5_BOOT_PAGE_COUNT)
        return false;
    (void)gv5_boot_get();
    nvs_handle_t handle;
    esp_err_t error = nvs_open("gauge_v5", NVS_READWRITE, &handle);
    if (error != ESP_OK)
        return false;
    error = nvs_set_u8(handle, "boot", page);
    if (error == ESP_OK)
        error = nvs_commit(handle);
    nvs_close(handle);
    if (error != ESP_OK)
        return false;
    current_page = page;
    return true;
}

uint8_t gv5_boot_prepare_preview6_start(void)
{
    uint8_t selected = gv5_boot_get();
    uint8_t native = selected >= GV5_BOOT_DEFAULT ? selected : GV5_BOOT_DEFAULT;
    nvs_handle_t handle;
    if (nvs_open("gauge_v5", NVS_READWRITE, &handle) != ESP_OK)
        return native;

    uint8_t marker = 0;
    esp_err_t error = nvs_get_u8(handle, "start6", &marker);
    if (error == ESP_OK && marker == 1 && !start6_retry) {
        nvs_close(handle);
        return selected;
    }
    if (error != ESP_OK && error != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return native;
    }

    /* Store the native choice before its marker. NVS has no multi-key
     * transaction; interruption before the marker leaves migration retryable. */
    if (selected < GV5_BOOT_DEFAULT)
        error = nvs_set_u8(handle, "boot", native);
    else
        error = ESP_OK;
    if (error == ESP_OK)
        error = nvs_set_u8(handle, "start6", 1);
    if (error == ESP_OK)
        error = nvs_commit(handle);
    nvs_close(handle);
    if (error == ESP_OK) {
        current_page = native;
        start6_retry = false;
    } else {
        /* Some backends expose set_u8 immediately, even before commit.
         * Do not accept such a marker as success on an in-process retry. */
        start6_retry = true;
    }
    return native;
}
