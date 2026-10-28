// BLE Scan & Select Page
// Shows saved device (with delete) + a list of discovered BLE devices
//
// Two use cases depending on device_role:
//  - MASTER/STANDALONE: scan and connect to an OBD ELM327 adapter (original logic unchanged)
//  - SLAVE: scan and pair with the triple-gauge master ("SkyGauge-XXYY" broadcast), see gauge_pair_ble_client.c

#include "../ui.h"
#include "../ui_gauge_v5.h"
#include "../ui_v5_about.h"
#include "../ui_phase1.h"
#include "../gauge_v5_fonts.h"
#include "bsp_obd_dsp/elm327_ble_client.h"
#include "bsp_obd_dsp/gauge_pair_ble_client.h"
#include "bsp_obd_dsp/espnow_link.h"
#include "bsp_obd_dsp/nvs_storage.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG_BLE_UI = "ble_scan_ui";

// UI elements (local)
static lv_obj_t *s_list = NULL;             // scanned device list
static lv_obj_t *s_label_status = NULL;     // status label
static lv_obj_t *s_spinner = NULL;          // scan spinner
static lv_obj_t *s_saved_panel = NULL;      // saved device panel
static lv_obj_t *s_label_saved_hdr = NULL;  // "SAVED" sub-header
static lv_obj_t *s_saved_name_lbl = NULL;   // saved device name label
static gv5_lang_t s_language;
static bool s_scanning = false;
static bool s_slave_mode = false;           // true=slave pairing with a master, false=OBD device scan (original logic)

// Slave mode: parallel table mapping scan list button index -> corresponding device MAC (one-to-one with the s_list child order)
static uint8_t s_gauge_macs[GAUGE_PAIR_SCAN_MAX_DEVICES][6];
static char s_gauge_names[GAUGE_PAIR_SCAN_MAX_DEVICES][32];
static int s_gauge_mac_count = 0;

// OBD device scan: same as above, records the MAC of each list item; after selection connects by exact MAC (avoids misconnecting to a same-name device)
static uint8_t s_obd_macs[BLE_SCAN_MAX_DEVICES][6];
static char s_obd_names[BLE_SCAN_MAX_DEVICES][32];
static int s_obd_mac_count = 0;

static const char *ble_tr(const char *zh,const char *en,const char *ja, const char *tw) {
    return s_language==GV5_EN?en:s_language==GV5_JA?ja:s_language==GV5_ZH_TW?tw:zh;
}
static void style_device(lv_obj_t *btn) {
    lv_obj_set_style_bg_color(btn,lv_color_hex(0x211d2a),0);
    lv_obj_set_style_bg_opa(btn,255,0);
    lv_obj_set_style_text_color(btn,lv_color_hex(0xf5f0f4),0);
    /* Keep the original clear device-name typeface; size 24 replaces size 20. */
    lv_obj_set_style_text_font(btn,&ui_font_FontTypoderSize24,0);
    lv_obj_set_style_pad_ver(btn,12,0);lv_obj_set_style_radius(btn,10,0);
    lv_obj_set_style_border_width(btn,0,0);lv_obj_set_height(btn,LV_SIZE_CONTENT);
    lv_obj_set_style_min_height(btn,48,0);
    lv_obj_t *l=lv_obj_get_child(btn,0);if(l)lv_label_set_long_mode(l,LV_LABEL_LONG_DOT);
}
// Forward declarations
static void start_scan(void);
static void on_device_selected(lv_event_t *e);
static void on_saved_device_delete(lv_event_t *e);
static void on_pair_result(bool ok, const char *name, const uint8_t mac[6]);

// Mutex for LVGL (defined in main)
extern SemaphoreHandle_t lvgl_mux;
static inline bool lvgl_lock_ui(int timeout_ms) {
    return xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
static inline void lvgl_unlock_ui(void) {
    xSemaphoreGive(lvgl_mux);
}

// BLE scan callback (called in the BT thread, LVGL must be updated thread-safely) -- OBD device scan (MASTER/STANDALONE)
static void scan_result_cb(const ble_scan_result_t *dev, int total_count) {
    if (!s_list) return;

    if (lvgl_lock_ui(100)) {
        if(!s_list || !lv_obj_is_valid(s_list)){lvgl_unlock_ui();return;}
        for(int i=0;i<s_obd_mac_count;i++)if(memcmp(s_obd_macs[i],dev->addr,6)==0){lvgl_unlock_ui();return;}
        if (s_obd_mac_count >= BLE_SCAN_MAX_DEVICES) {
            lvgl_unlock_ui();
            return;
        }

        // Add new device button
        lv_obj_t *btn = lv_list_add_btn(s_list, NULL, dev->name);
        style_device(btn);
        lv_obj_add_event_cb(btn, on_device_selected, LV_EVENT_CLICKED, NULL);

        memcpy(s_obd_macs[s_obd_mac_count], dev->addr, 6);
        snprintf(s_obd_names[s_obd_mac_count],sizeof s_obd_names[0],"%s",dev->name);
        s_obd_mac_count++;

        lv_label_set_text_fmt(s_label_status, ble_tr("发现 %d 个设备", "FOUND %d DEVICES", "%d 台を発見", "發現 %d 個設備"), total_count);
        lvgl_unlock_ui();
    }
}

// BLE scan callback -- slave pairing with a master (SLAVE); only devices with the "SkyGauge" prefix are received (see the filter in gauge_pair_ble_client.c)
static void scan_result_cb_gauge(const gauge_pair_scan_result_t *dev, int total_count) {
    if (!s_list) return;

    if (lvgl_lock_ui(100)) {
        if(!s_list || !lv_obj_is_valid(s_list)){lvgl_unlock_ui();return;}
        for(int i=0;i<s_gauge_mac_count;i++)if(memcmp(s_gauge_macs[i],dev->addr,6)==0){lvgl_unlock_ui();return;}
        if (s_gauge_mac_count >= GAUGE_PAIR_SCAN_MAX_DEVICES) {
            lvgl_unlock_ui();
            return;
        }

        lv_obj_t *btn = lv_list_add_btn(s_list, NULL, dev->name);
        style_device(btn);
        lv_obj_add_event_cb(btn, on_device_selected, LV_EVENT_CLICKED, NULL);

        memcpy(s_gauge_macs[s_gauge_mac_count], dev->addr, 6);
        snprintf(s_gauge_names[s_gauge_mac_count],sizeof s_gauge_names[0],"%s",dev->name);
        s_gauge_mac_count++;

        lv_label_set_text_fmt(s_label_status, ble_tr("发现 %d 个设备", "FOUND %d DEVICES", "%d 台を発見", "發現 %d 個設備"), total_count);
        lvgl_unlock_ui();
    }
}

// A device was tapped/selected
static void on_device_selected(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *lbl = lv_obj_get_child(btn, 0);
    if (!lbl) return;

    uint32_t idx = lv_obj_get_index(btn);
    /* DOT labels mutate their display string; use the original scan record. */
    const char *name = s_slave_mode ? (idx<(uint32_t)s_gauge_mac_count?s_gauge_names[idx]:NULL) :
                       (idx<(uint32_t)s_obd_mac_count?s_obd_names[idx]:NULL);
    if(!name)return;

    if (s_slave_mode) {
        if (idx >= (uint32_t)s_gauge_mac_count) return;
        uint8_t mac[6];
        memcpy(mac, s_gauge_macs[idx], 6);

        ESP_LOGI(TAG_BLE_UI, "Selected master: %s", name);
        gauge_pair_ble_scan_stop();
        s_scanning = false;

        lv_label_set_text(s_label_status, ble_tr("配对中…", "PAIRING…", "ペアリング中…", "配對中…"));
        if (s_spinner) lv_obj_clear_flag(s_spinner, LV_OBJ_FLAG_HIDDEN);
        gauge_pair_ble_connect(mac, name, on_pair_result);
        return;
    }

    ESP_LOGI(TAG_BLE_UI, "Selected BLE device: %s", name);

    if (idx >= (uint32_t)s_obd_mac_count) return;
    uint8_t mac[6];
    memcpy(mac, s_obd_macs[idx], 6);

    elm327_ble_scan_only_stop();
    s_scanning = false;

    nvs_user_cfg_t cfg = *nvs_cfg_get();
    strncpy(cfg.ble_device_name, name, sizeof(cfg.ble_device_name) - 1);
    cfg.ble_device_name[sizeof(cfg.ble_device_name) - 1] = '\0';
    memcpy(cfg.ble_obd_mac, mac, 6);
    if (nvs_cfg_set(&cfg) != ESP_OK) {
        if (s_label_status) lv_label_set_text(s_label_status, ble_tr("保存失败", "SAVE FAILED", "保存失敗", "保存失敗"));
        if (s_spinner) lv_obj_add_flag(s_spinner, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    // Refresh the saved device panel immediately
    if (s_saved_name_lbl) lv_label_set_text(s_saved_name_lbl, name);
    if (s_saved_panel)    lv_obj_clear_flag(s_saved_panel,    LV_OBJ_FLAG_HIDDEN);
    if (s_label_saved_hdr) lv_obj_clear_flag(s_label_saved_hdr, LV_OBJ_FLAG_HIDDEN);

    lv_label_set_text_fmt(s_label_status, ble_tr("连接 %s", "CONNECTING %s", "接続中 %s", "連接 %s"), name);
    if (s_spinner) lv_obj_clear_flag(s_spinner, LV_OBJ_FLAG_HIDDEN);

    elm327_ble_connect_by_addr(mac, name);
    ui_gauge_v5_open(0);
}

// BLE pairing result callback (called in the BT task context, lvgl_lock required)
static void on_pair_result(bool ok, const char *name, const uint8_t mac[6]) {
    if (!lvgl_lock_ui(200)) return;

    if (ok) {
        espnow_link_bind_master(mac);

        nvs_user_cfg_t cfg = *nvs_cfg_get();
        strncpy(cfg.ble_device_name, name ? name : "", sizeof(cfg.ble_device_name) - 1);
        cfg.ble_device_name[sizeof(cfg.ble_device_name) - 1] = '\0';
        nvs_cfg_set(&cfg);
        ESP_LOGI(TAG_BLE_UI, "Paired with master: %s", cfg.ble_device_name);

        if (s_saved_name_lbl) lv_label_set_text(s_saved_name_lbl, cfg.ble_device_name);
        if (s_saved_panel)    lv_obj_clear_flag(s_saved_panel,    LV_OBJ_FLAG_HIDDEN);
        if (s_label_saved_hdr) lv_obj_clear_flag(s_label_saved_hdr, LV_OBJ_FLAG_HIDDEN);

        lv_label_set_text(s_label_status, ble_tr("已配对", "PAIRED", "ペアリング完了", "已配對"));
        ui_gauge_v5_open(0);
    } else {
        ESP_LOGW(TAG_BLE_UI, "Pairing failed, rescanning");
        lv_label_set_text(s_label_status, ble_tr("重试配对…", "RETRYING…", "再試行中…", "重試配對…"));
        // When the native BLE scan window (15s) expires, no callback notifies here, so s_scanning stays true.
        // This is a place where a forced rescan is intended, so reset it before calling to avoid being blocked by the dedup check in start_scan().
        s_scanning = false;
        start_scan();
    }

    lvgl_unlock_ui();
}

// Delete the saved device
static void on_saved_device_delete(lv_event_t *e) {
    if (s_slave_mode) {
        espnow_link_unbind_master();
        nvs_user_cfg_t cfg = *nvs_cfg_get();
        cfg.ble_device_name[0] = '\0';
        nvs_cfg_set(&cfg);
        ESP_LOGI(TAG_BLE_UI, "Unbound saved master");
    } else {
        // If currently connected, disconnect BLE first
        if (elm327_ble_is_connected()) {
            elm327_ble_disconnect();
        }
        nvs_user_cfg_t cfg = *nvs_cfg_get();
        cfg.ble_device_name[0] = '\0';
        memset(cfg.ble_obd_mac, 0, sizeof(cfg.ble_obd_mac));
        nvs_cfg_set(&cfg);
        ESP_LOGI(TAG_BLE_UI, "Saved BLE device cleared");
    }

    if (s_saved_panel)    lv_obj_add_flag(s_saved_panel,    LV_OBJ_FLAG_HIDDEN);
    if (s_label_saved_hdr) lv_obj_add_flag(s_label_saved_hdr, LV_OBJ_FLAG_HIDDEN);
    if (s_label_status)   lv_label_set_text(s_label_status, ble_tr("已移除设备", "DEVICE REMOVED", "機器を削除しました", "已移除設備"));

    if (s_slave_mode) {
        s_scanning = false;   // the native scan window expiry does not reset via callback; reset before forcing a rescan (same reason as on_pair_result)
        start_scan();         // Slave: rescan immediately after deleting the binding, so a new master can be paired
    }
}

static void start_scan(void) {
    if (s_scanning) return;
    s_scanning = true;

    if (s_list) lv_obj_clean(s_list);
    if (s_label_status) lv_label_set_text(s_label_status, ble_tr("搜索设备…", "SCANNING…", "スキャン中…", "搜索設備…"));
    if (s_spinner) lv_obj_clear_flag(s_spinner, LV_OBJ_FLAG_HIDDEN);

    if (s_slave_mode) {
        s_gauge_mac_count = 0;
        gauge_pair_ble_scan_start(15, scan_result_cb_gauge);
    } else {
        s_obd_mac_count = 0;
        elm327_ble_scan_only_start(15, scan_result_cb);
    }
}

static bool saved_device_present(void) {
    if(s_slave_mode){const uint8_t *m=espnow_link_get_bound_master_mac();return (m[0]|m[1]|m[2]|m[3]|m[4]|m[5])!=0;}
    return nvs_cfg_get()->ble_device_name[0]!='\0';
}
static lv_obj_t *connection_label(lv_obj_t *parent,const char *value,int size,int width,int y) {
    lv_obj_t *l=lv_label_create(parent);lv_label_set_text(l,value);
    lv_obj_set_style_text_font(l,gv5_text_font(s_language,size),0);
    lv_obj_set_style_text_color(l,lv_color_hex(0xf5f0f4),0);
    lv_obj_set_style_text_align(l,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(l,width);lv_obj_align(l,LV_ALIGN_TOP_MID,0,y);
    lv_obj_clear_flag(l,LV_OBJ_FLAG_CLICKABLE);return l;
}
static void connection_back(lv_event_t *e){(void)e;ui_v5_about_open();}
static void connection_render(void) {
    s_slave_mode=nvs_cfg_get()->device_role==ESPNOW_ROLE_SLAVE;
    s_language=ui_gauge_v5_language();s_scanning=false;
    s_list=NULL;s_label_status=NULL;s_spinner=NULL;s_saved_panel=NULL;s_label_saved_hdr=NULL;s_saved_name_lbl=NULL;
    lv_obj_clean(ui_ScreenPageBLEScan);
    lv_obj_t *x=ui_phase1_x_create(ui_ScreenPageBLEScan,42);lv_obj_update_layout(x);
    int h=lv_obj_get_height(x),w=lv_obj_get_width(x);
    lv_obj_align(x,LV_ALIGN_TOP_MID,0,18+(h*42/w-h)/2);
    connection_label(ui_ScreenPageBLEScan,s_slave_mode?ble_tr("连接主表", "LINK MASTER", "親メーター接続", "連接主表"):ble_tr("连接车辆", "VEHICLE LINK", "車両に接続", "連接車輛"),22,264,67);
    s_label_status=connection_label(ui_ScreenPageBLEScan,ble_tr("搜索设备…", "SCANNING…", "スキャン中…", "搜索設備…"),18,264,97);
    lv_obj_set_style_text_color(s_label_status,lv_color_hex(0xb7a6d8),0);
    bool has_saved=saved_device_present();
    s_label_saved_hdr=connection_label(ui_ScreenPageBLEScan,ble_tr("已保存设备", "SAVED DEVICE", "保存済み機器", "已保存設備"),18,264,126);
    if(!has_saved)lv_obj_add_flag(s_label_saved_hdr,LV_OBJ_FLAG_HIDDEN);
    s_saved_panel=lv_obj_create(ui_ScreenPageBLEScan);lv_obj_remove_style_all(s_saved_panel);
    lv_obj_set_size(s_saved_panel,258,48);lv_obj_align(s_saved_panel,LV_ALIGN_TOP_MID,0,151);
    lv_obj_set_style_bg_color(s_saved_panel,lv_color_hex(0x211d2a),0);lv_obj_set_style_bg_opa(s_saved_panel,255,0);
    lv_obj_set_style_radius(s_saved_panel,10,0);lv_obj_clear_flag(s_saved_panel,LV_OBJ_FLAG_SCROLLABLE);
    if(!has_saved)lv_obj_add_flag(s_saved_panel,LV_OBJ_FLAG_HIDDEN);
    s_saved_name_lbl=lv_label_create(s_saved_panel);lv_label_set_text(s_saved_name_lbl,has_saved?nvs_cfg_get()->ble_device_name:"");
    lv_obj_set_style_text_font(s_saved_name_lbl,&ui_font_FontTypoderSize24,0);
    lv_obj_set_style_text_color(s_saved_name_lbl,lv_color_hex(0xf5f0f4),0);lv_obj_set_width(s_saved_name_lbl,190);
    lv_label_set_long_mode(s_saved_name_lbl,LV_LABEL_LONG_DOT);lv_obj_align(s_saved_name_lbl,LV_ALIGN_LEFT_MID,12,0);
    lv_obj_t *del=lv_btn_create(s_saved_panel);lv_obj_set_size(del,44,40);lv_obj_align(del,LV_ALIGN_RIGHT_MID,-4,0);
    lv_obj_set_style_bg_color(del,lv_color_hex(0x402036),0);lv_obj_set_style_shadow_width(del,0,0);lv_obj_set_style_radius(del,8,0);
    lv_obj_add_event_cb(del,on_saved_device_delete,LV_EVENT_CLICKED,NULL);
    lv_obj_t *dl=connection_label(del,"X",22,30,0);lv_obj_center(dl);lv_obj_set_style_text_color(dl,lv_color_hex(0xff0188),0);
    connection_label(ui_ScreenPageBLEScan,ble_tr("附近设备", "NEARBY", "近くの機器", "附近設備"),18,252,has_saved?207:129);
    s_list=lv_list_create(ui_ScreenPageBLEScan);lv_obj_remove_style_all(s_list);
    lv_obj_set_size(s_list,258,has_saved?62:136);lv_obj_align(s_list,LV_ALIGN_TOP_MID,0,has_saved?230:156);
    lv_obj_set_style_pad_row(s_list,6,0);lv_obj_set_flex_flow(s_list,LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(s_list,LV_DIR_VER);lv_obj_set_scrollbar_mode(s_list,LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(s_list,LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_t *back=lv_btn_create(ui_ScreenPageBLEScan);lv_obj_set_size(back,140,44);lv_obj_set_pos(back,110,302);
    lv_obj_set_style_bg_color(back,lv_color_hex(0x24202d),0);lv_obj_set_style_radius(back,12,0);lv_obj_set_ext_click_area(back,4);
    lv_obj_add_event_cb(back,connection_back,LV_EVENT_CLICKED,NULL);
    lv_obj_t *bl=connection_label(back,ble_tr("返回", "BACK", "戻る", "返回"),22,132,0);lv_obj_center(bl);
    start_scan();
}
static void connection_lifecycle(lv_event_t *e) {
    if(lv_event_get_code(e)==LV_EVENT_SCREEN_LOADED)connection_render();
    else if(lv_event_get_code(e)==LV_EVENT_SCREEN_UNLOADED){
        if(s_scanning){if(s_slave_mode)gauge_pair_ble_scan_stop();else elm327_ble_scan_only_stop();}s_scanning=false;
    }else if(lv_event_get_code(e)==LV_EVENT_DELETE){
        ui_ScreenPageBLEScan=NULL;s_list=NULL;s_label_status=NULL;s_spinner=NULL;s_saved_panel=NULL;s_label_saved_hdr=NULL;s_saved_name_lbl=NULL;
    }
}
void ui_ScreenPageBLEScan_screen_init(void) {
    ui_ScreenPageBLEScan=lv_obj_create(NULL);lv_obj_remove_style_all(ui_ScreenPageBLEScan);
    lv_obj_set_size(ui_ScreenPageBLEScan,360,360);lv_obj_set_style_bg_color(ui_ScreenPageBLEScan,lv_color_hex(0x000000),0);
    lv_obj_set_style_bg_opa(ui_ScreenPageBLEScan,255,0);lv_obj_clear_flag(ui_ScreenPageBLEScan,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(ui_ScreenPageBLEScan,connection_lifecycle,LV_EVENT_ALL,NULL);
    lv_obj_add_event_cb(ui_ScreenPageBLEScan,ui_event_ble_scan_background,LV_EVENT_GESTURE,NULL);
}
