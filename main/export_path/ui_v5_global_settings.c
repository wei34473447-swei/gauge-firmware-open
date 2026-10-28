#include "ui_v5_global_settings.h"
#include "ui_gauge_v5.h"
#include "ui_v5_about.h"
#include "gauge_v5_fonts.h"
#include "gauge_v5_boot.h"
#include "app_obd_dsp/vehicle_profiles.h"
#include "bsp_obd_dsp/elm327_ble_client.h"
#include "ui.h"
#include "bsp_obd_dsp/nvs_storage.h"
#include "bsp_obd_dsp/lcd_driver/ST77916.h"
#include <stdio.h>
#include <string.h>

static lv_obj_t *gs_screen, *gs_scroll, *gs_brightness, *gs_value, *gs_feedback;
typedef enum { GS_MAIN, GS_BOOT, GS_VEHICLE } gs_view_t;
static gs_view_t gs_view;
static gv5_lang_t gs_language;
static uint8_t gs_saved_brightness;
static const char *gs_tr(const char *zh, const char *en, const char *ja, const char *tw)
{
    return gs_language == GV5_EN ? en : gs_language == GV5_JA ? ja : gs_language == GV5_ZH_TW ? tw : zh;
}
static const char *gs_vehicle_name(const vehicle_profile_t *p) {
    return p ? p->name : "OBD2 Generic";
}
static lv_obj_t *gs_label(lv_obj_t *parent, const char *text, unsigned size, bool number)
{
    lv_obj_t *o = lv_label_create(parent);
    lv_label_set_text(o, text);
    lv_obj_set_style_text_font(o, number ? gv5_number_font(size) : gv5_text_font(gs_language, size), 0);
    lv_obj_set_style_text_color(o, lv_color_hex(0xf5f0f4), 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return o;
}
static void gs_back(lv_event_t *e)
{
    (void)e;
    if (gs_view == GS_MAIN) ui_v5_about_open();
    else ui_v5_global_settings_open();
}
static void gs_language_open(lv_event_t *e)
{
    (void)e;
    ui_gauge_v5_open_language();
}
static lv_obj_t *gs_row(int height)
{
    lv_obj_t *o = lv_obj_create(gs_scroll);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, 252, height);
    lv_obj_set_style_bg_color(o, lv_color_hex(0x211d2a), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(o, 12, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}
static void gs_show_feedback(const char *text)
{
    lv_label_set_text(gs_feedback, text);
    lv_obj_add_flag(lv_obj_get_child(gs_screen, 0), LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(gs_feedback, LV_OBJ_FLAG_HIDDEN);
}
static void gs_brightness_event(lv_event_t *e)
{
    int value = lv_slider_get_value(gs_brightness);
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        lv_label_set_text_fmt(gs_value, "%d%%", value);
        Set_Backlight((uint8_t)value);
    } else if (lv_event_get_code(e) == LV_EVENT_RELEASED && value != gs_saved_brightness) {
        nvs_user_cfg_t cfg = *nvs_cfg_get();
        cfg.brightness_day = (uint8_t)value;
        bool saved = nvs_cfg_set(&cfg) == 0;
        if (saved) gs_saved_brightness = (uint8_t)value;
        gs_show_feedback(saved ? gs_tr("已保存", "SAVED", "保存しました", "已保存") :
                          gs_tr("保存失败，仅本次有效", "SAVE FAILED / THIS SESSION ONLY", "保存失敗・今回のみ有効", "保存失敗，僅本次有效"));
    }
}
static void gs_frame(const char *heading)
{
    if (!gs_screen) {
        gs_screen = lv_obj_create(NULL);
        lv_obj_remove_style_all(gs_screen);
        lv_obj_set_size(gs_screen, 360, 360);
        lv_obj_clear_flag(gs_screen, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(gs_screen, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(gs_screen, LV_OPA_COVER, 0);
    }
    lv_obj_clean(gs_screen);
    gs_language = ui_gauge_v5_language();
    lv_obj_t *title = gs_label(gs_screen, heading, 22, false);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 47);
    gs_scroll = lv_obj_create(gs_screen);
    lv_obj_remove_style_all(gs_scroll);
    lv_obj_set_pos(gs_scroll, 52, 84);
    lv_obj_set_size(gs_scroll, 256, 184);
    lv_obj_set_style_pad_all(gs_scroll, 2, 0);
    lv_obj_set_style_pad_row(gs_scroll, 6, 0);
    lv_obj_set_flex_flow(gs_scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(gs_scroll, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(gs_scroll, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(gs_scroll, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLL_ELASTIC);

    gs_brightness = gs_value = NULL;
}
static void gs_finish(void)
{
    gs_feedback = gs_label(gs_screen, "", 18, false);
    lv_obj_add_flag(gs_feedback, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(gs_feedback, 254);
    lv_obj_set_style_text_align(gs_feedback, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(gs_feedback, 53, 47);
    lv_obj_set_style_bg_color(gs_feedback, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(gs_feedback, LV_OPA_COVER, 0);
    lv_label_set_long_mode(gs_feedback, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_t *back = lv_btn_create(gs_screen);
    lv_obj_set_size(back, 140, 44);
    lv_obj_set_pos(back, 110, 286);
    lv_obj_set_ext_click_area(back, 6);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x24202d), 0);
    lv_obj_clear_flag(back, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_add_event_cb(back, gs_back, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = gs_label(back, gs_tr("返回", "BACK", "戻る", "返回"), 22, false);
    lv_obj_center(l);
    lv_scr_load(gs_screen);
}

/* Exact persisted IDs; changing a summary never changes an existing choice. */
static const char *gs_boot_name(uint8_t id)
{
    static const char *const names[16] = {
        "LEGACY TEMP", "LEGACY INFO", "LEGACY OIL PRESSURE", "LEGACY NEEDLE",
        "LEGACY GEAR", "LEGACY RPM", "LEGACY SPEED", "MULTI A", "MULTI B",
        "MULTI C", "RPM", "COOLANT", "SPEED", "TURBO", "RADAR", "G METER"
    };
    return id < sizeof(names) / sizeof(names[0]) ? names[id] : "UNKNOWN";
}
static void gs_message(bool saved)
{
    gs_show_feedback(saved ? gs_tr("已保存", "SAVED", "保存しました", "已保存") :
        gs_tr("保存失败", "SAVE FAILED / UNCHANGED", "保存失敗", "保存失敗"));
}
static lv_obj_t *gs_choice(const char *text, bool selected, unsigned id, lv_event_cb_t cb)
{
    lv_obj_t *row = gs_row(48);
    lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_set_style_bg_color(row, lv_color_hex(selected ? 0x493449 : 0x211d2a), 0);
    lv_obj_add_event_cb(row, cb, LV_EVENT_CLICKED, (void *)(uintptr_t)(id + 1u));
    lv_obj_t *label = gs_label(row, text, 18, false);
    lv_obj_set_style_text_font(label, gv5_text_font(GV5_EN, 18), 0);
    lv_obj_set_width(label, 228);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 12, 0);
    return row;
}
static void gs_boot_select(lv_event_t *e)
{
    uint8_t id = (uint8_t)((uintptr_t)lv_event_get_user_data(e) - 1u);
    bool saved = id == gv5_boot_get() || gv5_boot_set(id);
    if (saved) ui_v5_global_settings_open();
    gs_message(saved);
}
static void gs_vehicle_select(lv_event_t *e)
{
    uint8_t id = (uint8_t)((uintptr_t)lv_event_get_user_data(e) - 1u);
    if (vehicle_profile_get(id) == vehicle_profile_get_active()) {
        ui_v5_global_settings_open();
        gs_message(true);
        return;
    }
    if (!vehicle_profile_try_set_active(id)) { gs_message(false); return; }
    bool restarted = elm327_ble_restart_current_session();
    ui_v5_global_settings_open();
    gs_show_feedback(restarted ?
        gs_tr("已保存，重新连接", "SAVED / RECONNECTING", "保存・再接続中", "已保存，重新連接") :
        gs_tr("已保存，下次连接", "SAVED / NEXT CONNECTION", "保存・次の接続", "已保存，下次連接"));
}
static void gs_boot_open(lv_event_t *e)
{
    (void)e;
    gs_language = ui_gauge_v5_language();
    gs_view = GS_BOOT;
    gs_frame(gs_tr("开机表盘", "BOOT GAUGE", "起動メーター", "開機表盤"));
    uint8_t current = gv5_boot_get();
    lv_obj_t *selected = NULL;
    for (unsigned id = 7; id < GV5_BOOT_PAGE_COUNT; id++) {
        lv_obj_t *row = gs_choice(gs_boot_name(id), current == id, id, gs_boot_select);
        if (current == id) selected = row;
    }
    gs_finish();
    if (selected) lv_obj_scroll_to_view(selected, LV_ANIM_OFF);
}
static void gs_vehicle_open(lv_event_t *e)
{
    (void)e;
    gs_language = ui_gauge_v5_language();
    gs_view = GS_VEHICLE;
    gs_frame(gs_tr("车型选择", "VEHICLE PROFILE", "車種の選択", "車型選擇"));
    uint8_t count = 0;
    const vehicle_profile_t *profiles = vehicle_profile_get_all(&count);
    const vehicle_profile_t *current = vehicle_profile_get_active();
    lv_obj_t *selected = NULL;
    for (unsigned id = 0; id < count; id++) {
        lv_obj_t *row = gs_choice(gs_vehicle_name(&profiles[id]), current == &profiles[id], id, gs_vehicle_select);
        if (current == &profiles[id]) selected = row;
    }
    gs_finish();
    if (selected) lv_obj_scroll_to_view(selected, LV_ANIM_OFF);
}
static void gs_summary(const char *title, const char *value, lv_event_cb_t cb)
{
    lv_obj_t *row = gs_row(64);
    lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(row, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = gs_label(row, title, 22, false);
    lv_obj_set_pos(label, 12, 5);
    label = gs_label(row, value, 18, false);
    lv_obj_set_style_text_font(label, gv5_text_font(GV5_EN, 18), 0);
    lv_obj_set_width(label, 228);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_pos(label, 12, 34);
}
void ui_v5_global_settings_open(void)
{
    gs_language = ui_gauge_v5_language();
    gs_view = GS_MAIN;
    gs_frame(gs_tr("全局设置", "GLOBAL SETTINGS", "全体設定", "全局設置"));
    lv_obj_t *row = gs_row(60);
    lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(row, gs_language_open, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = gs_label(row, gs_tr("语言", "LANGUAGE", "言語", "語言"), 22, false);
    lv_obj_set_pos(l, 12, 6);
    static const char *language_names[4] = {"简体中文", "ENGLISH", "日本語", "繁體中文"};
    l = gs_label(row, language_names[gs_language], 18, false);
    lv_obj_set_style_text_font(l, gv5_text_font(gs_language, 18), 0);
    lv_obj_set_pos(l, 12, 33);

    row = gs_row(80);
    l = gs_label(row, gs_tr("屏幕亮度", "BRIGHTNESS", "画面の明るさ", "屏幕亮度"), 22, false);
    lv_obj_set_pos(l, 12, 7);
    uint8_t brightness = nvs_cfg_get()->brightness_day;
    if (brightness < 10 || brightness > 100) brightness = 100;
    gs_saved_brightness = brightness;
    gs_value = gs_label(row, "", 18, true);
    lv_label_set_text_fmt(gs_value, "%u%%", brightness);
    lv_obj_align(gs_value, LV_ALIGN_TOP_RIGHT, -12, 8);
    gs_brightness = lv_slider_create(row);
    lv_obj_set_pos(gs_brightness, 20, 57);
    lv_obj_set_size(gs_brightness, 212, 10);
    lv_slider_set_range(gs_brightness, 10, 100);
    lv_slider_set_value(gs_brightness, brightness, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(gs_brightness, lv_color_hex(0xff0188), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(gs_brightness, lv_color_hex(0xb7a6d8), LV_PART_KNOB);
    lv_obj_set_style_pad_all(gs_brightness, 8, LV_PART_KNOB);
    lv_obj_set_ext_click_area(gs_brightness, 14);
    lv_obj_clear_flag(gs_brightness, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_add_event_cb(gs_brightness, gs_brightness_event, LV_EVENT_ALL, NULL);


    gs_summary(gs_tr("开机表盘", "BOOT GAUGE", "起動メーター", "開機表盤"), gs_boot_name(gv5_boot_get()), gs_boot_open);
    const vehicle_profile_t *profile = vehicle_profile_get_active();
    gs_summary(gs_tr("车型选择", "VEHICLE PROFILE", "車種の選択", "車型選擇"), gs_vehicle_name(profile), gs_vehicle_open);
    gs_finish();
}
bool ui_v5_global_settings_active(void)
{
    return gs_screen && lv_scr_act() == gs_screen;
}
