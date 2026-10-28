#include "ui_gauge_v5.h"
#include "esp_log.h"
#include "gauge_v5_fonts.h"
#include "lvgl.h"
#include "nvs.h"
#include "ui_phase1.h"
#include "ui_v5_imu.h"
#include "gauge_v5_demo.h"
#include "bsp_obd_dsp/elm327_ble_client.h"
#include "bsp_obd_dsp/racechrono_ble_diy.h"
#include "app_obd_dsp/ota_update_ble.h"
#include "theme_engine/theme_interface.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* The root integration owns these brand/navigation helpers. */
extern lv_obj_t *ui_phase1_x_create(lv_obj_t *parent, int size);
extern void ui_v5_about_open(void);
extern void ui_v5_global_settings_open(void);
#define V5_SCHEMA 3
#define V5_MAGIC 0x35475658u
#define PI_F 3.14159265358979323846f
#define PINK 0xFF0188u
#define LAVENDER 0xB7A6D8u
#define PAPER 0xF5F0F4u
#define MUTED 0xC8BFCBu
#define LINE 0x34303Eu
#define RED 0xFF5151u
static const char *TAG = "gauge_v5";

typedef struct {
    uint32_t magic;
    uint16_t schema;
    uint8_t page;
    uint8_t central[4], arcs[6], main, aux, trend, four, palette, ivory;
    uint8_t enabled[GV5_FIELD_COUNT], solid[GV5_FIELD_COUNT];
    float threshold[GV5_FIELD_COUNT];
    uint32_t checksum;
} v5_preset_t;
// Exact pre-pressure alarm layout: preserve existing saved pages on expansion.
#define V5_LEGACY_FIELD_COUNT 21
typedef struct {
    uint32_t magic; uint16_t schema; uint8_t page;
    uint8_t central[4], arcs[6], main, aux, trend, four, palette, ivory;
    uint8_t enabled[V5_LEGACY_FIELD_COUNT], solid[V5_LEGACY_FIELD_COUNT];
    float threshold[V5_LEGACY_FIELD_COUNT]; uint32_t checksum;
} v5_legacy_preset_t;
typedef enum {
    VIEW_GAUGE,
    VIEW_SETTINGS,
    VIEW_PICKER,
    VIEW_ALARMS,
    VIEW_ALARM,
    VIEW_LANGUAGE
} view_t;
typedef enum {
    ACT_CENTRAL = 1,
    ACT_ARC,
    ACT_TREND,
    ACT_MAIN,
    ACT_AUX,
    ACT_MODE,
    ACT_PALETTE,
    ACT_COLOR,
    ACT_ALARMS,
    ACT_LANGUAGE,
    ACT_TEXT,
    ACT_NUMBER,
    ACT_ALARM_FIELD,
    ACT_PICK,
    ACT_LANG,
    ACT_ENABLE,
    ACT_EFFECT,
    ACT_MINUS,
    ACT_PLUS
} action_t;
#define V5_PRESET_COUNT 7u
static v5_preset_t presets[V5_PRESET_COUNT], draft;
static gv5_frame_t frame;
static gv5_demo_t demo;
static gv5_lang_t language = GV5_ZH;
static bool initialized, loaded, press_active, navigated;
static bool migration_pending[3];
/* Compare displayed inputs exactly; timestamps/ages alone do not repaint a
 * static dial. Fixed storage, no hash collisions or per-frame allocation. */
static uint32_t painted_values[GV5_FIELD_COUNT], painted_epoch, painted_second, painted_demo;
static uint8_t painted_states[GV5_FIELD_COUNT], painted_page, painted_blink;
static bool painted_valid, painted_online, painted_demo_active;
/* 显示层固定状态：不改真实缓存、历史、报警或持久配置。 */
typedef struct { float from, target, shown; uint32_t started; } v5_motion_t;
// Only the existing animated fields need interpolation state. MAP/BARO are
// scalar measurements; adding selectable fields must not grow animation RAM.
#define V5_MOTION_FIELD_COUNT (GV5_BOOST + 1u)
static v5_motion_t motion[V5_MOTION_FIELD_COUNT];
static uint32_t motion_ready, painted_alarm_mask;
static bool motion_slave;
static float painted_trend[30];
static uint8_t painted_trend_count;
/* One visible trend only, not a second history buffer. Range follows accepted
 * history, never the interpolated display, and stays fixed while samples fit. */
static struct { float lo, hi; uint32_t epoch, narrowed_at; uint8_t field; bool valid, narrowing; } trend_range;

/* rate_limit_ms occupies the original cadence structure padding. */
static struct { uint32_t last_ms, changed_ms; bool seen; uint8_t rate_limit_ms; } cadence;
_Static_assert(sizeof(cadence) == 12, "显示节奏状态不增加内存");
#define V5_BLEND_MS 132u
_Static_assert(sizeof(motion) + sizeof(motion_ready) + sizeof(painted_alarm_mask) +
               sizeof(motion_slave) + sizeof(painted_trend) + sizeof(painted_trend_count) + sizeof(cadence) + sizeof(trend_range) <= 512, "显示状态预算");
static void motion_reset(void) { motion_ready = 0; }
static unsigned page;
static view_t view = VIEW_GAUGE;
static action_t picker_action;
static unsigned picker_index;
static gv5_field_t alarm_field;
static lv_obj_t *screen, *top_logo, *center_logo, *arc_names[4], *scroll, *feedback;
static lv_timer_t *timer;
/* The existing timer serves visible fast numbers and needles. Motion requests
 * 16ms only while interpolating; the independent load cap recovers after a
 * healthy second, even while new vehicle samples keep arriving. */
static uint32_t cadence_requested_period(void);
static void cadence_reset(void) {
    memset(&cadence, 0, sizeof cadence);
    if(timer)lv_timer_set_period(timer,33);
}
static void cadence_update(void) {
    uint32_t now=lv_tick_get(), period=timer->period;
    uint32_t requested=cadence_requested_period();
    if(!cadence.seen) {
        cadence.last_ms=cadence.changed_ms=now;
        cadence.seen=true;cadence.rate_limit_ms=16;
    } else {
        uint32_t elapsed=now-cadence.last_ms;cadence.last_ms=now;
        if(elapsed>period*2) {
            uint32_t limit=elapsed>100?100:elapsed>66?50:33;
            if(limit>cadence.rate_limit_ms)cadence.rate_limit_ms=(uint8_t)limit;
            cadence.changed_ms=now;
        } else if(cadence.rate_limit_ms>16) {
            uint32_t next=cadence.rate_limit_ms==100?50:cadence.rate_limit_ms==50?33:16;
            /* Deliberate waits at the current rate are not work overruns. */
            uint32_t late=elapsed>period?elapsed-period:0;
            if(late>next/2)cadence.changed_ms=now;
            else if(now-cadence.changed_ms>=1000) {
                cadence.rate_limit_ms=(uint8_t)next;cadence.changed_ms=now;
            }
        }
    }
    uint32_t next=requested>cadence.rate_limit_ms?requested:cadence.rate_limit_ms;
    if(next!=period)lv_timer_set_period(timer,next);
}

static void *arc_buffers[4];
static lv_point_t press_start;
static uint32_t feedback_started;
static bool feedback_visible;
static lv_obj_t *threshold_label, *threshold_slider;

/* All firmware strings are supplied to the font subset generation. */
static const char *words[][4] = {
    {"设置", "SETTINGS", "設定", "設置"},
    {"保存", "SAVE", "保存", "保存"},
    {"返回", "BACK", "戻る", "返回"},
    {"中央字段", "CENTER", "中央項目", "中央字段"},
    {"外圈", "ARC", "外周", "外圈"},
    {"趋势", "TREND", "履歴", "趨勢"},
    {"主指针", "MAIN POINTER", "メインの針", "主指針"},
    {"辅助字段", "AUX FIELD", "補助項目", "輔助字段"},
    {"左右半圈", "TWO HALVES", "左右半円", "左右半圈"},
    {"四象限", "FOUR QUADRANTS", "四つの領域", "四象限"},
    {"配色", "PALETTE", "配色", "配色"},
    {"粉色 × 雾紫", "PINK / LAVENDER", "ピンク / 薄紫", "粉色 × 霧紫"},
    {"粉色 × 冷银", "PINK / SILVER", "ピンク / シルバー", "粉色 × 冷銀"},
    {"粉色 × 香槟", "PINK / CHAMPAGNE", "ピンク / シャンパン", "粉色 × 香檳"},
    {"盘面颜色", "DIAL COLOR", "文字盤の色", "盤面顏色"},
    {"纯白色", "PURE WHITE", "純白", "純白色"},
    {"暖白色", "WARM WHITE", "暖かい白", "暖白色"},
    {"字段报警", "FIELD ALERTS", "項目アラート", "字段報警"},
    {"语言", "LANGUAGE", "言語", "語言"},
    {"文字字体", "TEXT FONT", "文字フォント", "文字字體"},
    {"数字字体", "NUMBER FONT", "数字フォント", "數字字體"},
    {"思源黑体", "Source Han Sans", "思源黒体", "思源黑體"},
    {"文悦新青年体", "WenYue", "文悦新青年体", "文悅新青年體"},
    {"Zen Old Mincho", "Zen Old Mincho", "Zen Old Mincho", "Zen Old Mincho"},
    {"无", "NONE", "なし", "無"},
    {"当前不可用", "UNAVAILABLE", "現在利用不可", "當前不可用"},
    {"选择字段", "CHOOSE FIELD", "項目を選択", "選擇字段"},
    {"上限报警", "HIGH ALERT", "上限アラート", "上限報警"},
    {"开启", "ON", "オン", "開啓"},
    {"关闭", "OFF", "オフ", "關閉"},
    {"达到或超过", "AT OR ABOVE", "以上で通知", "達到或超過"},
    {"闪烁提示", "FLASH", "点滅", "閃爍提示"},
    {"常亮提示", "SOLID", "点灯", "常亮提示"},
    {"已保存", "SAVED", "保存済み", "已保存"},
    {"保存失败，仅本次有效", "SAVE FAILED / SESSION ONLY", "保存失敗・今回のみ", "保存失敗，僅本次有效"},
    {"仅所选数值支持报警", "SELECT A NUMERIC FIELD", "数値項目を選択", "僅所選數值支持報警"},
    {"字体按语言，数字独立", "TEXT BY LANGUAGE / NUMBERS SEPARATE", "文字と言語・数字は独立", "字體按語言，數字獨立"},
    {"文字与数字分开配置", "TEXT / NUMBERS ARE SEPARATE", "文字と数字は別の設定", "文字與數字分開配置"}};
enum {
    W_SETTINGS,
    W_SAVE,
    W_BACK,
    W_CENTRAL,
    W_ARC,
    W_TREND,
    W_MAIN,
    W_AUX,
    W_TWO,
    W_FOUR,
    W_PALETTE,
    W_LAVENDER,
    W_SILVER,
    W_CHAMPAGNE,
    W_COLOR,
    W_WHITE,
    W_IVORY,
    W_ALARMS,
    W_LANGUAGE,
    W_TEXT,
    W_NUMBER,
    W_SOURCE,
    W_WENYUE,
    W_ZEN,
    W_NONE,
    W_UNAVAILABLE,
    W_PICK,
    W_ALERT,
    W_ON,
    W_OFF,
    W_THRESHOLD,
    W_FLASH,
    W_SOLID,
    W_SAVED,
    W_SAVE_FAILED,
    W_NO_ALERT,
    W_FONT_INFO,
    W_FONT_DIMENSION
};
static const char *word(unsigned n) { return words[n][language]; }
static const char *arc_positions[6][4] = {
    {"左半圈", "LEFT HALF", "左半円", "左半圈"}, {"右半圈", "RIGHT HALF", "右半円", "右半圈"},
    {"左上", "UPPER LEFT", "左上", "左上"},    {"左下", "LOWER LEFT", "左下", "左下"},
    {"右上", "UPPER RIGHT", "右上", "右上"},   {"右下", "LOWER RIGHT", "右下", "右下"}};
static void build_gauge(void);
static void build_settings(void);
static void build_picker(action_t action, unsigned index);
static void build_alarm_list(void);
static void build_alarm(gv5_field_t id);
static void build_language(void);
static void show_feedback(const char *message);
static void tick(lv_timer_t *t);
static bool field_ok(unsigned id) {
    return id < GV5_FIELD_COUNT && (id == GV5_NONE || (gv5_meta((gv5_field_t)id) &&
        (gv5_meta((gv5_field_t)id)->implemented || gv5_is_electric((gv5_field_t)id))));
}
static float clampf(float n, float lo, float hi) { return n < lo ? lo : n > hi ? hi : n; }
static uint32_t checksum(const v5_preset_t *p) {
    const uint8_t *b = (const uint8_t *)p;
    uint32_t c = 2166136261u;
    for (size_t i = 0; i < offsetof(v5_preset_t, checksum); i++)
        c = (c ^ b[i]) * 16777619u;
    return c;
}
static void defaults(unsigned i, v5_preset_t *p) {
    memset(p, 0, sizeof(*p));
    p->magic = V5_MAGIC;
    p->schema = i < 3 ? V5_SCHEMA : 1;
    p->page = i;
    static const uint8_t central_defaults[3][4] = {
        {GV5_RPM, GV5_COOLANT, GV5_SPEED, GV5_VOLTAGE},
        {GV5_COOLANT, GV5_OIL, GV5_IAT, GV5_BOOST},
        {GV5_RPM, GV5_THROTTLE, GV5_LOAD, GV5_BOOST}
    };
    if (i < 3) memcpy(p->central, central_defaults[i], sizeof p->central);
    else p->central[0] = GV5_RPM;
    p->four = i < 3;
    p->arcs[0] = GV5_RPM;
    p->arcs[1] = GV5_THROTTLE;
    p->arcs[2] = GV5_RPM;
    p->arcs[3] = GV5_LOAD;
    p->arcs[4] = GV5_THROTTLE;
    p->arcs[5] = GV5_BOOST;
    p->trend = i == 0 ? GV5_RPM : i == 1 ? GV5_COOLANT : i == 2 ? GV5_BOOST : GV5_OIL;
    p->palette = 0;
    p->main = i == 3 ? GV5_RPM : i == 4 ? GV5_COOLANT : i == 6 ? GV5_BOOST : GV5_SPEED;
    p->aux = i == 3 ? GV5_GEAR : i == 4 ? GV5_IAT : i == 6 ? GV5_NONE : GV5_VOLTAGE;
    /* Unsaved analog dials use warm white; valid saved presets stay intact. */
    p->ivory = i >= 3;
    for (unsigned id = 1; id < GV5_FIELD_COUNT; id++) {
        const gv5_meta_t *m = gv5_meta((gv5_field_t)id);
        p->threshold[id] = id == GV5_RPM ? 5000 : m ? m->max : 0;
    }
    p->checksum = checksum(p);
}
static bool valid_preset(const v5_preset_t *p, unsigned i) {
    if (p->magic != V5_MAGIC || (p->schema != 1 && !(i < 3 && (p->schema == 2 || p->schema == V5_SCHEMA))) || p->page != i ||
        p->checksum != checksum(p) || p->four > 1 || p->ivory > 1 || p->palette > 2)
        return false;
    for (unsigned n = 0; n < 4; n++)
        if (!field_ok(p->central[n]))
            return false;
    for (unsigned n = 0; n < 6; n++)
        if (!field_ok(p->arcs[n]) ||
            (p->arcs[n] != GV5_NONE && !gv5_continuous((gv5_field_t)p->arcs[n])))
            return false;
    if (!field_ok(p->main) || (p->main != GV5_NONE && !gv5_continuous((gv5_field_t)p->main)) ||
        !field_ok(p->aux))
        return false;
    if (!field_ok(p->trend) || (p->trend != GV5_NONE && !gv5_trend_eligible((gv5_field_t)p->trend)))
        return false;
    for (unsigned id = 1; id < GV5_FIELD_COUNT; id++) {
        const gv5_meta_t *m = gv5_meta((gv5_field_t)id);
        if (p->enabled[id] > 1 || p->solid[id] > 1 || !isfinite(p->threshold[id]) ||
            (m && (p->threshold[id] < m->min || p->threshold[id] > m->max)))
            return false;
    }
    return true;
}
static bool read_preset(nvs_handle_t h, const char *key, unsigned i, v5_preset_t *out) {
    union { v5_preset_t current; v5_legacy_preset_t legacy; } stored;
    size_t bytes=sizeof stored;
    if(nvs_get_blob(h,key,&stored,&bytes)!=ESP_OK)return false;
    if(bytes==sizeof(v5_preset_t)) {
        if(!valid_preset(&stored.current,i))return false;
        *out=stored.current;return true;
    }
    if(bytes!=sizeof(v5_legacy_preset_t))return false;
    const v5_legacy_preset_t *old=&stored.legacy;
    uint32_t c=2166136261u;
    for(size_t n=0;n<offsetof(v5_legacy_preset_t,checksum);n++)
        c=(c^((const uint8_t *)old)[n])*16777619u;
    if(c!=old->checksum)return false;
    // Never reinterpret a legacy out-of-range ID as a newly added field.
    for(unsigned n=0;n<4;n++)if(old->central[n]>=V5_LEGACY_FIELD_COUNT)return false;
    for(unsigned n=0;n<6;n++)if(old->arcs[n]>=V5_LEGACY_FIELD_COUNT)return false;
    if(old->main>=V5_LEGACY_FIELD_COUNT||old->aux>=V5_LEGACY_FIELD_COUNT||
       old->trend>=V5_LEGACY_FIELD_COUNT)return false;
    defaults(i,out);
    _Static_assert(offsetof(v5_preset_t,enabled)==offsetof(v5_legacy_preset_t,enabled),"legacy prefix");
    memcpy(out,old,offsetof(v5_legacy_preset_t,enabled));
    memcpy(out->enabled,old->enabled,sizeof old->enabled);
    memcpy(out->solid,old->solid,sizeof old->solid);
    memcpy(out->threshold,old->threshold,sizeof old->threshold);
    out->checksum=checksum(out);
    return valid_preset(out,i); // converted in RAM; no unsolicited NVS rewrite
}

static bool store_preset(unsigned i, v5_preset_t *p);
static void retry_default_migration(void) {
    for (unsigned i = 0; i < 3; i++) {
        if (migration_pending[i]) {
            bool saved = store_preset(i, &presets[i]);
            ESP_LOGI(TAG, "Four-field defaults preset=%u persisted=%u", i, saved);
        }
    }
}
static void load_preferences(void) {
    if (loaded) {
        retry_default_migration();
        return;
    }
    loaded = true;
    for (unsigned i = 0; i < V5_PRESET_COUNT; i++) defaults(i, &presets[i]);
    for (unsigned i = 0; i < 3; i++) migration_pending[i] = true;
    nvs_handle_t h;
    if (nvs_open("gauge_v5", NVS_READONLY, &h) == ESP_OK) {
        uint8_t l;
        if (nvs_get_u8(h, "lang", &l) == ESP_OK && l <= GV5_ZH_TW)
            language = (gv5_lang_t)l;
        for (unsigned i = 0; i < V5_PRESET_COUNT; i++) {
            char key[8]; snprintf(key, sizeof key, "p%u", i);
            v5_preset_t p;
            if (read_preset(h, key, i, &p)) {
                /* The new four-quadrant default is a one-time migration.
                 * Preserve saved center fields, alerts, colors and trends;
                 * schema 3 then preserves subsequent custom TWO/NONE choices. */
                if (i < 3 && p.schema == 2) {
                    presets[i] = p;
                    if (i < 2) {
                        presets[i].four = 1;
                        static const uint8_t quadrant_defaults[4] = {
                            GV5_RPM, GV5_LOAD, GV5_THROTTLE, GV5_BOOST
                        };
                        for (unsigned q = 0; q < 4; q++)
                            if (presets[i].arcs[q + 2] == GV5_NONE)
                                presets[i].arcs[q + 2] = quadrant_defaults[q];
                    }
                } else if (i >= 3 || p.schema == V5_SCHEMA) {
                    presets[i] = p;
                    if (i < 3) migration_pending[i] = false;
                }
            }
        }
        nvs_close(h);
    }
    retry_default_migration();
}
static bool store_preset(unsigned i, v5_preset_t *p) {
    p->schema = i < 3 ? V5_SCHEMA : 1;
    p->checksum = checksum(p);
    nvs_handle_t h;
    esp_err_t rc = nvs_open("gauge_v5", NVS_READWRITE, &h);
    if (rc != ESP_OK) {
        if (i < 3) migration_pending[i] = true;
        return false;
    }
    char key[8];
    snprintf(key, sizeof key, "p%u", i);
    rc = nvs_set_blob(h, key, p, sizeof(*p));
    if (rc == ESP_OK)
        rc = nvs_commit(h);
    nvs_close(h);
    if (i < 3) migration_pending[i] = rc != ESP_OK;
    return rc == ESP_OK;
}
gv5_lang_t ui_gauge_v5_language(void) {
    load_preferences();
    return language;
}
void ui_gauge_v5_set_language(gv5_lang_t l) {
    if (l < GV5_ZH || l > GV5_ZH_TW)
        return;
    load_preferences();
    nvs_handle_t h;
    esp_err_t rc = nvs_open("gauge_v5", NVS_READWRITE, &h);
    bool opened = rc == ESP_OK;
    if (opened) {
        rc = nvs_set_u8(h, "lang", l);
        if (rc == ESP_OK) rc = nvs_commit(h);
        nvs_close(h);
    }
    if (rc != ESP_OK) {
        /* IDF NVS writes eagerly in set_u8; commit is not a rollback boundary.
         * One best-effort compensation, not a retry of the rejected selection.
         * If storage itself is failing, a power-loss rollback cannot be promised. */
        if (opened) {
            nvs_handle_t recovery;
            esp_err_t restore = nvs_open("gauge_v5", NVS_READWRITE, &recovery);
            if (restore == ESP_OK) {
                restore = nvs_set_u8(recovery, "lang", language);
                if (restore == ESP_OK) restore = nvs_commit(recovery);
                nvs_close(recovery);
            }
            ESP_LOGW(TAG, "Language save failed rc=%d restore=%d", rc, restore);
        }
        /* Keep the previous accepted language; no session-only success. */
        static const char *failure[4] = {"保存失败", "SAVE FAILED", "保存失敗", "保存失敗"};
        if (ui_gauge_v5_active()) show_feedback(failure[language]);
        return;
    }
    language = l;
    if (ui_gauge_v5_active()) {
        if (view == VIEW_GAUGE)
            build_gauge();
        else if (view == VIEW_LANGUAGE)
            build_language();
        else
            build_settings();
        show_feedback(word(W_SAVED));
    }
}
bool ui_gauge_v5_active(void) { return screen && lv_scr_act() == screen; }
/* Downloaded palette is optional. Legacy community themes and no theme keep
 * the approved built-in pixels; settings, branding and alarms remain core UI.
 * The loader caches validated colors, so this does not parse JSON while drawing. */
static uint32_t gauge_color(uint8_t role, uint32_t fallback) {
    uint32_t color;
    return theme_get_gauge_color(role, &color) ? color : fallback;
}
static uint32_t accent(void) { return gauge_color(THEME_GAUGE_ACCENT, PINK); }
static uint32_t analog_ink(void) { return gauge_color(THEME_GAUGE_ANALOG_INK, 0x21232Au); }
static uint32_t analog_muted(void) { return gauge_color(THEME_GAUGE_ANALOG_MUTED, 0x555962u); }
static uint32_t secondary(void) {
    return gauge_color(THEME_GAUGE_SECONDARY, presets[page].palette == 1   ? 0xB5C1D0u
           : presets[page].palette == 2 ? 0xD5C2A4u
                                        : LAVENDER);
}
static uint32_t background(void) { return gauge_color(THEME_GAUGE_MULTI_BACKGROUND, 0x000000u); }

static bool demo_allowed(void) {
    return !frame.online && !elm327_ble_is_connected() &&
           !racechrono_ble_diy_is_connected() && !ota_update_ble_is_connected();
}
static bool fast_field(gv5_field_t id);
static gv5_reading_t display_read(gv5_field_t id) {
    gv5_reading_t r = gv5_read(&frame, id);
    if (demo.active && id > GV5_NONE && id < GV5_FIELD_COUNT) {
        const gv5_meta_t *m = gv5_meta(id);
        float zero = clampf(0, m->min, m->max);
        r.value = zero + (m->max - zero) * demo.ratio;
        if (!m->continuous) r.value = roundf(r.value);
        r.valid = true; r.state = GV5_CONFIRMED;
    }
    if (!demo.active && id > GV5_NONE && id < V5_MOTION_FIELD_COUNT &&
        (motion_ready & (1u << id)) && r.valid && r.state == GV5_CONFIRMED)
        r.value = motion[id].shown;
    return r;
}
static bool fast_field(gv5_field_t id) {
    return id == GV5_RPM || id == GV5_SPEED || id == GV5_THROTTLE ||
           id == GV5_LOAD || id == GV5_BOOST;
}
static uint32_t visible_fields(void) {
    const v5_preset_t *p = &presets[page]; uint32_t mask = 0;
    if (page < 3) {
        for (unsigned i=0;i<4;i++) if (p->central[i]) mask |= 1u << p->central[i];
        for (unsigned q=0;q<4;q++) {
            unsigned id=p->arcs[p->four?q+2:q<2?0:1];
            if (id) mask |= 1u << id;
        }
    } else {
        if (p->main) mask |= 1u << p->main;
        if (p->aux) mask |= 1u << p->aux;
    }
    return mask;
}
static uint32_t cadence_requested_period(void) {
    uint32_t visible=visible_fields()&motion_ready;
    for(unsigned id=1;id<V5_MOTION_FIELD_COUNT;id++)
        if((visible&(1u<<id)) && motion[id].shown!=motion[id].target)return 16;
    return 33;
}
static void motion_update(bool reset) {
    if (reset || demo.active) motion_reset();
    uint32_t visible = visible_fields(), now=lv_tick_get();
    for (unsigned id=1;id<GV5_FIELD_COUNT;id++) {
        uint32_t bit=1u<<id; const gv5_reading_t *r=&frame.readings[id];
        if (demo.active || id >= V5_MOTION_FIELD_COUNT || !(visible & bit) || !fast_field((gv5_field_t)id) ||
            !r->valid || r->state != GV5_CONFIRMED || !isfinite(r->value)) {
            motion_ready &= ~bit; continue;
        }
        v5_motion_t *m=&motion[id];
        if (!(motion_ready & bit)) {
            *m=(v5_motion_t){r->value,r->value,r->value,now}; motion_ready |= bit;
        }
        uint32_t elapsed=now-m->started;
        if (elapsed>=V5_BLEND_MS) m->shown=m->target;
        else {
            float u=1.0f-(float)elapsed/V5_BLEND_MS;
            m->shown=m->from+(m->target-m->from)*(1.0f-u*u*u);
        }
        /* Advance the previous segment before redirecting at this same tick. */
        if (m->target != r->value) {
            m->from=m->shown; m->target=r->value; m->started=now;
        }
    }
}
static bool alarm_on(gv5_field_t id, gv5_reading_t r) {
    /* 报警永远按真实输入；显示插值不能延迟或提前报警。 */
    r = gv5_read(&frame, id);
    return id > GV5_NONE && id < GV5_FIELD_COUNT && r.valid && r.state == GV5_CONFIRMED &&
           isfinite(r.value) && presets[page].enabled[id] && r.value >= presets[page].threshold[id];
}
static bool blink_lit(gv5_field_t id) {
    return presets[page].solid[id] || lv_tick_get() % 1200 < 600;
}
static void format_value(gv5_field_t id, gv5_reading_t r, char *out, size_t size) {
    if (!r.valid || !isfinite(r.value)) {
        snprintf(out, size, "--");
        return;
    }
    if (id == GV5_GEAR && r.value == -1) {
        snprintf(out, size, "R");
        return;
    }
    if (id == GV5_GEAR && r.value == 0) {
        snprintf(out, size, "N");
        return;
    }
    const gv5_meta_t *m = gv5_meta(id);
    snprintf(out, size, "%.*f", m ? m->precision : 0, (double)r.value);
}
static lv_area_t area(int x, int y, int w, int h) {
    lv_area_t a = {(lv_coord_t)x, (lv_coord_t)y, (lv_coord_t)(x + w - 1), (lv_coord_t)(y + h - 1)};
    return a;
}
static bool clip_hit(lv_draw_ctx_t *ctx,int x,int y,int w,int h) {
    const lv_area_t *c=ctx->clip_area;
    return x<=c->x2 && y<=c->y2 && x+w-1>=c->x1 && y+h-1>=c->y1;
}
static void text(lv_draw_ctx_t *ctx, const char *s, const lv_font_t *font, int cx, int cy,
                 int width, uint32_t color) {
    if (!clip_hit(ctx,cx-width/2-32,cy-font->line_height/2-8,width+64,font->line_height+16)) return;
    lv_draw_label_dsc_t d;
    lv_draw_label_dsc_init(&d);
    d.font = font;
    d.color = lv_color_hex(color);
    d.align = LV_TEXT_ALIGN_CENTER;
    d.flag = LV_TEXT_FLAG_EXPAND;
    lv_area_t a = area(cx - width / 2, cy - font->line_height / 2, width, font->line_height);
    lv_draw_label(ctx, &d, &a, s, NULL);
}
static int text_width(const char *s, const lv_font_t *font) {
    return lv_txt_get_width(s, (uint32_t)strlen(s), font, 0, LV_TEXT_FLAG_NONE);
}
static const lv_font_t *number_fit(const char *s, unsigned prefer, int width, int height) {
    const unsigned sizes[] = {88, 64, 56, 48, 40, 28, 24, 18, 14};
    const lv_font_t *last = gv5_number_font(14);
    for (unsigned i = 0; i < sizeof sizes / sizeof sizes[0]; i++) {
        if (sizes[i] > prefer)
            continue;
        const lv_font_t *f = gv5_number_font(sizes[i]);
        last = f;
        if (f->line_height <= height && text_width(s, f) <= width)
            return f;
    }
    return last;
}
static void line(lv_draw_ctx_t *ctx, int x1, int y1, int x2, int y2, unsigned width, uint32_t color,
                 lv_opa_t opa) {
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = lv_color_hex(color);
    d.width = width;
    d.opa = opa;
    lv_point_t a = {x1, y1}, b = {x2, y2};
    lv_draw_line(ctx, &d, &a, &b);
}
static void rect(lv_draw_ctx_t *ctx, int x, int y, int w, int h, int radius, uint32_t color) {
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(color);
    d.bg_opa = LV_OPA_COVER;
    d.radius = radius;
    lv_area_t a = area(x, y, w, h);
    lv_draw_rect(ctx, &d, &a);
}
static void lamp(lv_draw_ctx_t *ctx, int x, int y) { rect(ctx, x - 4, y - 4, 8, 8, 4, RED); }
static lv_point_t polar(float deg, float radius) {
    float rad = deg * PI_F / 180;
    return (lv_point_t){(lv_coord_t)lroundf(180 + cosf(rad) * radius),
                        (lv_coord_t)lroundf(180 + sinf(rad) * radius)};
}
static void arc(lv_draw_ctx_t *ctx, float start, float end, unsigned width, uint32_t color,
                lv_opa_t opa) {
    if (end - start < .5f)
        return;
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.width = width;
    d.color = lv_color_hex(color);
    d.opa = opa;
    d.rounded = true;
    lv_point_t center = {180, 180};
    lv_draw_arc(ctx, &d, &center, 177, (uint16_t)lroundf(start), (uint16_t)lroundf(end));
}
static float fraction(gv5_field_t id, gv5_reading_t r) {
    const gv5_meta_t *m = gv5_meta(id);
    if (!m || !r.valid || !isfinite(r.value) || m->max <= m->min)
        return 0;
    return clampf((r.value - m->min) / (m->max - m->min), 0, 1);
}
static void draw_arcs(lv_draw_ctx_t *ctx) {
    const v5_preset_t *p = &presets[page];
    const int start[4] = {180, 90, 270, 0};
    for (unsigned q = 0; q < 4; q++) {
        if (!clip_hit(ctx,q>=2?170:-10,q==1||q==3?170:-10,200,200)) continue;
        gv5_field_t id = (gv5_field_t)p->arcs[p->four ? q + 2 : q < 2 ? 0 : 1];
        if (id == GV5_NONE)
            continue;
        gv5_reading_t r = display_read(id);
        float f = fraction(id, r), a = start[q], b = a + 90;
        uint32_t c =
            p->four ? (q == 0 || q == 3 ? accent() : secondary()) : (q < 2 ? accent() : secondary());
        arc(ctx, a, b, 3, LINE, 120);
        if (q == 1 || q == 2)
            a = b - f * 90;
        else
            b = a + f * 90;
        bool active = alarm_on(id, r);
        lv_opa_t op = active && !blink_lit(id) ? 45 : LV_OPA_COVER;
        if (active && p->solid[id])
            c = RED;
        arc(ctx, a, b, active && p->solid[id] ? 6 : 5, c, op);
        if (p->four) {
            const float degrees[4] = {225, 135, 315, 45};
            lv_point_t inner = polar(degrees[q], 169), outer = polar(degrees[q], 173);
            uint32_t tick_color = q == 0 || q == 3 ? accent() : secondary();
            line(ctx, inner.x, inner.y, outer.x, outer.y, 2, tick_color, LV_OPA_COVER);
        }
    }
}
static void value_block(lv_draw_ctx_t *ctx, gv5_field_t id, int x, int title_y, int value_y,
                        unsigned preferred, int width, int height) {
    if (!clip_hit(ctx,x-width/2-14,title_y-27,width+28,value_y+height/2+8-(title_y-27))) return;
    const gv5_meta_t *m = gv5_meta(id);
    if (!m)
        return;
    gv5_reading_t r = display_read(id);
    char val[32];
    format_value(id, r, val, sizeof val);
    const char *name = gv5_label(id, language, true);
    const lv_font_t *f = gv5_text_font(language, 22);
    const lv_font_t *unit_font = gv5_unit_font();
    int unit_width = text_width(m->unit, unit_font);
    int gap = unit_width ? 4 : 0;
    int name_width = text_width(name, f);
    /* Center the complete name/unit group, with a small superscript unit.
     * Keep 12 px for a measured-value alarm, even when it is not currently lit. */
    if (name_width + gap + unit_width > width - 12) {
        f = gv5_text_font(language, 18);
        name_width = text_width(name, f);
    }
    if (name_width + gap + unit_width > width - 12) {
        /* Compact dial captions only; settings retain the full field name.
         * Text fonts have 18/22 px sizes, so requesting 14 would not shrink. */
        if (language == GV5_EN && id == GV5_EV_CONSUMPTION) name = "USE";
        else if (language == GV5_EN && id == GV5_TRIP_ENERGY) name = "TRIP";
        else if (language == GV5_JA && id == GV5_TRIP_ENERGY) name = "平均";
        else if (language == GV5_JA && id == GV5_TRIP_FUEL) name = "平均";
        else if ((language == GV5_ZH || language == GV5_ZH_TW) && id == GV5_TRIP_ENERGY) name = "均耗";
        name_width = text_width(name, f);
    }
    int group_width = name_width + gap + unit_width;
    int left = x - group_width / 2;
    text(ctx, name, f, left + name_width / 2, title_y, name_width, MUTED);
    if (unit_width)
        text(ctx, m->unit, unit_font, left + name_width + gap + unit_width / 2,
             title_y - 6, unit_width, MUTED);
    text(ctx, val, number_fit(val, preferred, width, height), x, value_y, width, PAPER);
    if (alarm_on(id, r) && blink_lit(id))
        lamp(ctx, left + group_width + 8, title_y - 2);
}
/* Per-field minimum windows in the same natural units as gv5_history. These
 * are visual spans, not sensor limits, alarm thresholds or new sample values. */
static float trend_min_span(gv5_field_t id) {
    static const float spans[GV5_FIELD_COUNT] = {
        0, 1000, 20, 20, 20, 20, 20, 40, 1, .5f, 2,
        200, 2, 0, 20, 10, 10, 5, 20, 20, 10, 20, 10
    };
    return id > GV5_NONE && id < GV5_FIELD_COUNT ? spans[id] : 0;
}
static bool trend_range_update(gv5_field_t id, const float *points, size_t n, uint32_t epoch) {
    float span = trend_min_span(id), low = INFINITY, high = -INFINITY;
    bool old_valid = trend_range.valid;
    float old_low = trend_range.lo, old_high = trend_range.hi;
    if (trend_range.field != id || trend_range.epoch != epoch) {
        memset(&trend_range, 0, sizeof trend_range);
        trend_range.field = id; trend_range.epoch = epoch;
    }
    for (size_t i = 0; i < n; i++) if (isfinite(points[i])) {
        low = fminf(low, points[i]); high = fmaxf(high, points[i]);
    }
    if (!(span > 0) || !isfinite(low)) {
        trend_range.valid = trend_range.narrowing = false;
    } else {
        float step = span / 4, extent = fmaxf(high - low, span);
        float center = low / 2 + high / 2;
        float lo = floorf((center - extent * .6f) / step) * step;
        float hi = ceilf((center + extent * .6f) / step) * step;
        /* Do not waste half a near-zero positive window below zero. A real
         * negative history point still extends the window normally. */
        if (low >= 0 && lo < 0) { lo = 0; hi = fmaxf(hi, ceilf(extent * 1.2f / step) * step); }
        uint32_t now = lv_tick_get();
        if (!trend_range.valid) {
            trend_range.lo = lo; trend_range.hi = hi; trend_range.valid = true;
        } else if (low < trend_range.lo || high > trend_range.hi) {
            /* Expand before painting: out-of-window actual values never clip. */
            trend_range.lo = fminf(trend_range.lo, lo);
            trend_range.hi = fmaxf(trend_range.hi, hi);
            trend_range.narrowing = false;
        } else if (hi - lo <= (trend_range.hi - trend_range.lo) * .6f) {
            if (!trend_range.narrowing) {
                trend_range.narrowing = true; trend_range.narrowed_at = now;
            } else if (now - trend_range.narrowed_at >= 10000u) {
                trend_range.lo = lo; trend_range.hi = hi; trend_range.narrowing = false;
            }
        } else trend_range.narrowing = false;
    }
    return old_valid != trend_range.valid || old_low != trend_range.lo || old_high != trend_range.hi;
}
static void draw_trend(lv_draw_ctx_t *ctx, gv5_field_t id) {
    if (id == GV5_NONE || !gv5_trend_eligible(id)) return;
    const gv5_meta_t *m = gv5_meta(id);
    const float *points = painted_trend;
    size_t n = painted_trend_count;
    gv5_reading_t r = gv5_read(&frame, id);
    text(ctx, m->unit, gv5_text_font(GV5_EN, 18), 192, 344, 156, MUTED);
    if (!trend_range.valid) {
        text(ctx, "--", gv5_number_font(18), 192, 312, 100, MUTED);
        return;
    }
    char bound[24];
    snprintf(bound, sizeof bound, "%.*f", m->precision > 1 ? 2 : m->precision, trend_range.hi);
    text(ctx, bound, number_fit(bound, 18, 48, 20), 103, 298, 48, MUTED);
    snprintf(bound, sizeof bound, "%.*f", m->precision > 1 ? 2 : m->precision, trend_range.lo);
    text(ctx, bound, number_fit(bound, 18, 48, 20), 103, 326, 48, MUTED);
    line(ctx, 130, 298, 264, 298, 1, LINE, LV_OPA_COVER);
    line(ctx, 130, 326, 264, 326, 1, LINE, LV_OPA_COVER);
    if (alarm_on(id, r) && blink_lit(id)) lamp(ctx, 276, 302);
    for (size_t i = 1; i < n; i++) {
        if (!isfinite(points[i - 1]) || !isfinite(points[i])) continue;
        float a = (points[i - 1] - trend_range.lo) / (trend_range.hi - trend_range.lo),
              b = (points[i] - trend_range.lo) / (trend_range.hi - trend_range.lo);
        line(ctx, 130 + (int)((i - 1) * 134 / 29), 326 - (int)(clampf(a, 0, 1) * 28),
                  130 + (int)(i * 134 / 29), 326 - (int)(clampf(b, 0, 1) * 28),
                  2, gauge_color(THEME_GAUGE_TREND, PINK), LV_OPA_COVER);
    }
}
static void draw_multi(lv_draw_ctx_t *ctx) {
    const v5_preset_t *p = &presets[page];
    draw_arcs(ctx);
    gv5_field_t ids[4];
    unsigned n = 0;
    for (unsigned i = 0; i < 4; i++)
        if (p->central[i] != GV5_NONE)
            ids[n++] = (gv5_field_t)p->central[i];
    bool trend = p->trend != GV5_NONE;
    if (n == 1)
        value_block(ctx, ids[0], 180, 126, trend ? 203 : 214, 88, 270, 100);
    else if (n == 2) {
        const int title_y[2] = {110, 215}, value_y[2] = {152, 254};
        for (unsigned i = 0; i < 2; i++) {
            value_block(ctx, ids[i], 180, title_y[i], value_y[i], 64, 260, 64);
        }
    } else if (n == 3) {
        value_block(ctx, ids[0], 108, 118, 156, 64, 144, 56);
        value_block(ctx, ids[1], 252, 118, 156, 64, 144, 56);
        value_block(ctx, ids[2], 180, 208, 246, 64, 240, 56);
    } else if (n == 4) {
        /* Selected D: a small central diamond with four quiet, short stems.
         * Coordinates match the 360px HTML prototype; no per-frame allocation. */
        static const lv_point_t segments[8][2] = {
            {{180, 176}, {184, 180}}, {{184, 180}, {180, 184}},
            {{180, 184}, {176, 180}}, {{176, 180}, {180, 176}},
            {{180, 156}, {180, 169}}, {{180, 191}, {180, 204}},
            {{155, 180}, {169, 180}}, {{191, 180}, {205, 180}}
        };
        lv_draw_line_dsc_t d;
        lv_draw_line_dsc_init(&d);
        d.color = lv_color_hex(0x4F455Bu);
        d.width = 1;
        d.opa = 158;
        d.round_start = 1;
        d.round_end = 1;
        for (unsigned i = 0; i < 8; i++)
            lv_draw_line(ctx, &d, &segments[i][0], &segments[i][1]);
        for (unsigned i = 0; i < 4; i++)
            value_block(ctx, ids[i], i % 2 ? 252 : 108, i < 2 ? 118 : 208, i < 2 ? 156 : 246,
                        64, 144, 56);
    }
    if (trend)
        draw_trend(ctx, (gv5_field_t)p->trend);
}
static float analog_needle_fraction(gv5_field_t id, gv5_reading_t r) {
    if (r.valid && r.state == GV5_CONFIRMED && isfinite(r.value))
        return fraction(id, r);
    const gv5_meta_t *m = gv5_meta(id);
    if (!m || m->max <= m->min)
        return 0;
    /* Reference position only: do not put this zero into the reading/cache. */
    return (clampf(0, m->min, m->max) - m->min) / (m->max - m->min);
}
static void draw_analog(lv_draw_ctx_t *ctx) {
    const v5_preset_t *p = &presets[page];
    gv5_field_t id = (gv5_field_t)p->main;
    const gv5_meta_t *m = gv5_meta(id);
    uint32_t ink = analog_ink(), muted = analog_muted();
    gv5_reading_t r = display_read(id);
    if (id != GV5_NONE && m) {
        unsigned steps = id == GV5_RPM ? 60 : 40, major = id == GV5_RPM ? 5 : 5;
        for (unsigned i = 0; i <= steps; i++) {
            float degrees = -222 + 264.0f * i / steps;
            lv_point_t a = polar(degrees, i % major ? 154 : 146), b = polar(degrees, 170);
            float v = m->min + (m->max - m->min) * i / steps;
            uint32_t c = id == GV5_RPM && v >= 7000 ? 0xC72227u : ink;
            line(ctx, a.x, a.y, b.x, b.y, i % major ? 2 : 3, c, LV_OPA_COVER);
            if (i % major == 0) {
                lv_point_t pt = polar(degrees, 130);
                if (!clip_hit(ctx,pt.x-40,pt.y-25,80,50)) continue;
                char num[24];
                snprintf(num, sizeof num,
                         id == GV5_RPM  ? "%.0f"
                         : m->precision ? "%.1f"
                                        : "%.0f",
                         (double)(id == GV5_RPM ? v / 1000 : v));
                text(ctx, num, number_fit(num, 28, 62, 36), pt.x, pt.y, 62, c);
            }
        }
        const char *name = gv5_label(id, language, true);
        const lv_font_t *font = gv5_text_font(language, 22);
        text(ctx, name, font, 180, 126, 180, ink);
        text(ctx, id == GV5_RPM ? "x1000 RPM" : m->unit, gv5_unit_font(), 180, 151, 180,
             muted);
        if (alarm_on(id, r) && blink_lit(id))
            lamp(ctx, 180 + text_width(name, font) / 2 + 9, 122);
        /* Keep the approved red needle even while offline. The -- reading
         * distinguishes the zero reference from a measured value. */
        bool live = r.valid && r.state == GV5_CONFIRMED && isfinite(r.value);
        float deg = (-222 + 264 * analog_needle_fraction(id, r)) * PI_F / 180;
        float dx = cosf(deg), dy = sinf(deg);
        lv_point_t triangle[3] = {
            {(lv_coord_t)(180 + dx * 145), (lv_coord_t)(180 + dy * 145)},
            {(lv_coord_t)(180 - dx * 23 - dy * 7), (lv_coord_t)(180 - dy * 23 + dx * 7)},
            {(lv_coord_t)(180 - dx * 23 + dy * 7), (lv_coord_t)(180 - dy * 23 - dx * 7)}};
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.bg_color = lv_color_hex(gauge_color(THEME_GAUGE_NEEDLE, 0xC72227u));
        d.bg_opa = LV_OPA_COVER;
        lv_draw_triangle(ctx, &d, triangle);
        rect(ctx, 168, 168, 24, 24, 12, ink);
        if (!live)
            text(ctx, "--", gv5_number_font(24), 180, 195, 80, muted);
    }
    if (p->aux != GV5_NONE) {
        gv5_field_t aid = (gv5_field_t)p->aux;
        const gv5_meta_t *am = gv5_meta(aid);
        gv5_reading_t ar = display_read(aid);
        char val[32];
        format_value(aid, ar, val, sizeof val);
        const char *name = gv5_label(aid, language, true);
        const lv_font_t *f = gv5_text_font(language, 18);
        text(ctx, name, f, 180, 267, 164, muted);
        text(ctx, val, number_fit(val, 48, 164, 50), 180, 298, 164, ink);
        text(ctx, am ? am->unit : "", gv5_unit_font(), 180, 332, 132, muted);
        if (alarm_on(aid, ar) && blink_lit(aid))
            lamp(ctx, 180 + text_width(name, f) / 2 + 9, 264);
    }
}
/* Dedicated white turbo preset, without moving any of the three saved analogs. */
static void draw_turbo(lv_draw_ctx_t *ctx) {
    const v5_preset_t *p=&presets[page];
    gv5_field_t id=(gv5_field_t)p->main;const gv5_meta_t *m=gv5_meta(id);
    if(id!=GV5_NONE) {
        for(unsigned i=0;i<=60;i++) {
            float degrees=-222+264.0f*i/60;
            lv_point_t a=polar(degrees,i%10?157:146),b=polar(degrees,170);
            float v=m->min+(m->max-m->min)*i/60;
            uint32_t color=gv5_turbo_scale_color(id==GV5_BOOST,v)==PINK?accent():analog_ink();
            line(ctx,a.x,a.y,b.x,b.y,i%10?1:3,color,LV_OPA_COVER);
            if(i%10==0) {
                lv_point_t pt=polar(degrees,129);
                if(!clip_hit(ctx,pt.x-40,pt.y-25,80,50))continue;
                char s[24];
                snprintf(s,sizeof s,m->precision?"%.1f":"%.0f",(double)v);
                text(ctx,s,number_fit(s,24,62,30),pt.x,pt.y,62,color);
            }
        }
        gv5_reading_t r=display_read(id);char s[32];format_value(id,r,s,sizeof s);
        text(ctx,id==GV5_BOOST?"TURBO":gv5_label(id,language,true),gv5_text_font(id==GV5_BOOST?GV5_EN:language,22),180,144,166,analog_ink());
        text(ctx,s,number_fit(s,48,160,50),140,231,160,analog_ink());
        text(ctx,m->unit,gv5_unit_font(),140,263,160,analog_muted());
        if(alarm_on(id,r)&&blink_lit(id))lamp(ctx,242,140);
    }
    if(p->aux!=GV5_NONE) {
        gv5_field_t aid=(gv5_field_t)p->aux;gv5_reading_t ar=display_read(aid);
        char val[32];format_value(aid,ar,val,sizeof val);
        text(ctx,gv5_label(aid,language,true),gv5_text_font(language,18),224,289,94,analog_muted());
        text(ctx,val,number_fit(val,28,94,30),224,315,94,analog_ink());
        text(ctx,gv5_meta(aid)->unit,gv5_unit_font(),224,339,94,analog_muted());
        if(alarm_on(aid,ar)&&blink_lit(aid))lamp(ctx,266,285);
    }
}
static void turbo_pointer(lv_draw_ctx_t *ctx) {
    gv5_field_t id=(gv5_field_t)presets[page].main;if(id==GV5_NONE)return;
    float angle=(-222+264*analog_needle_fraction(id,display_read(id)))*PI_F/180;
    float dx=cosf(angle),dy=sinf(angle);
    lv_point_t points[3]={{(lv_coord_t)(180+dx*145),(lv_coord_t)(180+dy*145)},
        {(lv_coord_t)(180-dx*23-dy*7),(lv_coord_t)(180-dy*23+dx*7)},
        {(lv_coord_t)(180-dx*23+dy*7),(lv_coord_t)(180-dy*23-dx*7)}};
    lv_draw_rect_dsc_t d;lv_draw_rect_dsc_init(&d);d.bg_color=lv_color_hex(accent());d.bg_opa=LV_OPA_COVER;
    lv_draw_triangle(ctx,&d,points);rect(ctx,168,168,24,24,12,analog_ink());
}
static void draw_event(lv_event_t *e) {
    if (view != VIEW_GAUGE)
        return;
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    if(lv_event_get_code(e)==LV_EVENT_DRAW_POST) {
        if(page==6)turbo_pointer(ctx);
        if(demo.active) {
            int x=page==6?110:180, y=page<3?24:page==6?282:237;
            text(ctx,"DEMO",gv5_text_font(GV5_EN,14),x,y,70,page<3?MUTED:analog_muted());
        }
        return;
    }
    if (page < 3)
        draw_multi(ctx);
    else if(page==6)
        draw_turbo(ctx);
    else
        draw_analog(ctx);
}
static void inert(lv_obj_t *o) {
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(o, LV_OBJ_FLAG_GESTURE_BUBBLE);
}
static void plain(lv_obj_t *o, uint32_t color) {
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, 360, 360);
    lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(o, 180, 0);
    lv_obj_set_style_clip_corner(o, true, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}
/* Image zoom changes visual bounds, not layout bounds. Align its visible top. */
static lv_obj_t *brand_at(int size, int visual_top) {
    lv_obj_t *o = ui_phase1_x_create(screen, size);
    if (o) {
        lv_obj_update_layout(o);
        int height = lv_obj_get_height(o);
        int native_width = lv_obj_get_width(o);
        int visual_height = native_width > 0 ? height * size / native_width : height;
        lv_obj_align(o, LV_ALIGN_TOP_MID, 0, visual_top + (visual_height - height) / 2);
    }
    return o;
}
static void clear_arc_buffers(void) {
    for (unsigned q = 0; q < 4; q++) {
        if (arc_buffers[q])
            lv_mem_free(arc_buffers[q]);
        arc_buffers[q] = NULL;
    }
}
/* Rotate a local alpha image, not a transparent object framebuffer. This
 * preserves arc bars and units behind the caption without changing board
 * LV_COLOR_SCREEN_TRANSP or framebuffer format. */
static lv_obj_t *arc_caption(unsigned q, const char *caption, uint32_t color, int angle) {
    const lv_font_t *font = gv5_text_font(language, 18);
    int width = text_width(caption, font), height = font->line_height;
    void *buffer = lv_mem_alloc(lv_img_buf_get_img_size(width, height, LV_IMG_CF_TRUE_COLOR_ALPHA));
    if (!buffer)
        return NULL;
    arc_buffers[q] = buffer;
    lv_obj_t *o = lv_canvas_create(screen);
    lv_canvas_set_buffer(o, buffer, width, height, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_canvas_fill_bg(o, lv_color_hex(background()), LV_OPA_TRANSP);
    lv_draw_label_dsc_t d;
    lv_draw_label_dsc_init(&d);
    d.font = font;
    d.color = lv_color_hex(color);
    d.align = LV_TEXT_ALIGN_CENTER;
    lv_canvas_draw_text(o, 0, 0, width, &d, caption);
    lv_img_set_pivot(o, width / 2, height / 2);
    lv_img_set_angle(o, angle < 0 ? angle + 3600 : angle);
    inert(o);
    return o;
}
static void build_gauge(void) {
    painted_valid = false;
    motion_reset();cadence_reset();
    gv5_demo_reset(&demo, lv_tick_get());
    view = VIEW_GAUGE;
    lv_obj_clean(screen);
    clear_arc_buffers();
    scroll = NULL;
    feedback = NULL;
    feedback_visible = false;
    memset(arc_names, 0, sizeof arc_names);
    center_logo = NULL;
    lv_obj_set_style_bg_color(screen,
                              lv_color_hex(page < 3              ? background()
                                           : gauge_color(THEME_GAUGE_ANALOG_FACE, presets[page].ivory ? 0xF0EADB
                                                                 : 0xFFFFFF)),
                              0);
    top_logo = brand_at(page < 3 ? 68 : page == 6 ? 48 : 38, page < 3 ? 32 : page == 6 ? 92 : 77);
    if (page < 3) {
        const v5_preset_t *p = &presets[page];
        unsigned n = 0;
        for (unsigned i = 0; i < 4; i++)
            n += p->central[i] != GV5_NONE;
        if (!n) {
            center_logo =
                brand_at(p->trend == GV5_NONE ? 164 : 120, p->trend == GV5_NONE ? 110 : 119);
        }
        for (unsigned q = 0; q < (p->four ? 4 : 2); q++) {
            gv5_field_t id = (gv5_field_t)p->arcs[p->four ? q + 2 : q];
            if (id == GV5_NONE)
                continue;
            uint32_t color =
                p->four ? (q == 0 || q == 3 ? accent() : secondary()) : (q == 0 ? accent() : secondary());
            lv_obj_t *label;
            if (p->four) {
                /* Lower captions sit farther toward six oclock so the larger
                 * numerals keep a clear boundary from peripheral arc labels. */
                const float deg[4] = {225, 126, 315, 54};
                const int rot[4] = {-450, 360, 450, -360};
                label = arc_caption(q, gv5_label(id, language, true), color, rot[q]);
                if (!label)
                    continue;
                lv_obj_update_layout(label);
                lv_point_t pt = polar(deg[q], 144);
                lv_obj_set_pos(label, pt.x - lv_obj_get_width(label) / 2,
                               pt.y - lv_obj_get_height(label) / 2);
            } else {
                label = lv_label_create(screen);
                lv_label_set_text(label, gv5_label(id, language, true));
                lv_obj_set_style_text_font(label, gv5_text_font(language, 18), 0);
                lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
                inert(label);
                lv_obj_align(label, q == 0 ? LV_ALIGN_TOP_LEFT : LV_ALIGN_TOP_RIGHT,
                             q == 0 ? 54 : -54, 77);
            }
            arc_names[q] = label;
        }
    } else if (presets[page].main == GV5_NONE) {
        center_logo = brand_at(presets[page].aux == GV5_NONE ? 144 : 88,
                               presets[page].aux == GV5_NONE ? 112 : 107);
    }
    gv5_capture(&frame);
    painted_trend_count = page<3 && presets[page].trend!=GV5_NONE
        ? (uint8_t)gv5_history((gv5_field_t)presets[page].trend,painted_trend,30) : 0;
    trend_range_update(page < 3 ? (gv5_field_t)presets[page].trend : GV5_NONE,
                       painted_trend, painted_trend_count, frame.epoch);
    lv_obj_invalidate(screen);
}
static void action_event(lv_event_t *e);
static lv_obj_t *label_obj(lv_obj_t *parent, const char *s, int size, bool numeric) {
    lv_obj_t *o = lv_label_create(parent);
    lv_label_set_text(o, s);
    lv_obj_set_style_text_font(o, numeric ? gv5_number_font(size) : gv5_text_font(language, size),
                               0);
    lv_obj_set_style_text_color(o, lv_color_hex(PAPER), 0);
    inert(o);
    return o;
}
static void menu_shell(const char *title) {
    lv_obj_clean(screen);
    clear_arc_buffers();
    memset(arc_names, 0, sizeof arc_names);
    top_logo = NULL;
    center_logo = NULL;
    feedback = NULL;
    feedback_visible = false;
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
    brand_at(25, 15);
    lv_obj_t *head = label_obj(screen, title, 22, false);
    lv_obj_set_width(head, 240);
    lv_obj_set_style_text_align(head, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(head, LV_LABEL_LONG_DOT);
    lv_obj_align(head, LV_ALIGN_TOP_MID, 0, 47);
    scroll = lv_obj_create(screen);
    lv_obj_remove_style_all(scroll);
    lv_obj_set_pos(scroll, 53, 84);
    lv_obj_set_size(scroll, 254, 180);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(scroll, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_set_style_bg_opa(scroll, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(scroll, 0, 0);
    lv_obj_set_style_bg_color(scroll, lv_color_hex(PINK), LV_PART_SCROLLBAR);
    lv_obj_set_style_width(scroll, 3, LV_PART_SCROLLBAR);
    lv_obj_set_flex_flow(scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(scroll, LV_OBJ_FLAG_GESTURE_BUBBLE);
}
static lv_obj_t *button(lv_obj_t *parent, const char *caption, int width, int height, action_t a,
                        unsigned index) {
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, width, height);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x17141C), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_set_style_radius(b, 5, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_add_event_cb(b, action_event, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)(((unsigned)a << 8) | (index & 255)));
    lv_obj_t *t = label_obj(b, caption, 22, false);
    lv_obj_center(t);
    return b;
}
static lv_obj_t *row(const char *name, const char *value, action_t a, unsigned index,
                     const char *chinese) {
    bool zh = language == GV5_ZH || language == GV5_ZH_TW;
    bool choice = a == ACT_PICK || a == ACT_LANG;
    lv_obj_t *b = button(scroll, "", 254, zh ? 56 : 60, a, index);
    lv_obj_clean(b);
    lv_obj_set_style_border_color(b, lv_color_hex(LINE), 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_side(b, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_t *n = label_obj(b, name, choice ? 22 : 18, false);
    lv_obj_set_style_text_color(n, lv_color_hex(choice ? PAPER : MUTED), 0);
    lv_obj_set_width(n, choice ? 207 : 90);
    lv_label_set_long_mode(n, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_pos(n, 8, zh ? 15 : 5);
    if (a == ACT_PICK && gv5_is_electric((gv5_field_t)index)) {
        /* Two inert filled rectangles form a crisp icon without a bitmap or
         * new font glyph. Reserve its horizontal space in every language. */
        lv_obj_set_pos(n, 26, zh ? 15 : 5);
        lv_obj_set_width(n, 189);
        lv_obj_t *body = lv_obj_create(b);
        lv_obj_remove_style_all(body);
        lv_obj_set_pos(body, 8, 7);
        lv_obj_set_size(body, 10, 6);
        lv_obj_set_style_radius(body, 1, 0);
        lv_obj_set_style_bg_color(body, lv_color_hex(0x65C790), 0);
        lv_obj_set_style_bg_opa(body, LV_OPA_COVER, 0);
        inert(body);
        lv_obj_t *terminal = lv_obj_create(b);
        lv_obj_remove_style_all(terminal);
        lv_obj_set_pos(terminal, 18, 9);
        lv_obj_set_size(terminal, 2, 2);
        lv_obj_set_style_bg_color(terminal, lv_color_hex(0x65C790), 0);
        lv_obj_set_style_bg_opa(terminal, LV_OPA_COVER, 0);
        inert(terminal);
    }
    lv_obj_t *v = label_obj(b, value, 22, false);
    lv_obj_set_width(v, choice ? 29 : 137);
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_long_mode(v, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_pos(v, choice ? 216 : 109, zh ? 13 : 3);
    if (!zh && chinese && *chinese) {
        lv_obj_t *z = lv_label_create(b);
        lv_label_set_text(z, chinese);
        lv_obj_set_style_text_font(z, gv5_chinese_font(18), 0);
        lv_obj_set_style_text_color(z, lv_color_hex(0xFFCBE6), 0);
        lv_obj_set_width(z, 236);
        lv_label_set_long_mode(z, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_pos(z, 8, 34);
        inert(z);
    }
    return b;
}
static void footer(bool save) {
    /* These targets, including the 6px extension, fit the circular display. */
    lv_obj_t *b = button(screen, word(W_BACK), save ? 100 : 140, 44, (action_t)0, 0);
    lv_obj_set_pos(b, save ? 73 : 110, save ? 270 : 286);
    lv_obj_set_ext_click_area(b, 6);
    if (save) {
        lv_obj_t *s = button(screen, word(W_SAVE), 100, 44, (action_t)0, 1);
        lv_obj_set_pos(s, 187, 270);
        lv_obj_set_ext_click_area(s, 6);
        lv_obj_set_style_bg_color(s, lv_color_hex(PINK), 0);
        lv_obj_set_style_bg_opa(s, LV_OPA_COVER, 0);
    }
}
static unsigned alarm_ids(gv5_field_t *out) {
    bool seen[GV5_FIELD_COUNT] = {0};
    unsigned n = 0;
    const v5_preset_t *p = &draft;
    if (page >= 3) {
        if (p->main)
            seen[p->main] = true;
        if (p->aux)
            seen[p->aux] = true;
    } else {
        for (unsigned i = 0; i < 4; i++)
            if (p->central[i])
                seen[p->central[i]] = true;
        if (p->trend)
            seen[p->trend] = true;
        for (unsigned i = p->four ? 2 : 0; i < (p->four ? 6 : 2); i++)
            if (p->arcs[i])
                seen[p->arcs[i]] = true;
    }
    for (unsigned i = 1; i < GV5_FIELD_COUNT; i++)
        if (seen[i])
            out[n++] = (gv5_field_t)i;
    return n;
}
static void build_settings(void) {
    view = VIEW_SETTINGS;
    menu_shell(word(W_SETTINGS));
    char name[64];
    if (page < 3) {
        for (unsigned i = 0; i < 4; i++) {
            snprintf(name, sizeof name, "%s %u", word(W_CENTRAL), i + 1);
            row(name, gv5_label((gv5_field_t)draft.central[i], language, false), ACT_CENTRAL, i,
                "中央字段：可选择无；其余字段自动收拢");
        }
        row(word(W_ARC), word(draft.four ? W_FOUR : W_TWO), ACT_MODE, 0,
            "外圈布局：左右半圈或四象限，起点在9点和3点");
        for (unsigned i = draft.four ? 2 : 0; i < (draft.four ? 6 : 2); i++)
            row(arc_positions[i][language], gv5_label((gv5_field_t)draft.arcs[i], language, false),
                ACT_ARC, i, "外圈字段：连续数据，挡位不可选");
        row(word(W_TREND), gv5_label((gv5_field_t)draft.trend, language, false), ACT_TREND, 0,
            "底部趋势：记录最近30秒；缺测留空，挡位不可选");
        row(word(W_PALETTE), word(W_LAVENDER + draft.palette), ACT_PALETTE, 0,
            "配色：品牌粉搭配雾紫、冷银或香槟");
    } else {
        row(word(W_MAIN), gv5_label((gv5_field_t)draft.main, language, false), ACT_MAIN, 0,
            "主指针：随真实有效读数变化，挡位不可选");
        row(word(W_AUX), gv5_label((gv5_field_t)draft.aux, language, false), ACT_AUX, 0,
            "中央辅助字段：可选择挡位或无");
        row(word(W_COLOR), word(draft.ivory ? W_IVORY : W_WHITE), ACT_COLOR, 0,
            "盘面颜色：纯白或暖白");
    }
    row(word(W_ALARMS), word(W_ALERT), ACT_ALARMS, 0, "各数值字段独立设置上限报警；缺测时清除红灯");
    footer(true);
}
static gv5_field_t selected_for(action_t action, unsigned index) {
    return (gv5_field_t)(action == ACT_CENTRAL ? draft.central[index]
                         : action == ACT_ARC   ? draft.arcs[index]
                         : action == ACT_TREND ? draft.trend
                         : action == ACT_MAIN  ? draft.main
                                               : draft.aux);
}
static bool picker_allowed(action_t action, gv5_field_t id) {
    if (!field_ok(id))
        return false;
    return id == GV5_NONE || (action == ACT_TREND                         ? gv5_trend_eligible(id)
                              : (action == ACT_ARC || action == ACT_MAIN) ? gv5_continuous(id)
                                                                          : true);
}
static void build_picker(action_t action, unsigned index) {
    picker_action = action;
    picker_index = index;
    view = VIEW_PICKER;
    menu_shell(word(W_PICK));
    gv5_capture(&frame);
    gv5_field_t selected = selected_for(action, index);
    lv_obj_t *chosen = NULL;
    for (unsigned i = 0; i < GV5_FIELD_COUNT; i++) {
        gv5_field_t id = (gv5_field_t)i;
        if (!picker_allowed(action, id))
            continue;
        /* Offline setup is also the user's bench-debugging workflow. The
         * reading capability gate applies only to the vehicle transport, not
         * to an arbitrary phone connected to our BLE peripheral service. */
        bool usable = id == GV5_NONE || !frame.online || gv5_usable(&frame, id);
        if (!usable && id != selected)
            continue;
        lv_obj_t *b = row(gv5_label(id, language, false),
                          usable ? (id == selected ? ">" : "") : word(W_UNAVAILABLE), ACT_PICK, i,
                          gv5_label(id, GV5_ZH, false));
        if (!usable)
            lv_obj_add_state(b, LV_STATE_DISABLED);
        if (id == selected) {
            chosen = b;
            lv_obj_set_style_bg_color(b, lv_color_hex(0x291322), 0);
            lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        }
    }
    footer(false);
    lv_obj_update_layout(scroll);
    if (chosen)
        lv_obj_scroll_to_view(chosen, LV_ANIM_OFF);
}
static void build_alarm_list(void) {
    view = VIEW_ALARMS;
    menu_shell(word(W_ALARMS));
    gv5_field_t ids[GV5_FIELD_COUNT];
    unsigned n = alarm_ids(ids);
    for (unsigned i = 0; i < n; i++)
        row(gv5_label(ids[i], language, false), word(draft.enabled[ids[i]] ? W_ON : W_OFF),
            ACT_ALARM_FIELD, ids[i], gv5_label(ids[i], GV5_ZH, false));
    if (!n)
        row(word(W_NO_ALERT), word(W_NONE), ACT_TEXT, 0, "选择数值字段后即可设置报警");
    footer(false);
}
static void threshold_render(void) {
    const gv5_meta_t *m = gv5_meta(alarm_field);
    char val[48];
    snprintf(val, sizeof val, "%.*f", m->precision, (double)draft.threshold[alarm_field]);
    lv_label_set_text(threshold_label, val);
}
static void threshold_event(lv_event_t *e) {
    const gv5_meta_t *m = gv5_meta(alarm_field);
    float scale = powf(10, m->precision);
    draft.threshold[alarm_field] =
        clampf(lv_slider_get_value(lv_event_get_target(e)) / scale, m->min, m->max);
    threshold_render();
}
static void build_alarm(gv5_field_t id) {
    view = VIEW_ALARM;
    alarm_field = id;
    menu_shell(gv5_label(id, language, false));
    row(word(W_ALERT), word(draft.enabled[id] ? W_ON : W_OFF), ACT_ENABLE, 0,
        "达到或超过阈值提醒；缺测撤警");
    const gv5_meta_t *m = gv5_meta(id);
    char val[48];
    snprintf(val, sizeof val, "%.*f", m->precision, (double)draft.threshold[id]);
    lv_obj_t *r = lv_obj_create(scroll);
    lv_obj_remove_style_all(r);
    lv_obj_set_size(r, 254, 110);
    lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *title = label_obj(r, word(W_THRESHOLD), 18, false);
    lv_obj_set_pos(title, 8, 0);
    threshold_label = label_obj(r, val, 28, true);
    lv_obj_align(threshold_label, LV_ALIGN_TOP_MID, 0, 29);
    lv_obj_t *unit = label_obj(r, m->unit, 18, false);
    lv_obj_set_style_text_font(unit, gv5_unit_font(), 0);
    lv_obj_align(unit, LV_ALIGN_TOP_MID, 0, 61);
    lv_obj_t *minus = button(r, "-", 42, 42, ACT_MINUS, 0);
    lv_obj_set_pos(minus, 0, 22);
    lv_obj_t *plus = button(r, "+", 42, 42, ACT_PLUS, 0);
    lv_obj_set_pos(plus, 212, 22);
    threshold_slider = lv_slider_create(r);
    lv_obj_set_size(threshold_slider, 208, 6);
    lv_obj_set_pos(threshold_slider, 23, 85);
    int scale = (int)lroundf(powf(10, m->precision));
    lv_slider_set_range(threshold_slider, (int32_t)lroundf(m->min * scale),
                        (int32_t)lroundf(m->max * scale));
    lv_slider_set_value(threshold_slider, (int32_t)lroundf(draft.threshold[id] * scale),
                        LV_ANIM_OFF);
    lv_obj_set_style_bg_color(threshold_slider, lv_color_hex(PINK), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(threshold_slider, lv_color_hex(PINK), LV_PART_KNOB);
    lv_obj_set_style_pad_all(threshold_slider, 10, LV_PART_KNOB);
    lv_obj_add_event_cb(threshold_slider, threshold_event, LV_EVENT_VALUE_CHANGED, NULL);
    row(word(W_ALARMS), word(draft.solid[id] ? W_SOLID : W_FLASH), ACT_EFFECT, 0,
        "中央小红灯；外圈仅对应弧条闪烁");
    footer(false);
}
static void build_language(void) {
    view = VIEW_LANGUAGE;
    menu_shell(word(W_LANGUAGE));
    const char *names[4] = {"简体中文", "ENGLISH", "日本語", "繁體中文"};
    lv_obj_t *chosen = NULL;
    for (unsigned i = 0; i < 4; i++) {
        lv_obj_t *b = row(names[i], i == language ? ">" : "", ACT_LANG, i, NULL);
        lv_obj_set_height(b, 52);
        lv_obj_set_pos(lv_obj_get_child(b, 0), 8, 12);
        lv_obj_set_pos(lv_obj_get_child(b, 1), 216, 10);
        lv_obj_set_style_text_font(lv_obj_get_child(b, 0), gv5_text_font((gv5_lang_t)i, 22), 0);
        if (i == language) {
            chosen = b;
            lv_obj_set_style_bg_color(b, lv_color_hex(0x291322), 0);
            lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        }
    }
    footer(false);
    lv_obj_update_layout(scroll);
    if (chosen) lv_obj_scroll_to_view(chosen, LV_ANIM_OFF);
}
static void show_feedback(const char *message) {
    if (feedback)
        lv_obj_del(feedback);
    feedback = label_obj(screen, message, 18, false);
    lv_obj_set_width(feedback, 244);
    lv_label_set_long_mode(feedback, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_bg_color(feedback, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(feedback, LV_OPA_COVER, 0);
    lv_obj_set_style_text_align(feedback, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(feedback, lv_color_hex(0xFF91C9), 0);
    lv_obj_align(feedback, LV_ALIGN_TOP_MID, 0, view == VIEW_LANGUAGE ? 264 : 47);
    feedback_started = lv_tick_get();
    feedback_visible = true;
}
static void action_event(lv_event_t *e) {
    uintptr_t code = (uintptr_t)lv_event_get_user_data(e);
    action_t action = (action_t)(code >> 8);
    unsigned i = code & 255;
    if (action == 0) {
        if (i == 1) {
            presets[page] = draft;
            bool saved = store_preset(page, &presets[page]);
            draft = presets[page];
            build_settings();
            show_feedback(word(saved ? W_SAVED : W_SAVE_FAILED));
        } else if (view == VIEW_LANGUAGE)
            ui_v5_global_settings_open();
        else if (view == VIEW_SETTINGS)
            build_gauge();
        else if (view == VIEW_ALARM)
            build_alarm_list();
        else
            build_settings();
        return;
    }
    if (action == ACT_CENTRAL || action == ACT_ARC || action == ACT_TREND || action == ACT_MAIN ||
        action == ACT_AUX) {
        build_picker(action, i);
        return;
    }
    if (action == ACT_PICK) {
        gv5_field_t id = (gv5_field_t)i;
        gv5_capture(&frame);
        if (frame.online && id != GV5_NONE && !gv5_usable(&frame, id)) {
            show_feedback(word(W_UNAVAILABLE));
            return;
        }
        if (!picker_allowed(picker_action, id))
            return;
        if (picker_action == ACT_CENTRAL)
            draft.central[picker_index] = id;
        else if (picker_action == ACT_ARC)
            draft.arcs[picker_index] = id;
        else if (picker_action == ACT_TREND)
            draft.trend = id;
        else if (picker_action == ACT_MAIN)
            draft.main = id;
        else
            draft.aux = id;
        build_settings();
        return;
    }
    if (action == ACT_MODE) {
        draft.four = !draft.four;
        build_settings();
    } else if (action == ACT_PALETTE) {
        draft.palette = (draft.palette + 1) % 3;
        build_settings();
    } else if (action == ACT_COLOR) {
        draft.ivory = !draft.ivory;
        build_settings();
    } else if (action == ACT_ALARMS)
        build_alarm_list();
    else if (action == ACT_LANGUAGE)
        build_language();
    else if (action == ACT_LANG) {
        ui_gauge_v5_set_language((gv5_lang_t)i);
    } else if (action == ACT_ALARM_FIELD)
        build_alarm((gv5_field_t)i);
    else if (action == ACT_ENABLE) {
        draft.enabled[alarm_field] = !draft.enabled[alarm_field];
        build_alarm(alarm_field);
    } else if (action == ACT_EFFECT) {
        draft.solid[alarm_field] = !draft.solid[alarm_field];
        build_alarm(alarm_field);
    } else if (action == ACT_PLUS || action == ACT_MINUS) {
        const gv5_meta_t *m = gv5_meta(alarm_field);
        float step = alarm_field == GV5_RPM ? 100 : powf(10, -(int)m->precision);
        float scale = powf(10, m->precision);
        draft.threshold[alarm_field] = clampf(
            roundf((draft.threshold[alarm_field] + (action == ACT_PLUS ? step : -step)) * scale) /
                scale,
            m->min, m->max);
        threshold_render();
        lv_slider_set_value(threshold_slider,
                            (int32_t)lroundf(draft.threshold[alarm_field] * scale), LV_ANIM_OFF);
    } else if (action == ACT_TEXT || action == ACT_NUMBER)
        show_feedback(word(W_FONT_DIMENSION));
}
static void navigate(lv_dir_t dir, lv_indev_t *input) {
    navigated = true;
    if (input)
        lv_indev_wait_release(input);
    if (dir == LV_DIR_LEFT) {
        if (page < V5_PRESET_COUNT - 1)
            ui_gauge_v5_open(page + 1);
        else
            ui_v5_imu_open();
    } else if (dir == LV_DIR_RIGHT) {
        if (page)
            ui_gauge_v5_open(page - 1);
        else
            ui_phase1_funny_open();
    } else {
        draft = presets[page];
        build_settings();
    }
}
static void screen_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *in = lv_indev_get_act();
    if (view != VIEW_GAUGE)
        return;
    if (code == LV_EVENT_PRESSED) {
        press_active = true;
        navigated = false;
        if (in)
            lv_indev_get_point(in, &press_start);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
        press_active = false;
    else if (code == LV_EVENT_LONG_PRESSED && !navigated) {
        navigate(LV_DIR_TOP, in);
    } else if (code == LV_EVENT_PRESSING && press_active && !navigated && in) {
        lv_point_t p;
        lv_indev_get_point(in, &p);
        int dx = p.x - press_start.x, dy = p.y - press_start.y, ax = dx < 0 ? -dx : dx,
            ay = dy < 0 ? -dy : dy;
        if (ax >= 28 && ax * 4 >= ay * 5)
            navigate(dx > 0 ? LV_DIR_RIGHT : LV_DIR_LEFT, in);
        else if (ay >= 28 && ay * 4 >= ax * 5)
            navigate(dy > 0 ? LV_DIR_BOTTOM : LV_DIR_TOP, in);
    } else if (code == LV_EVENT_GESTURE && !navigated && in) {
        lv_dir_t dir = lv_indev_get_gesture_dir(in);
        if (dir != LV_DIR_NONE)
            navigate(dir, in);
    }
}
static void dirty(lv_area_t a) {
    if (a.x1 < 0) a.x1=0;
    if (a.y1 < 0) a.y1=0;
    if (a.x2 > 359) a.x2=359;
    if (a.y2 > 359) a.y2=359;
    /* 与现有20行DMA条带同一分片原点，避免LVGL8软件AA在不同裁剪
     * 分片边界产生像素差。没有添加framebuffer或扩大DMA。 */
    a.x1=0; a.x2=359; a.y1=(a.y1/20)*20;
    a.y2=((a.y2/20)+1)*20-1;
    if (a.x1<=a.x2 && a.y1<=a.y2) lv_obj_invalidate_area(screen,&a);
}
static void dirty_value(int x,int title_y,int value_y,int width,int height) {
    dirty(area(x-width/2-14,title_y-27,width+28,value_y+height/2+3-(title_y-27)));
}
static void dirty_arc(float a,float b) {
    if (b<a) { float t=a; a=b; b=t; }
    lv_point_t p=polar(a,177),q=polar(b,177);
    lv_area_t box={p.x<q.x?p.x:q.x,p.y<q.y?p.y:q.y,
                   p.x>q.x?p.x:q.x,p.y>q.y?p.y:q.y};
    for (int d=0;d<=360;d+=90) if (d>=a && d<=b) {
        p=polar((float)d,177);
        if(p.x<box.x1)box.x1=p.x;
        if(p.y<box.y1)box.y1=p.y;
        if(p.x>box.x2)box.x2=p.x;
        if(p.y>box.y2)box.y2=p.y;
    }
    box.x1-=9;box.y1-=9;box.x2+=9;box.y2+=9;dirty(box);
}
static void needle_points(gv5_field_t id,gv5_reading_t r,lv_point_t p[3]) {
    float a=(-222+264*analog_needle_fraction(id,r))*PI_F/180;
    float dx=cosf(a),dy=sinf(a);
    p[0]=(lv_point_t){180+dx*145,180+dy*145};
    p[1]=(lv_point_t){180-dx*23-dy*7,180-dy*23+dx*7};
    p[2]=(lv_point_t){180-dx*23+dy*7,180-dy*23-dx*7};
}
static lv_area_t needle_bounds(const lv_point_t p[3]) {
    lv_area_t b={p[0].x,p[0].y,p[0].x,p[0].y};
    for(unsigned i=1;i<3;i++) {
        if(p[i].x<b.x1)b.x1=p[i].x;
        if(p[i].y<b.y1)b.y1=p[i].y;
        if(p[i].x>b.x2)b.x2=p[i].x;
        if(p[i].y>b.y2)b.y2=p[i].y;
    }
    b.x1-=3;b.y1-=3;b.x2+=3;b.y2+=3;return b;
}
static void __attribute__((optimize("Os"))) dirty_field(gv5_field_t id,gv5_reading_t old,gv5_reading_t current,bool alert) {
    const v5_preset_t *p=&presets[page];
    /* Only dynamic numeric consumers need a formatted-value comparison.
     * Arcs and the ordinary analog main needle use geometry/state below. */
    bool numeric_consumer=page<3
        ? p->central[0]==id || p->central[1]==id || p->central[2]==id || p->central[3]==id
        : p->aux==id || (page==6 && p->main==id);
    bool numeric=false;
    if(numeric_consumer) {
        char before[32],after[32];format_value(id,old,before,sizeof before);
        format_value(id,current,after,sizeof after);
        numeric=strcmp(before,after)!=0;
    }
    if(page<3) {
        const int start[4]={180,90,270,0};
        for(unsigned q=0;q<4;q++) if(p->arcs[p->four?q+2:q<2?0:1]==id) {
            float f= fraction(id,old),g=fraction(id,current);
            float lo=alert?0:fminf(f,g),hi=fmaxf(f,g);
            float a=start[q],b=a+90;
            float oa=q==1||q==2?b-f*90:a,ob=q==1||q==2?b:a+f*90;
            float na=q==1||q==2?b-g*90:a,nb=q==1||q==2?b:a+g*90;
            if(!alert && (ob-oa<.5f)==(nb-na<.5f) &&
                lroundf(oa)==lroundf(na) && lroundf(ob)==lroundf(nb)) continue;
            dirty_arc(q==1||q==2?b-hi*90:a+lo*90,
                      q==1||q==2?b-lo*90:a+hi*90);
        }
        gv5_field_t ids[4];unsigned n=0;
        for(unsigned i=0;i<4;i++)if(p->central[i])ids[n++]=(gv5_field_t)p->central[i];
        for(unsigned i=0;i<n;i++)if(ids[i]==id && (numeric || alert)) {
            if(n==1)dirty_value(180,126,p->trend!=GV5_NONE?203:214,270,100);
            else if(n==2)dirty_value(180,i?215:110,i?254:152,260,64);
            else if(n==3)dirty_value(i==2?180:i?252:108,i==2?208:118,i==2?246:156,i==2?240:144,56);
            else dirty_value(i%2?252:108,i<2?118:208,i<2?156:246,144,56);
        }
    } else {
        if(p->main==id) {
            /* 旧/新区域分开；LVGL按需合并，不画中间不存在的轨迹。 */
            lv_point_t before_points[3],after_points[3];
            needle_points(id,old,before_points);needle_points(id,current,after_points);
            if(memcmp(before_points,after_points,sizeof before_points)) {
                dirty(needle_bounds(before_points));dirty(needle_bounds(after_points));
            }
            if(page==6) { if(numeric)dirty(area(60,203,160,56)); if(alert)dirty(area(234,132,18,18)); }
            else {
                if(old.valid!=current.valid || old.state!=current.state)dirty(area(137,174,86,39));
                if(alert)dirty(area(80,108,210,28));
            }
        }
        if(p->aux==id && (numeric || alert)) {
            if(page==6)dirty_value(224,289,315,94,30);
            else dirty_value(180,267,298,164,50);
        }
    }
}
static bool __attribute__((optimize("Os"))) render_inputs_changed(void) {
    bool reset=!painted_valid || painted_page!=page || painted_epoch!=frame.epoch ||
               painted_online!=frame.online || painted_demo_active!=demo.active || motion_slave!=frame.slave;
    motion_update(reset);
    uint8_t blink=lv_tick_get()%1200<600?1:2;
    uint32_t alarm_mask=0,visible=visible_fields();bool changed=reset;
    for(unsigned id=1;id<GV5_FIELD_COUNT;id++) {
        gv5_reading_t r=display_read((gv5_field_t)id),old={0};uint32_t bits=0;
        if(r.valid)memcpy(&bits,&r.value,sizeof bits);
        uint8_t state=(uint8_t)r.state|(r.valid?0x80:0);
        memcpy(&old.value,&painted_values[id],sizeof old.value);
        old.valid=(painted_states[id]&0x80)!=0;old.state=(gv5_state_t)(painted_states[id]&0x7f);
        bool active=alarm_on((gv5_field_t)id,r);uint32_t bit=1u<<id;
        if(active)alarm_mask|=bit;
        bool alert=((painted_alarm_mask^alarm_mask)&bit)!=0 ||
                   (active&&!presets[page].solid[id]&&painted_blink!=blink);
        bool field_changed=bits!=painted_values[id] || state!=painted_states[id];
        if((visible&bit)&&(field_changed||alert)) {
            changed=true;if(!reset)dirty_field((gv5_field_t)id,old,r,alert);
        }
        painted_values[id]=bits;painted_states[id]=state;
    }
    uint32_t ratio=0;if(demo.active)memcpy(&ratio,&demo.ratio,sizeof ratio);
    if(painted_demo!=ratio || painted_demo_active!=demo.active) {
        changed=true;
        if(!reset)dirty(area((page==6?110:180)-38,(page<3?24:page==6?282:237)-12,76,24));
    }
    uint32_t second=0;
    if(page<3 && presets[page].trend!=GV5_NONE) {
        float samples[30];uint8_t count=(uint8_t)gv5_history((gv5_field_t)presets[page].trend,samples,30);
        uint32_t bit=1u<<presets[page].trend;
        bool trend_alert=((painted_alarm_mask^alarm_mask)&bit)!=0 ||
            ((alarm_mask&bit)&&!presets[page].solid[presets[page].trend]&&painted_blink!=blink);
        bool samples_changed = count != painted_trend_count || memcmp(samples, painted_trend, count * sizeof(float));
        bool range_changed = false;
        if (samples_changed || trend_range.narrowing || trend_range.field != presets[page].trend || trend_range.epoch != frame.epoch)
            range_changed = trend_range_update((gv5_field_t)presets[page].trend, samples, count, frame.epoch);
        if(samples_changed || trend_alert || range_changed) {
            changed=true;if(!reset)dirty(area(0,280,360,80));
        }
        memcpy(painted_trend,samples,count*sizeof(float));painted_trend_count=count;
    } else painted_trend_count=0;
    if(reset)lv_obj_invalidate(screen);
    painted_demo=ratio;painted_second=second;painted_blink=blink;painted_alarm_mask=alarm_mask;
    painted_page=page;painted_epoch=frame.epoch;painted_online=frame.online;
    painted_demo_active=demo.active;motion_slave=frame.slave;painted_valid=true;
    return changed;
}
static void tick(lv_timer_t *t) {
    (void)t;
    if (!ui_gauge_v5_active()) {
        gv5_demo_reset(&demo, lv_tick_get());
        motion_reset();cadence_reset(); painted_valid=false;
        return;
    }
    if (view == VIEW_GAUGE) {
        gv5_capture(&frame);
        bool was_demo = demo.active;
        gv5_demo_update(&demo, lv_tick_get(), demo_allowed());
        if(demo.active && !was_demo) ESP_LOGI(TAG,"Offline display demo preset=%u; cache/history unchanged",page);
        (void)render_inputs_changed();
        cadence_update();
    } else {
        motion_reset();cadence_reset(); painted_valid=false;
        gv5_demo_reset(&demo, lv_tick_get());
        if (feedback_visible && lv_tick_elaps(feedback_started) > 1800) {
        if (feedback)
            lv_obj_del(feedback);
        feedback = NULL;
        feedback_visible = false;
        }
    }
}
static void ensure_screen(void) {
    if (!initialized) {
        screen = lv_obj_create(NULL);
        plain(screen, 0x000000);
        lv_obj_add_event_cb(screen, draw_event, LV_EVENT_DRAW_MAIN, NULL);
        lv_obj_add_event_cb(screen, draw_event, LV_EVENT_DRAW_POST, NULL);
        lv_obj_add_event_cb(screen, screen_event, LV_EVENT_ALL, NULL);
        timer = lv_timer_create(tick, 33, NULL);
        initialized = true;
    }
}
void ui_gauge_v5_open_language(void) {
    load_preferences();
    ensure_screen();
    build_language();
    lv_scr_load(screen);
}
void ui_gauge_v5_open(unsigned p) {
    if (p >= V5_PRESET_COUNT)
        p = 0;
    load_preferences();
    page = p;
    ensure_screen();
    build_gauge();
    lv_scr_load(screen);
    ESP_LOGI(TAG, "Native gauge preset=%u", page);
}
