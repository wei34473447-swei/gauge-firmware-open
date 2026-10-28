#pragma once
#include "lvgl.h"
#include "app_obd_dsp/gauge_v5_data.h"
const lv_font_t *gv5_text_font(gv5_lang_t lang, unsigned size);
const lv_font_t *gv5_number_font(unsigned size);
const lv_font_t *gv5_chinese_font(unsigned size);
/* Language-independent small Source Han Sans Medium unit labels. */
const lv_font_t *gv5_unit_font(void);
