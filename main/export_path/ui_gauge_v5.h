#pragma once
#include "app_obd_dsp/gauge_v5_data.h"
#include <stdbool.h>
/* Native preview.5 UI; live readings only. These calls must run on the LVGL task. */
void ui_gauge_v5_open(unsigned page);
bool ui_gauge_v5_active(void);
gv5_lang_t ui_gauge_v5_language(void);
void ui_gauge_v5_set_language(gv5_lang_t language);
/* Global settings language subpage; Back returns to global settings. */
void ui_gauge_v5_open_language(void);
