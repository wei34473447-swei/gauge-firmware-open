#include "gauge_v5_fonts.h"
LV_FONT_DECLARE(gv5_zh_14); LV_FONT_DECLARE(gv5_zh_18); LV_FONT_DECLARE(gv5_zh_22);
LV_FONT_DECLARE(gv5_en_14); LV_FONT_DECLARE(gv5_en_18); LV_FONT_DECLARE(gv5_en_22);
LV_FONT_DECLARE(gv5_ja_14); LV_FONT_DECLARE(gv5_ja_18); LV_FONT_DECLARE(gv5_ja_22);
LV_FONT_DECLARE(gv5_number_14); LV_FONT_DECLARE(gv5_number_18); LV_FONT_DECLARE(gv5_number_24);
LV_FONT_DECLARE(gv5_number_40); LV_FONT_DECLARE(gv5_number_64); LV_FONT_DECLARE(gv5_number_88);
LV_FONT_DECLARE(gv5_number_28); LV_FONT_DECLARE(gv5_number_48);
LV_FONT_DECLARE(gv5_number_56); LV_FONT_DECLARE(gv5_unit_14);
LV_FONT_DECLARE(gv5_extra_zh_18); LV_FONT_DECLARE(gv5_extra_zh_22);
LV_FONT_DECLARE(gv5_extra_ja_18); LV_FONT_DECLARE(gv5_extra_ja_22);
static const unsigned text_sizes[]={18,22};
static const unsigned number_sizes[]={14,18,24,28,40,48,56,64,88};
static const lv_font_t *const text_fonts[4][2]={
 {&gv5_zh_18,&gv5_zh_22},
 {&gv5_en_18,&gv5_en_22},
 {&gv5_ja_18,&gv5_ja_22},
 {&gv5_zh_18,&gv5_zh_22}
};
static const lv_font_t *const number_fonts[]={&gv5_number_14,&gv5_number_18,&gv5_number_24,&gv5_number_28,&gv5_number_40,&gv5_number_48,&gv5_number_56,&gv5_number_64,&gv5_number_88};
static unsigned closest(unsigned value,const unsigned *sizes,unsigned count){unsigned best=0,diff=~0U;for(unsigned i=0;i<count;i++){unsigned d=value>sizes[i]?value-sizes[i]:sizes[i]-value;if(d<diff){diff=d;best=i;}}return best;}
const lv_font_t *gv5_text_font(gv5_lang_t lang,unsigned size){
 if(lang<GV5_ZH||lang>GV5_ZH_TW)lang=GV5_ZH;
 unsigned i=closest(size,text_sizes,2);
 /* Preserve the original bitmaps and family, adding only missing new glyphs. */
 static lv_font_t extended[4][2];static bool ready[4][2];
 if(lang==GV5_EN)return text_fonts[lang][i];
 if(!ready[lang][i]){
  extended[lang][i]=*text_fonts[lang][i];
  extended[lang][i].fallback=lang==GV5_JA?(i?&gv5_extra_ja_22:&gv5_extra_ja_18):(i?&gv5_extra_zh_22:&gv5_extra_zh_18);
  ready[lang][i]=true;
 }
 return &extended[lang][i];
}
const lv_font_t *gv5_number_font(unsigned size){return number_fonts[closest(size,number_sizes,9)];}
const lv_font_t *gv5_chinese_font(unsigned size){return gv5_text_font(GV5_ZH,size);}
const lv_font_t *gv5_unit_font(void){return &gv5_unit_14;}
