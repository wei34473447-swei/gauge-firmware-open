#include "ui_v5_about.h"
#include "ui_v5_global_settings.h"
#include "ui_gauge_v5.h"
#include "ui_v5_imu.h"
#include "gauge_v5_fonts.h"
#include "ui_phase1.h"
#include "preview5_assets.h"
#include "ui.h"
#include "esp_app_desc.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "bsp_obd_dsp/elm327_ble_client.h"
#include "bsp_obd_dsp/espnow_link.h"
#include "bsp_obd_dsp/nvs_storage.h"
#include "bsp_obd_dsp/racechrono_ble_diy.h"
#include "app_obd_dsp/ota_wifi_server.h"
#include "app_obd_dsp/ota_update_ble.h"
#include "theme_engine/theme_interface.h"
#include <stdio.h>
#include <stdlib.h>
static lv_obj_t *screen,*scroll,*status,*update_screen,*update_status;
static lv_timer_t *ticker;
static bool credits,dragging,wrapping;
static bool menu_press_active,menu_swipe_handled;
static lv_point_t menu_press_start;
static uint32_t hold_until;
static int qr_y[2],last_auto_qr=-1;
static gv5_lang_t language;
static const char *tr(const char *zh,const char *en,const char *ja,const char *tw)
{ return language==GV5_EN?en:language==GV5_JA?ja:language==GV5_ZH_TW?tw:zh; }
static lv_obj_t *label(lv_obj_t *parent,const char *text,int size,int width)
{
    lv_obj_t *obj=lv_label_create(parent);lv_label_set_text(obj,text);
    lv_obj_set_style_text_font(obj,gv5_text_font(language,size),0);
    lv_obj_set_style_text_color(obj,lv_color_hex(0xf5f0f4),0);
    lv_obj_set_style_text_align(obj,LV_TEXT_ALIGN_CENTER,0);
    if(width)lv_obj_set_width(obj,width);
    lv_obj_clear_flag(obj,LV_OBJ_FLAG_CLICKABLE);
    return obj;
}
static void open_system(lv_event_t *e)
{
    (void)e;
    ui_v5_global_settings_open();
}
static void open_connection(lv_event_t *e)
{
    if(!ui_ScreenPageBLEScan)ui_ScreenPageBLEScan_screen_init();
    lv_scr_load(ui_ScreenPageBLEScan);
}
static void open_classic(lv_event_t *e)
{
    if(theme_page_list_count()>0){
        ui_theme_gauge_page_index=0;
        if(!ui_ScreenPageThemeGauge)ui_ScreenPageThemeGauge_screen_init();
        lv_scr_load(ui_ScreenPageThemeGauge);
    }
}
static void update_back(lv_event_t *e)
{
    (void)e;
    ota_wifi_state_t state=ota_wifi_server_get_state();
    if((state!=OTA_WIFI_STATE_IDLE && state!=OTA_WIFI_STATE_ERROR) || ota_update_ble_is_active())return;
    bool restart=ota_wifi_server_restart_required();
    ota_wifi_server_stop();
    if(restart){esp_restart();return;}
    racechrono_ble_diy_set_ota_mode(false);ui_v5_about_open();
}
static void update_events(lv_event_t *e)
{
    if(lv_event_get_code(e)!=LV_EVENT_GESTURE)return;
    lv_indev_t *i=lv_indev_get_act();if(!i)return;
    lv_dir_t d=lv_indev_get_gesture_dir(i);
    if(d!=LV_DIR_LEFT && d!=LV_DIR_RIGHT)return;
    lv_indev_wait_release(i);
    update_back(e);
}
static void open_update(lv_event_t *e)
{
    if(!update_screen){
        update_screen=lv_obj_create(NULL);lv_obj_remove_style_all(update_screen);lv_obj_set_size(update_screen,360,360);
        lv_obj_clear_flag(update_screen,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_style_bg_color(update_screen,lv_color_hex(0x000000),0);lv_obj_set_style_bg_opa(update_screen,255,0);
        lv_obj_add_event_cb(update_screen,update_events,LV_EVENT_ALL,NULL);
    }
    lv_obj_clean(update_screen);
    language=ui_gauge_v5_language();
    lv_obj_t *x=ui_phase1_x_create(update_screen,72);
    lv_obj_update_layout(x);
    int h0=lv_obj_get_height(x), w0=lv_obj_get_width(x);
    int visual_h=w0>0?h0*72/w0:h0;
    lv_obj_align(x,LV_ALIGN_TOP_MID,0,28+(visual_h-h0)/2);
    lv_obj_t *h=label(update_screen,tr("手机升级","PHONE UPDATE","スマホで更新","手機升級"),22,272);lv_obj_set_pos(h,44,117);
    uint8_t mac[6]={0};char device[40];
    esp_read_mac(mac,ESP_MAC_BT);
    snprintf(device,sizeof(device),"Community / %02X%02X",mac[4],mac[5]);
    lv_obj_t *name=label(update_screen,device,22,280);
    lv_obj_set_style_text_font(name,gv5_text_font(GV5_EN,22),0);lv_obj_set_pos(name,40,156);
    lv_obj_t *guide=label(update_screen,"Use a compatible OTA client\nDownload before Wi-Fi",18,274);lv_obj_set_pos(guide,43,190);
    update_status=label(update_screen,"BLE / Wi-Fi OTA",18,252);lv_obj_set_pos(update_status,54,263);
    lv_obj_t *back=lv_btn_create(update_screen);lv_obj_set_pos(back,110,286);lv_obj_set_size(back,140,44);
    lv_obj_set_ext_click_area(back,6);lv_obj_set_style_bg_color(back,lv_color_hex(0x24202d),0);
    lv_obj_set_style_radius(back,12,0);lv_obj_clear_flag(back,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(back,update_back,LV_EVENT_CLICKED,NULL);
    lv_obj_t *bl=label(back,tr("返回","BACK","戻る","返回"),22,132);lv_obj_center(bl);
    racechrono_ble_diy_set_ota_mode(true);lv_scr_load(update_screen);lv_timer_resume(ticker);
}
static lv_obj_t *button(const char *text,int x,lv_event_cb_t cb)
{
    lv_obj_t *b=lv_btn_create(screen);lv_obj_set_pos(b,x,286);lv_obj_set_size(b,80,44);lv_obj_set_ext_click_area(b,4);
    /* A slow drag beginning on a menu button must reach the page's press
     * tracker too; wait_release cancels the button click after navigation. */
    lv_obj_add_flag(b,LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_style_bg_color(b,lv_color_hex(0x24202d),0);lv_obj_set_style_radius(b,12,0);
    lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,NULL);lv_obj_t *l=label(b,text,18,78);lv_obj_center(l);return b;
}
static void render(void);
/* Keep the menu's deliberate slow swipes consistent with the Radar page.
 * Credits own their scrolling; no press-travel navigation runs there. */
static void menu_navigate(lv_dir_t d,lv_indev_t *i)
{
    menu_press_active=false;menu_swipe_handled=true;lv_indev_wait_release(i);
    if(d==LV_DIR_LEFT)ui_phase1_funny_open();
    else if(d==LV_DIR_RIGHT)ui_v5_imu_open();
    else if(d==LV_DIR_TOP){credits=true;render();}
}
static void events(lv_event_t *e)
{
    lv_event_code_t code=lv_event_get_code(e);
    lv_indev_t *i=lv_indev_get_act();
    if(code==LV_EVENT_SCREEN_UNLOADED){
        lv_timer_pause(ticker);menu_press_active=false;menu_swipe_handled=true;
    }else if(code==LV_EVENT_SCREEN_LOADED){
        menu_press_active=false;menu_swipe_handled=false;lv_timer_resume(ticker);
    }
    else if(code==LV_EVENT_DELETE){
        if(ticker){lv_timer_del(ticker);}
        ticker=NULL;screen=NULL;
        menu_press_active=false;menu_swipe_handled=true;
    }else if(credits){
        if(code==LV_EVENT_GESTURE&&i&&lv_indev_get_gesture_dir(i)==LV_DIR_RIGHT){
            lv_indev_wait_release(i);credits=false;render();
        }
    }else if(code==LV_EVENT_PRESSED&&i){
        lv_indev_get_point(i,&menu_press_start);
        menu_press_active=true;menu_swipe_handled=false;
    }else if(code==LV_EVENT_PRESS_LOST)menu_press_active=false;
    else if(code==LV_EVENT_PRESSING&&i&&menu_press_active&&!menu_swipe_handled){
        lv_point_t p;lv_indev_get_point(i,&p);
        int dx=p.x-menu_press_start.x,dy=p.y-menu_press_start.y;
        int ax=abs(dx),ay=abs(dy);
        if(ax>=28&&ax*4>=ay*5)menu_navigate(dx>0?LV_DIR_RIGHT:LV_DIR_LEFT,i);
        else if(ay>=28&&ay*4>=ax*5&&dy<0)menu_navigate(LV_DIR_TOP,i);
    }else if(code==LV_EVENT_GESTURE&&i&&!menu_swipe_handled){
        lv_dir_t d=lv_indev_get_gesture_dir(i);
        if(d==LV_DIR_LEFT||d==LV_DIR_RIGHT||d==LV_DIR_TOP)menu_navigate(d,i);
    }
}
static void scroll_event(lv_event_t *e)
{
    lv_event_code_t c=lv_event_get_code(e);
    if(c==LV_EVENT_PRESSED){dragging=true;hold_until=lv_tick_get()+10000;}
    else if(c==LV_EVENT_RELEASED || c==LV_EVENT_PRESS_LOST){dragging=false;hold_until=lv_tick_get()+10000;}
    else if(c==LV_EVENT_SCROLL_END && !wrapping && lv_obj_get_scroll_bottom(scroll)<=0 && lv_obj_get_scroll_y(scroll)>0){
        wrapping=true;lv_obj_scroll_to_y(scroll,0,LV_ANIM_OFF);wrapping=false;last_auto_qr=-1;
        hold_until=lv_tick_get()+3000;
    }
}
static int add_text(const char *text,int y,int size)
{
    lv_obj_t *l=label(scroll,text,size,278);lv_obj_set_pos(l,0,y);lv_obj_update_layout(l);
    return y+lv_obj_get_height(l)+8;
}
static int add_image(const phase1_asset_t *asset,int y)
{
    lv_obj_t *i=ui_phase1_asset_create(scroll,asset);lv_obj_set_pos(i,(278-asset->width)/2,y);return y+asset->height+8;
}
/* Logo-only scale; QR images keep their original resolution and geometry. */
static int add_small_social_logo(const phase1_asset_t *asset,int y)
{
    lv_obj_t *i=ui_phase1_asset_create(scroll,asset);
    const int w=asset->width*3/4,h=asset->height*3/4;
    lv_img_set_pivot(i,0,0);lv_img_set_zoom(i,192);
    lv_obj_set_pos(i,(278-w)/2,y);
    return y+h+8;
}
/* Keep the community mark in its original Conthrax wordmark treatment.
 * This is a logo exception to the selected language's body font. */
static int add_community_mark(int y)
{
    lv_obj_t *sky=label(scroll,"SKY",22,278);
    lv_obj_set_style_text_font(sky,&ui_font_FontTypoderSize44,0);
    lv_obj_set_style_text_letter_space(sky,6,0);
    lv_obj_set_pos(sky,0,y);lv_obj_update_layout(sky);
    y+=lv_obj_get_height(sky)+2;
    lv_obj_t *gauge=label(scroll,"GAUGE",22,278);
    lv_obj_set_style_text_font(gauge,&ui_font_FontTypoderSize24,0);
    lv_obj_set_style_text_letter_space(gauge,9,0);
    lv_obj_set_style_text_color(gauge,lv_color_hex(0xaaaaaa),0);
    lv_obj_set_pos(gauge,0,y);lv_obj_update_layout(gauge);
    return y+lv_obj_get_height(gauge)+12;
}
static int add_brand_link(const char *text,int y)
{
    lv_obj_t *l=label(scroll,text,22,278);
    lv_obj_set_style_text_font(l,gv5_text_font(GV5_EN,22),0);
    lv_obj_set_pos(l,0,y);lv_obj_update_layout(l);
    return y+lv_obj_get_height(l)+12;
}
static void render(void)
{
    lv_obj_clean(screen);scroll=NULL;status=NULL;dragging=false;wrapping=false;language=ui_gauge_v5_language();last_auto_qr=-1;
    if(!credits){
        lv_obj_t *word=ui_phase1_asset_create(screen,&preview5_brand_wordmark);lv_obj_align(word,LV_ALIGN_TOP_MID,0,38);
        lv_obj_t *x=ui_phase1_x_create(screen,136);lv_obj_align(x,LV_ALIGN_CENTER,0,-22);
        char version[64];snprintf(version,sizeof(version),esp_app_get_description()->version[0]=='v'?"%s":"v%s",esp_app_get_description()->version);
        lv_obj_t *v=label(screen,version,18,270);lv_obj_set_pos(v,45,235);
        if(theme_page_list_count()>0){
        lv_obj_t *classic=lv_btn_create(screen);lv_obj_set_pos(classic,55,258);lv_obj_set_size(classic,250,26);
        lv_obj_set_style_bg_opa(classic,LV_OPA_TRANSP,0);lv_obj_set_style_shadow_width(classic,0,0);
        lv_obj_add_event_cb(classic,open_classic,LV_EVENT_CLICKED,NULL);
        lv_obj_t *cl=label(classic,tr("表盘与主题","CLASSIC / THEMES","メーターとテーマ","錶盤與主題"),18,246);lv_obj_center(cl);
        lv_obj_set_style_text_color(cl,lv_color_hex(0xb7a6d8),0);
        }
        button(tr("设置","SETUP","設定","設定"),50,open_system);
        button(tr("连接","LINK","接続","連接"),140,open_connection);
        button(tr("升级","UPDATE","更新","升級"),230,open_update);
    }else{
        lv_obj_t *h=label(screen,tr("关于与鸣谢","ABOUT / THANKS","紹介と感謝","關於與鳴謝"),22,260);lv_obj_set_pos(h,50,32);
        scroll=lv_obj_create(screen);lv_obj_remove_style_all(scroll);lv_obj_set_size(scroll,284,268);lv_obj_set_pos(scroll,38,66);
        lv_obj_set_style_pad_all(scroll,3,0);lv_obj_set_scroll_dir(scroll,LV_DIR_VER);lv_obj_set_scrollbar_mode(scroll,LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(scroll,LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM);lv_obj_add_flag(scroll,LV_OBJ_FLAG_GESTURE_BUBBLE);
        lv_obj_add_event_cb(scroll,scroll_event,LV_EVENT_ALL,NULL);
        int y=4;y=add_image(&preview5_brand_wordmark,y);y=add_text(tr("关于","ABOUT","私たちについて","關於"),y,22);
        char v[64];snprintf(v,sizeof(v),esp_app_get_description()->version[0]=='v'?"%s":"v%s",esp_app_get_description()->version);y=add_text(v,y,18);
        y=add_brand_link("Community Gauge",y);
        qr_y[0]=-1000;
        qr_y[1]=-1000;
        y=add_text(tr("鸣谢","THANK YOU","感謝","鳴謝"),y,22);
        y=add_text(tr("感谢开源协作者\n感谢每一份共享与贡献。","To all open-source collaborators,\nthank you for sharing.","オープンソースに協力する\n皆さまへ、感謝します。","感謝開源協作者\n感謝每一份分享與貢獻。"),y,18);
        y=add_community_mark(y);y=add_brand_link("steveEcode",y);
        y=add_text("ctos.stevectos.xyz:6003",y,18);
        y=add_text("github.com/steveEcode/\nobd_brz_gauge",y,18);
        y=add_text(tr("继续创造，继续分享。","Keep creating. Keep sharing.","一緒に、次のアイデアへ。","繼續創造，繼續分享。"),y,18);
        lv_obj_t *tail=lv_obj_create(scroll);lv_obj_remove_style_all(tail);lv_obj_set_pos(tail,0,y);lv_obj_set_size(tail,1,12);
        hold_until=lv_tick_get()+3000;lv_obj_update_layout(scroll);
    }
}
static void tick(lv_timer_t *t)
{
    if(lv_scr_act()==update_screen){
        ota_wifi_state_t state=ota_wifi_server_get_state();
        const char *message=state==OTA_WIFI_STATE_ERROR?tr("升级失败，请断电重启","ERROR / RESTART GAUGE","失敗・再起動してください","升級失敗，請斷電重啟"):state==OTA_WIFI_STATE_IDLE?"BLE / Wi-Fi OTA":tr("升级中，请停留","UPDATING / STAY HERE","更新中・待機してください","升級中，請停留");
        if(update_status)lv_label_set_text(update_status,message);
        return;
    }
    if(lv_scr_act()!=screen)return;
    if(status){
        bool online=nvs_cfg_get()->device_role==ESPNOW_ROLE_SLAVE?espnow_link_slave_has_data():elm327_ble_is_connected();
        lv_label_set_text(status,online?tr("蓝牙已连接","BLE CONNECTED","BLE接続済み","藍牙已連接"):tr("等待连接","WAITING FOR BLE","BLE接続待ち","等待連接"));
    }
    if(!scroll||dragging || (int32_t)(lv_tick_get()-hold_until)<0)return;
    int y=lv_obj_get_scroll_y(scroll),next=y+2;
    for(int i=0;i<2;i++)if(last_auto_qr<i && y+119<=qr_y[i]+111 && next+119>=qr_y[i]+111){last_auto_qr=i;hold_until=lv_tick_get()+10000;return;}
    if(lv_obj_get_scroll_bottom(scroll)<=2){wrapping=true;lv_obj_scroll_to_y(scroll,0,LV_ANIM_OFF);wrapping=false;last_auto_qr=-1;hold_until=lv_tick_get()+3000;}
    else lv_obj_scroll_to_y(scroll,next,LV_ANIM_OFF);
}
void ui_v5_about_open(void)
{
    if(!screen){screen=lv_obj_create(NULL);lv_obj_remove_style_all(screen);lv_obj_set_size(screen,360,360);lv_obj_clear_flag(screen,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(screen,lv_color_hex(0x000000),0);lv_obj_set_style_bg_opa(screen,255,0);
        lv_obj_add_event_cb(screen,events,LV_EVENT_ALL,NULL);ticker=lv_timer_create(tick,100,NULL);}
    credits=false;render();lv_scr_load(screen);lv_timer_resume(ticker);
}
bool ui_v5_about_active(void){return (screen&&lv_scr_act()==screen)||(update_screen&&lv_scr_act()==update_screen);}
