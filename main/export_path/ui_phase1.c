#include "ui_phase1.h"
#include "phase1_assets.h"
#include "preview5_assets.h"
#include "gauge_v5_fonts.h"
#include "ui_gauge_v5.h"
#include "ui_v5_about.h"
#include "ui.h"
#include "ui_helpers.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "phase1";
#define PI_F 3.14159265358979323846f
#define RAYS 32
/* Immutable assets expand lazily. Only boot branding expires with its images. */
typedef struct { const phase1_asset_t *source; lv_img_dsc_t image; uint8_t *pixels; bool mirror; uint16_t live_images; } cached_image_t;
static cached_image_t images[20];
static bool boot_asset(const phase1_asset_t *asset)
{
    return asset==&phase1_brand_x || asset==&phase1_brand_top;
}
/* The cache owner travels with the image without a dynamic event descriptor.
 * Keep the public LVGL user_data available to callers. */
typedef struct { lv_img_t image; cached_image_t *owned_slot; } boot_image_t;
static void boot_image_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    (void)class_p;
    boot_image_t *image=(boot_image_t *)obj;
    cached_image_t *slot=image->owned_slot;
    image->owned_slot=NULL;
    /* DELETE observers have finished before the class destructor. Detach
     * before releasing our original slot, even if a caller replaced src. */
    lv_img_set_src(obj,NULL);
    if(!slot || !slot->live_images) return;
    if(--slot->live_images) return;
    lv_img_cache_invalidate_src(&slot->image);
    free(slot->pixels);
    memset(slot,0,sizeof(*slot));
}
static const lv_obj_class_t boot_image_class={
    .base_class=&lv_img_class,
    .destructor_cb=boot_image_destructor,
    .instance_size=sizeof(boot_image_t)
};
static const lv_img_dsc_t *asset_image(const phase1_asset_t *asset, bool mirror)
{
    cached_image_t *slot = NULL;
    for (unsigned i=0;i<20;i++) {
        if (images[i].source==asset && images[i].mirror==mirror) return &images[i].image;
        if (!images[i].source && !slot) slot=&images[i];
    }
    if (!slot) return NULL;
    size_t count=(size_t)asset->width*asset->height;
    uint8_t *pixels=heap_caps_malloc(count*3, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if (!pixels) { ESP_LOGE(TAG,"No PSRAM for phase-one image"); return NULL; }
    size_t index=0;
    for (size_t offset=0; offset+5<=asset->size; offset+=5) {
        unsigned n=asset->rle[offset]|(asset->rle[offset+1]<<8);
        uint16_t rgb=asset->rle[offset+2]|(asset->rle[offset+3]<<8);
        uint8_t alpha=asset->rle[offset+4];
        if (!n || index+n>count) { free(pixels); return NULL; }
        lv_color_t color=lv_color_make((rgb>>8)&0xf8,(rgb>>3)&0xfc,(rgb<<3)&0xf8);
        for (unsigned j=0;j<n;j++,index++) {
            size_t pos=mirror?(index/asset->width*asset->width+asset->width-1-index%asset->width):index;
            memcpy(pixels+pos*3,&color,2); pixels[pos*3+2]=alpha;
        }
    }
    if (index!=count || asset->size%5) { free(pixels); return NULL; }
    slot->pixels=pixels; slot->source=asset; slot->mirror=mirror;
    slot->image=(lv_img_dsc_t){.header={.cf=LV_IMG_CF_TRUE_COLOR_ALPHA,.w=asset->width,.h=asset->height},.data_size=count*3,.data=pixels};
    return &slot->image;
}
static lv_obj_t *picture(lv_obj_t *parent,const phase1_asset_t *asset)
{
    lv_obj_t *image;
    if(boot_asset(asset)) {
        image=lv_obj_class_create_obj(&boot_image_class,parent);
        if(!image) return NULL;
        lv_obj_class_init_obj(image);
    } else image=lv_img_create(parent);
    const lv_img_dsc_t *data=asset_image(asset,false);
    if (data) {
        if(boot_asset(asset)) {
            cached_image_t *slot=NULL;
            for(unsigned i=0;i<20;i++) if(&images[i].image==data) {slot=&images[i];break;}
            /* Keep a saturated cache alive rather than wrap its owner count. */
            if(slot && slot->live_images<UINT16_MAX) {
                ((boot_image_t *)image)->owned_slot=slot;
                slot->live_images++;
                lv_img_set_src(image,data);
            }
        } else lv_img_set_src(image,data);
    }
    lv_obj_clear_flag(image,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(image,LV_OBJ_FLAG_GESTURE_BUBBLE);
    return image;
}
lv_obj_t *ui_phase1_asset_create(lv_obj_t *parent, const void *asset)
{
    return picture(parent,(const phase1_asset_t *)asset);
}
lv_obj_t *ui_phase1_x_create(lv_obj_t *parent,int size)
{
    lv_obj_t *image=picture(parent,&preview5_x);
    lv_img_set_zoom(image,size*256/preview5_x.width);
    return image;
}
static void plain_screen(lv_obj_t *screen)
{
    lv_obj_remove_style_all(screen);
    lv_obj_set_size(screen,360,360);
    lv_obj_set_style_bg_color(screen,lv_color_hex(0x000000),0);
    lv_obj_set_style_bg_opa(screen,LV_OPA_COVER,0);
    lv_obj_set_style_radius(screen,0,0);
    lv_obj_set_style_clip_corner(screen,false,0);
    lv_obj_clear_flag(screen,LV_OBJ_FLAG_SCROLLABLE);
}
static lv_obj_t *boot_top,*boot_x,*boot_arc,*boot_parent;
static lv_timer_t *boot_timer;
static uint32_t boot_started;
static bool boot_clock_started;
static void brand_frame(lv_timer_t *timer)
{
    uint32_t ms=lv_tick_elaps(boot_started);
    if (!boot_clock_started || !boot_parent || lv_scr_act()!=boot_parent) return;
    uint8_t top=255,x=255;
    if (ms>=650 && ms<4050) {
        uint32_t elapsed=ms-650;
        top=elapsed<448?(uint8_t)(elapsed*255/448):255;
        x=elapsed<512?0:elapsed<1216?(uint8_t)((elapsed-512)*255/704):255;
        uint16_t zoom=elapsed<2100?248+(8*elapsed/2100):256;
        if(boot_x) lv_img_set_zoom(boot_x,zoom);
        lv_arc_set_rotation(boot_arc,(elapsed*300/3200)%360);
        lv_obj_set_style_arc_opa(boot_arc,elapsed>2880?0:110,LV_PART_INDICATOR);
    } else lv_obj_set_style_arc_opa(boot_arc,0,LV_PART_INDICATOR);
    if (ms>3900) { uint8_t out=ms<4080?(4080-ms)*255/180:0; top=top*out/255; x=x*out/255; }
    if(boot_top) lv_obj_set_style_img_opa(boot_top,top,0);
    if(boot_x) lv_obj_set_style_img_opa(boot_x,x,0);
    if (ms>=4080) lv_timer_pause(timer);
}
static void brand_deleted(lv_event_t *e)
{
    if (lv_event_get_target(e)!=boot_parent) return;
    if (boot_timer) {lv_timer_del(boot_timer);boot_timer=NULL;}
    boot_parent=NULL;boot_top=NULL;boot_x=NULL;boot_arc=NULL;
}
void ui_phase1_brand_create(lv_obj_t *parent)
{
    plain_screen(parent);
    if (boot_timer) {lv_timer_del(boot_timer);boot_timer=NULL;}
    boot_parent=parent;boot_started=lv_tick_get();boot_clock_started=false;
    boot_x=picture(parent,&phase1_brand_x);
    boot_top=picture(parent,&phase1_brand_top);
    boot_arc=lv_arc_create(parent);lv_obj_remove_style_all(boot_arc);
    lv_obj_set_size(boot_arc,344,344);lv_obj_center(boot_arc);
    lv_arc_set_bg_angles(boot_arc,0,360);lv_arc_set_angles(boot_arc,20,95);
    lv_obj_set_style_arc_width(boot_arc,2,LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(boot_arc,lv_color_hex(0xff0188),LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(boot_arc,0,LV_PART_INDICATOR);
    lv_obj_clear_flag(boot_arc,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(parent,brand_deleted,LV_EVENT_DELETE,NULL);
    boot_timer=lv_timer_create(brand_frame,40,NULL);
    ESP_LOGI(TAG,"iN/X boot started");
}
bool ui_phase1_boot_finished(void) { if(!boot_clock_started){boot_started=lv_tick_get();boot_clock_started=true;} return lv_tick_elaps(boot_started)>=4080; }

static lv_obj_t *funny_screen,*rays[RAYS],*dots[7],*echoes[7];
static lv_timer_t *funny_timer;
static lv_point_t ray_points[RAYS][2];
static float target_angles[7];
static uint32_t funny_started,radar_cycle=UINT32_MAX;
static lv_obj_t *round_obj(lv_obj_t *parent,int size,uint32_t color)
{
    lv_obj_t *obj=lv_obj_create(parent);lv_obj_remove_style_all(obj);
    lv_obj_set_size(obj,size,size);lv_obj_set_style_radius(obj,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_color(obj,lv_color_hex(color),0);lv_obj_set_style_bg_opa(obj,255,0);
    lv_obj_clear_flag(obj,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);lv_obj_add_flag(obj,LV_OBJ_FLAG_GESTURE_BUBBLE);
    return obj;
}
static void reset_targets(void)
{
    for(int i=0;i<7;i++) {
        float angle=(esp_random()%36000)/100.f, radius=45+(esp_random()%109);
        target_angles[i]=angle;
        int x=180+radius*sinf(angle*PI_F/180),y=180-radius*cosf(angle*PI_F/180);
        lv_obj_set_pos(dots[i],x-7,y-7);lv_obj_set_pos(echoes[i],x-7,y-7);
        lv_obj_set_style_bg_opa(dots[i],6,0);
    }
}
static void radar_frame(float seconds)
{
    uint32_t cycle=(uint32_t)(seconds/5.6f);float local=fmodf(seconds,5.6f),theta=local/5.6f*360;
    if(cycle!=radar_cycle){radar_cycle=cycle;reset_targets();}
    for(int i=0;i<RAYS;i++) {
        float angle=(theta-58+i*58.f/(RAYS-1))*PI_F/180;
        ray_points[i][0]=(lv_point_t){180,180};ray_points[i][1]=(lv_point_t){180+180*sinf(angle),180-180*cosf(angle)};
        lv_line_set_points(rays[i],ray_points[i],2);
    }
    for(int i=0;i<7;i++) {
        float age=local-target_angles[i]/360.f*5.6f;
        float light=age<0?0:expf(-age*.87f)*(.73f+.27f*cosf(age*17+i));
        if(light<.04f)light=0;
        lv_obj_set_style_bg_opa(dots[i],(uint8_t)(light*255),0);
        uint8_t echo=age>=0&&age<.9f?(uint8_t)((1-age/.9f)*170):0;
        int size=14+(int)(fminf(1,fmaxf(0,age))*42);
        lv_obj_set_size(echoes[i],size,size);
        lv_obj_set_pos(echoes[i],lv_obj_get_x(dots[i])+7-size/2,lv_obj_get_y(dots[i])+7-size/2);
        lv_obj_set_style_border_opa(echoes[i],echo,0);
    }
}
static void funny_frame(lv_timer_t *timer)
{
    if(!funny_screen||lv_scr_act()!=funny_screen)return;
    float seconds=lv_tick_elaps(funny_started)/1000.f;
    radar_frame(seconds);
}
/* A filled marker uses geometry rather than a font chevron. Its centroid
 * remains at (180,180); no border or additional raster buffer is needed. */
static void radar_center_draw(lv_event_t *event)
{
    lv_area_t area;lv_obj_get_coords(lv_event_get_target(event),&area);
    lv_point_t points[3]={{area.x1+13,area.y1},{area.x1,area.y2},{area.x2,area.y2}};
    lv_draw_rect_dsc_t dsc;lv_draw_rect_dsc_init(&dsc);
    dsc.bg_color=lv_color_hex(0xff3b30);dsc.bg_opa=LV_OPA_COVER;
    dsc.border_width=0;dsc.shadow_width=0;
    lv_draw_triangle(lv_event_get_draw_ctx(event),&dsc,points);
}
static void render_funny(void)
{
    lv_obj_clean(funny_screen);funny_started=lv_tick_get();radar_cycle=UINT32_MAX;
    picture(funny_screen,&phase1_radar_bg);
    for(int i=0;i<RAYS;i++) {
        rays[i]=lv_line_create(funny_screen);lv_obj_set_pos(rays[i],0,0);
        lv_obj_set_style_line_color(rays[i],lv_color_hex(0xf5fff0),0);
        lv_obj_set_style_line_width(rays[i],i==RAYS-1?2:5,0);
        lv_obj_set_style_line_opa(rays[i],i==RAYS-1?230:8+i*3,0);
        lv_obj_clear_flag(rays[i],LV_OBJ_FLAG_CLICKABLE);lv_obj_add_flag(rays[i],LV_OBJ_FLAG_GESTURE_BUBBLE);
    }
    for(int i=0;i<7;i++){
        echoes[i]=round_obj(funny_screen,14,0);lv_obj_set_style_bg_opa(echoes[i],0,0);
        lv_obj_set_style_border_color(echoes[i],lv_color_hex(0xfff18a),0);lv_obj_set_style_border_width(echoes[i],1,0);
        dots[i]=round_obj(funny_screen,14,0xffe23f);
    }
    lv_obj_t *center=lv_obj_create(funny_screen);lv_obj_remove_style_all(center);
    lv_obj_set_size(center,27,25);lv_obj_set_pos(center,167,164);
    lv_obj_clear_flag(center,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(center,LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(center,radar_center_draw,LV_EVENT_DRAW_MAIN,NULL);
    lv_obj_t *search=lv_label_create(funny_screen);lv_label_set_text(search,"サーチ中…");
    lv_obj_set_style_text_font(search,gv5_text_font(GV5_JA,18),0);
    lv_obj_set_style_text_color(search,lv_color_hex(0xe8efcd),0);
    lv_obj_align(search,LV_ALIGN_TOP_MID,0,44);
    lv_obj_clear_flag(search,LV_OBJ_FLAG_CLICKABLE);
    radar_frame(0);

}
/* LVGL's default gesture detector resets its sum below 3 px/sample.
 * Track total travel on this screen so deliberate slow swipes work too. */
static lv_point_t funny_press_start;
static bool funny_press_active, funny_swipe_handled;
static void funny_navigate(lv_dir_t dir, lv_indev_t *input)
{
    funny_swipe_handled=true;funny_press_active=false;
    lv_indev_wait_release(input);
    ESP_LOGI(TAG,"Funny swipe mode=Radar direction=%d",(int)dir);
    if(dir==LV_DIR_TOP||dir==LV_DIR_BOTTOM){render_funny();}
    /* Radar is the first page: finger-right wraps to the last settings menu;
     * finger-left advances to native preset 0. */
    else if(dir==LV_DIR_RIGHT)ui_v5_about_open();
    else if(dir==LV_DIR_LEFT)ui_gauge_v5_open(0);
}
static void funny_event(lv_event_t *event)
{
    lv_event_code_t code=lv_event_get_code(event);
    if(code==LV_EVENT_SCREEN_LOADED){funny_started=lv_tick_get();lv_timer_resume(funny_timer);}
    else if(code==LV_EVENT_SCREEN_UNLOADED){
        lv_timer_pause(funny_timer);funny_press_active=false;funny_swipe_handled=true;
    }
    else if(code==LV_EVENT_DELETE){if(funny_timer)lv_timer_del(funny_timer);funny_timer=NULL;funny_screen=NULL;}
    else if(code==LV_EVENT_PRESSED){
        lv_indev_t *input=lv_indev_get_act();if(!input)return;
        lv_indev_get_point(input,&funny_press_start);
        funny_press_active=true;funny_swipe_handled=false;
    }
    else if(code==LV_EVENT_PRESS_LOST)funny_press_active=false;
    else if(code==LV_EVENT_PRESSING&&funny_press_active&&!funny_swipe_handled){
        lv_indev_t *input=lv_indev_get_act();if(!input)return;
        lv_point_t point;lv_indev_get_point(input,&point);
        int dx=point.x-funny_press_start.x,dy=point.y-funny_press_start.y;
        int ax=abs(dx),ay=abs(dy);
        /* 28 px dead zone plus dominant axis: taps/jitter do not turn pages. */
        if(ax>=28&&ax*4>=ay*5)funny_navigate(dx>0?LV_DIR_RIGHT:LV_DIR_LEFT,input);
        else if(ay>=28&&ay*4>=ax*5)funny_navigate(dy>0?LV_DIR_BOTTOM:LV_DIR_TOP,input);
    }
    else if(code==LV_EVENT_GESTURE&&!funny_swipe_handled){
        lv_indev_t *input=lv_indev_get_act();if(!input)return;
        lv_dir_t dir=lv_indev_get_gesture_dir(input);
        if(dir!=LV_DIR_NONE)funny_navigate(dir,input);
    }
}
static void funny_open(bool immediate)
{
    /* Radar only; removed Digivice remains in prior source/archive assets. */
    if(!funny_screen){
        funny_screen=lv_obj_create(NULL);plain_screen(funny_screen);render_funny();
        funny_timer=lv_timer_create(funny_frame,40,NULL);lv_timer_pause(funny_timer);
        lv_obj_add_event_cb(funny_screen,funny_event,LV_EVENT_ALL,NULL);
    }
    /* Boot releases Logo immediately: switch act_scr before it is deleted.
     * Ordinary navigation retains its fade and keeps the previous page alive. */
    if(immediate)lv_scr_load(funny_screen);
    else lv_scr_load_anim(funny_screen,LV_SCR_LOAD_ANIM_FADE_ON,120,0,false);
    ESP_LOGI(TAG,"Funny mode Radar (simulation)");
}

void ui_phase1_funny_open(void){funny_open(false);}
void ui_phase1_funny_open_boot(void){funny_open(true);}

bool ui_phase1_funny_active(void){return funny_screen && lv_scr_act()==funny_screen;}
