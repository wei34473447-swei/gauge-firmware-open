#include "ui_v5_imu.h"
#include "ui_gauge_v5.h"
#include "ui_v5_about.h"
#include "ui_phase1.h"
#include "gauge_v5_fonts.h"
#include "app_obd_dsp/gauge_imu.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static lv_obj_t *imu_screen;
static lv_timer_t *imu_timer;
static lv_obj_t *imu_mount_label;
static gauge_imu_frame_t imu_frame;
static bool imu_settings, imu_press, imu_navigated;
static lv_point_t imu_start;
/* Session range changes only through the explicit RANGE button. */
static float imu_range = 2.f;
static gauge_imu_frame_t imu_last_valid;
static uint32_t imu_hold_since;
static bool imu_holding;
#define IMU_RADIUS 174
#define IMU_HOLD_MS 200u
typedef struct {
    lv_point_t history[68],dot;
    char values[4][20],scale[20];
    uint8_t count,dot_visible;
} imu_paint_key_t;
_Static_assert(sizeof(imu_paint_key_t)<400,"bounded exact IMU paint key");
static imu_paint_key_t imu_drawn_key;
static bool imu_drawn_valid,imu_paint_pending;
static const char *imu_words[][4] = {
    {"陀螺仪", "IMU", "IMU", "陀螺儀"},
    {"归零", "ZERO", "ゼロ", "歸零"},
    {"清除轨迹", "CLEAR HISTORY", "履歴をクリア", "清除軌跡"},
    {"返回", "BACK", "戻る", "返回"}
};
static const char *iw(unsigned n) { return imu_words[n][ui_gauge_v5_language()]; }
enum {
    IMU_FEEDBACK_NONE, IMU_FEEDBACK_BUSY, IMU_FEEDBACK_NOT_READY,
    IMU_FEEDBACK_REST_REQUIRED, IMU_FEEDBACK_CAL_APPLIED, IMU_FEEDBACK_FAILED
};
static uint8_t imu_feedback;
/* Existing font subsets only; these immutable messages need no label object.
 * NOT_READY is deliberately a condition, not a guessed sensor diagnosis. */
static const char *const imu_feedback_words[][4] = {
    {"使用中，请再试", "Busy: retry", "BUSY・再試行", "使用中，請再試"},
    {"当前不可，请再试", "Not ready: retry", "今は不可・再試行", "當前不可，請再試"},
    {"请停车静止再试", "Park still, then retry", "静止して再試行", "請停車靜止再試"},
    {"CAL 已设置", "CAL applied", "CAL 済み", "CAL 已設置"},
    {"设置失败，请再试", "Failed: retry", "失敗・再試行", "設置失敗，請再試"}
};
static void imu_command_feedback(unsigned action, esp_err_t err) {
    imu_feedback=err==ESP_OK ? (action==8?IMU_FEEDBACK_CAL_APPLIED:IMU_FEEDBACK_NONE) :
        err==ESP_ERR_TIMEOUT?IMU_FEEDBACK_BUSY:
        err==ESP_ERR_INVALID_STATE?(action==8||action==7?IMU_FEEDBACK_REST_REQUIRED:IMU_FEEDBACK_NOT_READY):
        IMU_FEEDBACK_FAILED;
    lv_obj_invalidate(imu_screen);
}
static void imu_capture(void) {
    gauge_imu_frame_t next;gauge_imu_get(&next);
    int64_t now=esp_timer_get_time();
    if(next.valid&&next.projection_ready&&next.gravity_tracking&&next.gravity_quiet_validated&&isfinite(next.plane_g[0])&&isfinite(next.plane_g[1])) {
        imu_last_valid=next;imu_holding=false;
    } else if(next.error==ESP_ERR_TIMEOUT&&imu_last_valid.timestamp_us>0) {
        int64_t age=now-imu_last_valid.timestamp_us;
        if(!imu_holding){imu_holding=true;imu_hold_since=lv_tick_get();}
        /* Hold only a real bounded-age reading. Original 250ms TTL remains;
         * no I/O, fabricated samples, peak additions or silent stale extension. */
        if(age>=0&&age<=(int64_t)GAUGE_IMU_STALE_MS*1000&&
           lv_tick_elaps(imu_hold_since)<=IMU_HOLD_MS) {
            gm_peak_snapshot_t latest=next.timestamp_us?next.peak:imu_frame.peak;
            next=imu_last_valid;next.age_ms=(uint32_t)(age/1000);
            next.peak=latest;next.stationary=false;
        }
    } else imu_holding=false;
    if(!next.timestamp_us&&next.error==ESP_ERR_TIMEOUT) {
        next.peak=imu_frame.peak;next.calibrated=imu_frame.calibrated;
        next.projection_ready=imu_frame.projection_ready;
        next.reverse_lr=imu_frame.reverse_lr;next.reverse_fb=imu_frame.reverse_fb;
        next.mount_axis=imu_frame.mount_axis;next.effective_forward_axis=imu_frame.effective_forward_axis;
    }
    imu_frame=next;
}
static void imu_text(lv_draw_ctx_t *ctx, const char *s, const lv_font_t *f, int x, int y,
                     int width, unsigned color) {
    lv_draw_label_dsc_t d; lv_draw_label_dsc_init(&d);
    d.font = f; d.color = lv_color_hex(color); d.align = LV_TEXT_ALIGN_CENTER;
    d.flag = LV_TEXT_FLAG_EXPAND;
    lv_area_t a = {x-width/2, y-f->line_height/2, x+width/2-1, y+(f->line_height+1)/2-1};
    lv_draw_label(ctx, &d, &a, s, NULL);
}
static void imu_line(lv_draw_ctx_t *ctx, lv_point_t a, lv_point_t b, unsigned width,
                     unsigned color, unsigned opacity) {
    lv_draw_line_dsc_t d; lv_draw_line_dsc_init(&d);
    d.color=lv_color_hex(color); d.width=width; d.opa=opacity; d.round_start=true; d.round_end=true;
    lv_draw_line(ctx,&d,&a,&b);
}
static lv_point_t imu_pixel(gm_peak_point_t p) {
    return (lv_point_t){(lv_coord_t)lroundf(180+p.x*IMU_RADIUS/imu_range),
                        (lv_coord_t)lroundf(180-p.y*IMU_RADIUS/imu_range)};
}
/* Same fixed directions as the conservative trip envelope. Const data stays
 * in flash; clipping needs only 64 float limits (256 bytes), no heap/scratch
 * polygon. A 64-sided circle has <0.21px radial error at radius 174. */
static const gm_peak_point_t imu_normals[64] = {
    {1.f, 0.f},
    {0.995184727f, 0.0980171403f},
    {0.98078528f, 0.195090322f},
    {0.956940336f, 0.290284677f},
    {0.923879533f, 0.382683432f},
    {0.881921264f, 0.471396737f},
    {0.831469612f, 0.555570233f},
    {0.773010453f, 0.634393284f},
    {0.707106781f, 0.707106781f},
    {0.634393284f, 0.773010453f},
    {0.555570233f, 0.831469612f},
    {0.471396737f, 0.881921264f},
    {0.382683432f, 0.923879533f},
    {0.290284677f, 0.956940336f},
    {0.195090322f, 0.98078528f},
    {0.0980171403f, 0.995184727f},
    {0.f, 1.f},
    {-0.0980171403f, 0.995184727f},
    {-0.195090322f, 0.98078528f},
    {-0.290284677f, 0.956940336f},
    {-0.382683432f, 0.923879533f},
    {-0.471396737f, 0.881921264f},
    {-0.555570233f, 0.831469612f},
    {-0.634393284f, 0.773010453f},
    {-0.707106781f, 0.707106781f},
    {-0.773010453f, 0.634393284f},
    {-0.831469612f, 0.555570233f},
    {-0.881921264f, 0.471396737f},
    {-0.923879533f, 0.382683432f},
    {-0.956940336f, 0.290284677f},
    {-0.98078528f, 0.195090322f},
    {-0.995184727f, 0.0980171403f},
    {-1.f, 0.f},
    {-0.995184727f, -0.0980171403f},
    {-0.98078528f, -0.195090322f},
    {-0.956940336f, -0.290284677f},
    {-0.923879533f, -0.382683432f},
    {-0.881921264f, -0.471396737f},
    {-0.831469612f, -0.555570233f},
    {-0.773010453f, -0.634393284f},
    {-0.707106781f, -0.707106781f},
    {-0.634393284f, -0.773010453f},
    {-0.555570233f, -0.831469612f},
    {-0.471396737f, -0.881921264f},
    {-0.382683432f, -0.923879533f},
    {-0.290284677f, -0.956940336f},
    {-0.195090322f, -0.98078528f},
    {-0.0980171403f, -0.995184727f},
    {0.f, -1.f},
    {0.0980171403f, -0.995184727f},
    {0.195090322f, -0.98078528f},
    {0.290284677f, -0.956940336f},
    {0.382683432f, -0.923879533f},
    {0.471396737f, -0.881921264f},
    {0.555570233f, -0.831469612f},
    {0.634393284f, -0.773010453f},
    {0.707106781f, -0.707106781f},
    {0.773010453f, -0.634393284f},
    {0.831469612f, -0.555570233f},
    {0.881921264f, -0.471396737f},
    {0.923879533f, -0.382683432f},
    {0.956940336f, -0.290284677f},
    {0.98078528f, -0.195090322f},
    {0.995184727f, -0.0980171403f},
};
static int32_t imu_cross(lv_point_t a,lv_point_t b,lv_point_t c) {
    return (int32_t)(b.x-a.x)*(c.y-a.y)-(int32_t)(b.y-a.y)*(c.x-a.x);
}
static unsigned imu_pixel_hull(lv_point_t *points,unsigned count) {
    /* Pixel rounding can introduce 1px reflex corners. Andrew's monotone
     * hull retains all rounded points and removes those corners, including
     * duplicate/collinear facets. One bounded 272-byte temporary array. */
    for(unsigned i=1;i<count;i++) {
        lv_point_t p=points[i];unsigned j=i;
        while(j&&(points[j-1].x>p.x||(points[j-1].x==p.x&&points[j-1].y>p.y))) {
            points[j]=points[j-1];--j;
        }
        points[j]=p;
    }
    unsigned unique=0;
    for(unsigned i=0;i<count;i++)
        if(!unique||points[i].x!=points[unique-1].x||points[i].y!=points[unique-1].y)points[unique++]=points[i];
    if(unique<2)return unique;
    lv_point_t hull[68];unsigned n=0;
    for(unsigned i=0;i<unique;i++) {
        while(n>=2&&imu_cross(hull[n-2],hull[n-1],points[i])<=0)--n;
        hull[n++]=points[i];
    }
    unsigned lower=n;
    for(unsigned i=unique-1;i>0;--i) {
        while(n>lower&&imu_cross(hull[n-2],hull[n-1],points[i-1])<=0)--n;
        hull[n++]=points[i-1];
    }
    --n;memcpy(points,hull,n*sizeof(*points));return n;
}
static unsigned imu_visible_history(lv_point_t *out) {
    const gm_peak_snapshot_t *peak=&imu_frame.peak;
    if(peak->vertex_count<2||peak->vertex_count>GM_PEAK_MAX_VERTICES||peak->radius_g<=.015f)return 0;
    float limits[64];
    const float circle_limit=imu_range*.998795456f; /* cos(pi/64), inscribed circle */
    for(unsigned i=0;i<64;i++) {
        float support=0;
        for(unsigned v=0;v<peak->vertex_count;v++) {
            float projection=imu_normals[i].x*peak->vertices[v].x+imu_normals[i].y*peak->vertices[v].y;
            if(!isfinite(projection))return 0;
            support=fmaxf(support,projection);
        }
        limits[i]=fminf(support,circle_limit*sqrtf(imu_normals[i].x*imu_normals[i].x+imu_normals[i].y*imu_normals[i].y));
    }
    unsigned count=0;
    /* Intersect each facet line with every other half-plane. Inactive facets
     * have an empty t interval; do not assume neighbouring normals are active.
     * K includes the origin. Increasing historical supports cannot shrink
     * K intersect fixed P, unlike independently clamping hull vertices/rays. */
    for(unsigned i=0;i<64;i++) {
        gm_peak_point_t n=imu_normals[i],t={-n.y,n.x};
        float norm2=n.x*n.x+n.y*n.y;
        gm_peak_point_t base={n.x*limits[i]/norm2,n.y*limits[i]/norm2};
        float lo=-INFINITY,hi=INFINITY;
        bool empty=false;
        for(unsigned j=0;j<64;j++)if(j!=i) {
            float divisor=imu_normals[j].x*t.x+imu_normals[j].y*t.y;
            float slack=limits[j]-imu_normals[j].x*base.x-imu_normals[j].y*base.y;
            if(divisor>1e-7f)hi=fminf(hi,slack/divisor);
            else if(divisor<-1e-7f)lo=fmaxf(lo,slack/divisor);
            else if(slack<-1e-6f){empty=true;break;}
        }
        if(empty||!isfinite(hi)||hi<lo-1e-6f)continue;
        lv_point_t p=imu_pixel((gm_peak_point_t){base.x+hi*t.x,base.y+hi*t.y});
        if(!count||p.x!=out[count-1].x||p.y!=out[count-1].y)out[count++]=p;
    }
    if(count>1&&out[count-1].x==out[0].x&&out[count-1].y==out[0].y)--count;
    return imu_pixel_hull(out,count);
}
static lv_point_t imu_live_pixel(void) {
    gm_peak_point_t p={imu_frame.plane_g[0],imu_frame.plane_g[1]};
    float radius=hypotf(p.x,p.y),visible=imu_range*(IMU_RADIUS-8.f)/IMU_RADIUS;
    if(radius>visible){float ratio=visible/radius;p.x*=ratio;p.y*=ratio;}
    return imu_pixel(p); /* Entire 7px dot + border stays visible; numbers stay real. */
}
static bool imu_live(void) {
    int64_t age=esp_timer_get_time()-imu_frame.timestamp_us;
    return imu_frame.valid&&imu_frame.projection_ready&&imu_frame.gravity_tracking&&imu_frame.gravity_quiet_validated&&isfinite(imu_frame.plane_g[0])&&isfinite(imu_frame.plane_g[1])&&
           imu_frame.timestamp_us>0&&age>=0&&age<=(int64_t)GAUGE_IMU_STALE_MS*1000;
}
static void imu_paint_key(imu_paint_key_t *key) {
    memset(key,0,sizeof(*key));bool live=imu_live();
    key->count=(uint8_t)imu_visible_history(key->history);
    key->dot_visible=live||!imu_frame.projection_ready||!imu_frame.gravity_quiet_validated;
    if(key->dot_visible)key->dot=live?imu_live_pixel():(lv_point_t){180,180};
    float values[4]={fmaxf(0,imu_frame.plane_g[1]),fmaxf(0,-imu_frame.plane_g[1]),fmaxf(0,-imu_frame.plane_g[0]),fmaxf(0,imu_frame.plane_g[0])};
    for(unsigned i=0;i<4;i++)if(live)snprintf(key->values[i],sizeof(key->values[i]),"%.2f",(double)values[i]);
        else snprintf(key->values[i],sizeof(key->values[i]),"--");
    snprintf(key->scale,sizeof(key->scale),"%.1fg%s",(double)imu_range,
             live&&hypotf(imu_frame.plane_g[0],imu_frame.plane_g[1])>imu_range?"+":"");
}
static void imu_draw(lv_event_t *e) {
    if (imu_settings) {
        if(imu_feedback)imu_text(lv_event_get_draw_ctx(e),
            imu_feedback_words[imu_feedback-1][ui_gauge_v5_language()],
            gv5_text_font(ui_gauge_v5_language(),18),180,45,210,0xF5F0F4);
        return;
    }
    lv_draw_ctx_t *ctx=lv_event_get_draw_ctx(e);
    lv_draw_arc_dsc_t arc; lv_draw_arc_dsc_init(&arc);
    arc.width=2; arc.color=lv_color_hex(0xB7A6D8); arc.opa=210;
    lv_point_t center={180,180};
    lv_draw_arc(ctx,&arc,&center,IMU_RADIUS,0,360);
    lv_draw_arc(ctx,&arc,&center,IMU_RADIUS/2,0,360);
    imu_line(ctx,(lv_point_t){80,180},(lv_point_t){280,180},1,0xB7A6D8,160);
    imu_line(ctx,(lv_point_t){180,80},(lv_point_t){180,280},1,0xB7A6D8,160);
    /* Render exactly the same bounded clipped geometry/text used by the key. */
    /* LVGL may draw one screen in several buffer strips. Prepare once and
     * reuse the same exact key for every strip; geometry is not re-clipped. */
    if(!imu_drawn_valid){imu_paint_key(&imu_drawn_key);imu_drawn_valid=true;}
    const imu_paint_key_t *key=&imu_drawn_key;
    for(unsigned i=0;i<key->count;i++)
        imu_line(ctx,key->history[i],key->history[(i+1)%key->count],2,0xFF0188,LV_OPA_COVER);
    if(key->dot_visible) {
        lv_point_t p=key->dot;
        lv_draw_rect_dsc_t dot;lv_draw_rect_dsc_init(&dot);
        dot.radius=LV_RADIUS_CIRCLE;dot.bg_color=lv_color_hex(0xFF0188);
        dot.bg_opa=LV_OPA_COVER;dot.border_color=lv_color_white();
        dot.border_width=1;dot.border_opa=LV_OPA_COVER;
        lv_area_t a={p.x-7,p.y-7,p.x+7,p.y+7};lv_draw_rect(ctx,&dot,&a);
    }
    /* Values at the four axis ends show positive magnitude per direction;
     * mapped +forward above, -forward below, -right left, +right right.
     * Unmapped center is a reference only: axis text is --, never fake zero. */
    const int xs[4]={180,180,31,329},ys[4]={31,329,180,180};
    for(unsigned i=0;i<4;i++) {
        const char *val=key->values[i];
        const lv_font_t *nf=gv5_number_font(24);
        if(lv_txt_get_width(val,(uint32_t)strlen(val),nf,0,LV_TEXT_FLAG_NONE)>56)nf=gv5_number_font(18);
        if(lv_txt_get_width(val,(uint32_t)strlen(val),nf,0,LV_TEXT_FLAG_NONE)>56)nf=gv5_number_font(14);
        imu_text(ctx,val,nf,xs[i],ys[i],60,0xF5F0F4);
    }
    imu_text(ctx,key->scale,gv5_text_font(GV5_EN,18),180,110,90,0xB7A6D8);
    /* Cache the exact drawn pixels/text, not timestamps or sample counts. */
    imu_paint_pending=false;


}
static void imu_render(void);
static void imu_mount_text(char *text,size_t capacity) {
    const char *axis=imu_frame.mount_axis==GAUGE_IMU_MOUNT_X?"X":
                     imu_frame.mount_axis==GAUGE_IMU_MOUNT_Y?"Y":"Z";
    if(imu_frame.mount_axis==GAUGE_IMU_MOUNT_AUTO) {
        const char *effective=imu_frame.effective_forward_axis==GAUGE_IMU_MOUNT_X?"-X":
                              imu_frame.effective_forward_axis==GAUGE_IMU_MOUNT_Y?"-Y":
                              imu_frame.effective_forward_axis==GAUGE_IMU_MOUNT_Z?"-Z":"";
        snprintf(text,capacity,"FWD: AUTO%s",effective);
    } else snprintf(text,capacity,"FWD: %s",axis);
}
static void imu_refresh_mount_label(void) {
    if(imu_settings&&imu_mount_label) {
        char text[24];imu_mount_text(text,sizeof text);
        if(strcmp(lv_label_get_text(imu_mount_label),text))lv_label_set_text(imu_mount_label,text);
    }
}
static void imu_action(lv_event_t *e) {
    unsigned a=(unsigned)(uintptr_t)lv_event_get_user_data(e);
    /* Keep a rejected action visible until the next explicit action or page
     * entry; never retry a command or allocate UI state in the sample timer. */
    if(imu_feedback){imu_feedback=IMU_FEEDBACK_NONE;lv_obj_invalidate(imu_screen);}
    if(a==1) {
        gauge_imu_frame_t accepted_frame;
        esp_err_t err=gauge_imu_zero_capture(&accepted_frame);
        if(err==ESP_OK) {
            memset(&imu_last_valid,0,sizeof imu_last_valid);imu_holding=false;
            imu_frame=accepted_frame;
            /* Snapshot the acknowledged command before the next real sample.
             * LVGL may first draw after that sample; show this accepted cache
             * for one complete frame, then normal ticks resume real linear G. */
            imu_paint_key_t accepted;imu_paint_key(&accepted);
            imu_settings=false;imu_render();
            imu_drawn_key=accepted;imu_drawn_valid=true;
        }
        else {ESP_LOGI("gauge_imu_ui","zero rejected: %d",(int)err);imu_command_feedback(a,err);}
    }
    else if(a==2) {gauge_imu_clear_peak(); memset(&imu_last_valid,0,sizeof imu_last_valid);imu_holding=false;imu_capture();}
    else if(a==4||a==5) {
        bool lr=imu_frame.reverse_lr,fb=imu_frame.reverse_fb;
        if(a==4)lr=!lr;else fb=!fb;
        esp_err_t err=gauge_imu_set_orientation(lr,fb);
        if(err==ESP_OK) {
            memset(&imu_last_valid,0,sizeof imu_last_valid);imu_holding=false;
            imu_capture();imu_render();
        } else {ESP_LOGI("gauge_imu_ui","orientation change rejected: %d",(int)err);imu_command_feedback(a,err);}
    }
    else if(a==8) {
        /* Explicit parked quiet alignment recovers attitude after lost tracking.
         * It is separate from immediate moving ZERO and never changes range. */
        esp_err_t err=gauge_imu_align_at_rest();
        if(err==ESP_OK) {
            memset(&imu_last_valid,0,sizeof imu_last_valid);imu_holding=false;
            imu_capture();imu_drawn_valid=false;imu_refresh_mount_label();
        } else ESP_LOGI("gauge_imu_ui","park cal rejected: %d",(int)err);
        imu_command_feedback(a,err);
    }
    else if(a==7) {
        unsigned current=(unsigned)imu_frame.mount_axis;
        if(current>GAUGE_IMU_MOUNT_Z)current=GAUGE_IMU_MOUNT_AUTO;
        esp_err_t err=ESP_ERR_INVALID_STATE;
        /* Skip only an unavailable parallel axis. A lock timeout or any other
         * failure stops immediately; no I/O/quiet wait or automatic retry. */
        for(unsigned step=1;step<=4;step++) {
            gauge_imu_mount_axis_t axis=(gauge_imu_mount_axis_t)((current+step)%4);
            err=gauge_imu_set_mount_axis(axis);
            if(err==ESP_OK) {
                memset(&imu_last_valid,0,sizeof imu_last_valid);imu_holding=false;
                imu_capture();imu_drawn_valid=false;imu_refresh_mount_label();break;
            }
            if(err!=ESP_ERR_INVALID_STATE)break;
        }
        if(err!=ESP_OK){ESP_LOGI("gauge_imu_ui","mount change rejected: %d",(int)err);imu_command_feedback(a,err);}
    }
    else if(a==6) {
        imu_range=imu_range==.5f?1.f:imu_range==1.f?2.f:imu_range==2.f?4.f:.5f;
        char label[24];snprintf(label,sizeof label,"RANGE: %.1fg",(double)imu_range);
        lv_label_set_text(lv_obj_get_child(lv_event_get_target(e),0),label);
        imu_drawn_valid=false; /* Preserve scroll position and selected session range. */
    }
    else {imu_settings=false; imu_render();}
}
static void imu_button(lv_obj_t *parent,unsigned action,int y,int width) {
    lv_obj_update_layout(parent);
    lv_obj_t *b=lv_btn_create(parent); lv_obj_set_size(b,width,44); lv_obj_set_pos(b,(lv_obj_get_width(parent)-width)/2,y);
    lv_obj_set_style_bg_color(b,lv_color_hex(0x24202D),0);lv_obj_set_style_radius(b,12,0);
    lv_obj_set_ext_click_area(b,6); lv_obj_add_event_cb(b,imu_action,LV_EVENT_CLICKED,(void*)(uintptr_t)action);
    char range_label[24],mount_label[24];snprintf(range_label,sizeof range_label,"RANGE: %.1fg",(double)imu_range);
    imu_mount_text(mount_label,sizeof mount_label);
    lv_obj_t *l=lv_label_create(b);lv_label_set_text(l,action==8?"PARK CAL":action==7?mount_label:action==6?range_label:action==4?(imu_frame.reverse_lr?"L/R: -":"L/R: +"):
        action==5?(imu_frame.reverse_fb?"F/B: -":"F/B: +"):iw(action==3?3:action));
    if(action==7)imu_mount_label=l;
    lv_obj_set_style_text_font(l,gv5_text_font(action>=4?GV5_EN:ui_gauge_v5_language(),22),0);lv_obj_center(l);
}
static int imu_header_ink_middle(const char *s,const lv_font_t *font) {
    int low=32767,high=-32768;uint32_t at=0;
    while(s[at]) {
        uint32_t cp=_lv_txt_encoded_next(s,&at);lv_font_glyph_dsc_t d;
        if(!lv_font_get_glyph_dsc(font,&d,cp,0))continue;
        const uint8_t *bits=lv_font_get_glyph_bitmap(font,cp);
        if(!bits||!d.bpp||d.bpp>8||8%d.bpp)continue;
        int top=font->line_height-font->base_line-d.box_h-d.ofs_y;
        for(unsigned y=0;y<d.box_h;y++)for(unsigned x=0;x<d.box_w;x++) {
            unsigned pixel=y*d.box_w+x,perbyte=8/d.bpp;
            if((bits[pixel/perbyte]>>(8-d.bpp-(pixel%perbyte)*d.bpp))&((1u<<d.bpp)-1u)) {
                if(top+(int)y<low)low=top+(int)y;
                if(top+(int)y>high)high=top+(int)y;
            }
        }
    }
    return low<=high?(low+high)/2:font->line_height/2;
}
static void imu_render(void) {
    imu_drawn_valid=false;imu_paint_pending=true;
    imu_mount_label=NULL;lv_obj_clean(imu_screen);
    int header_y=imu_settings?20:70;
    const char *title_text=imu_settings?iw(0):"G-FORCE";
    const lv_font_t *header_font=gv5_text_font(imu_settings?ui_gauge_v5_language():GV5_EN,18);
    int title_width=lv_txt_get_width(title_text,(uint32_t)strlen(title_text),header_font,0,LV_TEXT_FLAG_NONE);
    int x_size=imu_settings?18:32;
    int x_distance=title_width/2+8+x_size/2;
    for(unsigned side=0;side<(imu_settings?1u:2u);side++){
        lv_obj_t *x=ui_phase1_x_create(imu_screen,x_size);
        if(x){lv_obj_update_layout(x);int height=lv_obj_get_height(x),width=lv_obj_get_width(x);
            int visual_height=width>0?height*x_size/width:height;
            int offset=imu_settings?-41:(side?x_distance:-x_distance);
            lv_obj_align(x,LV_ALIGN_TOP_MID,offset,header_y-visual_height/2+(visual_height-height)/2);}
    }
    lv_obj_t *title=lv_label_create(imu_screen);lv_label_set_text(title,title_text);
    lv_obj_set_style_text_font(title,header_font,0);
    lv_obj_set_style_text_color(title,lv_color_hex(0xF5F0F4),0);
    lv_obj_align(title,LV_ALIGN_TOP_MID,imu_settings?16:0,header_y-imu_header_ink_middle(title_text,header_font));
    if(imu_settings) {
        lv_obj_t *scroll=lv_obj_create(imu_screen);lv_obj_remove_style_all(scroll);
        lv_obj_set_size(scroll,250,195);lv_obj_set_pos(scroll,55,68);
        lv_obj_set_scroll_dir(scroll,LV_DIR_VER);lv_obj_set_scrollbar_mode(scroll,LV_SCROLLBAR_MODE_AUTO);
        imu_button(scroll,1,2,246);imu_button(scroll,8,58,246);imu_button(scroll,2,114,246);
        imu_button(scroll,7,170,246);imu_button(scroll,4,226,246);
        imu_button(scroll,5,282,246);imu_button(scroll,6,338,246);
        imu_button(imu_screen,3,286,140);
    }

    lv_obj_invalidate(imu_screen);
}
static void imu_navigate(lv_dir_t d,lv_indev_t *in) {
    imu_navigated=true;if(in)lv_indev_wait_release(in);
    if(imu_settings)return;
    if(d==LV_DIR_LEFT)ui_v5_about_open();
    else if(d==LV_DIR_RIGHT)ui_gauge_v5_open(6);
    else {imu_settings=true;imu_render();}
}
static void imu_events(lv_event_t *e) {
    lv_event_code_t c=lv_event_get_code(e);lv_indev_t *in=lv_indev_get_act();
    if(c==LV_EVENT_SCREEN_UNLOADED) {lv_timer_pause(imu_timer);imu_press=false;}
    else if(c==LV_EVENT_SCREEN_LOADED)lv_timer_resume(imu_timer);
    else if(imu_settings)return;
    else if(c==LV_EVENT_PRESSED) {imu_press=true;imu_navigated=false;if(in)lv_indev_get_point(in,&imu_start);}
    else if(c==LV_EVENT_RELEASED||c==LV_EVENT_PRESS_LOST)imu_press=false;
    else if(c==LV_EVENT_LONG_PRESSED&&!imu_navigated)imu_navigate(LV_DIR_TOP,in);
    else if(c==LV_EVENT_PRESSING&&imu_press&&!imu_navigated&&in) {
        lv_point_t p;lv_indev_get_point(in,&p);int dx=p.x-imu_start.x,dy=p.y-imu_start.y;
        if(abs(dx)>=28&&abs(dx)*4>=abs(dy)*5)imu_navigate(dx>0?LV_DIR_RIGHT:LV_DIR_LEFT,in);
        else if(abs(dy)>=28&&abs(dy)*4>=abs(dx)*5)imu_navigate(LV_DIR_TOP,in);
    } else if(c==LV_EVENT_GESTURE&&!imu_navigated&&in)imu_navigate(lv_indev_get_gesture_dir(in),in);
}
static void imu_tick(lv_timer_t *t) {
    (void)t;if(!ui_v5_imu_active())return;imu_capture();imu_refresh_mount_label();
    if(!imu_settings&&!imu_paint_pending){
        imu_paint_key_t key;imu_paint_key(&key);
        if(!imu_drawn_valid||memcmp(&key,&imu_drawn_key,sizeof(key))){
            imu_drawn_key=key;imu_drawn_valid=true;
            imu_paint_pending=true;lv_obj_invalidate(imu_screen);
        }
    }
}
bool ui_v5_imu_active(void) {return imu_screen&&lv_scr_act()==imu_screen;}
void ui_v5_imu_open(void) {
    if(!imu_screen) {imu_screen=lv_obj_create(NULL);lv_obj_remove_style_all(imu_screen);
        lv_obj_set_size(imu_screen,360,360);lv_obj_set_style_bg_color(imu_screen,lv_color_black(),0);
        lv_obj_set_style_bg_opa(imu_screen,LV_OPA_COVER,0);lv_obj_clear_flag(imu_screen,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(imu_screen,imu_draw,LV_EVENT_DRAW_MAIN,NULL);
        lv_obj_add_event_cb(imu_screen,imu_events,LV_EVENT_ALL,NULL);imu_timer=lv_timer_create(imu_tick,50,NULL);}
    imu_feedback=IMU_FEEDBACK_NONE;imu_settings=false;imu_capture();imu_render();lv_scr_load(imu_screen);
    ESP_LOGI("gauge_imu_ui","Independent IMU gauge opened");
}
