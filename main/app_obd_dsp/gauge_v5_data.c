#include "gauge_v5_data.h"
#include "gauge_fuel.h"
#include "bsp_obd_dsp/elm327_ble_client.h"
#include "bsp_obd_dsp/espnow_link.h"
#include "bsp_obd_dsp/nvs_storage.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include <limits.h>

#define META(k,zh,en,ja,tw,zs,es,js,ts,u,lo,hi,p,impl,cont,ttl) {k,{zh,en,ja,tw},{zs,es,js,ts},u,lo,hi,p,impl,cont,cont,ttl}
static const gv5_meta_t s_meta[GV5_FIELD_COUNT] = {
 [GV5_NONE]=META("none","无","NONE","なし","無","无","NONE","なし","無","",0,1,0,false,false,0),
 [GV5_RPM]=META("rpm","转速","ENGINE RPM","回転数","轉速","转速","RPM","回転数","轉速","RPM",0,12000,0,true,true,5000),
 [GV5_COOLANT]=META("coolant","水温","COOLANT","水温","水溫","水温","CLT","水温","水溫","°C",40,120,0,true,true,30000),
 [GV5_IAT]=META("iat","进气温度","INTAKE AIR","吸気温度","進氣溫度","进气","IAT","吸気","進氣","°C",0,80,0,true,true,30000),
 [GV5_OIL]=META("oil","油温","OIL TEMP","油温","油溫","油温","OIL","油温","油溫","°C",40,140,0,true,true,30000),
 [GV5_THROTTLE]=META("throttle","节气门","THROTTLE","スロットル","節氣門","节气门","TPS","開度","節氣門","%",0,100,0,true,true,15000),
 [GV5_LOAD]=META("load","发动机负荷","ENGINE LOAD","エンジン負荷","發動機負荷","负荷","LOAD","負荷","負荷","%",0,100,0,true,true,15000),
 [GV5_SPEED]=META("speed","车速","SPEED","車速","車速","车速","SPD","車速","車速","km/h",0,240,0,true,true,5000),
 [GV5_VOLTAGE]=META("voltage","电压","VOLTAGE","電圧","電壓","电压","VOLT","電圧","電壓","V",10,16,1,true,true,30000),
 [GV5_BOOST]=META("boost","涡轮压力","BOOST PRESSURE","過給圧","渦輪壓力","涡轮","BOOST","過給","渦輪","bar",-1,2,2,true,true,15000),
 [GV5_OIL_PRESSURE]=META("oilPressure","机油压力","OIL PRESS","油圧","機油壓力","油压","OIL P","油圧","油壓","bar",0,8,1,true,true,30000),
 [GV5_BRAKE_TEMP]=META("brakeTemp","刹车温度","BRAKE TEMP","ブレーキ温度","剎車溫度","刹温","BKT","制動","剎溫","°C",0,800,0,true,true,30000),
 [GV5_AFR]=META("afr","空燃比","AIR FUEL RATIO","空燃比","空燃比","空燃比","AFR","空燃比","空燃比","",8,22,1,true,true,30000),
 [GV5_GEAR]=META("gear","挡位","GEAR","ギア","檔位","挡位","GEAR","ギア","檔位","",-1,8,0,true,false,5000),
 [GV5_PEDAL]=META("pedal","加速踏板","PEDAL","アクセル","加速踏板","踏板","PEDAL","ペダル","踏板","%",0,100,0,false,true,15000),
 [GV5_FUEL_RATE]=META("fuelRate","燃油流量","FUEL RATE","燃料流量","燃油流量","燃油流量","FLOW","燃料流量","燃油流量","L/h",0,90,1,true,true,5000),
 [GV5_FUEL_CONSUMPTION]=META("fuelConsumption","瞬时油耗","INSTANT FUEL","瞬間燃費","瞬時油耗","油耗","FUEL","燃費","油耗","L/100km",0,30,1,true,true,5000),
 [GV5_TRIP_FUEL]=META("tripFuel","行程油耗","TRIP FUEL","平均燃費","行程油耗","均耗","TRIP F","平均燃費","均耗","L/100km",0,25,1,true,true,5000),
 [GV5_BATTERY_POWER]=META("batteryPower","电池功率","BATTERY POWER","電池出力","電池功率","电池功率","POWER","電池出力","電池功率","kW",-60,120,1,false,true,0),
 [GV5_EV_CONSUMPTION]=META("evConsumption","瞬时电耗","INSTANT ENERGY","瞬間電費","瞬時電耗","电耗","ENERGY","電費","電耗","kWh/100km",-40,80,1,false,true,0),
 [GV5_TRIP_ENERGY]=META("tripEnergy","行程净电耗","TRIP NET ENERGY","平均電費","行程淨電耗","均电耗","TRIP kWh","平均電費","均電耗","kWh/100km",-30,60,1,false,true,0),
 [GV5_MAP_ABS]=META("mapAbsolute","进气绝压","MANIFOLD ABS","吸気絶対圧","進氣絕壓","进气压","MAP ABS","吸気圧","進氣壓","kPa",0,255,0,true,true,5000),
 [GV5_BARO]=META("barometricPressure","大气压力","BAROMETRIC","大気圧","大氣壓力","大气压","BARO","大気圧","大氣壓","kPa",50,120,0,true,true,5000)
};
#undef META

typedef struct { int32_t raw; int64_t updated_us; bool seen, invalid; } sample_t;
static portMUX_TYPE s_mux=portMUX_INITIALIZER_UNLOCKED;
static sample_t s_samples[GV5_FIELD_COUNT];
static bool s_link_seen, s_online;
static uint8_t s_role;
static uint32_t s_epoch=1;
static float s_history[GV5_FIELD_COUNT][30];
static uint8_t s_history_count,s_history_next;
static int64_t s_history_us;
static gauge_fuel_t s_fuel={.complete=true};

const gv5_meta_t *gv5_meta(gv5_field_t f) { return f>=0&&f<GV5_FIELD_COUNT?&s_meta[f]:&s_meta[GV5_NONE]; }
const char *gv5_label(gv5_field_t f,gv5_lang_t l,bool short_name) { if(l<GV5_ZH||l>GV5_ZH_TW)l=GV5_ZH;return short_name?gv5_meta(f)->short_label[l]:gv5_meta(f)->label[l]; }
bool gv5_continuous(gv5_field_t f) { return gv5_meta(f)->continuous; }
bool gv5_trend_eligible(gv5_field_t f) { return gv5_meta(f)->trend; }
bool gv5_is_electric(gv5_field_t f) { return f==GV5_BATTERY_POWER||f==GV5_EV_CONSUMPTION||f==GV5_TRIP_ENERGY; }
static bool raw_valid(gv5_field_t f,int32_t r) {
 switch(f) {
 case GV5_RPM:return r>=0&&r<=16383;
 case GV5_SPEED:return r>=0&&r<=255;
 case GV5_COOLANT:case GV5_IAT:return r>-40&&r<=215;
 case GV5_OIL:return r>=-20&&r<=150;
 case GV5_THROTTLE:case GV5_LOAD:return r>=0&&r<=100;
 case GV5_VOLTAGE:return r>0&&r<=60000;
 case GV5_BOOST:return r>=-15&&r<=300;
 case GV5_OIL_PRESSURE:return r>=0&&r<=200;
 case GV5_BRAKE_TEMP:return r>=-500&&r<=12000;
 case GV5_AFR:return r>=800&&r<=2200;
 case GV5_GEAR:return r>=-1&&r<=8;
 case GV5_FUEL_RATE:return r>=0&&r<=65535;
 case GV5_MAP_ABS:case GV5_BARO:return r>=0&&r<=255;
 default:return false;
 }
}
static float natural(gv5_field_t f,int32_t r) {
 switch(f) {case GV5_FUEL_RATE:return r/20.0f;case GV5_VOLTAGE:return r/1000.0f;case GV5_BOOST:case GV5_OIL_PRESSURE:case GV5_BRAKE_TEMP:return r/10.0f;case GV5_AFR:return r/100.0f;default:return (float)r;}
}
static bool transport_online(uint8_t role) { return role==ESPNOW_ROLE_SLAVE?espnow_link_slave_has_data():elm327_ble_is_connected(); }
/* A falling link edge/role change invalidates prior samples. A rising edge does
 * not clear observations already received by setters before the first capture. */
static void sync_link_locked(uint8_t role,bool online) {
 if(s_link_seen&&((s_role!=role)||(s_online&&!online))) {memset(s_samples,0,sizeof(s_samples));gauge_fuel_reset(&s_fuel);s_epoch++;}
 s_link_seen=true;s_role=role;s_online=online;
}
void gv5_data_observed(gv5_field_t f,int32_t raw) {
 if(f<=GV5_NONE||f>=GV5_FIELD_COUNT||!s_meta[f].implemented)return;
 const uint8_t role=nvs_cfg_get()->device_role;const bool online=transport_online(role);const int64_t now=esp_timer_get_time();
 portENTER_CRITICAL(&s_mux);sync_link_locked(role,online);s_samples[f]=(sample_t){raw,now,true,!raw_valid(f,raw)};
 if(f==GV5_FUEL_RATE){const sample_t *speed=&s_samples[GV5_SPEED];
  bool paired=online&&speed->seen&&!speed->invalid&&now>=speed->updated_us&&now-speed->updated_us<=5000000;
  gauge_fuel_step(&s_fuel,natural(f,raw),natural(GV5_SPEED,speed->raw),now,paired&&!s_samples[f].invalid);
 }
 portEXIT_CRITICAL(&s_mux);
}
void gv5_data_invalidated(gv5_field_t f) {
 if(f<=GV5_NONE||f>=GV5_FIELD_COUNT)return;
 portENTER_CRITICAL(&s_mux);s_samples[f].seen=true;s_samples[f].invalid=true;s_samples[f].updated_us=esp_timer_get_time();if(f==GV5_FUEL_RATE||f==GV5_SPEED)gauge_fuel_missing(&s_fuel);portEXIT_CRITICAL(&s_mux);
}
void gv5_data_reset_session(void) {
 portENTER_CRITICAL(&s_mux);memset(s_samples,0,sizeof(s_samples));gauge_fuel_reset(&s_fuel);s_epoch++;portEXIT_CRITICAL(&s_mux);
}
static gv5_reading_t unavailable(gv5_state_t s) {return (gv5_reading_t){.value=NAN,.valid=false,.state=s,.age_ms=UINT32_MAX};}
static void history_locked(const gv5_frame_t *frame) {
 if(s_history_count&&frame->captured_us-s_history_us<1000000)return;
 int64_t skipped=s_history_count?(frame->captured_us-s_history_us)/1000000-1:0;
 if(skipped>29)skipped=29;
 for(int64_t n=0;n<=skipped;n++) {for(int i=0;i<GV5_FIELD_COUNT;i++)s_history[i][s_history_next]=(n<skipped||!frame->readings[i].valid)?NAN:frame->readings[i].value;s_history_next=(s_history_next+1)%30;if(s_history_count<30)s_history_count++;}
 s_history_us=frame->captured_us;
}
void gv5_capture(gv5_frame_t *out) {
 if(!out)return;
 const uint8_t role=nvs_cfg_get()->device_role;const bool online=transport_online(role);const int64_t now=esp_timer_get_time();
 portENTER_CRITICAL(&s_mux);sync_link_locked(role,online);memset(out,0,sizeof(*out));out->online=online;out->slave=role==ESPNOW_ROLE_SLAVE;out->epoch=s_epoch;out->captured_us=now;
 for(int i=0;i<GV5_FIELD_COUNT;i++) {
  const sample_t *s=&s_samples[i];gv5_reading_t r=unavailable(GV5_UNSUPPORTED);
  if(s_meta[i].implemented) {
   if(!online)r.state=GV5_OFFLINE;
   else if(!s->seen)r.state=GV5_UNSEEN;
   else {
    r.updated_us=s->updated_us;int64_t age=now-s->updated_us;if(age<0)age=0;r.age_ms=age/1000>UINT32_MAX?UINT32_MAX:(uint32_t)(age/1000);
    if(s->invalid)r.state=GV5_INVALID;
    else if(age>(int64_t)s_meta[i].ttl_ms*1000)r.state=GV5_STALE;
    else {r.value=natural((gv5_field_t)i,s->raw);r.valid=isfinite(r.value);r.state=r.valid?GV5_CONFIRMED:GV5_INVALID;}
   }
  }
  out->readings[i]=r;
 }
 const gv5_reading_t fuel=out->readings[GV5_FUEL_RATE],speed=out->readings[GV5_SPEED];
 gv5_reading_t instant=unavailable(online?GV5_UNSEEN:GV5_OFFLINE),trip=instant;
 if(online&&fuel.valid&&speed.valid){
  instant.updated_us=trip.updated_us=fuel.updated_us<speed.updated_us?fuel.updated_us:speed.updated_us;
  instant.age_ms=trip.age_ms=fuel.age_ms>speed.age_ms?fuel.age_ms:speed.age_ms;
  if(speed.value>=1.0f){instant.value=100.0f*fuel.value/speed.value;instant.valid=isfinite(instant.value);instant.state=instant.valid?GV5_CONFIRMED:GV5_INVALID;}
  float avg;if(gauge_fuel_average(&s_fuel,&avg)){trip.value=avg;trip.valid=true;trip.state=GV5_CONFIRMED;}
 }else if(online&&s_fuel.started)gauge_fuel_missing(&s_fuel);
 out->readings[GV5_FUEL_CONSUMPTION]=instant;out->readings[GV5_TRIP_FUEL]=trip;
 history_locked(out);portEXIT_CRITICAL(&s_mux);
}
gv5_reading_t gv5_read(const gv5_frame_t *frame,gv5_field_t f) {return frame&&f>=0&&f<GV5_FIELD_COUNT?frame->readings[f]:unavailable(GV5_UNSUPPORTED);}
bool gv5_usable(const gv5_frame_t *frame,gv5_field_t f) {const gv5_reading_t r=gv5_read(frame,f);return r.valid&&r.state==GV5_CONFIRMED;}
size_t gv5_history(gv5_field_t f,float *out,size_t cap) {
 if(!out||!cap||f<=GV5_NONE||f>=GV5_FIELD_COUNT||!s_meta[f].trend)return 0;
 portENTER_CRITICAL(&s_mux);size_t count=s_history_count;if(count>cap)count=cap;const size_t first=(s_history_next+30-count)%30;for(size_t i=0;i<count;i++)out[i]=s_history[f][(first+i)%30];portEXIT_CRITICAL(&s_mux);return count;
}
