#include "gauge_fuel.h"
#include <math.h>
#include <string.h>
_Static_assert(sizeof(gauge_fuel_t)<100,"bounded fuel state");
void gauge_fuel_reset(gauge_fuel_t *s){if(s){memset(s,0,sizeof(*s));s->complete=true;}}
void gauge_fuel_missing(gauge_fuel_t *s){if(s&&s->started)s->complete=false;}
void gauge_fuel_step(gauge_fuel_t *s,float lph,float kmh,int64_t now,bool fresh){
 if(!s)return;
 if(!fresh||!isfinite(lph)||!isfinite(kmh)||lph<0||lph>3276.75f||kmh<0||kmh>255||now<=0){gauge_fuel_missing(s);return;}
 if(!s->started){s->started=true;s->stamp_us=now;s->previous_lph=lph;s->previous_kmh=kmh;return;}
 int64_t dt=now-s->stamp_us;
 if(dt==0)return; /* Never double-integrate a duplicated reading. */
 if(dt<0||dt>5000000){s->complete=false;}else if(s->complete){
  double hours=(double)dt/3600000000.0;
  s->litres+=((double)s->previous_lph+lph)*0.5*hours;
  s->kilometres+=((double)s->previous_kmh+kmh)*0.5*hours;
  if(!isfinite(s->litres)||!isfinite(s->kilometres))s->complete=false;
 }
 s->stamp_us=now;s->previous_lph=lph;s->previous_kmh=kmh;
}
bool gauge_fuel_average(const gauge_fuel_t *s,float *out){
 if(!s||!out||!s->started||!s->complete||s->kilometres<0.01)return false;
 double value=100.0*s->litres/s->kilometres;
 if(!isfinite(value)||value<0||value>3.402823466e38)return false;
 *out=(float)value;return isfinite(*out);
}
bool gauge_fuel_supports_5e(const uint32_t *d,size_t n){
 if(!d||n!=4)return false;
 for(size_t i=0;i<n;i++)if(d[i]>255)return false;
 return (d[3]&0x04u)!=0;
}
bool gauge_fuel_decode_5e(const uint32_t *d,size_t n,uint16_t *out){
 if(!d||!out||n!=2||d[0]>255||d[1]>255)return false;
 *out=(uint16_t)((d[0]<<8)|d[1]);return true;
}
static int nibble(char c){
 if(c>='0'&&c<='9')return c-'0';
 if(c>='a'&&c<='f')return c-'a'+10;
 if(c>='A'&&c<='F')return c-'A'+10;
 return -1;
}
bool gauge_fuel_line_bytes(const char *p,uint8_t pid,uint32_t *out,size_t count){
 if(!p||!out||(count!=2&&count!=4))return false;
 uint32_t tokens[6];size_t used=0;
 while(*p&&*p!='\r'&&*p!='\n'&&*p!='>'){
  if(*p==' '||*p=='\t'){p++;continue;}
  if(used>=count+2)return false;
  int hi=nibble(*p++);if(hi<0)return false;
  int lo=nibble(*p++);if(lo<0)return false;
  if(*p&&*p!=' '&&*p!='\t'&&*p!='\r'&&*p!='\n'&&*p!='>')return false;
  tokens[used++]=(uint32_t)((hi<<4)|lo);
 }
 if(used!=count+2||tokens[0]!=0x41||tokens[1]!=pid)return false;
 memcpy(out,tokens+2,count*sizeof(*out));return true;
}
