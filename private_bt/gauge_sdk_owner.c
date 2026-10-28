#include "gauge_sdk_owner.h"
#include <string.h>
#ifdef GSO_HOST
#include <pthread.h>
static pthread_mutex_t s_mux=PTHREAD_MUTEX_INITIALIZER;
#define SELF() ((uintptr_t)pthread_self())
#define ENTER() pthread_mutex_lock(&s_mux)
#define EXIT() pthread_mutex_unlock(&s_mux)
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static portMUX_TYPE s_mux=portMUX_INITIALIZER_UNLOCKED;
#define SELF() ((uintptr_t)xTaskGetCurrentTaskHandle())
#define ENTER() portENTER_CRITICAL(&s_mux)
#define EXIT() portEXIT_CRITICAL(&s_mux)
#endif
/* A single app-owned client. All SDK/NVS/logger/parse work stays outside mux. */
typedef struct {
    void *descriptor;
    uint32_t next,birth,open,job;
    uint16_t conn,read_ops,producers,job_conn;
    uint8_t peer[6],open_peer[6];
    uint8_t client,registered,alive,closing,freed,job_state,open_state,exhausted,terminal,open_consumed;
} sdk_owner_t;
static sdk_owner_t s;
/* Context is native-worker scoped and nested callbacks save/restore it. */
static gso_context_t contexts[2];
static bool matching(uint32_t t,uint16_t c) { return t && t!=GSO_INVALID && t==s.birth && s.alive && s.conn==c; }
void gso_register(uint8_t c) { ENTER(); if(!s.registered) {s.registered=1;s.client=c;s.conn=UINT16_MAX;} EXIT(); }
bool gso_owned(uint16_t c) { ENTER();bool b=s.registered && (uint8_t)c==s.client;EXIT();return b; }
uint32_t gso_birth(void *p,uint8_t c,const uint8_t peer[6]) {
    uint32_t t=0;ENTER();
    if(s.registered && c==s.client) {
        if(s.alive || s.read_ops || s.producers || !s.open || s.exhausted ||
           memcmp(peer,s.open_peer,6)!=0 || s.next>=GSO_OPEN_BIT-2) {s.exhausted|=s.next>=GSO_OPEN_BIT-2;t=GSO_INVALID;}
        else {s.open_consumed=1;s.birth=++s.next;s.descriptor=p;s.alive=1;s.closing=0;s.freed=0;s.terminal=0;s.conn=UINT16_MAX;memcpy(s.peer,peer,6);s.job_state=GSO_NONE;t=s.birth;
            /* API_OPEN's context becomes the actual descriptor incarnation. */
            if(contexts[1].task==SELF() && contexts[1].token==s.open) {contexts[1].token=t;contexts[1].conn=UINT16_MAX;s.producers++;contexts[1].kind|=0x80u;}
        }
    }
    else if(s.descriptor==p && !s.alive)s.descriptor=NULL;
    EXIT();return t;
}
void gso_bind(void *p,uint16_t c) {ENTER();if(s.alive && s.descriptor==p) {s.conn=c;if(contexts[1].token==s.birth)contexts[1].conn=c;}EXIT();}
uint32_t gso_descriptor(const void *p) {ENTER();uint32_t t=s.descriptor==p?(s.alive?s.birth:GSO_INVALID):0;EXIT();return t;}
uint32_t gso_connection(uint16_t c) {ENTER();uint32_t t=s.registered && (uint8_t)c==s.client?(s.alive && s.conn==c?s.birth:GSO_INVALID):0;EXIT();return t;}
uint32_t gso_peer(uint8_t c,const uint8_t p[6]) {ENTER();uint32_t t=s.registered && c==s.client?(s.alive && memcmp(p,s.peer,6)==0?s.birth:GSO_INVALID):0;EXIT();return t;}
bool gso_auth(uint32_t t,uint16_t c) {ENTER();bool b=matching(t,c);EXIT();return b;}
bool gso_operation_auth(uint32_t t,uint16_t c) {ENTER();bool b=matching(t,c)&&!s.terminal;EXIT();return b;}
void gso_deallocated(void *p) {ENTER();if(s.alive && s.descriptor==p) {s.alive=0;s.freed=1;/* Retain retired slot identity until native allocation explicitly reuses it. */}EXIT();}
bool gso_operation_allocated(uint32_t t) {ENTER();bool b=t && t==s.birth && s.alive  && s.read_ops!=UINT16_MAX;if(b)s.read_ops++;EXIT();return b;}
void gso_operation_deallocated(uint32_t t) {ENTER();if(t && t==s.birth && s.read_ops)s.read_ops--;EXIT();}
gso_context_t gso_enter(uint32_t t,uint16_t c,uint8_t kind,uint8_t slot) {
    ENTER();gso_context_t old=contexts[slot];old.slot=slot;contexts[slot]=(gso_context_t){t,c,kind,slot,SELF()};
    if(t && t==s.birth && (s.alive || s.read_ops)) {
        if(s.producers==UINT16_MAX)s.exhausted=1;
        else {s.producers++;contexts[slot].kind|=0x80u;}
    }
    EXIT();return old;
}
void gso_leave(gso_context_t old) {ENTER();gso_context_t now=contexts[old.slot];if((now.kind&0x80u) && now.token && now.token==s.birth && s.producers)s.producers--;contexts[old.slot]=old;EXIT();}
uint32_t gso_context(uint16_t c,bool operation) {ENTER();gso_context_t x=contexts[1];uint32_t t=(contexts[1].task==SELF() && (x.conn==c || c==UINT16_MAX) && (!operation||(x.kind&0x7fu)==GSO_OPERATION))?x.token:0;EXIT();return t;}
uint32_t gso_outgoing(uint16_t c) {ENTER();uint32_t t=contexts[0].task==SELF()?contexts[0].token:0;if(!t||contexts[0].conn!=c)t=s.registered&&(uint8_t)c==s.client?(s.alive&&s.conn==c?s.birth:GSO_INVALID):0;EXIT();return t;}
uint32_t gso_open_reserve(uint8_t c,const uint8_t p[6]) {
    ENTER();uint32_t t=0;
    bool settled=s.job_state!=GSO_QUEUED;
    if(s.registered && c==s.client && !s.alive && !s.read_ops && !s.producers && settled && !s.exhausted &&
       (s.open_state==GSO_NONE || s.open_state==GSO_REJECTED || s.open_state==GSO_RELEASED)) {
        if(s.next>=GSO_OPEN_BIT-2)s.exhausted=1;
        else {s.open_consumed=0;s.open=GSO_OPEN_BIT|(s.next+1);s.open_state=GSO_QUEUED;memcpy(s.open_peer,p,6);t=s.open;}
    }
    EXIT();return t;
}
bool gso_open_auth(uint32_t t,uint8_t c,const uint8_t p[6]) {ENTER();bool b=t && t==s.open && c==s.client && !s.alive && !s.open_consumed && (s.open_state==GSO_QUEUED||s.open_state==GSO_ACCEPTED) && memcmp(p,s.open_peer,6)==0;EXIT();return b;}
void gso_open_settled(uint32_t t,bool ok) {ENTER();if(t==s.open && (ok || (!s.alive && !s.open_consumed)))s.open_state=ok?GSO_ACCEPTED:GSO_REJECTED;EXIT();}
void gso_mark_closing(uint32_t t) {ENTER();if(t && t==s.birth)s.closing=1;EXIT();}
bool gso_close_claim(uint32_t t,uint16_t c) {ENTER();bool b=matching(t,c) && !s.terminal && (s.job_state==GSO_NONE || s.job_state==GSO_REJECTED);if(b){s.closing=1;s.job=t;s.job_conn=c;s.job_state=GSO_QUEUED;}EXIT();return b;}
void gso_close_job(uint32_t *t,uint16_t *c) {ENTER();*t=s.job;*c=s.job_conn;EXIT();}
void gso_close_settled(uint32_t t,bool ok) {ENTER();if(t==s.job && s.job_state==GSO_QUEUED)s.job_state=ok?GSO_ACCEPTED:GSO_REJECTED;EXIT();}
gso_state_t gso_state(uint32_t t) {
    ENTER();gso_state_t r=GSO_NONE;
    if(t==s.birth && t) {
        if(!s.exhausted && s.freed && !s.read_ops && !s.producers && s.job_state!=GSO_QUEUED){r=GSO_RELEASED;s.open_state=GSO_RELEASED;}
        else r=s.terminal?GSO_ACCEPTED:(gso_state_t)s.job_state;
    } else if(t==s.open && (t&GSO_OPEN_BIT)) r=(gso_state_t)s.open_state;
    EXIT();return r;
}
size_t gso_static_bytes(void) { return sizeof(s)+sizeof(s_mux)+sizeof(contexts); }

bool gso_current(uint32_t t,bool alive) {ENTER();bool b=t&&t==s.birth&&(!alive||s.alive);EXIT();return b;}
uint32_t gso_open_current(uint8_t c,const uint8_t p[6]) {ENTER();uint32_t t=s.registered&&c==s.client?(memcmp(p,s.open_peer,6)==0&&s.open_state!=GSO_REJECTED?s.open:GSO_INVALID):0;EXIT();return t;}

uint16_t gso_conn_for(uint32_t t) {ENTER();uint16_t c=t==s.birth?s.conn:UINT16_MAX;EXIT();return c;}

void gso_terminal_produced(uint32_t t) {ENTER();if(t&&t==s.birth)s.terminal=1;EXIT();}
bool gso_execution_auth(uint32_t t,uint16_t c) {ENTER();bool b=matching(t,c)&&!s.terminal;EXIT();return b;}
uint32_t gso_callback_token(void) {ENTER();uint32_t t=contexts[0].task==SELF()&&(contexts[0].kind&0x7fu)==GSO_BTC_CALLBACK?contexts[0].token:0;EXIT();return t;}
