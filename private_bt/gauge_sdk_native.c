#include "gauge_sdk_owner.h"
#include <string.h>
#include "stack/bt_types.h"
#include "bta/bta_gatt_api.h"
#include "bta_gattc_int.h"
#include "btc/btc_task.h"
#include "btc_gattc.h"
#include "osi/thread.h"
#include "osi/allocator.h"
#include "stack/btu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
extern osi_thread_t *btu_thread;
extern osi_thread_t *btc_get_current_thread(void);
extern bool btu_task_post(uint32_t sig,void *param,uint32_t timeout);
extern bool gauge_BTA_GATTC_Close_checked(uint16_t conn,uint32_t token);
static tBTA_GATTC_CLOSE saved_close;
static uint32_t saved_token;
static uint8_t saved_pending,saved_queued;
static portMUX_TYPE delivery_mux=portMUX_INITIALIZER_UNLOCKED;
uint32_t gauge_sdk_header(const void *h) {const BT_HDR *p=h;return (uint32_t)p->len|((uint32_t)p->offset<<16);}
void gauge_sdk_stamp(void *h,uint32_t t) {BT_HDR *p=h;p->len=(uint16_t)t;p->offset=(uint16_t)(t>>16);}
uint32_t gauge_sdk_operation_token(const void *p,size_t n,uint16_t c) {uint32_t t=0;if(gso_owned(c))memcpy(&t,(const uint8_t *)p+n,4);return t;}
static bool header_scoped(uint16_t e) {
    switch(e) {
    case BTA_GATTC_API_CLOSE_EVT:case BTA_GATTC_API_CFG_MTU_EVT:
    case BTA_GATTC_API_SEARCH_EVT:case BTA_GATTC_API_READ_EVT:
    case BTA_GATTC_API_WRITE_EVT:case BTA_GATTC_API_EXEC_EVT:
    case BTA_GATTC_API_READ_MULTI_EVT:case BTA_GATTC_API_READ_BY_TYPE_EVT:
    case BTA_GATTC_API_READ_MULTI_VAR_EVT:
    case BTA_GATTC_API_CONFIRM_EVT:case BTA_GATTC_OP_CMPL_EVT:
    case BTA_GATTC_INT_CONN_EVT:case BTA_GATTC_INT_DISCONN_EVT:return true;
    default:return false;
    }
}
bool gauge_sdk_packet_accept(const void *h) {
    const BT_HDR *p=h;uint32_t t=gauge_sdk_header(p);
    if(p->event==BTA_GATTC_API_OPEN_EVT) {const tBTA_GATTC_API_OPEN *o=h;
        uint32_t owned=gso_open_current(o->client_if,o->remote_bda);
        return !owned || gso_open_auth(t,o->client_if,o->remote_bda);
    }
    if(p->event==BTA_GATTC_INT_CONN_EVT || p->event==BTA_GATTC_INT_DISCONN_EVT) {
        const tBTA_GATTC_INT_CONN *c=h;uint32_t current=gso_peer(c->client_if,c->remote_bda);
        return !current || (t==current && gso_execution_auth(t,gso_conn_for(t)));
    }
    if(header_scoped(p->event))return !gso_owned(p->layer_specific)||gso_execution_auth(t,p->layer_specific);
    if(p->event==BTA_GATTC_ENC_CMPL_EVT)return !t||gso_execution_auth(t,gso_conn_for(t));
    return !t || (t!=GSO_INVALID && gso_current(t,true));
}
void gauge_sdk_sendmsg_token(void *m,uint32_t t) {
    BT_HDR *h=m;uint16_t event=h->event;gauge_sdk_stamp(h,t);
    if(t && !gauge_sdk_packet_accept(h)) {osi_free(m);return;}
    if(!t) {bta_sys_sendmsg(m);return;} /* unowned native blocking/post semantics unchanged */
    if(!btu_task_post(SIG_BTU_BTA_MSG,m,0)) {if(event==BTA_GATTC_API_OPEN_EVT)gso_open_settled(t,false);osi_free(m);}
    else if(event==BTA_GATTC_API_OPEN_EVT)gso_open_settled(t,true);
}
void gauge_sdk_sendmsg(void *m) {
    BT_HDR *h=m;uint32_t t=0;
    if(h->event==BTA_GATTC_API_OPEN_EVT) {tBTA_GATTC_API_OPEN *o=m;t=gso_open_current(o->client_if,o->remote_bda);}
    else if(header_scoped(h->event)) {t=gso_context(h->layer_specific,false);if(!t)t=gso_outgoing(h->layer_specific);}
    gauge_sdk_sendmsg_token(m,t);
}
static uint16_t callback_connection(uint8_t e,const tBTA_GATTC *a,bool *scoped) {
    *scoped=true;
    switch(e) {
    case BTA_GATTC_OPEN_EVT:return a->open.conn_id;
    case BTA_GATTC_CONNECT_EVT:return a->connect.conn_id;
    case BTA_GATTC_CLOSE_EVT:return a->close.conn_id;
    case BTA_GATTC_DISCONNECT_EVT:return a->disconnect.conn_id;
    case BTA_GATTC_NOTIF_EVT:return a->notify.conn_id;
    case BTA_GATTC_READ_CHAR_EVT:case BTA_GATTC_READ_DESCR_EVT:case BTA_GATTC_READ_MULTIPLE_EVT:case BTA_GATTC_READ_MULTI_VAR_EVT:return a->read.conn_id;
    case BTA_GATTC_WRITE_CHAR_EVT:case BTA_GATTC_WRITE_DESCR_EVT:case BTA_GATTC_PREP_WRITE_EVT:return a->write.conn_id;
    case BTA_GATTC_EXEC_EVT:return a->exec_cmpl.conn_id;
    case BTA_GATTC_SEARCH_RES_EVT:return a->srvc_res.conn_id;
    case BTA_GATTC_SEARCH_CMPL_EVT:return a->search_cmpl.conn_id;
    case BTA_GATTC_CFG_MTU_EVT:return a->cfg_mtu.conn_id;
    case BTA_GATTC_CONGEST_EVT:return a->congest.conn_id;
    case BTA_GATTC_QUEUE_FULL_EVT:return a->queue_full.conn_id;
    case BTA_GATTC_SRVC_CHG_EVT:return a->srvc_chg.conn_id;
    case BTA_GATTC_DIS_SRVC_CMPL_EVT:return a->dis_cmpl.conn_id;
    default:*scoped=false;return 0;
    }
}
static uint16_t api_connection(uint8_t act,const btc_ble_gattc_args_t *a,bool *scoped) {
    *scoped=true;
    switch(act) {
    case BTC_GATTC_ACT_CLOSE:return a->close.conn_id;
    case BTC_GATTC_ACT_CFG_MTU:return a->cfg_mtu.conn_id;
    case BTC_GATTC_ACT_SEARCH_SERVICE:return a->search_srvc.conn_id;
    case BTC_GATTC_ACT_READ_CHAR:return a->read_char.conn_id;
    case BTC_GATTC_ACT_READ_MULTIPLE_CHAR:case BTC_GATTC_ACT_READ_MULTIPLE_VARIABLE_CHAR:return a->read_multiple.conn_id;
    case BTC_GATTC_ACT_READ_CHAR_DESCR:return a->read_descr.conn_id;
    case BTC_GATTC_ACT_READ_BY_TYPE:return a->read_by_type.conn_id;
    case BTC_GATTC_ACT_WRITE_CHAR:return a->write_char.conn_id;
    case BTC_GATTC_ACT_WRITE_CHAR_DESCR:return a->write_descr.conn_id;
    case BTC_GATTC_ACT_PREPARE_WRITE:return a->prep_write.conn_id;
    case BTC_GATTC_ACT_PREPARE_WRITE_CHAR_DESCR:return a->prep_write_descr.conn_id;
    case BTC_GATTC_ACT_EXECUTE_WRITE:return a->exec_write.conn_id;
    default:*scoped=false;return 0;
    }
}
bool gauge_sdk_btc_capture(const void *m,const void *v,int n,uint32_t *t) {
    const btc_msg_t *msg=m;bool scoped=false;*t=0;
    if(msg->pid!=BTC_PID_GATTC || !v)return false;
    if(msg->sig==BTC_SIG_API_CB && n==(int)sizeof(tBTA_GATTC)) {
        const tBTA_GATTC *a=v;uint16_t c=callback_connection(msg->act,a,&scoped);
        if(!scoped) {
            if(msg->act==BTA_GATTC_ENC_CMPL_CB_EVT) {if(!gso_owned(a->enc_cmpl.client_if))return false;*t=gso_context(UINT16_MAX,false);if(!*t)*t=gso_peer(a->enc_cmpl.client_if,a->enc_cmpl.remote_bda);return *t!=0;}
            return false;
        }
        *t=gso_context(c,false);
        if(msg->act==BTA_GATTC_OPEN_EVT && !*t && gso_owned(a->open.client_if)) *t=gso_context(UINT16_MAX,false);
        if(!*t)*t=gso_connection(c);
        if(msg->act==BTA_GATTC_OPEN_EVT && !*t)*t=gso_open_current(a->open.client_if,a->open.remote_bda);
        return *t!=0;
    }
    if(msg->sig==BTC_SIG_API_CALL && n==(int)sizeof(btc_ble_gattc_args_t)) {
        const btc_ble_gattc_args_t *a=v;
        if(msg->act==BTC_GATTC_ACT_OPEN) {*t=gso_open_current(a->open.gattc_if,a->open.remote_bda);return *t!=0;}
        if(msg->act==BTC_GATTC_ACT_REG_FOR_NOTIFY) {*t=gso_peer(a->reg_for_notify.gattc_if,a->reg_for_notify.remote_bda);return *t!=0;}
        if(msg->act==BTC_GATTC_ACT_UNREG_FOR_NOTIFY) {*t=gso_peer(a->unreg_for_notify.gattc_if,a->unreg_for_notify.remote_bda);return *t!=0;}
        uint16_t c=api_connection(msg->act,a,&scoped);if(scoped)*t=gso_connection(c);return *t!=0;
    }
    return false;
}
bool gauge_sdk_btc_auth(const void *m,const void *v,int n,uint32_t *t) {
    const btc_msg_t *msg=m;*t=0;if(!(msg->aid&GSO_BTC_TAG))return true;
    memcpy(t,(const uint8_t *)v+n,4);if(*t==GSO_INVALID || !*t)return false;
    if(msg->sig==BTC_SIG_API_CALL && msg->act==BTC_GATTC_ACT_OPEN) {const btc_ble_gattc_args_t *a=v;return gso_open_auth(*t,a->open.gattc_if,a->open.remote_bda);}
    if(msg->sig==BTC_SIG_API_CB && msg->act==BTA_GATTC_OPEN_EVT) {const tBTA_GATTC *a=v;
        if(*t&GSO_OPEN_BIT)return gso_open_auth(*t,a->open.client_if,a->open.remote_bda);
        return gso_current(*t,a->open.status==BTA_GATT_OK);
    }
    if(msg->sig==BTC_SIG_API_CB && (msg->act==BTA_GATTC_CLOSE_EVT || msg->act==BTA_GATTC_DISCONNECT_EVT))return gso_current(*t,true);
    return gso_execution_auth(*t,gso_conn_for(*t));
}
static void close_job(void *unused) {(void)unused;uint32_t t;uint16_t c;gso_close_job(&t,&c);bool ok=gso_auth(t,c)&&gauge_BTA_GATTC_Close_checked(c,t);gso_close_settled(t,ok);}
bool gauge_sdk_close_submit(uint32_t t,uint16_t c) {
    if(!gso_close_claim(t,c))return false;
    osi_thread_t *thread=btc_get_current_thread();
    if(!thread || !osi_thread_post(thread,close_job,NULL,0,0)) {gso_close_settled(t,false);return false;}return true;
}
void gauge_sdk_btc_failed(const void *m,const void *v,uint32_t t) {
    const btc_msg_t *msg=m;
    if(msg->sig==BTC_SIG_API_CB && msg->act==BTA_GATTC_CLOSE_EVT && t)gauge_sdk_save_close(&((const tBTA_GATTC *)v)->close,t);
    if(msg->sig==BTC_SIG_API_CALL && msg->act==BTC_GATTC_ACT_OPEN && t)gso_open_settled(t,false);
}
void gauge_sdk_save_close(const void *v,uint32_t t) {
    if(!gso_current(t,true))return;
    portENTER_CRITICAL(&delivery_mux);
    if(!saved_pending) {saved_close=*(const tBTA_GATTC_CLOSE *)v;saved_token=t;saved_pending=1;}
    portEXIT_CRITICAL(&delivery_mux);
}
static void saved_delivery(void *unused) {
    (void)unused;tBTA_GATTC_CLOSE c;uint32_t t;
    portENTER_CRITICAL(&delivery_mux);c=saved_close;t=saved_token;portEXIT_CRITICAL(&delivery_mux);
    gauge_sdk_deliver_close(&c,t); /* real native BTC destructor/callback path */
    portENTER_CRITICAL(&delivery_mux);saved_pending=0;saved_queued=0;portEXIT_CRITICAL(&delivery_mux);
}
void gauge_sdk_service_delivery(void) {
    portENTER_CRITICAL(&delivery_mux);bool send=saved_pending&&!saved_queued;if(send)saved_queued=1;portEXIT_CRITICAL(&delivery_mux);
    if(send) {osi_thread_t *thread=btc_get_current_thread();if(!thread||!osi_thread_post(thread,saved_delivery,NULL,0,0)) {portENTER_CRITICAL(&delivery_mux);saved_queued=0;portEXIT_CRITICAL(&delivery_mux);}}
}

bool gauge_sdk_sm_accept(void *descriptor,uint16_t event,const void *data)
{
    tBTA_GATTC_CLCB *c=descriptor;
    uint32_t expected=gso_descriptor(descriptor);
    if(expected==GSO_INVALID) return false;
    uint32_t inherited=gso_context(c->bta_conn_id,false);
    if(expected) {
        uint32_t token=data?gauge_sdk_header(data):inherited;
        if(event==BTA_GATTC_API_OPEN_EVT && (token&GSO_OPEN_BIT)) token=gso_context(UINT16_MAX,false);
        return token==expected && gso_execution_auth(expected,gso_conn_for(expected));
    }
    /* An old owned operation cannot use a slot which now belongs to another app. */
    if(inherited && !(inherited&GSO_OPEN_BIT) && !gso_current(inherited,true))return false;
    return true;
}
