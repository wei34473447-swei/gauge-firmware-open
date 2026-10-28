#ifndef GAUGE_SDK_OWNER_H
#define GAUGE_SDK_OWNER_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define GSO_INVALID UINT32_MAX
#define GSO_OPEN_BIT UINT32_C(0x80000000)
#define GSO_BTC_TAG 0x80u
/* ID comes from actual SDK CLCB allocation; intent IDs are disjoint. */
typedef enum { GSO_NONE, GSO_REJECTED, GSO_QUEUED, GSO_ACCEPTED, GSO_RELEASED } gso_state_t;
typedef struct { uint32_t token; uint16_t conn; uint8_t kind; uint8_t slot; uintptr_t task; } gso_context_t;
enum { GSO_PACKET=1, GSO_OPERATION=2, GSO_BTC_API=3, GSO_BTC_CALLBACK=4 };
void gso_register(uint8_t client);
bool gso_owned(uint16_t conn);
uint32_t gso_birth(void *descriptor,uint8_t client,const uint8_t peer[6]);
void gso_bind(void *descriptor,uint16_t conn);
uint32_t gso_descriptor(const void *descriptor);
uint16_t gso_conn_for(uint32_t token);
uint32_t gso_connection(uint16_t conn);
uint32_t gso_peer(uint8_t client,const uint8_t peer[6]);
void gso_terminal_produced(uint32_t token);
bool gso_execution_auth(uint32_t token,uint16_t conn);
uint32_t gso_callback_token(void);
bool gso_auth(uint32_t token,uint16_t conn);
bool gso_operation_auth(uint32_t token,uint16_t conn);
void gso_deallocated(void *descriptor);
bool gso_operation_allocated(uint32_t token);
void gso_operation_deallocated(uint32_t token);
gso_context_t gso_enter(uint32_t token,uint16_t conn,uint8_t kind,uint8_t slot);
void gso_leave(gso_context_t prior);
uint32_t gso_context(uint16_t conn,bool operation);
uint32_t gso_outgoing(uint16_t conn);
uint32_t gso_open_reserve(uint8_t client,const uint8_t peer[6]);
bool gso_open_auth(uint32_t token,uint8_t client,const uint8_t peer[6]);
uint32_t gso_open_current(uint8_t client,const uint8_t peer[6]);
bool gso_current(uint32_t token,bool require_alive);
void gso_open_settled(uint32_t token,bool accepted);
bool gso_close_claim(uint32_t token,uint16_t conn);
void gso_close_settled(uint32_t token,bool accepted);
gso_state_t gso_state(uint32_t token);
void gso_close_job(uint32_t *token,uint16_t *conn);
void gso_mark_closing(uint32_t token);
size_t gso_static_bytes(void);
/* Native adapters use existing BTC/BTU queue 0 only, no new task/queue. */
bool gauge_sdk_close_submit(uint32_t token,uint16_t conn);
void gauge_sdk_service_delivery(void);
bool gauge_sdk_sm_accept(void *descriptor,uint16_t event,const void *data);
bool gauge_sdk_packet_accept(const void *hdr);
void gauge_sdk_sendmsg(void *message);
void gauge_sdk_sendmsg_token(void *message,uint32_t token);
uint32_t gauge_sdk_operation_token(const void *clcb,size_t native_size,uint16_t conn);
void gauge_sdk_stamp(void *hdr,uint32_t token);
uint32_t gauge_sdk_header(const void *hdr);
bool gauge_sdk_btc_capture(const void *msg,const void *arg,int arg_len,uint32_t *token);
bool gauge_sdk_btc_auth(const void *msg,const void *arg,int arg_len,uint32_t *token);
void gauge_sdk_btc_failed(const void *msg,const void *arg,uint32_t token);
void gauge_sdk_deliver_close(const void *close,uint32_t token);
void gauge_sdk_save_close(const void *close,uint32_t token);
#endif
