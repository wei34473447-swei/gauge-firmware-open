#include "elm327_ble_client.h"
#include "../gauge_memory_diag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "app_obd_dsp/obd_data_cache.h"
#include "app_obd_dsp/gauge_v5_data.h"
#include "app_obd_dsp/gauge_fuel.h"
#include <stdatomic.h>
#include "gauge_sdk_owner.h"
#include "app_obd_dsp/vehicle_profiles.h"
#include "app_obd_dsp/vehicle_custom_config.h"
#include "racechrono_ble_diy.h"
#include "ble_adv_util.h"
#include "esp_task_wdt.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include "nvs_storage.h"

// UUID constants
#define UUID16_OBD_SERVICE      0xFFF0  // common ELM327 BLE adapters (FFF1 write / FFF2 notify)
#define UUID16_OBD_SERVICE_18F0  0x18F0  // IOS-Vlink / Vlink (2AF1 write / 2AF0 notify)
#define UUID16_OBD_SERVICE_FF12  0xFF12  // config service of some adapters (e.g. Viecar) (FF15 write / FF14 notify)
#define UUID16_OBD_WRITE_CHAR    0xFFF1
#define UUID16_CCCD              0x2902

static const char *TAG = "elm327_ble";

static esp_gatt_if_t s_gattc_if = ESP_GATT_IF_NONE;   // 0 is a valid real interface number; NONE distinguishes "not registered yet"
static uint16_t s_conn_id = 0xFFFF;
static esp_bd_addr_t s_peer_bda = {0};
static volatile bool s_connected = false;
static bool s_have_service = false;
static uint16_t s_service_start = 0x0001, s_service_end = 0xFFFF; // default: full range
static uint16_t s_all_attr_end = 0xFFFF; // tracks highest seen end handle
static bool s_have_18f0 = false;          // 0x18F0 service (the real OBD communication service of IOS-Vlink)
static uint16_t s_18f0_start = 0, s_18f0_end = 0;
static bool s_have_ff12 = false;          // 0xFF12 service (fallback)
static uint16_t s_ff12_start = 0, s_ff12_end = 0;
static uint16_t s_char_write_handle = 0; // FFF1
static uint16_t s_char_notify_handle = 0; // prefer FFF2, fall back to FFF1 if absent
static uint16_t s_cccd_handle = 0;
static esp_gatt_write_type_t s_write_type = ESP_GATT_WRITE_TYPE_RSP; // write type, auto-selected from characteristic properties
static elm327_ble_callbacks_t s_cbs = {0};
static char s_target_name[32] = "OBDII";
// Exact MAC match: once set, match_device_target accepts only this address and ignores the name (prevents misconnecting to same-named devices)
static esp_bd_addr_t s_target_bda = {0};
static bool s_target_bda_valid = false;
// An ACL CONNECT also arrives for phones using our GATT server. Only an OPEN
// that we requested for the bound adapter establishes the OBD client session.
static bool s_open_pending = false;
static esp_bd_addr_t s_open_bda = {0};
static atomic_uint s_obd_target_generation;
static bool s_retarget_close_pending;
static uint16_t s_retarget_old_conn;
static esp_bd_addr_t s_retarget_old_peer;
/* Native descriptor incarnation, never a timeout-derived "closed" flag.
 * The permanent OBD worker performs final cleanup only after real SDK release. */
static uint32_t s_active_sdk_epoch;
static uint32_t s_retarget_sdk_epoch;
static bool s_retarget_cleaning;
static portMUX_TYPE s_retarget_mux = portMUX_INITIALIZER_UNLOCKED;
static void retarget_service(void);
static void retarget_close_explicit(void);
static void retarget_terminal(uint8_t gatt_if, uint16_t conn, const uint8_t peer[6]);

/* Scan lifecycle: fixed-size state, serviced by the existing OBD worker.
 * Requests survive asynchronous parameter/start/stop completion. */
static volatile bool s_scan_params_ready;
static volatile bool s_scan_params_pending;
static volatile bool s_scan_requested;
static volatile bool s_scan_start_pending;
static volatile bool s_ble_scan_active;
static volatile bool s_scan_stop_requested;
static volatile bool s_scan_stop_pending;
static int64_t s_scan_api_retry_after_us;
static int s_scan_duration_s = 10;
static void scan_service(void);
static void scan_stop(void);
static void configure_scan_params(void);

// ---- Scan mode related ----
static bool s_scan_only_mode = false;  // true=scan only, no connect
static ble_scan_found_cb_t s_scan_cb = NULL;
static ble_scan_result_t s_scan_list[BLE_SCAN_MAX_DEVICES];
static int s_scan_count = 0;
static bool s_ble_inited = false;  // whether the BLE stack has been initialized
static atomic_bool s_fuel_probe_pending;
static atomic_bool s_fuel_supported;
static atomic_uint s_fuel_attempts;
static int64_t s_fuel_last_request_us;
static atomic_bool s_poll_task_started = false; // poll task created or creation in progress
static volatile bool s_ota_paused = false; // during WiFi OTA: suppress auto-reconnect + polling so BLE stops competing for the radio
static TaskHandle_t s_poll_task_handle = NULL; // poll task handle, used for task-notification wakeup
static volatile bool s_notify_ready = false;   // set only after CCCD notify subscription completes; init/poll proceed based on this (prevents losing handshake responses)
static volatile int64_t s_last_obd_valid_us = 0; // time of the last valid OBD data; a timeout without data triggers self-heal re-init
static volatile bool s_got_valid_data = false;   // whether a genuinely valid frame has been parsed since the last poll round; set only by the parse path (not by do_elm_init), lets the poll task clear the self-heal counter
static uint8_t s_oil_query_mode = 0;     // current query mode index (0-2)

/* Bounded startup evidence, not a second command queue. Raw command/reply
 * association is frozen at the first fragment BEFORE prompt wakes the sender.
 * A forced timeout/overlap makes later attribution explicitly uncertain. */
#define ELM_DIAG_RESPONSE_LIMIT 24u
static uint8_t s_effective_obd_protocol;
static char s_scan_fixed_header[20];
static void obd_scan_reset_session(void);
static bool s_fast_waiting,s_fast_done,s_fast_ok,s_fast_enabled,s_fast_associated;
static portMUX_TYPE s_elm_diag_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned s_elm_diag_count;
static bool s_elm_diag_pending, s_elm_diag_uncertain, s_elm_diag_pid_seen;
static char s_elm_diag_command[20];
static char s_elm_diag_response_command[20]; /* GATT callback owns fragments. */
static bool s_elm_diag_response_associated;
static void elm_diag_reset(void)
{
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_elm_diag_count=0;s_elm_diag_pending=false;s_elm_diag_uncertain=false;s_elm_diag_pid_seen=false;
    s_elm_diag_command[0]=0;
    portEXIT_CRITICAL(&s_elm_diag_mux);
    s_elm_diag_response_command[0]=0;s_elm_diag_response_associated=false;
}
static void elm_diag_uncertain(void)
{
    portENTER_CRITICAL(&s_elm_diag_mux);s_elm_diag_uncertain=true;
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
static void elm_diag_request(const char *command)
{
    char clean[20];size_t n=0;
    for(const unsigned char *p=(const unsigned char *)command;*p&&n<sizeof clean-1;p++)
        if(*p>32&&*p<127)clean[n++]=(char)*p;
    clean[n]=0;
    portENTER_CRITICAL(&s_elm_diag_mux);
    if(s_elm_diag_pending)s_elm_diag_uncertain=true;
    memcpy(s_elm_diag_command,clean,n+1);s_elm_diag_pending=true;
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
static void elm_diag_begin_response(void)
{
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_elm_diag_response_associated=s_elm_diag_pending&&!s_elm_diag_uncertain;
    if(s_elm_diag_response_associated)memcpy(s_elm_diag_response_command,s_elm_diag_command,sizeof s_elm_diag_response_command);
    else {s_elm_diag_response_command[0]='?';s_elm_diag_response_command[1]=0;}
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
static void elm_diag_prompt(void)
{
    /* Release only this response's pending request before waking poll task. */
    portENTER_CRITICAL(&s_elm_diag_mux);s_elm_diag_pending=false;
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
static const char *elm_diag_response_kind(const char *response)
{
    if(strstr(response,"NO DATA"))return "NO_DATA";
    if(strstr(response,"UNABLE"))return "UNABLE";
    if(strstr(response,"CAN ERROR"))return "CAN_ERROR";
    if(strstr(response,"ERROR"))return "ERROR";
    if(strchr(response,'?'))return "QUERY";
    if(strstr(response,"41 ")||strstr(response,"62 ")||strstr(response,"61 "))return "PID_HEADER";
    if(strstr(response,"410C")||strstr(response,"4105")||strstr(response,"4142"))return "COMPACT";
    if(strstr(response,"OK"))return "OK";
    return "OTHER";
}
static void elm_diag_complete_response(const char *response)
{
    unsigned count;
    portENTER_CRITICAL(&s_elm_diag_mux);
    count=s_elm_diag_count;if(count<ELM_DIAG_RESPONSE_LIMIT)s_elm_diag_count++;
    portEXIT_CRITICAL(&s_elm_diag_mux);
    if(count>=ELM_DIAG_RESPONSE_LIMIT)return;
    char clean[41];size_t n=0;
    for(const unsigned char *p=(const unsigned char *)response;*p&&n<sizeof clean-1;p++)
        clean[n++]=(*p>=32&&*p<127)?(char)*p:' ';
    clean[n]=0;
    char line[97];snprintf(line,sizeof line,"rx n=%u c=%s a=%u t=%s r=%s",count+1,
        s_elm_diag_response_command,(unsigned)s_elm_diag_response_associated,
        elm_diag_response_kind(response),clean);
    ESP_LOGI(TAG,"%s",line); /* No console in critical section. */
}
static void elm_diag_first_pid(unsigned pid)
{
    bool first;
    portENTER_CRITICAL(&s_elm_diag_mux);first=!s_elm_diag_pid_seen;s_elm_diag_pid_seen=true;
    portEXIT_CRITICAL(&s_elm_diag_mux);
    if(first)ESP_LOGI(TAG,"first parsed standard PID=%02X",pid);
}

// Called when valid OBD data is parsed: refreshes the "valid data" timestamp and sets the flag.
// do_elm_init only sets the timestamp and does NOT go through here — just having initialized does not mean "data is flowing". Otherwise heal_attempts
// would be cleared right after every re-init, and "3 consecutive self-heals escalate to forced reconnect" could never be reached.
static uint32_t s_valid_frame_count;
static int64_t s_real_valid_us;

static inline void mark_obd_data_valid(void) {
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_valid_frame_count++;
    s_real_valid_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_elm_diag_mux);
    s_last_obd_valid_us = esp_timer_get_time();
    s_got_valid_data = true;
}

bool elm327_ble_send_ascii_blocking(const char *ascii_cmd);

static bool send_rpm_request(const char *site)
{
    bool ok = elm327_ble_send_ascii_blocking("01 0C\r");
    if (!ok) {
        ESP_LOGW(TAG, "[RPM][REQ:%s] send failed", site);
    }
    return ok;
}

// ---- Data-driven override state ----
static const vehicle_override_t *s_ov = NULL;  // override config of the current vehicle profile
static const oil_formula_t *s_oil_formula_pri = NULL;   // primary oil-temp formula
static const oil_formula_t *s_oil_formula_sec = NULL;   // secondary oil-temp formula
static bool s_oil_use_override = false;  // true=use override formulas, false=use the legacy enum
static uint8_t s_oil_override_idx = 0;   // 0=primary, 1=secondary
static uint8_t s_oil_override_fail = 0;  // consecutive failure count of the current formula
#define OIL_OVERRIDE_FAIL_MAX 5
static int s_mode21_oil_idx = 33;        // Mode21 oil-temp byte index, adaptively updated
static int16_t s_last_mode21_oil = -100; // oil temp parsed from the last Mode21 response
static int s_mode21_hold_cnt = 0;        // ZC6 consistency hold count; consecutive noise frames, new value accepted only past a threshold
static int64_t s_last_mode21_oil_us = 0; // timestamp (us) of the last accepted oil-temp value

// ---- Vehicle-profile-based oil-temp query strategy ----
static oil_temp_query_mode_t s_oil_mode_priority[4] = {
    OIL_TEMP_MODE_PID_5C,
    OIL_TEMP_MODE_UDS_22_10_17,
    OIL_TEMP_MODE_TOYOTA_21_01,
};  // default priority, updated from the vehicle profile config after boot
static uint32_t s_oil_mode_fail_count[12] = {0};  // consecutive failure count per mode (poll idx 0~11)
#define OIL_MODE_FAIL_THRESHOLD 5  // switch to the next mode only after a mode fails this many times
#define OBD_POLL_SLOT_GAP_MS   30  // idle gap between poll slots (ms); was 100, lowered to raise refresh rate — too small overwhelms clone adapters
static bool s_vehicle_profile_inited = false;

// Oil-temp diagnostic stats
static struct {
    uint32_t mode0_ok;  // 01 5C success count
    uint32_t mode1_ok;  // 22 10 17 success count
    uint32_t mode2_ok;  // 21 01 success count
    uint32_t mode3_ok;  // 22 11 1F (Mazda) success count
    uint32_t mode4_ok;  // 22 13 10 (Mazda) success count
    uint32_t mode5_ok;  // reserved
    uint32_t mode6_ok;  // 22 58 22 (MINI/BMW) success count
    uint32_t mode7_ok;  // 22 44 02 (BMW F-series) success count
    uint32_t mode8_ok;  // 22 03 F3 (BMW G-series) success count
    uint32_t mode9_ok;  // 22 44 02 G formula (BMW G-series) success count
    uint32_t mode10_ok; // 22 D0 02 (BMW G-series) success count
    uint32_t mode11_ok; // 22 11 1F (BMW, A-50) success count
    uint32_t mode0_fail;
    uint32_t mode1_fail;
    uint32_t mode2_fail;
    uint32_t mode3_fail;
    uint32_t mode4_fail;
    uint32_t mode5_fail;
    uint32_t mode6_fail;
    uint32_t mode7_fail;
    uint32_t mode8_fail;
    uint32_t mode9_fail;
    uint32_t mode10_fail;
    uint32_t mode11_fail;
    int16_t last_raw_temp; // raw temperature (unfiltered)
    int16_t last_filtered_temp; // filtered temperature
} s_oil_diag = {0};

static int8_t s_oil_temp_offset = 0;  // user calibration offset, in °C
static volatile int64_t s_can_oil_last_us = 0;      // last CAN oil-temp sample time
static volatile int64_t s_can_coolant_last_us = 0;  // last CAN coolant-temp sample time

// Global ready flag
static volatile bool s_elm_ready = true; // initially true so the first ATZ can be sent
static volatile bool s_expect_mode21 = false; // true=last command was 21 01, waiting for a 61 01 response
// ---- CAN continuous monitor mode (ATMA, parse each frame as it arrives) ----
static volatile bool s_zc6_can_monitor_active = false;
static bool s_zc_can_obd_phase = false;          // true=running the standard OBD poll
static bool s_zc_can_obd_round_started = false;  // false=just entered OBD phase, true=already consumed slot0 if needed
#define ZC6_CAN_MONITOR_BUF_SIZE 160
static char s_zc6_can_monitor_buf[ZC6_CAN_MONITOR_BUF_SIZE];
static size_t s_zc6_can_monitor_len = 0;
static int64_t s_zc6_can_monitor_entered_us = 0;   // time the monitor was entered
static uint32_t s_zc6_can_monitor_obd_cycle = 0;   // OBD query cycle counter while monitoring
#define ZC6_CAN_OBD_INTERVAL 50                     // exit ATMA every 50 cycles (~6s) to query OBD PIDs once
#define ZC6_CAN_TEMP_PROBE_INTERVAL_US 500000LL     // brief CAN-temp probe every 0.5s
#define ZC6_CAN_TEMP_PROBE_WINDOW_MS 120u           // shorter window to reduce RPM disturbance
#define ZC6_CAN_TEMP_STALE_US 15000000LL            // CAN temp channels stale after 15s without fresh frames
static int64_t s_zc6_can_temp_probe_last_us = 0;    // last time we briefly entered ATMA to refresh ZC6 CAN temps
bool elm327_ble_send_ascii_blocking(const char *ascii_cmd);
static bool can_rules_have_channel(const vehicle_override_t *ov, uint8_t channel);

// Accumulation buffer for multi-packet responses (21 01 responses span multiple BLE packets)
#define ACCUM_BUF_SIZE 512
static char s_accum_buf[ACCUM_BUF_SIZE];
static size_t s_accum_len = 0;
static int64_t s_accum_start_us = 0; // accumulation start time (us)

static const char *const s_zc6_can_monitor_enter_cmds_plain[] = {
    "ATE0\r", "ATL0\r", "ATS1\r", "ATH1\r", "ATMA\r",
};

// ---- Auto protocol detection ----
static volatile int s_protocol_detect_idx = -1;  // -1=not detecting, 0-10=protocol number being tried
static volatile bool s_protocol_detect_got_response = false;  // whether a valid response was received
static volatile int32_t s_protocol_detect_rpm = -1;  // detected RPM value (-1=none)

// ---- Oil-temp query mode conversion ----
// Convert oil_temp_query_mode_t to poll index (0-2)
static inline uint8_t oil_mode_to_poll_idx(oil_temp_query_mode_t mode) {
    switch (mode) {
        case OIL_TEMP_MODE_PID_5C: return 0;
        case OIL_TEMP_MODE_UDS_22_10_17: return 1;
        case OIL_TEMP_MODE_TOYOTA_21_01: return 2;
        case OIL_TEMP_MODE_MAZDA_22_111F: return 3;
        case OIL_TEMP_MODE_MAZDA_22_1310: return 4;
        case OIL_TEMP_MODE_MINI_22_5822: return 5;
        case OIL_TEMP_MODE_BMW_22_4402: return 6;
        case OIL_TEMP_MODE_BMW_22_03F3: return 7;
        case OIL_TEMP_MODE_BMW_G_22_4402: return 8;
        case OIL_TEMP_MODE_BMW_22_D002: return 9;
        case OIL_TEMP_MODE_BMW_22_111F: return 10;
        default: return 0;
    }
}

static const char *get_vehicle_fixed_header_cmd(void) {
    const vehicle_profile_t *vp = vehicle_profile_get_active();
    if (vehicle_profile_uses_standard_poll()) {
        if(s_scan_fixed_header[0])return s_scan_fixed_header;
        if(s_effective_obd_protocol==6||s_effective_obd_protocol==8)return "ATSH7DF\r";
        if(s_effective_obd_protocol==7||s_effective_obd_protocol==9)return "ATSH18DB33F1\r";
        return NULL; /* ISO/KWP/J1850: retain ELM's protocol-specific default. */
    }
    if (vp && vp->obd_functional_addr) {
        // 29-bit CAN functional broadcast (Honda Integra/Civic 11th gen and similar vehicles)
        if (vp->obd_29bit_functional) {
            return "ATSH18DB33F1\r";
        }
        // 11-bit functional addressing (vehicle-wide broadcast), same as phone apps; BMW needs this
        return "ATSH7DF\r";
    }
    return "ATSH7E0\r";     // physical addressing to the engine ECU; Subaru/default
}

// Initialize the oil-temp query strategy (reads the primary/secondary/tertiary priority chain from the vehicle profile config)
static void init_oil_temp_strategy(void) {
    // Take the profile's full priority chain: after the primary fails consecutively (threshold times), fall back to secondary, tertiary.
    const oil_temp_strategy_t *st = vehicle_profile_get_oil_temp_strategy();
    s_oil_mode_priority[0] = st ? st->primary    : OIL_TEMP_MODE_PID_5C;
    s_oil_mode_priority[1] = st ? st->secondary  : OIL_TEMP_MODE_NONE;
    s_oil_mode_priority[2] = st ? st->tertiary   : OIL_TEMP_MODE_NONE;
    s_oil_mode_priority[3] = st ? st->quaternary : OIL_TEMP_MODE_NONE;

    // ---- Data-driven override init ----
    s_ov = vehicle_profile_get_override();
    s_oil_formula_pri = s_ov ? s_ov->oil_primary : NULL;
    s_oil_formula_sec = s_ov ? s_ov->oil_secondary : NULL;
    s_oil_use_override = (s_oil_formula_pri != NULL);
    s_oil_override_idx = 0;
    s_oil_override_fail = 0;

    if (s_oil_mode_priority[0] == OIL_TEMP_MODE_TOYOTA_21_01) {
        // ZC/N6 fixed at d[33]
        s_mode21_oil_idx = 33;
    }
    s_last_mode21_oil = -100;
    s_last_mode21_oil_us = 0;
    s_mode21_hold_cnt = 0;

    // Reset failure counts
    memset(s_oil_mode_fail_count, 0, sizeof(s_oil_mode_fail_count));
    s_oil_query_mode = 0;
    s_vehicle_profile_inited = true;

    const vehicle_profile_t *profile = vehicle_profile_get_active();
    ESP_LOGD(TAG, "Oil temp strategy for [%s]: Primary=%d",
             profile ? profile->name : "UNKNOWN", s_oil_mode_priority[0]);
}

// Select the next query mode based on the current strategy
static oil_temp_query_mode_t get_next_oil_query_mode(uint8_t *poll_idx) {
    if (!s_vehicle_profile_inited) {
        init_oil_temp_strategy();
    }
    
    // Walk the priority list and find the first valid mode that hasn't failed too much
    for (int i = 0; i < 4; i++) {
        oil_temp_query_mode_t mode = s_oil_mode_priority[i];
        if (mode == OIL_TEMP_MODE_NONE) continue;
        
        uint8_t idx = oil_mode_to_poll_idx(mode);
        // Use this mode if its failure count is below the threshold
        if (s_oil_mode_fail_count[idx] < OIL_MODE_FAIL_THRESHOLD) {
            *poll_idx = idx;
            return mode;
        }
    }
    
    // All modes have failed too much; reset failure counts and use the primary mode
    memset(s_oil_mode_fail_count, 0, sizeof(s_oil_mode_fail_count));
    *poll_idx = oil_mode_to_poll_idx(s_oil_mode_priority[0]);
    return s_oil_mode_priority[0];
}

// Record oil-temp query success/failure (internal use)
static void record_oil_temp_success(oil_temp_query_mode_t mode) {
    uint8_t idx = oil_mode_to_poll_idx(mode);
    s_oil_mode_fail_count[idx] = 0;  // success, clear the failure count

    // Update diagnostic stats
    switch (mode) {
        case OIL_TEMP_MODE_PID_5C:
            s_oil_diag.mode0_ok++;
            break;
        case OIL_TEMP_MODE_UDS_22_10_17:
            s_oil_diag.mode1_ok++;
            break;
        case OIL_TEMP_MODE_TOYOTA_21_01:
            s_oil_diag.mode2_ok++;
            break;
        case OIL_TEMP_MODE_MAZDA_22_111F:
            s_oil_diag.mode3_ok++;
            break;
        case OIL_TEMP_MODE_MAZDA_22_1310:
            s_oil_diag.mode4_ok++;
            break;
        case OIL_TEMP_MODE_MINI_22_5822:
            s_oil_diag.mode6_ok++;
            break;
        case OIL_TEMP_MODE_BMW_22_4402:
            s_oil_diag.mode7_ok++;
            break;
        case OIL_TEMP_MODE_BMW_22_03F3:
            s_oil_diag.mode8_ok++;
            break;
        case OIL_TEMP_MODE_BMW_G_22_4402:
            s_oil_diag.mode9_ok++;
            break;
        case OIL_TEMP_MODE_BMW_22_D002:
            s_oil_diag.mode10_ok++;
            break;
        case OIL_TEMP_MODE_BMW_22_111F:
            s_oil_diag.mode11_ok++;
            break;
        default:
            break;
    }
}

static void record_oil_temp_failure(oil_temp_query_mode_t mode) {
    uint8_t idx = oil_mode_to_poll_idx(mode);
    s_oil_mode_fail_count[idx]++;

    // Update diagnostic stats
    switch (mode) {
        case OIL_TEMP_MODE_PID_5C:
            s_oil_diag.mode0_fail++;
            break;
        case OIL_TEMP_MODE_UDS_22_10_17:
            s_oil_diag.mode1_fail++;
            break;
        case OIL_TEMP_MODE_TOYOTA_21_01:
            s_oil_diag.mode2_fail++;
            break;
        case OIL_TEMP_MODE_MAZDA_22_111F:
            s_oil_diag.mode3_fail++;
            break;
        case OIL_TEMP_MODE_MAZDA_22_1310:
            s_oil_diag.mode4_fail++;
            break;
        case OIL_TEMP_MODE_MINI_22_5822:
            s_oil_diag.mode6_fail++;
            break;
        case OIL_TEMP_MODE_BMW_22_4402:
            s_oil_diag.mode7_fail++;
            break;
        case OIL_TEMP_MODE_BMW_22_03F3:
            s_oil_diag.mode8_fail++;
            break;
        case OIL_TEMP_MODE_BMW_G_22_4402:
            s_oil_diag.mode9_fail++;
            break;
        case OIL_TEMP_MODE_BMW_22_D002:
            s_oil_diag.mode10_fail++;
            break;
        case OIL_TEMP_MODE_BMW_22_111F:
            s_oil_diag.mode11_fail++;
            break;
        default:
            break;
    }
}

// Default callbacks and poll task (optional)
static void default_on_connected(void) { ESP_LOGD(TAG, "OBD BLE connected"); }
static void default_on_disconnected(void) { ESP_LOGD(TAG, "OBD BLE disconnected"); }
static void default_on_raw_notify(const uint8_t *data, size_t len) {
    // Receiving '>' means the ELM is ready; the next command can be sent
    // xTaskNotify wakes the poll task immediately, avoiding the 10ms polling overhead
    for (size_t i = 0; i < len; ++i) {
        if (data[i] == '>') {
            s_elm_ready = true;
            if (s_poll_task_handle) xTaskNotify(s_poll_task_handle, 0, eNoAction);
            break;
        }
    }
}

// ---- Protocol auto-detection ----
// Try all protocols (1-11); send 01 0C (read RPM) to check whether the protocol is valid
static int elm327_auto_detect_protocol(void) {
    ESP_LOGD(TAG, "=== Starting protocol auto-detect ===");

    // Send generic init commands first (no protocol selection involved)
    const char *init_cmds[] = {
        "ATZ\r",        // reset
        "ATE0\r",       // Echo off
        "ATAT1\r",      // adaptive timing
        "ATST 19\r",    // set timeout
    };

    for (size_t i = 0; i < sizeof(init_cmds) / sizeof(init_cmds[0]); ++i) {
        if (!s_connected) { ESP_LOGW(TAG, "[DETECT] BLE disconnected, aborting"); s_protocol_detect_idx = -1; return 0; }
        elm327_ble_send_ascii_blocking(init_cmds[i]);
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    // Try protocols 1-11
    for (int proto = 1; proto <= 11; proto++) {
        // Abort immediately on BLE disconnect — without this check, a disconnect mid-detect
        // would blindly cycle 11 protocols × 2s = up to 22s without feeding the watchdog → TWDT reboot.
        if (!s_connected) { ESP_LOGW(TAG, "[DETECT] BLE disconnected, aborting"); s_protocol_detect_idx = -1; return 0; }
        esp_task_wdt_reset();  // feed watchdog between protocols (the 2s wait below also feeds)
        ESP_LOGD(TAG, "[DETECT] Trying protocol %d...", proto);

        // Set the protocol
        char atsp_cmd[16];
        snprintf(atsp_cmd, sizeof(atsp_cmd), "ATSP%d\r", proto);
        elm327_ble_send_ascii_blocking(atsp_cmd);
        vTaskDelay(pdMS_TO_TICKS(50));

        // Init detection state
        s_protocol_detect_idx = proto;
        s_protocol_detect_got_response = false;
        s_protocol_detect_rpm = -1;

        // Send test command 01 0C (read RPM)
        elm327_ble_send_ascii_blocking("01 0C\r");
        esp_log_level_t prev_level = esp_log_level_get(TAG);
        esp_log_level_set(TAG, ESP_LOG_INFO);
        ESP_LOGD(TAG, "[DETECT] Sent 01 0C, waiting...");
        esp_log_level_set(TAG, prev_level);

        // Wait for a response, up to 2 seconds
        uint32_t wait_ms = 0;
        while (wait_ms < 2000) {
            if (!s_connected) { ESP_LOGW(TAG, "[DETECT] BLE disconnected, aborting"); s_protocol_detect_idx = -1; return 0; }
            esp_task_wdt_reset();  // feed the watchdog in the 2s wait loop (11 protocols × 2s could otherwise exceed TWDT)
            vTaskDelay(pdMS_TO_TICKS(50));
            wait_ms += 50;

            if (s_protocol_detect_got_response) {
                // Success!
                s_protocol_detect_idx = -1;  // exit detection mode
                ESP_LOGD(TAG, "[DETECT] Protocol %d: SUCCESS! (RPM=%ld)", proto, s_protocol_detect_rpm);
                return proto;
            }
        }

        ESP_LOGW(TAG, "[DETECT] Protocol %d: No valid response (timeout)", proto);
    }

    s_protocol_detect_idx = -1;  // exit detection mode
    ESP_LOGW(TAG, "=== Protocol auto-detect FAILED ===");
    return 0;  // 0 means detection failed; the default protocol 6 will be used
}

/* Generic runtime-only standard ECU discovery. No manufacturer identity inference. */
#define OBD_SCAN_ECUS 8u
#define OBD_SCAN_BUDGET_US 24000000LL
#define OBD_SCAN_RESTORE_US 4000000LL
#define OBD_SEEN_BITMAP 1u
#define OBD_SEEN_RPM 2u
#define OBD_SEEN_CLT 4u
#define OBD_SEEN_VOLT 8u
typedef struct {
    uint32_t response_id;
    uint32_t advertised[3]; /* 0100/0120/0140; never substituted for observed data. */
    uint16_t hits;
    uint8_t seen;
    uint8_t advertised_valid;
} obd_ecu_evidence_t;
static obd_ecu_evidence_t s_scan_ecus[OBD_SCAN_ECUS];
static uint8_t s_ecu_count,s_scan_protocol,s_scan_seen;
static bool s_scan_active,s_scan_done,s_scan_ok;
static uint32_t s_scan_epoch,s_scan_pending_epoch,s_scan_fragment_epoch;
static uint32_t s_scan_target_response;
static char s_scan_request[20];
static int64_t s_scan_deadline;
static uint32_t s_scan_run_generation;
static int64_t s_scan_retry_after_us;
static bool s_scan_retry_pending;
static atomic_bool s_scan_skip_verified;
static void start_scan(void);

static void obd_scan_reset_session(void)
{
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_scan_epoch++;s_scan_active=false;s_scan_done=false;s_ecu_count=0;s_scan_seen=0;
    s_fast_waiting=false;s_fast_enabled=false;
    s_scan_fragment_epoch=0;s_scan_target_response=0;s_scan_fixed_header[0]=0;
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
static void obd_scan_begin_fragment(void)
{
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_scan_fragment_epoch=s_scan_active?s_scan_pending_epoch:0;
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
/* Strict one-line standard frames only: CAN response ID, optional SF length,
 * Mode41, requested PID, exact required bytes. Different ECUs never combine. */
static bool obd_scan_line(const char *line,size_t len,uint8_t pid,uint8_t protocol,
                          uint32_t *id,uint32_t *bitmap)
{
    uint32_t t[12];uint8_t widths[12];unsigned n=0;size_t i=0;
    while(i<len&&n<12){
        while(i<len&&(line[i]==' '||line[i]=='\t'))i++;
        if(i==len||line[i]=='>')break;
        uint32_t v=0;unsigned w=0;
        while(i<len&&isxdigit((unsigned char)line[i])){
            unsigned char c=(unsigned char)line[i++];
            if(++w>8)return false;
            v=(v<<4)|(c<='9'?c-'0':(unsigned)(toupper(c)-'A'+10));
        }
        if(!w||(i<len&&line[i]!=' '&&line[i]!='\t'&&line[i]!='>'))return false;
        t[n]=v;widths[n++]=(uint8_t)w;
    }
    if(i<len&&line[i]!='>')return false;
    unsigned start=0;*id=0;
    bool can=protocol==0||(protocol>=6&&protocol<=9);
    if(n&&widths[0]==3&&t[0]>=0x7E8&&t[0]<=0x7EF){*id=t[0];start=1;}
    else if(n&&widths[0]==8&&(t[0]&0xFFFFFF00u)==0x18DAF100u){*id=t[0];start=1;}
    else if(n>=4&&t[0]==0x18&&t[1]==0xDA&&t[2]==0xF1&&t[3]<=255&&
            widths[0]==2&&widths[1]==2&&widths[2]==2&&widths[3]==2){*id=0x18DAF100u|t[3];start=4;}
    else if(can)return false;
    if(protocol==6||protocol==8){if(*id>0x7FF)return false;}
    if(protocol==7||protocol==9){if(*id&&*id<=0x7FF)return false;}
    unsigned header_tokens=start;
    unsigned sf_len=0;
    if(*id){
        if(start>=n||widths[start]!=2||t[start]>7||t[start]<3)return false;
        sf_len=t[start++]; /* CAF1+ATH1, ATD0: exact ISO-TP SF PCI. */
    }
    if(!*id){ /* Non-CAN ATH1 may prepend three ISO/J1850 header bytes. */
        while(start<n&&t[start]!=0x41)start++;
    }
    unsigned need=(pid==0||pid==0x20||pid==0x40)?4:(pid==0x0C||pid==0x42||pid==0x44||pid==0x5E)?2:(pid==0x05||pid==0x0D||pid==0x11||pid==0x04||pid==0x0F||pid==0x5C||pid==0x0B||pid==0x33)?1:0;
    if(!need||start+2+need>n||t[start]!=0x41||t[start+1]!=pid)return false;
    if(*id&&sf_len!=need+2)return false;
    if(!*id&&start!=0&&start!=3)return false;
    for(unsigned k=0;k<start&&!*id;k++)if(t[k]>255||widths[k]!=2)return false;
    for(unsigned k=start;k<n;k++)if(t[k]>255||widths[k]>2)return false;
    /* CAN padding is outside the SF payload and may be nonzero. */
    if(!*id&&n>start+2+need+1)return false;
    if(*id&&n>header_tokens+8)return false;
    if(pid==0x0C&&((t[start+2]<<8)|t[start+3])>60000)return false;
    if(pid==0x42&&((t[start+2]<<8)|t[start+3])>40000)return false;
    if(!*id&&start==3)*id=(t[0]<<16)|(t[1]<<8)|t[2]; /* Real legacy header, no invented CAN address. */
    *bitmap=need==4?(t[start+2]<<24)|(t[start+3]<<16)|(t[start+4]<<8)|t[start+5]:0;
    return true;
}
static bool obd_scan_has_ok(const char *response)
{
    if(strstr(response,"ERROR")||strstr(response,"NO DATA")||strstr(response,"UNABLE")||strchr(response,'?'))return false;
    for(const char *p=response;*p;){
        while(*p=='\r'||*p=='\n'||*p==' '||*p=='\t')p++;
        const char *e=p;while(*e&&*e!='\r'&&*e!='\n'&&*e!='>')e++;
        const char *trim=e;while(trim>p&&(trim[-1]==' '||trim[-1]=='\t'))trim--;
        if(trim-p==2&&p[0]=='O'&&p[1]=='K')return true;
        if(*e=='>')break;
        p=*e?e+1:e;
    }
    return false;
}
static void obd_scan_response(const char *response)
{
    char expected[20];uint32_t epoch;uint8_t protocol;bool active,associated;
    portENTER_CRITICAL(&s_elm_diag_mux);
    active=s_scan_active;epoch=s_scan_pending_epoch;protocol=s_scan_protocol;
    memcpy(expected,s_scan_request,sizeof expected);
    associated=s_elm_diag_response_associated&&!s_elm_diag_uncertain&&s_scan_fragment_epoch==epoch&&
        strcmp(expected,s_elm_diag_response_command)==0;
    portEXIT_CRITICAL(&s_elm_diag_mux);
    if(!active)return;
    bool ok=false;uint8_t detected=protocol;
    if(associated&&!strcmp(expected,"ATDPN")){
        const char *p=response;while(*p&&isspace((unsigned char)*p))p++;
        if(*p=='A')p++;
        if(isxdigit((unsigned char)*p)){
            unsigned x=*p<='9'?*p-'0':toupper((unsigned char)*p)-'A'+10;p++;
            while(*p&&isspace((unsigned char)*p))p++;
            if((*p=='>'||!*p)&&x>=1&&x<=9){detected=(uint8_t)x;ok=true;}
        }
    }else if(associated&&expected[0]=='A'){
        /* Reset replies identify the adapter, not a vehicle hit. */
        ok=!strcmp(expected,"ATZ")?(strstr(response,"ELM")!=NULL||strstr(response,"STN")!=NULL):obd_scan_has_ok(response);
    }else if(associated&&strlen(expected)==4&&expected[0]=='0'&&expected[1]=='1'){
        char *end;unsigned pid=(unsigned)strtoul(expected+2,&end,16);
        if(!*end&&pid<=255){
            for(const char *p=response;*p;){
                const char *e=p;while(*e&&*e!='\r'&&*e!='\n'&&*e!='>')e++;
                uint32_t id,bitmap;
                if(obd_scan_line(p,(size_t)(e-p),(uint8_t)pid,protocol,&id,&bitmap)&&
                   (!s_scan_target_response||id==s_scan_target_response)){
                    uint8_t bit=pid==0||pid==0x20||pid==0x40?OBD_SEEN_BITMAP:
                        pid==0x0C?OBD_SEEN_RPM:pid==0x05?OBD_SEEN_CLT:OBD_SEEN_VOLT;
                    portENTER_CRITICAL(&s_elm_diag_mux);
                    if(s_scan_active&&s_scan_pending_epoch==epoch){
                        s_scan_seen|=bit;
                        unsigned index=0;while(index<s_ecu_count&&s_scan_ecus[index].response_id!=id)index++;
                        if(id&&(index<s_ecu_count||s_ecu_count<OBD_SCAN_ECUS)){
                            if(index==s_ecu_count){memset(&s_scan_ecus[index],0,sizeof s_scan_ecus[index]);s_scan_ecus[index].response_id=id;s_ecu_count++;}
                            obd_ecu_evidence_t *v=&s_scan_ecus[index];
                            if(v->hits<UINT16_MAX)v->hits++;
                            v->seen|=bit;
                            if(bit==OBD_SEEN_BITMAP){unsigned b=pid/0x20;v->advertised[b]=bitmap;v->advertised_valid|=(uint8_t)(1u<<b);}
                        }
                    }
                    portEXIT_CRITICAL(&s_elm_diag_mux);ok=true;
                }
                if(*e=='>')break;
                p=*e?e+1:e;
            }
        }
    }
    s_accum_len=0;s_accum_buf[0]=0; /* Last shared buffer write before waking waiter. */
    portENTER_CRITICAL(&s_elm_diag_mux);
    if(associated&&s_scan_active&&s_scan_pending_epoch==epoch){s_scan_protocol=detected;s_scan_ok=ok;s_scan_done=true;}
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
/* The receive callback sets done only AFTER clearing its accumulation buffer.
 * A timed out command is never overlapped with a different request. */
static bool obd_scan_exchange(const char *command,unsigned wait_ms,bool *positive)
{
    if(s_scan_run_generation!=atomic_load(&s_obd_target_generation)||
       !s_connected||!s_notify_ready||s_ota_paused||!s_elm_ready||
       esp_timer_get_time()>=s_scan_deadline)return false;
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_scan_pending_epoch=++s_scan_epoch;s_scan_done=false;s_scan_ok=false;
    size_t n=0;for(const char *p=command;*p&&n<sizeof s_scan_request-1;p++)
        if((unsigned char)*p>32&&(unsigned char)*p<127)s_scan_request[n++]=*p;
    s_scan_request[n]=0;
    portEXIT_CRITICAL(&s_elm_diag_mux);
    if(!elm327_ble_send_ascii_blocking(command))return false;
    int64_t started=esp_timer_get_time();
    int64_t stop=started+(int64_t)wait_ms*1000;
    if(stop>s_scan_deadline)stop=s_scan_deadline;
    while(s_scan_run_generation==atomic_load(&s_obd_target_generation)&&
          s_connected&&s_notify_ready&&!s_ota_paused&&esp_timer_get_time()<stop){
        esp_task_wdt_reset();
        portENTER_CRITICAL(&s_elm_diag_mux);bool done=s_scan_done,ok=s_scan_ok;portEXIT_CRITICAL(&s_elm_diag_mux);
        if(done){if(positive)*positive=ok;return true;}
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    elm_diag_uncertain();
    ESP_LOGW(TAG,"scan wait cmd=%s ms=%u",s_scan_request,(unsigned)((esp_timer_get_time()-started)/1000));
    return false;
}
static bool obd_scan_required(const char *command)
{
    /* Real iCarPro2S ATZ banner takes ~1.3-1.5s; ordinary ACK is faster.
     * Reset has its own bounded allowance inside the same total scan budget. */
    bool positive=false;return obd_scan_exchange(command,!strcmp(command,"ATZ\r")?3000:1200,&positive)&&positive;
}
static void obd_scan_clear_evidence(uint8_t protocol)
{
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_ecu_count=0;s_scan_seen=0;s_scan_protocol=protocol;s_scan_target_response=0;
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
static bool obd_scan_setup(void)
{
    const char *setup[]={"ATZ\r","ATE0\r","ATL0\r","ATS1\r","ATH1\r","ATD0\r","ATM0\r","ATCRA\r","ATCAF1\r","ATAT1\r","ATST19\r"};
    for(unsigned i=0;i<sizeof setup/sizeof setup[0];i++)if(!obd_scan_required(setup[i]))return false;
    return true;
}
static bool obd_scan_generic_init(void)
{
    bool positive=false,verified=false,hint_stage=false;
    bool skipped=atomic_load(&s_scan_skip_verified);
    s_scan_run_generation=atomic_load(&s_obd_target_generation);
    obd_scan_reset_session();
    portENTER_CRITICAL(&s_elm_diag_mux);s_scan_active=true;s_scan_protocol=0;portEXIT_CRITICAL(&s_elm_diag_mux);
    s_scan_deadline=esp_timer_get_time()+OBD_SCAN_BUDGET_US-OBD_SCAN_RESTORE_US;
    ESP_LOGI(TAG,"scan begin Generic verified-hint then auto maxECU=8");
    if(!obd_scan_setup())goto fail;
    /* This is a verified transport route, not a BMW identity/profile match. */
    if(!skipped){
        hint_stage=true;obd_scan_clear_evidence(6);
        if(!obd_scan_required("ATSP6\r")||!obd_scan_required("ATSH7DF\r")||
           !obd_scan_exchange("0100\r",2500,&positive))goto fail;
        verified=positive;
        if(!verified){if(!obd_scan_exchange("010C\r",1500,&positive))goto fail;verified=positive;}
        if(verified){
            if(!obd_scan_exchange("ATDPN\r",1200,&positive))goto fail;
            verified=positive&&s_scan_protocol==6;
        }
        hint_stage=false;
    }
    if(verified)ESP_LOGI(TAG,"scan verified hint=6/7DF positive Mode01, no make inference");
    else{
        /* ATZ clears custom7DF before auto searches 29-bit/non-CAN defaults.
         * If hint lost its prompt, the NEXT connection bypasses it safely. */
        obd_scan_clear_evidence(0);
        if((!skipped&&!obd_scan_setup())||!obd_scan_required("ATSP0\r")||
           !obd_scan_exchange("0100\r",6500,&positive)||
           !obd_scan_exchange("ATDPN\r",1200,&positive))goto fail;
    }
    uint8_t protocol=s_scan_protocol;
    if(protocol){
        if(!obd_scan_exchange("010C\r",2000,&positive)||!obd_scan_exchange("0105\r",1200,&positive)||!obd_scan_exchange("0142\r",1200,&positive))goto fail;
        if(s_scan_seen&OBD_SEEN_BITMAP){
            if(!obd_scan_exchange("0120\r",1200,&positive)||!obd_scan_exchange("0140\r",1200,&positive))goto fail;
        }
    }else{
        /* A cached protocol is only an unverified physical-probe candidate.
         * It is never persisted or promoted without genuine Mode01 data. */
        uint8_t hint=nvs_cfg_get()->protocol;
        protocol=hint>=6&&hint<=9?hint:6;
        char sp[12];snprintf(sp,sizeof sp,"ATSP%u\r",(unsigned)protocol);
        if(!obd_scan_required(sp))goto fail;
        obd_scan_clear_evidence(protocol);
        ESP_LOGI(TAG,"scan unknown protocol; candidate=%u unverified",(unsigned)protocol);
    }
    if(!(s_scan_seen&(OBD_SEEN_RPM|OBD_SEEN_CLT|OBD_SEEN_VOLT))&&protocol>=6&&protocol<=9){
        /* Standard read-only request addresses, one pass, no manufacturer UDS. */
        for(unsigned i=0;i<8&&esp_timer_get_time()+5000000<s_scan_deadline;i++){
            char header[20];snprintf(header,sizeof header,protocol==6||protocol==8?"ATSH%03X\r":"ATSH18DA%02XF1\r",protocol==6||protocol==8?0x7E0+i:0x10+i);
            portENTER_CRITICAL(&s_elm_diag_mux);
            s_scan_target_response=protocol==6||protocol==8?0x7E8+i:0x18DAF100u+0x10+i;
            portEXIT_CRITICAL(&s_elm_diag_mux);
            if(!obd_scan_required(header)||!obd_scan_exchange("010C\r",1200,&positive))goto fail;
            bool rpm=positive,usable=rpm;
            if(!obd_scan_exchange("0105\r",1200,&positive))goto fail;
            usable|=positive;
            if(!obd_scan_exchange("0142\r",1200,&positive))goto fail;
            usable|=positive;
            if(usable&&(rpm||!s_scan_fixed_header[0]))memcpy(s_scan_fixed_header,header,strlen(header)+1);
            if(rpm)break; /* Retain first truly observed RPM route, including zero. */
        }
        if(s_scan_fixed_header[0]){
            if(!obd_scan_exchange("ATDPN\r",1200,&positive)||!positive||s_scan_protocol!=protocol)goto fail;
        }
    }
    if(!protocol||!(s_scan_seen&(OBD_SEEN_BITMAP|OBD_SEEN_RPM|OBD_SEEN_CLT|OBD_SEEN_VOLT)))goto fail;
    s_effective_obd_protocol=protocol;
    s_scan_deadline+=OBD_SCAN_RESTORE_US;
    char sp[12];snprintf(sp,sizeof sp,"ATSP%u\r",(unsigned)protocol);
    if(!obd_scan_required("ATH0\r"))goto fail;
    if(!obd_scan_required(sp))goto fail;
    const char *header=get_vehicle_fixed_header_cmd();
    if(header&&!obd_scan_required(header))goto fail;
    uint32_t preferred=0;unsigned best_score=0;
    for(unsigned i=0;i<s_ecu_count;i++){
        const obd_ecu_evidence_t *v=&s_scan_ecus[i];
        unsigned score=(v->seen&OBD_SEEN_RPM?16u:0u)+(v->seen&OBD_SEEN_CLT?8u:0u)+
            (v->seen&OBD_SEEN_VOLT?4u:0u)+(v->seen&OBD_SEEN_BITMAP?1u:0u);
        if(score>best_score){best_score=score;preferred=v->response_id;}
        ESP_LOGI(TAG,"scan hit id=%08lX n=%u seen=%u adv=%u",(unsigned long)v->response_id,(unsigned)v->hits,(unsigned)v->seen,(unsigned)v->advertised_valid);
    }
    ESP_LOGI(TAG,"scan end p=%u ECU=%u seen=%u e=%08lX route=%s",(unsigned)protocol,(unsigned)s_ecu_count,(unsigned)s_scan_seen,(unsigned long)preferred,s_scan_fixed_header[0]?"physical-observed":"broadcast/default");
    s_scan_retry_pending=false;atomic_store(&s_scan_skip_verified,false);
    portENTER_CRITICAL(&s_elm_diag_mux);s_scan_active=false;portEXIT_CRITICAL(&s_elm_diag_mux);
    return true;
fail:
    if(s_scan_run_generation!=atomic_load(&s_obd_target_generation))return false;
    if(hint_stage)atomic_store(&s_scan_skip_verified,true);
    portENTER_CRITICAL(&s_elm_diag_mux);s_scan_active=false;s_scan_epoch++;s_scan_fixed_header[0]=0;portEXIT_CRITICAL(&s_elm_diag_mux);
    s_scan_retry_after_us=esp_timer_get_time()+5000000;
    s_scan_retry_pending=true;
    s_notify_ready=false; /* No normal polling in an un-restored context. */
    ESP_LOGW(TAG,"scan aborted; no guessed route, reconnect after cooldown");
    if(s_connected)esp_ble_gattc_close(s_gattc_if,s_conn_id);
    return false;
}


/* STAGED30: Generic standard read-only deadline scheduler. No interpolated data.
 * Due times are objectives, not an impossible guarantee across slow wire replies.
 * Two consecutive RPM requests maximum when another advertised field is overdue. */
static const struct { uint8_t pid; uint16_t period_ms; } s_fast_fields[]={
    {0x0C,150},{0x0D,300},{0x11,500},{0x04,500},
    {0x05,1000},{0x42,2000},{0x0F,1000},{0x44,2000},
    {0x5C,2000},{0x5E,2000},{0x0B,500},{0x33,2000}
};
#define FAST_FIELDS (sizeof s_fast_fields/sizeof s_fast_fields[0])
static uint32_t s_fast_due[FAST_FIELDS];
static int64_t s_fast_last_step_us;
static uint32_t s_fast_generation;
static uint8_t s_fast_pid,s_fast_rpm_run;

static void obd_fast_reset(void)
{
    s_fast_last_step_us=esp_timer_get_time();
    for(unsigned i=0;i<FAST_FIELDS;i++)s_fast_due[i]=(uint32_t)s_fast_last_step_us;
    s_fast_rpm_run=0;
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_fast_waiting=false;s_fast_done=false;s_fast_generation=atomic_load(&s_obd_target_generation);
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
/* 1 advertised, 0 unknown, -1 confirmed absent. Union only the selected route.
 * Do not use bitmap from unrelated physical ECU; absent higher segment is unknown. */
static int obd_fast_support(uint8_t pid)
{
    unsigned segment=(pid-1u)/32u,bit=31u-(pid-1u)%32u;bool known=false;
    uint32_t response=0;
    if(s_scan_fixed_header[0]){
        uint32_t request=(uint32_t)strtoul(s_scan_fixed_header+4,NULL,16);
        response=request<=0x7FF?request+8:0x18DAF100u|((request>>8)&255u);
    }
    for(unsigned i=0;i<s_ecu_count;i++){
        const obd_ecu_evidence_t *e=&s_scan_ecus[i];
        if(response&&e->response_id!=response)continue;
        unsigned observed=pid==0x0C?OBD_SEEN_RPM:pid==0x05?OBD_SEEN_CLT:pid==0x42?OBD_SEEN_VOLT:0;
        if(e->seen&observed)return 1; /* A genuine observed reply outranks an advertised bit. */
        if(!(e->advertised_valid&(1u<<segment)))continue;
        known=true;if(e->advertised[segment]&(1u<<bit))return 1;
    }
    return known?-1:0;
}
/* Called only after complete prompt, before normal cache publication. False
 * means discard this unrelated/invalid frame; never promote late data to fresh. */
static bool obd_fast_response(const char *response,bool complete)
{
    uint8_t pid;bool trusted;
    portENTER_CRITICAL(&s_elm_diag_mux);
    pid=s_fast_pid;
    trusted=s_fast_waiting&&!s_fast_done&&s_fast_generation==atomic_load(&s_obd_target_generation)&&
        s_elm_diag_response_associated&&!s_elm_diag_uncertain;
    portEXIT_CRITICAL(&s_elm_diag_mux);
    char command[8];snprintf(command,sizeof command,"01%02X",pid);
    trusted=trusted&&complete&&!strcmp(command,s_elm_diag_response_command);
    bool positive=false;
    if(trusted)for(const char *line=response;*line;){
        const char *end=line;while(*end&&*end!='\r'&&*end!='\n'&&*end!='>')end++;
        uint32_t id,bitmap;
        bool valid=obd_scan_line(line,(size_t)(end-line),pid,1,&id,&bitmap);
        const char *mode41=strstr(line,"41 ");
        if(mode41&&mode41<end&&!valid){positive=false;break;}
        if(valid)positive=true;
        if(!*end||*end=='>')break;
        line=end+1;
    }
    portENTER_CRITICAL(&s_elm_diag_mux);s_fast_ok=trusted&&positive;s_fast_associated=trusted;portEXIT_CRITICAL(&s_elm_diag_mux);
    return trusted&&positive;
}
/* Shared buffer must be fully cleared and real parsing complete before done. */
static void obd_fast_complete(void)
{
    portENTER_CRITICAL(&s_elm_diag_mux);
    if(s_fast_waiting&&s_fast_generation==atomic_load(&s_obd_target_generation)&&
       s_fast_associated&&!s_elm_diag_uncertain)s_fast_done=true;
    portEXIT_CRITICAL(&s_elm_diag_mux);
}
static void obd_fast_abort(unsigned generation)
{
    if(generation!=atomic_load(&s_obd_target_generation))return; /* Never close a new peer. */
    portENTER_CRITICAL(&s_elm_diag_mux);s_fast_waiting=false;s_scan_epoch++;portEXIT_CRITICAL(&s_elm_diag_mux);
    elm_diag_uncertain();s_notify_ready=false;s_scan_retry_pending=true;
    s_scan_retry_after_us=esp_timer_get_time()+5000000;
    esp_ble_gattc_close(s_gattc_if,s_conn_id);
}
static bool obd_fast_profile_allows(uint8_t pid,int support) {
    if(pid==0x0B||pid==0x33)return false;
    return support>=0 && ((pid!=0x5C&&pid!=0x5E)||support==1);
}
static void obd_fast_step(void)
{
    int64_t clock=esp_timer_get_time();uint32_t now=(uint32_t)clock;
    if(clock-s_fast_last_step_us>1000000)obd_fast_reset(); /* Long showroom/OTA pause: re-anchor, no burst. */
    s_fast_last_step_us=clock;int chosen=-1,slow=-1;int32_t oldest=-1;
    for(unsigned i=0;i<FAST_FIELDS;i++){
        uint8_t requested=s_fast_fields[i].pid;
        int support=obd_fast_support(requested);
        if(!obd_fast_profile_allows(requested,support))continue;
        int32_t late=(int32_t)(now-s_fast_due[i]);if(late<0)continue;
        if(i==0)chosen=0;
        else if(late>oldest){oldest=late;slow=(int)i;}
    }
    if(slow>=0&&(chosen<0||s_fast_rpm_run>=2))chosen=slow;
    if(chosen<0){vTaskDelay(pdMS_TO_TICKS(10));return;}
    unsigned generation=atomic_load(&s_obd_target_generation);
    /* Never call legacy forced-send path unless already ready. */
    if(!s_elm_ready){obd_fast_abort(generation);return;}
    uint8_t pid=s_fast_fields[chosen].pid;char command[8];snprintf(command,sizeof command,"01%02X\r",pid);
    portENTER_CRITICAL(&s_elm_diag_mux);
    s_fast_generation=generation;s_fast_pid=pid;s_fast_waiting=true;s_fast_done=false;s_fast_ok=false;s_fast_associated=false;
    portEXIT_CRITICAL(&s_elm_diag_mux);
    if(!elm327_ble_send_ascii_blocking(command)){obd_fast_abort(generation);return;}
    int64_t deadline=clock+700000;bool done=false,ok=false;
    while(s_connected&&s_notify_ready&&generation==atomic_load(&s_obd_target_generation)&&!s_ota_paused&&esp_timer_get_time()<deadline){
        portENTER_CRITICAL(&s_elm_diag_mux);done=s_fast_done;ok=s_fast_ok;portEXIT_CRITICAL(&s_elm_diag_mux);
        if(done)break;
        esp_task_wdt_reset();ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(10));
    }
    if(!done){obd_fast_abort(generation);return;}
    portENTER_CRITICAL(&s_elm_diag_mux);s_fast_waiting=false;portEXIT_CRITICAL(&s_elm_diag_mux);
    int support=obd_fast_support(pid);
    if(!ok)s_fast_due[chosen]=(uint32_t)esp_timer_get_time()+5000000u; /* NO DATA is session cooldown, not unsupported. */
    else if(chosen==0){
        uint32_t next=s_fast_due[0]+150000u;
        s_fast_due[0]=(int32_t)(now-next)>150000?now+150000u:next; /* bounded phase-preserving re-anchor, no bursts */
    }else s_fast_due[chosen]=(uint32_t)esp_timer_get_time()+(uint32_t)(support==0?3000:s_fast_fields[chosen].period_ms)*1000u;
    if(chosen==0){if(s_fast_rpm_run<2)s_fast_rpm_run++;}else s_fast_rpm_run=0;
}

static void default_on_parsed_rpm(uint16_t rpm) { obd_data_set_rpm(rpm); }
static void default_on_parsed_speed(uint8_t kmh) {
    const vehicle_profile_t *p = vehicle_profile_get_active();
    float sc = (p && p->speed_scale > 0.0f) ? p->speed_scale : 1.0f;
    int32_t v = (int32_t)((float)kmh * sc + 0.5f);
    if (v > 255) v = 255;
    if (p && p->can_broadcast_mode && obd_data_get_rpm() < 800)
        v = 0;
    if (v <= 2) v = 0;
    obd_data_set_speed((uint8_t)v);
}
static void default_on_parsed_coolant_temp(uint32_t coolant_temp) {
    const vehicle_profile_t *profile = vehicle_profile_get_active();
    const vehicle_override_t *ov = vehicle_profile_get_override();
    if (profile && profile->can_broadcast_mode && can_rules_have_channel(ov, CH_COOLANT)) {
        s_can_coolant_last_us = esp_timer_get_time();
    }
    obd_data_set_coolant_temp((int16_t)coolant_temp);
}
static void default_on_parsed_intake_temp(uint32_t intake_temp) { obd_data_set_intake_temp((int16_t)intake_temp); }

// Inline helper: apply the oil-temp offset before storing
static inline void obd_data_set_oil_temp_with_offset(int16_t temp) {
    int16_t adjusted = temp + s_oil_temp_offset;
    if (adjusted < -20) adjusted = -20;
    if (adjusted > 150) adjusted = 150;
    obd_data_set_oil_temp(adjusted);
}

// float → int16_t rounding (safe for positive values; negative-value precision is a non-issue in the oil-temp range)
static inline int16_t oil_f2i(float f) { return (int16_t)(f + 0.5f); }

static void default_on_parsed_oil_temp(uint32_t oil_temp)
{
    static float s_oil_filtered = -100.0f;
    static int16_t s_oil_pending = -100;
    static uint8_t s_oil_pending_cnt = 0;
    const vehicle_profile_t *profile = vehicle_profile_get_active();
    const vehicle_override_t *ov = vehicle_profile_get_override();
    bool can_direct_oil = profile && profile->can_broadcast_mode &&
                          can_rules_have_channel(ov, CH_OIL_TEMP);

    int16_t in = (int16_t)oil_temp;
    s_oil_diag.last_raw_temp = in;

    if (in < -20 || in > 150) return;

    if (can_direct_oil) {
        s_can_oil_last_us = esp_timer_get_time();
        s_oil_filtered = (float)in;
        s_oil_pending = in;
        s_oil_pending_cnt = 0;
        s_oil_diag.last_filtered_temp = in;
        obd_data_set_oil_temp_with_offset(in);
        return;
    }

    if (s_oil_filtered <= -40.0f) {
        s_oil_filtered = (float)in;
        s_oil_pending = in;
        s_oil_pending_cnt = 1;
        s_oil_diag.last_filtered_temp = in;
        obd_data_set_oil_temp_with_offset(in);
        return;
    }

    float fdiff = s_oil_filtered - (float)in;
    int diff = (int)(fdiff >= 0 ? fdiff : -fdiff);

    if (diff <= 5) {
        s_oil_pending = -100;
        s_oil_pending_cnt = 0;
        s_oil_filtered = 0.65f * s_oil_filtered + 0.35f * (float)in;
        int16_t disp = oil_f2i(s_oil_filtered);
        s_oil_diag.last_filtered_temp = disp;
        obd_data_set_oil_temp_with_offset(disp);
        return;
    }

    if (diff <= 15) {
        if (s_oil_pending == in) {
            s_oil_pending_cnt++;
        } else {
            s_oil_pending = in;
            s_oil_pending_cnt = 1;
            return;
        }
        if (s_oil_pending_cnt >= 4) {
            s_oil_filtered = 0.5f * s_oil_filtered + 0.5f * (float)in;
            s_oil_pending = -100;
            s_oil_pending_cnt = 0;
            int16_t disp = oil_f2i(s_oil_filtered);
            s_oil_diag.last_filtered_temp = disp;
            obd_data_set_oil_temp_with_offset(disp);
        }
        return;
    }

    if (diff >= 20) {
        if (s_oil_pending == in) {
            s_oil_pending_cnt++;
        } else {
            s_oil_pending = in;
            s_oil_pending_cnt = 1;
            return;
        }
        if (s_oil_pending_cnt >= 3) {
            s_oil_filtered = (float)in;
            s_oil_pending = -100;
            s_oil_pending_cnt = 0;
            s_oil_diag.last_filtered_temp = in;
            obd_data_set_oil_temp_with_offset(in);
        }
    }
}
static void default_on_parsed_load_pct(uint32_t load_pct) { obd_data_set_load_pct((int16_t)load_pct); }
static void default_on_parsed_control_module_voltage(uint32_t bat_mv) { obd_data_set_bat_mv((int32_t)bat_mv); }
static void default_on_parsed_throttle_position(uint32_t tps_pct) { obd_data_set_tps((int16_t)tps_pct); }
static void default_on_parsed_gear(int8_t gear) {
    // Direct gear decode when available, raw value -> UI gear:
    //   3→R, 4→N, 5→1, 6→2, …  (0-2 reserved/Park etc.)
    if (gear == 3) {
        obd_data_set_gear(-1);  // R
    } else if (gear == 4) {
        obd_data_set_gear(GEAR_NEUTRAL);  // N/P
    } else if (gear >= 5 && gear <= 12) {
        obd_data_set_gear((int8_t)(gear - 4));  // 5→1, 6→2, ...
    } else {
        obd_data_set_gear(127);  // invalid/unknown
    }
}
// MAP(kPa) → turbo gauge pressure (0.1bar): gauge = (MAP - atmospheric ≈100kPa), 10kPa = 0.1bar
static void default_on_parsed_manifold_pressure(uint32_t map_kpa) {
    int16_t boost_x10 = (int16_t)(((int32_t)map_kpa - 100) / 10);
    if (boost_x10 < 0) boost_x10 = 0; // don't display negative pressure (vacuum), floor at 0
    obd_data_set_boost_x10(boost_x10);
}

static void can_expire_stale_temp_channels(void)
{
    const vehicle_profile_t *profile = vehicle_profile_get_active();
    const vehicle_override_t *ov = vehicle_profile_get_override();
    int64_t now_us;

    if (!profile || !profile->can_broadcast_mode || !ov || !ov->can_rules || ov->can_rule_count == 0) {
        return;
    }

    now_us = esp_timer_get_time();

    if (can_rules_have_channel(ov, CH_OIL_TEMP) && s_can_oil_last_us > 0 &&
        (now_us - s_can_oil_last_us) > ZC6_CAN_TEMP_STALE_US) {
        obd_data_set_oil_temp_invalid();
        s_oil_diag.last_raw_temp = -100;
        s_oil_diag.last_filtered_temp = -100;
        s_can_oil_last_us = 0;
    }

    if (can_rules_have_channel(ov, CH_COOLANT) && s_can_coolant_last_us > 0 &&
        (now_us - s_can_coolant_last_us) > ZC6_CAN_TEMP_STALE_US) {
        obd_data_set_coolant_temp(-40);
        s_can_coolant_last_us = 0;
    }
}

// PID 01 44: Commanded Equivalence Ratio (λ), formula: (A*256+B)/32768, range 0~<2
// λ=1.0 stoichiometric AFR (gasoline ~14.7:1), λ<1 rich, λ>1 lean
// Conversion: AFR = λ × 14.7, stored ×100 (1470 = 14.70:1)
static void default_on_parsed_afr(uint32_t afr_x100) {
    obd_data_set_afr_x100((int16_t)afr_x100);
}
// Mode 22 oil pressure (4436=B58, 586F=N55): absolute hPa → 0.1 bar (x10), setter clamps to [0, 200]
static void default_on_parsed_oil_pressure(uint32_t oil_pressure_hpa) {
    int32_t x10 = (int32_t)oil_pressure_hpa / 100;   // 100 hPa = 0.1 bar
    if (x10 < 0) x10 = 0;
    if (x10 > 200) x10 = 200;
    obd_data_set_oil_pressure_x10((int16_t)x10);
}
// Mode 22 gear (BMW 22 D0 31, ZF 8HP "BMW_GEAR_V2"): 0x00=N, 0x01..0x08=1..8; anything else invalid → ratio-calc fallback
static void default_on_parsed_obd_gear(uint8_t raw_gear) {
    if (raw_gear <= 0x08) {
        obd_data_set_gear((int8_t)raw_gear);   // 0=N, 1..8 = forward gear
    } else {
        obd_data_set_gear(127);                // invalid/unknown (R/P/0xFF) → falls back to the computed gear
    }
}

// ---- CAN continuous monitor: enter/exit/byte-wise feed/line-wise parse ----

static void zc6_can_monitor_enter(void)
{
    const char *const *cmds = s_zc6_can_monitor_enter_cmds_plain;
    size_t cmd_count = sizeof(s_zc6_can_monitor_enter_cmds_plain) / sizeof(s_zc6_can_monitor_enter_cmds_plain[0]);
    s_zc6_can_monitor_active = false;
    s_zc6_can_monitor_len = 0;
    s_zc6_can_monitor_buf[0] = '\0';
    s_zc6_can_monitor_entered_us = 0;
    s_accum_len = 0;
    s_accum_buf[0] = '\0';

    for (size_t i = 0; i < cmd_count; i++) {
        elm327_ble_send_ascii_blocking(cmds[i]);
        vTaskDelay(pdMS_TO_TICKS(i + 1 == cmd_count ? 80 : 30));
    }
    s_zc6_can_monitor_active = true;
    s_zc6_can_monitor_entered_us = esp_timer_get_time();
}

static void zc6_can_monitor_exit(void)
{
    s_zc6_can_monitor_active = false;
    s_elm_ready = true;
    uint8_t stop = '\r';
    elm327_ble_send_command(&stop, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
    // Restore CAN filtering to the adapter default before normal OBD polling resumes.
    elm327_ble_send_ascii_blocking("ATCRA\r");
    s_accum_len = 0;
    s_accum_buf[0] = '\0';
    // Light restore: clear filter + headers off, no full ATZ reset (saves ~500ms)
    elm327_ble_send_ascii_blocking("AT AR\r");
    elm327_ble_send_ascii_blocking("AT H0\r");
}

static void zc6_can_monitor_probe_window(uint32_t window_ms)
{
    if (window_ms == 0) return;
    zc6_can_monitor_enter();
    vTaskDelay(pdMS_TO_TICKS(window_ms));
    zc6_can_monitor_exit();
}

static bool can_monitor_parse_hex_token(const char **cursor, uint32_t *value_out)
{
    const char *p = (cursor != NULL) ? *cursor : NULL;
    uint32_t value = 0;
    bool saw_digit = false;

    if (!p || !value_out) return false;

    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        p += 2;
    }

    while (*p && isxdigit((unsigned char)*p)) {
        uint8_t nibble;
        if (*p >= '0' && *p <= '9') {
            nibble = (uint8_t)(*p - '0');
        } else if (*p >= 'A' && *p <= 'F') {
            nibble = (uint8_t)(*p - 'A' + 10);
        } else {
            nibble = (uint8_t)(*p - 'a' + 10);
        }
        value = (value << 4) | nibble;
        saw_digit = true;
        ++p;
    }

    if (!saw_digit) return false;
    *cursor = p;
    *value_out = value;
    return true;
}

static bool can_monitor_parse_hex_byte(const char **cursor, uint8_t *value_out)
{
    uint32_t value = 0;
    if (!can_monitor_parse_hex_token(cursor, &value) || value > 0xFFu || !value_out) return false;
    *value_out = (uint8_t)value;
    return true;
}

static bool can_monitor_parse_line_fast(const char *line, uint16_t *can_id_out,
                                        uint8_t data[8], uint8_t *data_len_out)
{
    const char *p = line;
    uint32_t token = 0;
    uint32_t tokens[9] = {0};
    uint8_t count = 0;
    uint8_t start = 0;

    if (!line || !can_id_out || !data || !data_len_out) return false;

    while (*p && !isxdigit((unsigned char)*p)) {
        if (*p == '>' || *p == '\r' || *p == '\n') return false;
        ++p;
    }

    if (!can_monitor_parse_hex_token(&p, &token)) return false;
    if (*p == ':' && token <= 0xFFu) {
        ++p;
        while (*p && !isxdigit((unsigned char)*p)) {
            if (*p == '>' || *p == '\r' || *p == '\n') return false;
            ++p;
        }
        if (!can_monitor_parse_hex_token(&p, &token)) return false;
    }
    if (token > 0x7FFu) return false;
    *can_id_out = (uint16_t)token;

    while (*p && count < 9) {
        while (*p && !isxdigit((unsigned char)*p)) {
            if (*p == '>' || *p == '\r' || *p == '\n') goto out;
            ++p;
        }
        if (!*p) break;

        if (!can_monitor_parse_hex_token(&p, &token) || token > 0xFFu) return false;
        tokens[count++] = token;
    }

    if (count == 9 && tokens[0] <= 8u) {
        start = 1;
    }
    for (uint8_t i = start; i < count && (i - start) < 8; ++i) {
        data[i - start] = (uint8_t)tokens[i];
    }

out:
    *data_len_out = (uint8_t)(count - start);
    return *data_len_out > 0;
}

static bool can_rules_have_channel(const vehicle_override_t *ov, uint8_t channel)
{
    if (!ov || !ov->can_rules || ov->can_rule_count == 0) return false;
    for (uint8_t i = 0; i < ov->can_rule_count; ++i) {
        if (ov->can_rules[i].channel == channel) return true;
    }
    return false;
}

// Line-wise parse: generic rule tables; skipped automatically if the frame is not on the bus
static bool zc6_can_monitor_parse_line(const char *line)
{
    // Generic CAN frame parse: extract CAN ID + data from the ATMA line and apply the override rules
    const vehicle_override_t *ov = vehicle_profile_get_override();
    if (!ov || !ov->can_rules || ov->can_rule_count == 0) {
        return false;
    }

    // Parse the CAN ID and payload once. Most adapters emit a plain "<ID> <D0> <D1> ..." monitor line;
    // the fast path handles that without `sscanf`, and the slow path below keeps compatibility with odd adapters.
    uint16_t line_id = 0;
    uint8_t data[8] = {0};
    uint8_t data_len = 0;
    bool parsed_fast = can_monitor_parse_line_fast(line, &line_id, data, &data_len);

    bool id_watched = false;
    for (uint8_t i = 0; i < ov->can_rule_count; i++) {
        if (ov->can_rules[i].can_id == line_id) { id_watched = true; break; }
    }
    if (!id_watched) return false;

    if (!parsed_fast) {
        int vals = sscanf(line, "%hx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                          &line_id,
                          &data[0],&data[1],&data[2],&data[3],
                          &data[4],&data[5],&data[6],&data[7]);
        if (vals < 2 || line_id == 0) return false;
        data_len = (uint8_t)(vals - 1);
    }

    float channels[CH_COUNT];
    for (int c = 0; c < CH_COUNT; c++) channels[c] = -32768.0f;
    can_apply_rules(ov->can_rules, ov->can_rule_count, line_id, data, channels);

    if (channels[CH_RPM] >= 0 && s_cbs.on_parsed_rpm)
        s_cbs.on_parsed_rpm((uint16_t)channels[CH_RPM]);
    if (channels[CH_SPEED] >= 0 && s_cbs.on_parsed_speed_kmh)
        s_cbs.on_parsed_speed_kmh((uint8_t)channels[CH_SPEED]);
    if (channels[CH_OIL_TEMP] > -40 && channels[CH_OIL_TEMP] <= 215 && s_cbs.on_parsed_oil_temp)
        s_cbs.on_parsed_oil_temp((uint32_t)(int16_t)channels[CH_OIL_TEMP]);
    if (channels[CH_COOLANT] > -40 && channels[CH_COOLANT] <= 215 && s_cbs.on_parsed_coolant_temp)
        s_cbs.on_parsed_coolant_temp((uint32_t)(int16_t)channels[CH_COOLANT]);
    if (channels[CH_TPS] >= 0 && s_cbs.on_parsed_throttle_position)
        s_cbs.on_parsed_throttle_position((uint32_t)channels[CH_TPS]);
    if (channels[CH_GEAR] > 0 && channels[CH_GEAR] < 127 && s_cbs.on_parsed_gear)
        s_cbs.on_parsed_gear((int8_t)channels[CH_GEAR]);

    mark_obd_data_valid();
    return true;
}

// Byte-wise feed: split into lines on \r\n and call parse_line for each line
static void zc6_can_monitor_feed(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char ch = (char)data[i];
        if (ch == '>') { s_elm_ready = true; continue; }
        if (ch == '\r' || ch == '\n') {
            if (s_zc6_can_monitor_len > 0) {
                s_zc6_can_monitor_buf[s_zc6_can_monitor_len] = '\0';
                zc6_can_monitor_parse_line(s_zc6_can_monitor_buf);
                s_zc6_can_monitor_len = 0;
            }
            continue;
        }
        if ((unsigned char)ch < 0x20 || (unsigned char)ch > 0x7E) continue;
        if (s_zc6_can_monitor_len + 1 >= ZC6_CAN_MONITOR_BUF_SIZE)
            s_zc6_can_monitor_len = 0;
        s_zc6_can_monitor_buf[s_zc6_can_monitor_len++] = ch;
    }
}

// ELM327 init sequence (protocol selection + AT config + bus warm-up + oil-temp strategy).
// Called on every (re)connect and only after the notify subscription is ready, so handshake responses aren't lost before subscribing.
static void do_elm_init(void) {
    atomic_store(&s_fuel_probe_pending,false);
    atomic_store(&s_fuel_supported,false);
    s_fuel_last_request_us=0;
    atomic_store(&s_fuel_attempts,0);
    char atsp_cmd[16];
    const nvs_user_cfg_t *cfg = nvs_cfg_get();

    if(vehicle_profile_uses_standard_poll()){
        if(!obd_scan_generic_init())return;
        goto elm_init_standard_ready;
    }

    // ---- Protocol selection ----
    uint8_t protocol_to_use = cfg->protocol;
    // Vehicle-forced protocol takes priority (some ECUs/adapters are unstable with auto-detect; skip detection when a profile sets one)
    const vehicle_profile_t *vp_proto = vehicle_profile_get_active();
    if (vp_proto && vp_proto->forced_protocol != 0) {
        protocol_to_use = vp_proto->forced_protocol;
        ESP_LOGD(TAG, "Vehicle '%s' forces protocol %d (skip auto-detect)", vp_proto->name, protocol_to_use);
    } else if (protocol_to_use == 0) {
        // Auto protocol detection — abort immediately if BLE dropped during detection
        ESP_LOGD(TAG, "Protocol auto-detect enabled (current NVS: 0-auto)");
        int detected_proto = elm327_auto_detect_protocol();
        if (!s_connected) {
            ESP_LOGW(TAG, "BLE disconnected during protocol auto-detect, aborting init");
            return;
        }
        if (detected_proto > 0) {
            protocol_to_use = (uint8_t)detected_proto;
            nvs_user_cfg_t new_cfg = *cfg;
            new_cfg.protocol = protocol_to_use;
            nvs_cfg_set(&new_cfg);
            ESP_LOGD(TAG, "Protocol auto-detect SUCCESS! Saving protocol %d to NVS", protocol_to_use);
        } else {
            protocol_to_use = 6;
            ESP_LOGW(TAG, "Protocol auto-detect FAILED, using fallback protocol 6");
        }
    }

    s_effective_obd_protocol=protocol_to_use;
    snprintf(atsp_cmd, sizeof(atsp_cmd), "ATSP%d\r", protocol_to_use);
    const char *fixed_header_cmd = get_vehicle_fixed_header_cmd();
    // Timeout command: prefer the override's obd_timeout first, then the profile's obd_timeout, default 0x19
    const vehicle_override_t *vp_ov = vehicle_profile_get_override();
    char atst_cmd[12];
    uint8_t timeout_val = (vp_ov && vp_ov->obd_timeout) ? vp_ov->obd_timeout
                          : ((vp_proto && vp_proto->obd_timeout) ? vp_proto->obd_timeout : 0x19);
    snprintf(atst_cmd, sizeof(atst_cmd), "ATST %02X\r", timeout_val);
    char header_diag[20]="default";
    if(fixed_header_cmd)snprintf(header_diag,sizeof header_diag,"%.*s",
        (int)strcspn(fixed_header_cmd+4,"\r\n"),fixed_header_cmd+4);
    ESP_LOGI(TAG,"init profile=%s proto=%u hdr=%s",vp_proto?vp_proto->name:"?",
             (unsigned)protocol_to_use,header_diag);
    const char *init_cmds[] = {
        "ATZ\r", "ATE0\r", "ATL0\r", "ATS1\r", "ATH0\r", "ATAT1\r", atst_cmd,
        atsp_cmd, fixed_header_cmd,
    };
    for (size_t i = 0; i < (sizeof(init_cmds) / sizeof(init_cmds[0])); ++i) {
        if (!init_cmds[i])continue;
        if (!s_connected) { ESP_LOGW(TAG, "BLE disconnected during ELM init seq, aborting"); return; }
        elm327_ble_send_ascii_blocking(init_cmds[i]);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    // Bus warm-up: send 01 00 a few extra times to give the vehicle CAN/ELM protocol time to handshake (the bus may not be awake on a cold start)
    for (int probe = 0; probe < 3; ++probe) {
        if (!s_connected) { ESP_LOGW(TAG, "BLE disconnected during bus warm-up, aborting"); return; }
        elm327_ble_send_ascii_blocking("01 00\r");
        vTaskDelay(pdMS_TO_TICKS(150));
    }

elm_init_standard_ready:
    // ---- Init the oil-temp query strategy (based on vehicle profile config) ----
    init_oil_temp_strategy();
    obd_data_reset_temp_cache();
    /* One bounded standard read in the restored standard header/protocol context.
     * No fuel query is scheduled until a complete positive 41 40 bitmap arrives. */
    if(vehicle_profile_uses_standard_poll()){
        /* Discovery completed0140 before ATH0 restore; no asynchronous init tail
         * can race first deadline request. Reuse real selected-route evidence. */
        atomic_store(&s_fuel_supported,obd_fast_support(0x5E)==1);
    }else if(s_connected&&s_protocol_detect_idx<0){
        atomic_store(&s_fuel_probe_pending,true);
        if(!elm327_ble_send_ascii_blocking("01 40\r"))atomic_store(&s_fuel_probe_pending,false);
    }
    s_zc_can_obd_phase = false;
    s_zc_can_obd_round_started = false;
    s_zc6_can_temp_probe_last_us = 0;
    s_last_obd_valid_us = esp_timer_get_time();   // give a fresh "valid data" baseline so self-heal doesn't trigger right after init
}

/* A connected ACL without a confirmed CCCD must not wait forever. The real
 * DISCONNECT callback owns state changes and reconnects the saved target.
 * Late, successful subscription is allowed to restore this valid connection. */
static void obd_wait_notify_step(int64_t *deadline_us)
{
    if (!s_connected || s_notify_ready || s_ota_paused) {
        *deadline_us = 0;
        return;
    }
    int64_t now = esp_timer_get_time();
    if (*deadline_us == 0) {
        *deadline_us = now + 10000000;
        return;
    }
    if (now < *deadline_us) return;
    *deadline_us = now + 5000000;
    ESP_LOGW(TAG, "Notify setup timeout: reconnect bound adapter");
    esp_err_t err = esp_ble_gattc_close(s_gattc_if, s_conn_id);
    if (err != ESP_OK) ESP_LOGW(TAG, "Notify reconnect close failed: %d", (int)err);
}

/* Bounded, integer-only health log, called once per 10s outside the UI lock.
 * Ams is actual parsed data age; ELM initialization never refreshes it.
 * Cache values are a snapshot, not proof that each individual PID is fresh. */
void elm327_ble_log_diagnostics(void)
{
    uint32_t count;
    int64_t last;
    portENTER_CRITICAL(&s_elm_diag_mux);
    count = s_valid_frame_count;
    last = s_real_valid_us;
    portEXIT_CRITICAL(&s_elm_diag_mux);
    int64_t age = last ? (esp_timer_get_time() - last) / 1000 : -1;
    if (age > INT32_MAX) age = INT32_MAX;
    obd_data_snapshot_t cache;
    obd_data_get_snapshot(&cache);
    ESP_LOGI(TAG, "obd C=%u N=%u F=%u Ams=%ld cache R=%u V=%u W=%d I=%d B=%d",
             (unsigned)s_connected, (unsigned)s_notify_ready, (unsigned)count,
             (long)age, (unsigned)cache.rpm, (unsigned)cache.speed,
             (int)cache.coolant_temp, (int)cache.intake_temp, (int)cache.boost_x10);
}

static void obd_poll_task(void *arg) {
    s_poll_task_handle = xTaskGetCurrentTaskHandle();
    esp_task_wdt_add(NULL);  // register with the watchdog
    gauge_stack_diag_record_self(GAUGE_STACK_OBD);
    int64_t stack_diag_at_us = esp_timer_get_time();
    int64_t notify_deadline_us = 0;
    uint32_t tick_count = 0;
    bool inited = false;
    uint8_t heal_attempts = 0;
    unsigned poll_generation=atomic_load(&s_obd_target_generation);   // consecutive self-heal count; escalates to a forced reconnect if resending ATZ a few times still yields no data

    // Original 12 slots; optional slot12 only while fuel-rate support is confirmed, <=0.5Hz: 0=RPM, 1=IAT, 2=Speed, 3=CLT, 4=Load(0x04), 5=TPS(0x11), 6=OIL(vehicle strategy), 7=BAT(0x42), 8=Boost, 9=AFR, 10=Oil pressure, 11=Gear
    while (1)
    {
        /* Before every pause/disconnect continue, but scan at most once/10s. */
        int64_t stack_diag_now_us = esp_timer_get_time();
        if (stack_diag_now_us - stack_diag_at_us >= 10000000) {
            gauge_stack_diag_record_self(GAUGE_STACK_OBD);
            stack_diag_at_us = stack_diag_now_us;
        }
        unsigned generation=atomic_load(&s_obd_target_generation);
        if(generation!=poll_generation){poll_generation=generation;notify_deadline_us=0;inited=false;tick_count=0;heal_attempts=0;obd_fast_reset();}
        esp_task_wdt_reset();  // feed the watchdog
        retarget_service(); /* final native cleanup/delivery only; never retries CLOSE */
        scan_service(); /* bounded retry; no additional task or allocation */
        // Showroom mode: pause OBD polling to avoid overwriting the dummy data
        extern bool ui_showroom_is_active(void);
        if (ui_showroom_is_active()) {
            notify_deadline_us = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        // WiFi OTA: keep the ELM327 polling quiet so BLE stops competing for the radio.
        if (s_ota_paused) {
            notify_deadline_us = 0;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        // Not connected or notify subscription not ready → mark for re-init and wait
        if (!s_connected || !s_notify_ready) {
            obd_wait_notify_step(&notify_deadline_us);
            if(!s_connected&&s_scan_retry_pending&&esp_timer_get_time()>=s_scan_retry_after_us){
                s_scan_retry_pending=false;
                if(s_target_bda_valid&&!s_scan_only_mode)start_scan();
            }
            inited = false;
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        notify_deadline_us = 0;
        // Init only after the notify subscription is ready following a (re)connect; re-runs on every reconnect (fixes "must disconnect/reconnect to get readings")
        if (!inited) {
            vTaskDelay(pdMS_TO_TICKS(300));   // give the subscription a bit more time to settle
            if (!s_connected || !s_notify_ready) continue;  // dropped again while waiting — re-check before init
            do_elm_init();
            // do_elm_init may have aborted early due to disconnect; don't mark inited in that case
            if (!s_connected || !s_notify_ready) continue;
            inited = true;
            obd_fast_reset();
            s_fast_enabled=vehicle_profile_uses_standard_poll();
            tick_count = 0;
            continue;
        }
        // Real valid data is flowing (any frame parsed since the last round) → clear the self-heal counter.
        // Must NOT judge by timestamp: do_elm_init also refreshes the timestamp; if that counted as "data flowing",
        // heal_attempts would be cleared right after every re-init and the "3 consecutive self-heals → forced reconnect" escalation could never run.
        if (s_got_valid_data) {
            s_got_valid_data = false;
            heal_attempts = 0;
        }
        // Self-heal: no valid data for >10s straight (covers no response / SEARCHING / UNABLE TO CONNECT / NO DATA).
        if ((esp_timer_get_time() - s_last_obd_valid_us) > 10000000) {
            s_last_obd_valid_us = esp_timer_get_time();
            heal_attempts++;
            if (heal_attempts >= 3) {
                // Retried ATZ a few times with still no data → force-close BLE; the DISCONNECT callback auto-reconnects,
                // equivalent to automatically doing a "manual disconnect/reconnect" (re-subscribe notify + re-run init; by then the bus is usually awake).
                ESP_LOGW(TAG, "Self-heal escalate: force BLE reconnect (re-init didn't help)");
                heal_attempts = 0;
                inited = false;
                esp_ble_gattc_close(s_gattc_if, s_conn_id);
                vTaskDelay(pdMS_TO_TICKS(300));
                continue;
            }
            ESP_LOGW(TAG, "No valid OBD data >10s, re-init ELM (self-heal #%u)...", heal_attempts);
            inited = false;
            continue;
        }
        // ---- Mixed CAN/OBD mode ----
        //  - OBD-RPM profiles (e.g. ZN/C6 CAN): keep OBD as the main loop and open short ATMA probe windows only
        //    when we need to refresh CAN temps/TPS.
        const vehicle_profile_t *vp_poll = vehicle_profile_get_active();
        const vehicle_override_t *ov_poll = vehicle_profile_get_override();
        if(vehicle_profile_uses_standard_poll()){obd_fast_step();continue;}
        portENTER_CRITICAL(&s_elm_diag_mux);s_fast_enabled=false;portEXIT_CRITICAL(&s_elm_diag_mux);
        bool can_broadcast = vp_poll && vp_poll->can_broadcast_mode;
        bool can_has_rpm = can_rules_have_channel(ov_poll, CH_RPM);
        bool can_has_coolant = can_rules_have_channel(ov_poll, CH_COOLANT);
        bool can_has_tps = can_rules_have_channel(ov_poll, CH_TPS);
        bool can_has_oil = can_rules_have_channel(ov_poll, CH_OIL_TEMP);
        bool can_needs_monitor_probe = can_has_coolant || can_has_tps || can_has_oil;
        bool can_obd_primary = can_broadcast && !can_has_rpm;
        uint32_t can_obd_interval = can_has_rpm ? ZC6_CAN_OBD_INTERVAL : 1;
        uint32_t can_atma_delay_ms = can_has_rpm ? 120u : 30u;
        can_expire_stale_temp_channels();

        if (!can_obd_primary && can_broadcast && !s_zc_can_obd_phase) {
            // ---- ATMA phase: receive CAN broadcast frames ----
            if (!s_zc6_can_monitor_active) {
                zc6_can_monitor_enter();
                s_zc6_can_monitor_obd_cycle = 0;
            }
            // Self-heal
            if (s_zc6_can_monitor_entered_us > 0 &&
                (esp_timer_get_time() - s_last_obd_valid_us) > 10000000) {
                zc6_can_monitor_exit();
                zc6_can_monitor_enter();
                s_zc6_can_monitor_obd_cycle = 0;
                vTaskDelay(pdMS_TO_TICKS(can_atma_delay_ms));
                continue;
            }
            s_zc6_can_monitor_obd_cycle++;
            if (s_zc6_can_monitor_obd_cycle >= can_obd_interval) {
                // Switch to the OBD phase
                zc6_can_monitor_exit();
                s_zc_can_obd_phase = true;
                s_zc_can_obd_round_started = can_has_rpm;
                tick_count = can_has_rpm ? 1 : 0;
            }
            vTaskDelay(pdMS_TO_TICKS(can_atma_delay_ms));
            continue;
        }

        // ---- Standard OBD polling (ZC/N6 CAN's OBD phase, or non-CAN profiles) ----
        // Fully reuses the switch(tick_count) below, identical to regular ZC/N6
        if (!can_obd_primary && can_broadcast && s_zc_can_obd_phase && tick_count == 0) {
            if (!s_zc_can_obd_round_started) {
                s_zc_can_obd_round_started = true;
            } else {
                // One standard OBD round done, return to the ATMA phase
                s_zc_can_obd_phase = false;
                s_zc_can_obd_round_started = false;
                zc6_can_monitor_enter();
                s_zc6_can_monitor_obd_cycle = 0;
                vTaskDelay(pdMS_TO_TICKS(can_atma_delay_ms));
                continue;
            }
        }

        bool completed_obd_round = tick_count == 12 || (tick_count == 11 && !atomic_load(&s_fuel_supported));
        {
        switch(tick_count)
        {
            case 0:// Engine RPM
                send_rpm_request("slot0");
                break;
            case 1:// Intake air temp
                elm327_ble_send_ascii_blocking("01 0F\r");
                break;
            case 6: // Auto oil-temp query (based on vehicle strategy) (CAN mode gets it from passive CAN, skip when available)
                if (can_broadcast && can_has_oil) break;
                {
                    const oil_formula_t *oil_f = NULL;
                    if (s_oil_use_override) {
                        oil_f = (s_oil_override_idx == 0) ? s_oil_formula_pri : s_oil_formula_sec;
                    }

                    if (oil_f && oil_f->type != OIL_SPECIAL) {
                        char cmd_buf[24];
                        const char *uds_hdr = (s_ov && s_ov->uds_header_cmd) ? s_ov->uds_header_cmd : NULL;
                        bool need_phys = !uds_hdr && s_ov && s_ov->functional_addr && oil_f->type == OIL_UDS_22;
                        if (uds_hdr) elm327_ble_send_ascii_blocking(uds_hdr);
                        else if (need_phys) elm327_ble_send_ascii_blocking("ATSH7E0\r");
                        if (oil_formula_build_cmd(oil_f, cmd_buf, sizeof(cmd_buf))) {
                            elm327_ble_send_ascii_blocking(cmd_buf);
                        }
                        if (uds_hdr) elm327_ble_send_ascii_blocking(get_vehicle_fixed_header_cmd());
                        else if (need_phys) elm327_ble_send_ascii_blocking("ATSH7DF\r");
                        s_expect_mode21 = false;
                    } else if (oil_f && oil_f->type == OIL_SPECIAL && oil_f->special_id == 0) {
                        elm327_ble_send_ascii_blocking("21 01\r");
                        s_expect_mode21 = true;
                    } else {
                        uint8_t poll_idx = 0;
                        oil_temp_query_mode_t mode = get_next_oil_query_mode(&poll_idx);
                        s_expect_mode21 = (mode == OIL_TEMP_MODE_TOYOTA_21_01);
                        if (mode == OIL_TEMP_MODE_PID_5C)
                            elm327_ble_send_ascii_blocking("01 5C\r");
                        else if (mode == OIL_TEMP_MODE_TOYOTA_21_01)
                            elm327_ble_send_ascii_blocking("21 01\r");
                        else {
                            char cmd_buf[24];
                            oil_formula_t legacy_f = {0};
                            legacy_f.type = OIL_UDS_22;
                            legacy_f.pid_len = 2;
                            switch (mode) {
                                case OIL_TEMP_MODE_UDS_22_10_17: legacy_f.pid[0]=0x10; legacy_f.pid[1]=0x17; break;
                                case OIL_TEMP_MODE_MAZDA_22_111F: legacy_f.pid[0]=0x11; legacy_f.pid[1]=0x1F; break;
                                case OIL_TEMP_MODE_MAZDA_22_1310: legacy_f.pid[0]=0x13; legacy_f.pid[1]=0x10; break;
                                case OIL_TEMP_MODE_MINI_22_5822: legacy_f.pid[0]=0x58; legacy_f.pid[1]=0x22; break;
                                case OIL_TEMP_MODE_BMW_22_4402:
                                case OIL_TEMP_MODE_BMW_G_22_4402: legacy_f.pid[0]=0x44; legacy_f.pid[1]=0x02; break;
                                case OIL_TEMP_MODE_BMW_22_03F3: legacy_f.pid[0]=0x03; legacy_f.pid[1]=0xF3; break;
                                case OIL_TEMP_MODE_BMW_22_D002: legacy_f.pid[0]=0xD0; legacy_f.pid[1]=0x02; break;
                                case OIL_TEMP_MODE_BMW_22_111F: legacy_f.pid[0]=0x11; legacy_f.pid[1]=0x1F; break;
                                default: legacy_f.type = OIL_STD_PID; legacy_f.pid[0]=0x5C; legacy_f.pid_len=1; break;
                            }
                            bool need_phys = (mode == OIL_TEMP_MODE_BMW_22_03F3 ||
                                              mode == OIL_TEMP_MODE_BMW_G_22_4402 ||
                                              mode == OIL_TEMP_MODE_BMW_22_D002 ||
                                              mode == OIL_TEMP_MODE_BMW_22_111F);
                            if (need_phys) elm327_ble_send_ascii_blocking("ATSH7E0\r");
                            if (oil_formula_build_cmd(&legacy_f, cmd_buf, sizeof(cmd_buf)))
                                elm327_ble_send_ascii_blocking(cmd_buf);
                            if (need_phys) elm327_ble_send_ascii_blocking("ATSH7DF\r");
                        }
                        s_oil_query_mode = poll_idx;
                    }
                }
                break;
            case 2:// Vehicle speed
                elm327_ble_send_ascii_blocking("01 0D\r");
                break;
            case 3:// Coolant temp (skip when CAN already provides it)
                if (!(can_broadcast && can_has_coolant)) {
                    elm327_ble_send_ascii_blocking("01 05\r");
                }
                break;
            case 4:// Engine load (0x04, 0~100%)
                elm327_ble_send_ascii_blocking("01 04\r");
                break;
            case 5:// Throttle position TPS (skip when CAN already provides it)
                if (!(can_broadcast && can_has_tps)) {
                    elm327_ble_send_ascii_blocking("01 11\r");
                }
                break;
            case 7:// Battery voltage (0x42)
                elm327_ble_send_ascii_blocking("01 42\r");
                break;
            case 8:// Boost pressure: intake manifold absolute pressure (0x0B, kPa), queried only for turbo profiles
                {
                    const vehicle_profile_t *vp = vehicle_profile_get_active();
                    if (vp && vp->has_boost) {
                        elm327_ble_send_ascii_blocking("01 0B\r");
                    }
                }
                break;
            case 9:// Air-fuel ratio AFR (01 44, Commanded Equivalence Ratio)
                elm327_ble_send_ascii_blocking("01 44\r");
                break;
            case 10:// Engine oil pressure (Mode 22 DID, per-profile: 4436=B58, 586F=N55) — physical header, only for OBD-oil-pressure profiles
                {
                    const vehicle_profile_t *vp = vehicle_profile_get_active();
                    if (vp && vp->obd_oil_pressure_did != 0) {
                        const vehicle_override_t *ov = vehicle_profile_get_override();
                        const char *phys_hdr = (ov && ov->uds_header_cmd) ? ov->uds_header_cmd : "ATSH7E0\r";
                        char cmd[16];
                        elm327_ble_send_ascii_blocking(phys_hdr);
                        snprintf(cmd, sizeof(cmd), "22 %02X %02X\r",
                                 (vp->obd_oil_pressure_did >> 8) & 0xFF, vp->obd_oil_pressure_did & 0xFF);
                        elm327_ble_send_ascii_blocking(cmd);
                        elm327_ble_send_ascii_blocking(get_vehicle_fixed_header_cmd());
                    }
                }
                break;
            case 11:// Transmission gear (Mode 22 DID, per-profile: D031=BMW ZF 8HP) — EGS functional header, only for OBD-gear profiles
                {
                    const vehicle_profile_t *vp = vehicle_profile_get_active();
                    if (vp && vp->obd_gear_did != 0) {
                        const vehicle_override_t *ov = vehicle_profile_get_override();
                        const char *gear_hdr = (ov && ov->obd_gear_header_cmd) ? ov->obd_gear_header_cmd
                                             : ((ov && ov->uds_header_cmd) ? ov->uds_header_cmd : "ATSH7E0\r");
                        char cmd[16];
                        elm327_ble_send_ascii_blocking(gear_hdr);
                        snprintf(cmd, sizeof(cmd), "22 %02X %02X\r",
                                 (vp->obd_gear_did >> 8) & 0xFF, vp->obd_gear_did & 0xFF);
                        elm327_ble_send_ascii_blocking(cmd);
                        elm327_ble_send_ascii_blocking(get_vehicle_fixed_header_cmd());
                    }
                }
                break;
            case 12: /* PID 5E, only explicitly supported by this session's standard ECU response. */
                if(atomic_load(&s_fuel_supported)){
                    int64_t now=esp_timer_get_time();
                    if(atomic_load(&s_fuel_attempts)>=3){
                        atomic_store(&s_fuel_supported,false);
                        gv5_data_invalidated(GV5_FUEL_RATE);
                    }else if(s_fuel_last_request_us==0||now-s_fuel_last_request_us>=2000000){
                        atomic_fetch_add(&s_fuel_attempts,1);
                        elm327_ble_send_ascii_blocking("01 5E\r");
                        /* The blocking helper may wait for the preceding response.
                         * Rate-limit from completion of the actual send attempt. */
                        s_fuel_last_request_us=esp_timer_get_time();
                    }
                }
                break;
            default:
                break;
        }

        if ((!can_broadcast || can_obd_primary) && tick_count != 0) {
            send_rpm_request("dup");
        }

        tick_count++;
        if(tick_count >= (atomic_load(&s_fuel_supported)?13u:12u))
        {
            tick_count = 0;
        }
        } // end standard OBD poll block

        if (can_obd_primary && can_needs_monitor_probe && completed_obd_round) {
            int64_t now_us = esp_timer_get_time();
            if (s_zc6_can_temp_probe_last_us == 0 ||
                (now_us - s_zc6_can_temp_probe_last_us) >= ZC6_CAN_TEMP_PROBE_INTERVAL_US) {
                zc6_can_monitor_probe_window(ZC6_CAN_TEMP_PROBE_WINDOW_MS);
                s_zc6_can_temp_probe_last_us = esp_timer_get_time();
            }
        }

        // Inter-slot idle gap: prefer the override's poll_gap_ms first, then the profile's poll_gap_ms (e.g. ZC/N6, MX-5 ND use 1ms); fall back to the global default 30ms.
        // Too small overwhelms cheap BLE adapters; profiles with fast CAN-bus response can safely go smaller.
        // If the user sets poll_gap_ms = 0, skip vTaskDelay and move on directly.
        {
            const vehicle_override_t *ov_gap = vehicle_profile_get_override();
            const vehicle_profile_t *vp_gap = vehicle_profile_get_active();
            uint32_t gap = (ov_gap && ov_gap->poll_gap_ms > 0)
                           ? ov_gap->poll_gap_ms
                           : ((vp_gap && vp_gap->poll_gap_ms > 0)
                              ? vp_gap->poll_gap_ms : OBD_POLL_SLOT_GAP_MS);
            if (gap > 0) vTaskDelay(pdMS_TO_TICKS(gap));
        }
    }
}

// Mode 21 multi-frame parser: extract all data bytes after "61 01".
// Skips ELM327 line-number prefixes ("N: ") and ISO-TP consecutive-frame sequence bytes (0x20~0x2F).
// Returns the number of bytes extracted; results stored in out[].
static int parse_mode21_data(const char *buf, uint32_t *out, int max_out) {
    const char *p = strstr(buf, "61 01");
    if (!p) return 0;
    p += 5; // skip "61 01"
    if (*p == ' ') p++;

    int count = 0;
    bool new_line = false;

    while (*p && count < max_out) {
        if (*p == '>') break;
        if (*p == '\r' || *p == '\n') {
            new_line = true;
            p++;
            continue;
        }
        if (new_line) {
            // Skip the "N: " prefix (one or more digits + colon + space)
            while (isdigit((unsigned char)*p)) p++;
            if (*p == ':') p++;
            while (*p == ' ') p++;
            // Skip ISO-TP consecutive-frame sequence bytes (0x20~0x2F)
            {
                const char *peek = p;
                uint8_t bval = 0;
                if (can_monitor_parse_hex_byte(&peek, &bval) && bval >= 0x20 && bval <= 0x2F) {
                    p = peek;
                    if (*p == ' ') p++;
                }
            }
            new_line = false;
            continue;
        }
        // Parse one hex byte pair
        {
            const char *peek = p;
            uint8_t bval = 0;
            if (can_monitor_parse_hex_byte(&peek, &bval)) {
                out[count++] = (uint32_t)bval;
                p = peek;
            } else {
                p++;
            }
        }
        if (*p == ' ') p++;
    }
    return count;
}

// Extract the oil-temp byte from Mode21 data (using ZC/N6 as the reference).
// ZC/N6: always use only d[33], never enter the adaptive search (which may mis-pick another byte and jump to 60/70°C).
static bool extract_mode21_oil_temp(const uint32_t *d, int count, int32_t *oil_c) {
    if (!d || count <= 0 || !oil_c) return false;

    int16_t coolant = obd_data_get_coolant_temp();

    // ---- ZC/N6: locate by tail offset, handling both 38- and 39-byte response lengths ----
    // With 38 bytes oil temp is at d[33]=d[38-5]; with 39 bytes at d[34]=d[39-5]. A fixed d[33] reads the wrong byte on 39-byte frames.
    // Identify by profile name (not index): both the "ZN/C6 CAN" and "ZN/C6 PID" variants take this parse path.
    const vehicle_profile_t *vp_m21 = vehicle_profile_get_active();
    if (vp_m21 && strncmp(vp_m21->name, "ZN/C6", 5) == 0) {
        #define ZC_MODE21_OIL_TAIL_OFFSET 5
        int zc_idx = count - ZC_MODE21_OIL_TAIL_OFFSET;
        if (zc_idx < 0 || zc_idx >= count) {
            s_oil_diag.mode2_fail++;
            return false;
        }
        int32_t zc_temp = (int32_t)d[zc_idx] - 40;
        if (zc_temp < -10 || zc_temp > 150) {
            s_oil_diag.mode2_fail++;
            return false;
        }
        // Consistency check: oil temp physically cannot jump more than 8°C between two polls (~270ms).
        int64_t now_us = esp_timer_get_time();
        bool time_gap = (s_last_mode21_oil_us == 0) || ((now_us - s_last_mode21_oil_us) > 3000000);
        bool consistent = (s_last_mode21_oil <= -50) || time_gap ||
                          (abs((int)zc_temp - (int)s_last_mode21_oil) <= 8);
        if (consistent) {
            s_last_mode21_oil = (int16_t)zc_temp;
            s_last_mode21_oil_us = now_us;
            s_mode21_hold_cnt = 0;
            s_oil_diag.mode2_ok++;
            *oil_c = zc_temp;
            return true;
        }
        if (s_mode21_hold_cnt < 30) {
            s_mode21_hold_cnt++;
            s_oil_diag.mode2_ok++;
            *oil_c = s_last_mode21_oil;
            return true;
        }
        s_last_mode21_oil = (int16_t)zc_temp;
        s_last_mode21_oil_us = esp_timer_get_time();
        s_mode21_hold_cnt = 0;
        s_oil_diag.mode2_ok++;
        *oil_c = zc_temp;
        return true;
    }

    if (s_mode21_oil_idx >= 0 && s_mode21_oil_idx < count) {
        int32_t c = (int32_t)d[s_mode21_oil_idx] - 40;
        bool in_range = (c >= -10 && c <= 150);
        bool consistent = (s_last_mode21_oil <= -50) || (abs((int)c - (int)s_last_mode21_oil) <= 8);
        if (in_range && consistent) {
            s_last_mode21_oil = (int16_t)c;
            s_mode21_hold_cnt = 0;
            *oil_c = c;
            s_oil_diag.mode2_ok++;
            return true;
        }
        if (in_range && !consistent) {
            if (s_mode21_hold_cnt < 30) {
                s_mode21_hold_cnt++;
                s_oil_diag.mode2_ok++;
                *oil_c = s_last_mode21_oil;
                return true;
            }
            s_last_mode21_oil = (int16_t)c;
            s_mode21_hold_cnt = 0;
            s_oil_diag.mode2_ok++;
            *oil_c = c;
            return true;
        }
    }

    int best_idx = -1;
    int32_t best_temp = 0;
    int best_distance = -1;
    int strict_count = 0;

    for (int idx = 0; idx < count; idx++) {
        int32_t c = (int32_t)d[idx] - 40;

        if (c < -10 || c > 150) continue;

        if (coolant > -40) {
            int diff = (int)c - (int)coolant;

            if (diff >= -25 && diff <= 25) {
                int priority = (diff > 0 && diff <= 20) ? 1000 :   // ideal: oil > coolant
                               (diff >= -5 && diff <= 0) ? 700  :   // cold engine or equal temp: acceptable
                               (diff > 20 && diff <= 25) ? 400  : 100;
                int score = priority - abs(diff);

                if (score > best_distance) {
                    best_distance = score;
                    best_idx = idx;
                    best_temp = c;
                    strict_count++;
                }
            }
        } else {
            if (best_idx < 0) {
                best_idx = idx;
                best_temp = c;
            }
        }
    }

    if (best_idx >= 0) {
        s_mode21_oil_idx = best_idx;
        s_last_mode21_oil = (int16_t)best_temp;
        s_mode21_hold_cnt = 0;
        s_oil_diag.mode2_ok++;
        *oil_c = best_temp;
        return true;
    }

    s_oil_diag.mode2_fail++;
    return false;
}

static bool scan_can_run(void)
{
    return !s_ota_paused && !s_open_pending && !s_retarget_close_pending &&
           (s_scan_only_mode || (!s_connected && s_target_bda_valid));
}

static void configure_scan_params(void)
{
    esp_ble_scan_params_t params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x60, .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    portENTER_CRITICAL(&s_retarget_mux);
    if (s_scan_params_ready || s_scan_params_pending || s_gattc_if == ESP_GATT_IF_NONE ||
        esp_timer_get_time() < s_scan_api_retry_after_us) {
        portEXIT_CRITICAL(&s_retarget_mux);
        return;
    }
    s_scan_params_pending = true;
    portEXIT_CRITICAL(&s_retarget_mux);
    esp_err_t err = esp_ble_gap_set_scan_params(&params);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_retarget_mux);
        s_scan_params_pending = false;
        s_scan_api_retry_after_us = esp_timer_get_time() + 5000000;
        portEXIT_CRITICAL(&s_retarget_mux);
        ESP_LOGW(TAG, "OBD scan params request failed: %d; retry in 5s", err);
    }
}

static void scan_service(void)
{
    bool start = false, stop = false, params = false;
    int duration = 10;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_retarget_mux);
    if (now < s_scan_api_retry_after_us) {
        portEXIT_CRITICAL(&s_retarget_mux);
        return;
    }
    if (s_scan_stop_requested) {
        if (s_ble_scan_active && !s_scan_stop_pending) {
            s_scan_stop_pending = true;
            stop = true;
        } else if (!s_ble_scan_active && !s_scan_start_pending && !s_scan_stop_pending) {
            s_scan_stop_requested = false;
        }
    }
    if (!stop && !s_scan_stop_requested && s_scan_requested && scan_can_run()) {
        if (!s_scan_params_ready) {
            params = !s_scan_params_pending && s_gattc_if != ESP_GATT_IF_NONE;
        } else if (!s_ble_scan_active && !s_scan_start_pending && !s_scan_stop_pending &&
                   now >= s_scan_retry_after_us) {
            s_scan_start_pending = true;
            duration = s_scan_duration_s;
            start = true;
        }
    }
    portEXIT_CRITICAL(&s_retarget_mux);
    if (params) configure_scan_params();
    if (!start && !stop) return;
    ESP_LOGI(TAG, "OBD scan %s request target=%d", stop ? "stop" : "start", s_target_bda_valid);
    esp_err_t err = stop ? esp_ble_gap_stop_scanning() : esp_ble_gap_start_scanning(duration);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_retarget_mux);
        if (stop) s_scan_stop_pending = false;
        else s_scan_start_pending = false;
        s_scan_api_retry_after_us = esp_timer_get_time() + 5000000;
        portEXIT_CRITICAL(&s_retarget_mux);
        ESP_LOGW(TAG, "OBD scan %s request failed: %d; retry in 5s", stop ? "stop" : "start", err);
    }
}

static void start_scan(void)
{
    portENTER_CRITICAL(&s_retarget_mux);
    s_scan_requested = s_scan_only_mode || s_target_bda_valid;
    portEXIT_CRITICAL(&s_retarget_mux);
    scan_service();
}

static void scan_stop(void)
{
    portENTER_CRITICAL(&s_retarget_mux);
    s_scan_requested = false;
    s_scan_stop_requested = true;
    portEXIT_CRITICAL(&s_retarget_mux);
    scan_service();
}

static bool match_device_target(const esp_ble_gap_cb_param_t *pr, const char *target_name,
                                char *found_name, size_t found_name_len) {
    if (!pr) return false;
    ble_adv_extract_name(pr->scan_rst.ble_adv, pr->scan_rst.adv_data_len, pr->scan_rst.scan_rsp_len,
                     found_name, found_name_len);
    (void)target_name;

    // Only exact MAC matches: never auto-connect when no MAC is bound (no fuzzy name matching,
    // to prevent connecting to a nearby same-named adapter that yields no data). The user must manually select and bind a MAC on the BLE SCAN page.
    if (!s_target_bda_valid) return false;
    return memcmp(pr->scan_rst.bda, s_target_bda, sizeof(esp_bd_addr_t)) == 0;
}

static void request_discovery(void) {
    if (!s_connected || s_ota_paused || s_gattc_if == ESP_GATT_IF_NONE ||
        s_conn_id == 0xFFFF) return;
    // NULL = discover all services, to stay compatible with ELM327 adapters using different UUIDs
    esp_ble_gattc_search_service(s_gattc_if, s_conn_id, NULL);
}

static void enable_notify_if_ready(void) {
    if (s_cccd_handle) {
        uint8_t notify_en[2] = {0x01, 0x00};
        esp_ble_gattc_write_char_descr(s_gattc_if, s_conn_id, s_cccd_handle,
                                       sizeof(notify_en), notify_en,
                                       ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    }
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

void elm327_ble_init_and_start(const char *target_name, const elm327_ble_callbacks_t *cbs) {
    if (cbs) s_cbs = *cbs;
    if (target_name && target_name[0]) {
        strncpy(s_target_name, target_name, sizeof(s_target_name)-1);
        s_target_name[sizeof(s_target_name)-1] = '\0';
    }

    portENTER_CRITICAL(&s_retarget_mux);
    bool initialize = !s_ble_inited;
    s_ble_inited = true; // reserve the single app registration before asynchronous callbacks
    portEXIT_CRITICAL(&s_retarget_mux);
    if (!initialize) return;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    }
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    }
    if (!esp_bluedroid_get_status()) {
        // Raise the local ATT MTU from the 23-byte default: the phone OTA app requests
        // 517 and the negotiated MTU is min(both). 23 bytes would cripple OTA throughput
        // (20-byte writes) and force long-read fragmentation for the manifest.
        esp_ble_gatt_set_local_mtu(517);
        ESP_ERROR_CHECK(esp_bluedroid_init());
        ESP_ERROR_CHECK(esp_bluedroid_enable());
    } else if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bluedroid_enable());
    }

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_event_handler));
    ESP_LOGI(TAG, "OBD GATTC register request (once)");
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));
    s_ble_inited = true;
}

bool elm327_ble_send_command(const uint8_t *data, size_t len) {
    if (!s_connected || !s_target_bda_valid||memcmp(s_peer_bda,s_target_bda,sizeof s_peer_bda)!=0||s_char_write_handle == 0) {
        s_elm_ready = true; // restore the flag when unable to send, to prevent a permanent timeout
        return false;
    }
    if (len == 0 || data == NULL) { s_elm_ready = true; return false; }
    esp_err_t err = esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_char_write_handle,
                                             len, (uint8_t *)data,
                                             s_write_type, ESP_GATT_AUTH_REQ_NONE);
    if (err != ESP_OK) s_elm_ready = true; // also restore on send failure
    return err == ESP_OK;
}

// Block until the previous response ends ('>' received) before sending.
// Uses FreeRTOS task notifications instead of 10ms polling: xTaskNotify wakes immediately on '>', zero wait overhead.
bool elm327_ble_send_ascii_blocking(const char *ascii_cmd)
{
    if(!ascii_cmd||!ascii_cmd[0])return false;
    // Fast-exit when disconnected: no point waiting for a '>' that will never arrive.
    // Prevents up to 3s of pointless blocking per command after a BLE drop.
    if (!s_connected) {
        s_elm_ready = true;
        return false;
    }
    if (!s_elm_ready) {
        uint32_t waited_ms = 0;
        while (!s_elm_ready && waited_ms < 3000) {
            // Disconnect mid-wait → abort immediately instead of waiting up to 3s
            if (!s_connected) { s_elm_ready = true; return false; }
            // Wait at most 10ms (as a fallback); xTaskNotify wakes early when '>' arrives
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            waited_ms += 10;
            // Feed the watchdog: this wait loop itself can run close to 3s (adapter unresponsive / protocol detection timing out repeatedly).
            // The TWDT timeout is only 5s; without feeding here, a few consecutive protocol-detect timeouts would
            // trip the obd_poll task's watchdog and reboot (reproduced in testing).
            esp_task_wdt_reset();
        }
        if (!s_elm_ready) {
            elm_diag_uncertain();
            ESP_LOGW(TAG, "Timeout (>3s) waiting previous response, forcing send: %s", ascii_cmd);
            s_elm_ready = true;
        }
    }
    s_elm_ready = false;
    uint8_t buf[32];
    size_t n = elm327_ble_ascii_cmd_to_bytes(ascii_cmd, buf, sizeof(buf));
    if (n) {
        elm_diag_request(ascii_cmd);
        bool sent=elm327_ble_send_command(buf,n);
        if(!sent)elm_diag_uncertain();
        return sent;
    }
    else {
        s_elm_ready = true;
        return false;
    }
}

// Copy an ASCII command (e.g. "01 0C\r") into the output buffer, stripping whitespace while keeping the ASCII format the ELM327 expects
size_t elm327_ble_ascii_cmd_to_bytes(const char *ascii, uint8_t *out_buf, size_t out_buf_len) {
    size_t out = 0;
    const char *p = ascii;
    while (*p && out < out_buf_len) {
        if (*p == ' ' || *p == '\t') {
            p++;                // skip whitespace
            continue;
        }
        out_buf[out++] = (uint8_t)(*p++); // copy the ASCII byte directly
    }
    return out;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    racechrono_ble_diy_handle_gap_event(event, param);

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        portENTER_CRITICAL(&s_retarget_mux);
        s_scan_params_pending = false;
        s_scan_params_ready = param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS;
        if (!s_scan_params_ready) s_scan_api_retry_after_us = esp_timer_get_time() + 5000000;
        portEXIT_CRITICAL(&s_retarget_mux);
        ESP_LOGI(TAG, "OBD scan params complete status=%d target=%d", param->scan_param_cmpl.status, s_target_bda_valid);
        scan_service();
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
        portENTER_CRITICAL(&s_retarget_mux);
        s_scan_start_pending = false;
        s_ble_scan_active = param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS;
        if (!s_ble_scan_active) s_scan_api_retry_after_us = esp_timer_get_time() + 5000000;
        portEXIT_CRITICAL(&s_retarget_mux);
        ESP_LOGI(TAG, "OBD scan start complete status=%d", param->scan_start_cmpl.status);
        scan_service();
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *pr = param;
        if (pr->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            char dev_name[32] = {0};
            ble_adv_extract_name(pr->scan_rst.ble_adv, pr->scan_rst.adv_data_len,
                             pr->scan_rst.scan_rsp_len, dev_name, sizeof(dev_name));

            if (s_scan_only_mode) {
                // Scan mode: collect the device list
                if (dev_name[0] != '\0' && s_scan_count < BLE_SCAN_MAX_DEVICES) {
                    // Check if it already exists
                    bool exists = false;
                    for (int i = 0; i < s_scan_count; i++) {
                        if (strcmp(s_scan_list[i].name, dev_name) == 0) {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists) {
                        strncpy(s_scan_list[s_scan_count].name, dev_name, 31);
                        memcpy(s_scan_list[s_scan_count].addr, pr->scan_rst.bda, 6);
                        s_scan_list[s_scan_count].rssi = pr->scan_rst.rssi;
                        s_scan_count++;
                        ESP_LOGD(TAG, "Scan found [%d]: %s (RSSI %d)", s_scan_count, dev_name, pr->scan_rst.rssi);
                        if (s_scan_cb) s_scan_cb(&s_scan_list[s_scan_count - 1], s_scan_count);
                    }
                }
            } else {
                // Normal mode: connect after matching
                bool matched = match_device_target(pr, s_target_name, dev_name, sizeof(dev_name));
                ESP_LOGD(TAG, "Scan: name=%s rssi=%d target=%s match=%d",
                         dev_name[0] ? dev_name : "<no-name>", pr->scan_rst.rssi,
                         s_target_name, matched);
                if (matched && !s_connected && !s_open_pending && !s_ota_paused &&
                    !s_retarget_close_pending && s_gattc_if != ESP_GATT_IF_NONE) {
                    ESP_LOGI(TAG, "Found bound OBD target %s (dev=%s), connecting...",
                             s_target_name, dev_name[0] ? dev_name : "<no-name>");
                    uint32_t opening = gso_open_reserve(s_gattc_if, pr->scan_rst.bda);
                    if (!opening) break; /* old native incarnation has not retired */
                    scan_stop();
                    memcpy(s_open_bda, pr->scan_rst.bda, sizeof(esp_bd_addr_t));
                    s_open_pending = true;
                    esp_err_t err = esp_ble_gattc_open(s_gattc_if, pr->scan_rst.bda,
                                                      pr->scan_rst.ble_addr_type, true);
                    if (err != ESP_OK) {
                        gso_open_settled(opening, false); /* this API job was not queued */
                        s_open_pending = false;
                        ESP_LOGW(TAG, "Adapter OPEN request failed: %d", err);
                        start_scan();
                    }
                }
            }
        } else if (pr->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            // A completed window retires the active scan; bound-target requests persist.
            portENTER_CRITICAL(&s_retarget_mux);
            s_ble_scan_active = false;
            s_scan_start_pending = false;
            if (s_scan_only_mode) s_scan_requested = false; // manual list scan is finite
            portEXIT_CRITICAL(&s_retarget_mux);
            scan_service();
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
        portENTER_CRITICAL(&s_retarget_mux);
        s_scan_stop_pending = false;
        if (param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            s_ble_scan_active = false;
            s_scan_stop_requested = false;
        } else {
            s_scan_api_retry_after_us = esp_timer_get_time() + 5000000;
        }
        portEXIT_CRITICAL(&s_retarget_mux);
        scan_service();
        break;
    }
    default:
        break;
    }
}

// Locate a CAN ID's frame-header token in the multi-line accumulation buffer; only accept matches "at the line start".
// Must not strstr the whole buffer directly: if some frame's data-byte text happens to equal another monitored ID
// (e.g. oil-temp byte =0x40 while the profile also monitors 0x040), it false-matches inside the data segment and mislabels the frame.
static const char *find_can_id_token(const char *buf, uint16_t id) {
    char upper[8], lower[8];
    snprintf(upper, sizeof(upper), "%X ", id);
    snprintf(lower, sizeof(lower), "%x ", id);
    for (int pass = 0; pass < 2; pass++) {
        const char *needle = (pass == 0) ? upper : lower;
        const char *scan = buf;
        while ((scan = strstr(scan, needle)) != NULL) {
            if (scan == buf || scan[-1] == '\r' || scan[-1] == '\n') return scan;
            scan += 1; // coincidental match inside data bytes; skip it and keep looking for a real line start
        }
    }
    return NULL;
}

/* Same session reset as the original DISCONNECT case; scan publication is
 * delayed until native descriptor, operation and callback leases are gone. */
static void retarget_retire_session(void)
{
        atomic_store(&s_fuel_probe_pending,false);
        atomic_store(&s_fuel_supported,false);
        atomic_store(&s_fuel_attempts,0);
        atomic_fetch_add(&s_obd_target_generation,1);
        obd_scan_reset_session();
        s_connected = false;
        s_notify_ready = false;   // disconnect → notifications invalid; must re-subscribe + re-init after reconnect
        s_elm_ready = true;       // release any blocking send wait immediately (no '>' will ever arrive)
        if (s_poll_task_handle) xTaskNotify(s_poll_task_handle, 0, eNoAction); // wake the poll task so it sees the disconnect without waiting up to 3s
        s_conn_id = 0xFFFF;
        s_have_service = false;
        s_service_start = 0x0001;
        s_service_end = 0xFFFF;
        s_all_attr_end = 0xFFFF;
        s_have_18f0 = false;
        s_18f0_start = s_18f0_end = 0;
        s_have_ff12 = false;
        s_ff12_start = s_ff12_end = 0;
        s_write_type = ESP_GATT_WRITE_TYPE_RSP; // reset the write type after disconnect
        s_expect_mode21 = false;
        s_char_write_handle = s_char_notify_handle = s_cccd_handle = 0;
        s_accum_len = 0; s_accum_buf[0] = '\0'; // clear the response accumulation buffer
        s_zc6_can_monitor_active = false;        // reset the CAN continuous monitor
        s_zc6_can_monitor_len = 0;
        obd_data_reset_temp_cache();
        s_zc_can_obd_phase = false;
        s_zc_can_obd_round_started = false;
        s_zc6_can_temp_probe_last_us = 0;
        s_got_valid_data = false;                // prevent the stale pre-disconnect flag from being mis-consumed after reconnect
        s_last_mode21_oil = -100;
        s_mode21_hold_cnt = 0;
        s_protocol_detect_idx = -1;  // clear protocol detection state
        s_protocol_detect_got_response = false;
        s_protocol_detect_rpm = -1;
        s_oil_query_mode = 0;  // reset the oil-temp query counter
        if (s_cbs.on_disconnected) s_cbs.on_disconnected();
}

static void retarget_wake(void)
{
    if (s_poll_task_handle) xTaskNotify(s_poll_task_handle, 0, eNoAction);
}

/* Only an explicit public operation can recover a definitely unposted job.
 * The SDK claims and validates the immutable (incarnation, conn) atomically;
 * old jobs cannot close a later connection reusing the same numeric conn_id. */
static void retarget_close_explicit(void)
{
    uint32_t epoch = 0;
    uint16_t conn = UINT16_MAX;
    portENTER_CRITICAL(&s_retarget_mux);
    if (s_connected && !s_retarget_close_pending) {
        s_retarget_old_conn = s_conn_id;
        memcpy(s_retarget_old_peer, s_peer_bda, sizeof(esp_bd_addr_t));
        s_retarget_sdk_epoch = s_active_sdk_epoch;
        s_retarget_close_pending = true;
        s_connected = false;
        s_notify_ready = false;
        s_elm_ready = true;
    }
    if (s_retarget_close_pending && !s_retarget_cleaning) {
        epoch = s_retarget_sdk_epoch;
        conn = (uint16_t)((s_retarget_old_conn << 8) | s_gattc_if);
    }
    portEXIT_CRITICAL(&s_retarget_mux);
    if (epoch) {
        gso_state_t state = gso_state(epoch);
        if (state == GSO_NONE || state == GSO_REJECTED) {
            bool queued = gauge_sdk_close_submit(epoch, conn);
            if (!queued && gso_state(epoch) == GSO_REJECTED)
                ESP_LOGW(TAG, "Adapter CLOSE not queued; retry on next explicit selection");
        }
        /* QUEUED/ACCEPTED without a completion is held, never replayed. */
    }
    retarget_wake();
}

static void retarget_terminal(uint8_t gatt_if, uint16_t conn, const uint8_t peer[6])
{
    uint32_t token = gso_callback_token();
    if (!token || token == GSO_INVALID) return;
    portENTER_CRITICAL(&s_retarget_mux);
    if (gatt_if == s_gattc_if && token == s_active_sdk_epoch &&
        conn == s_conn_id && memcmp(peer, s_peer_bda, sizeof(esp_bd_addr_t)) == 0) {
        if (!s_retarget_close_pending) {
            s_retarget_old_conn = conn;
            memcpy(s_retarget_old_peer, peer, sizeof(esp_bd_addr_t));
            s_retarget_sdk_epoch = token;
            s_retarget_close_pending = true;
        }
        s_connected = false;
        s_notify_ready = false;
        s_elm_ready = true;
    }
    portEXIT_CRITICAL(&s_retarget_mux);
    retarget_wake();
}

static void retarget_service(void)
{
    gauge_sdk_service_delivery(); /* retry a saved callback, not the CLOSE command */
    uint32_t epoch;
    portENTER_CRITICAL(&s_retarget_mux);
    epoch = s_retarget_sdk_epoch;
    portEXIT_CRITICAL(&s_retarget_mux);
    if (!epoch || gso_state(epoch) != GSO_RELEASED) return;
    bool cleanup = false;
    portENTER_CRITICAL(&s_retarget_mux);
    if (s_retarget_close_pending && !s_retarget_cleaning && s_retarget_sdk_epoch == epoch) {
        s_retarget_cleaning = true;
        cleanup = true;
    }
    portEXIT_CRITICAL(&s_retarget_mux);
    if (!cleanup) return;
    retarget_retire_session(); /* no app/native mux during parser reset or callbacks */
    portENTER_CRITICAL(&s_retarget_mux);
    s_active_sdk_epoch = 0;
    s_retarget_sdk_epoch = 0;
    s_retarget_close_pending = false;
    s_retarget_cleaning = false;
    portEXIT_CRITICAL(&s_retarget_mux);
    if ((s_target_bda_valid || s_scan_only_mode) && !s_ota_paused) start_scan();
}

static bool retarget_open_complete(uint8_t gatt_if, uint16_t conn, const uint8_t peer[6])
{
    uint32_t token = gso_callback_token();
    if (!token || token == GSO_INVALID ||
        !gso_auth(token, (uint16_t)((conn << 8) | gatt_if))) return false;
    bool live = false;
    portENTER_CRITICAL(&s_retarget_mux);
    if (gatt_if == s_gattc_if && s_open_pending && !s_retarget_close_pending &&
        memcmp(peer, s_open_bda, sizeof(esp_bd_addr_t)) == 0) {
        s_open_pending = false;
        s_active_sdk_epoch = token;
        s_conn_id = conn;
        memcpy(s_peer_bda, peer, sizeof(esp_bd_addr_t));
        live = !s_ota_paused && !s_scan_only_mode && s_target_bda_valid &&
               memcmp(peer, s_target_bda, sizeof(esp_bd_addr_t)) == 0;
        s_connected = true; /* cancelled successes still own a real closeable descriptor */
    }
    portEXIT_CRITICAL(&s_retarget_mux);
    if (!live) retarget_close_explicit();
    return live;
}

static bool owned_connection(uint16_t conn_id, const uint8_t *peer)
{
    return s_connected && s_conn_id == conn_id &&
           s_target_bda_valid&&memcmp(s_peer_bda,s_target_bda,sizeof(esp_bd_addr_t))==0&&
           (!peer || memcmp(peer, s_peer_bda, sizeof(esp_bd_addr_t)) == 0);
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    if (!param) return;
    if (event != ESP_GATTC_REG_EVT &&
        (s_gattc_if == ESP_GATT_IF_NONE || gattc_if != s_gattc_if)) return;
    if (event == ESP_GATTC_CLOSE_EVT) {
        retarget_terminal(gattc_if, param->close.conn_id, param->close.remote_bda);
        return;
    }
    if (event == ESP_GATTC_DISCONNECT_EVT) {
        retarget_terminal(gattc_if, param->disconnect.conn_id, param->disconnect.remote_bda);
        return;
    }
    // Client interfaces and connection IDs are separate from the simultaneous
    // OTA/pairing GATT server link. Ignore unrelated and late completions.
    switch (event) {
    case ESP_GATTC_SEARCH_RES_EVT:
        if (!owned_connection(param->search_res.conn_id, NULL)) return;
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (!owned_connection(param->search_cmpl.conn_id, NULL) ||
            param->search_cmpl.status != ESP_GATT_OK) return;
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (!owned_connection(param->write.conn_id, NULL)) return;
        if (event == ESP_GATTC_WRITE_DESCR_EVT &&
            param->write.handle != s_cccd_handle) return;
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (!owned_connection(param->notify.conn_id, param->notify.remote_bda) ||
            param->notify.handle != s_char_notify_handle || !s_notify_ready) return;
        break;
    default:
        break;
    }
    switch (event) {
    case ESP_GATTC_REG_EVT: {
        ESP_LOGI(TAG, "OBD GATTC register complete app=%u status=%d", param->reg.app_id, param->reg.status);
        if (param->reg.app_id != 0 || param->reg.status != ESP_GATT_OK) return;
        s_gattc_if = gattc_if;
        gso_register(gattc_if);
        configure_scan_params();
        break;
    }
    case ESP_GATTC_CONNECT_EVT: {
        // Physical ACL connection is not proof that our GATT client OPEN
        // succeeded. Inbound phones must never be searched as ELM327 adapters.
        break;
    }
    case ESP_GATTC_OPEN_EVT: {
        if (!s_open_pending ||
            memcmp(param->open.remote_bda, s_open_bda, sizeof(esp_bd_addr_t)) != 0) return;
        if (param->open.status != ESP_GATT_OK) {
            gso_open_settled(gso_callback_token(), false);
            s_open_pending = false;
            ESP_LOGE(TAG, "Open failed status=%d", param->open.status);
            if (!s_ota_paused) start_scan();
            break;
        }
        if (!retarget_open_complete(gattc_if, param->open.conn_id, param->open.remote_bda)) break;
        atomic_fetch_add(&s_obd_target_generation,1);
        elm_diag_reset();
        obd_scan_reset_session();
        gv5_data_reset_session();
        ESP_LOGI(TAG, "Bound adapter GATT client opened conn=%u", s_conn_id);
        if (s_cbs.on_connected) s_cbs.on_connected();
        request_discovery();
        break;
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
        const esp_gatt_id_t *srvc_id = &param->search_res.srvc_id;
        uint16_t sh = param->search_res.start_handle;
        uint16_t eh = param->search_res.end_handle;
        if (srvc_id->uuid.len == ESP_UUID_LEN_16) {
            ESP_LOGD(TAG, "Service found: UUID=0x%04X handle=%04X~%04X",
                     srvc_id->uuid.uuid.uuid16, sh, eh);
            if (srvc_id->uuid.uuid.uuid16 == UUID16_OBD_SERVICE) {
                s_have_service = true;
                s_service_start = sh;
                s_service_end = eh;
                ESP_LOGD(TAG, "Target service FFF0 matched");
            } else if (srvc_id->uuid.uuid.uuid16 == UUID16_OBD_SERVICE_18F0) {
                s_have_18f0 = true;
                s_18f0_start = sh;
                s_18f0_end = eh;
                ESP_LOGD(TAG, "Target service 18F0 matched (IOS-Vlink OBD)");
            } else if (srvc_id->uuid.uuid.uuid16 == UUID16_OBD_SERVICE_FF12) {
                s_have_ff12 = true;
                s_ff12_start = sh;
                s_ff12_end = eh;
                ESP_LOGD(TAG, "Target service FF12 matched");
            }
        } else {
            ESP_LOGD(TAG, "Service found: UUID(long) handle=%04X~%04X", sh, eh);
        }
        // Record the max handle range, used for the full-range fallback search
        if (eh > s_all_attr_end || s_all_attr_end == 0xFFFF) s_all_attr_end = eh;
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGD(TAG, "Service discovery complete. have_FFF0=%d have_18F0=%d have_FF12=%d",
                 s_have_service, s_have_18f0, s_have_ff12);

        // Preference order: 0xFFF0 > 0x18F0 (IOS-Vlink) > 0xFF12 > full-range fallback
        if (!s_have_service) {
            if (s_have_18f0) {
                s_service_start = s_18f0_start;
                s_service_end   = s_18f0_end;
                ESP_LOGD(TAG, "Using 18F0 service range 0x%04X~0x%04X", s_service_start, s_service_end);
            } else if (s_have_ff12) {
                s_service_start = s_ff12_start;
                s_service_end   = s_ff12_end;
                ESP_LOGD(TAG, "Using FF12 service range 0x%04X~0x%04X", s_service_start, s_service_end);
            } else {
                s_service_start = 0x0001;
                s_service_end = (s_all_attr_end > 0x0001) ? s_all_attr_end : 0xFFFF;
                ESP_LOGW(TAG, "FFF0/18F0/FF12 not found, using full range 0x0001~0x%04X", s_service_end);
            }
        }

        // Enumerate all characteristics and select by property (WRITE/NOTIFY)
        uint16_t char_count = 0;
        esp_err_t ret = esp_ble_gattc_get_attr_count(gattc_if, s_conn_id,
            ESP_GATT_DB_CHARACTERISTIC, s_service_start, s_service_end, 0, &char_count);
        ESP_LOGD(TAG, "get_attr_count ret=%d, char_count=%d", ret, char_count);

        if (ret != ESP_OK || char_count == 0) {
            ESP_LOGE(TAG, "No characteristics found in range! Cannot communicate.");
            break;
        }

        // Allocate the characteristic array
        uint16_t alloc_count = char_count;
        esp_gattc_char_elem_t *chars = (esp_gattc_char_elem_t *)malloc(alloc_count * sizeof(esp_gattc_char_elem_t));
        if (!chars) { ESP_LOGE(TAG, "malloc failed"); break; }

        ret = esp_ble_gattc_get_all_char(gattc_if, s_conn_id,
            s_service_start, s_service_end, chars, &alloc_count, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "get_all_char failed: %d", ret);
            free(chars); break;
        }

        // Print all characteristics and auto-select the write/notify handles
        ESP_LOGD(TAG, "=== All characteristics (%d) ===", alloc_count);
        for (int i = 0; i < alloc_count; i++) {
            esp_gattc_char_elem_t *c = &chars[i];
            if (c->uuid.len == ESP_UUID_LEN_16) {
                ESP_LOGD(TAG, "  [%d] UUID=0x%04X handle=0x%04X prop=0x%02X",
                         i, c->uuid.uuid.uuid16, c->char_handle, c->properties);
            } else if (c->uuid.len == ESP_UUID_LEN_128) {
                ESP_LOGD(TAG, "  [%d] UUID128=%02X%02X...%02X%02X handle=0x%04X prop=0x%02X",
                         i, c->uuid.uuid.uuid128[15], c->uuid.uuid.uuid128[14],
                            c->uuid.uuid.uuid128[1],  c->uuid.uuid.uuid128[0],
                            c->char_handle, c->properties);
            }
            // Pick the first characteristic with the WRITE property as the write handle
            if (s_char_write_handle == 0 &&
                (c->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR))) {
                s_char_write_handle = c->char_handle;
                // Prefer WRITE_NR (write without response) to avoid status=3 (WRITE_NOT_PERMIT)
                s_write_type = (c->properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR)
                               ? ESP_GATT_WRITE_TYPE_NO_RSP
                               : ESP_GATT_WRITE_TYPE_RSP;
                ESP_LOGD(TAG, "  >> Selected as WRITE handle: 0x%04X (write_type=%s)",
                         s_char_write_handle,
                         s_write_type == ESP_GATT_WRITE_TYPE_NO_RSP ? "NO_RSP" : "RSP");
            }
            // Pick the first characteristic with the NOTIFY property as the notify handle
            if (s_char_notify_handle == 0 &&
                (c->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
                s_char_notify_handle = c->char_handle;
                ESP_LOGD(TAG, "  >> Selected as NOTIFY handle: 0x%04X", s_char_notify_handle);
            }
        }
        free(chars);

        if (s_char_write_handle == 0) {
            ESP_LOGE(TAG, "No WRITE characteristic found! Cannot send commands.");
            break;
        }
        // If there's no dedicated NOTIFY characteristic, reuse the write handle
        if (s_char_notify_handle == 0) {
            s_char_notify_handle = s_char_write_handle;
            ESP_LOGD(TAG, "No NOTIFY char found, using WRITE handle 0x%04X for notify", s_char_notify_handle);
        }

        // Register for notifications
        int sret = esp_ble_gattc_register_for_notify(gattc_if, s_peer_bda, s_char_notify_handle);
        ESP_LOGD(TAG, "register_for_notify handle=0x%04X ret=%d", s_char_notify_handle, sret);

        // Find the CCCD
        esp_gattc_descr_elem_t descr_elems[2];
        uint16_t count = 2;
        esp_bt_uuid_t cccd_uuid = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = UUID16_CCCD } };
        ret = esp_ble_gattc_get_descr_by_char_handle(gattc_if, s_conn_id,
            s_char_notify_handle, cccd_uuid, descr_elems, &count);
        if (ret == ESP_OK && count > 0) {
            s_cccd_handle = descr_elems[0].handle;
            ESP_LOGD(TAG, "Found CCCD, handle: 0x%04X", s_cccd_handle);
        } else {
            ESP_LOGW(TAG, "CCCD not found (ret=%d cnt=%d)", ret, count);
        }
        ESP_LOGI(TAG,"service=%04X h=%04X-%04X wr=%04X nt=%04X cc=%04X",
            s_have_service?UUID16_OBD_SERVICE:s_have_18f0?UUID16_OBD_SERVICE_18F0:
            s_have_ff12?UUID16_OBD_SERVICE_FF12:0,s_service_start,s_service_end,
            s_char_write_handle,s_char_notify_handle,s_cccd_handle);
        enable_notify_if_ready();
        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT: {
        if (param->write.status == ESP_GATT_OK) {
            ESP_LOGD(TAG, "Notifications enabled");
            ESP_LOGI(TAG,"CCCD ready h=%04X",s_cccd_handle);
            s_notify_ready = true;   // subscription ready → let the poll task proceed with ELM init
        } else {
            ESP_LOGW(TAG, "Enable notify failed status=%d", param->write.status);
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
        const uint8_t *v = param->notify.value;
        int n = param->notify.value_len;
        if(!s_zc6_can_monitor_active&&s_accum_len>0&&
           esp_timer_get_time()-s_accum_start_us>10000000){
            s_accum_len=0;s_accum_buf[0]=0;s_elm_ready=true;elm_diag_uncertain();
        }
        if(!s_zc6_can_monitor_active&&s_accum_len==0){elm_diag_begin_response();obd_scan_begin_fragment();}
        if(memchr(v,'>',(size_t)n))elm_diag_prompt();
        if (s_cbs.on_raw_notify) s_cbs.on_raw_notify(v,param->notify.value_len);

        // CAN continuous monitor mode: feed byte-wise, parse line-wise, bypass the accumulation buffer
        if (s_zc6_can_monitor_active) {
            zc6_can_monitor_feed(v, (size_t)n);
            break;
        }

        // ---- Accumulate multi-packet data until '>' (the ELM327 prompt) is received ----
        // Accumulation timeout guard: force-flush if '>' doesn't arrive within 10s (ATMA mode may go long without a '>' prompt)
        if (s_accum_len > 0) {
            int64_t now_us = esp_timer_get_time();
            if ((now_us - s_accum_start_us) > 10000000) {
                s_accum_len = 0;
                s_accum_buf[0] = '\0';
                s_elm_ready = true;elm_diag_uncertain();
            }
        }
        if (s_accum_len == 0) {
            s_accum_start_us = esp_timer_get_time();
        }
        size_t space_left = ACCUM_BUF_SIZE - 1 - s_accum_len;
        size_t copy_n = ((size_t)n < space_left) ? (size_t)n : space_left;
        memcpy(s_accum_buf + s_accum_len, v, copy_n);
        s_accum_len += copy_n;
        s_accum_buf[s_accum_len] = '\0';

        // Keep waiting if '>' hasn't arrived
        if (memchr(s_accum_buf, '>', s_accum_len) == NULL) break;

        char *buf = s_accum_buf;
        elm_diag_complete_response(buf);
        portENTER_CRITICAL(&s_elm_diag_mux);bool scanning=s_scan_active;portEXIT_CRITICAL(&s_elm_diag_mux);
        if(scanning){
            if(copy_n!=(size_t)n){elm_diag_uncertain();}
            /* Clear first, then publish scan completion; no request/fragment race. */
            obd_scan_response(buf);
            break;
        }

        portENTER_CRITICAL(&s_elm_diag_mux);bool fast_poll=s_fast_enabled;portEXIT_CRITICAL(&s_elm_diag_mux);
        if(fast_poll&&!vehicle_profile_uses_standard_poll())fast_poll=false;
        if(fast_poll&&!obd_fast_response(buf,copy_n==(size_t)n)){
            s_accum_len=0;s_accum_buf[0]=0;obd_fast_complete();break;
        }

        char *p61 = strstr(buf, "61 01"); // Mode 21 response header (exact match "61 01")
        char *p41 = strstr(buf, "41 ");
        char *p62 = strstr(buf, "62 ");
        // ---- CAN broadcast frame parsing: data-driven ----
        const vehicle_profile_t *vp_can = vehicle_profile_get_active();
        const vehicle_override_t *ov_can = vehicle_profile_get_override();
        bool has_can_rules = ov_can && ov_can->can_rules && ov_can->can_rule_count > 0;
        // Compat with the legacy can_broadcast_mode flag
        bool ft86_can_mode = (vp_can && vp_can->can_broadcast_mode) || has_can_rules;

        // Collect all unique CAN IDs from the override and search for them in buf
        if (ft86_can_mode) {
            uint16_t seen_ids[8] = {0};
            uint8_t seen_count = 0;
            // Decide which rule table to scan: override first, otherwise the legacy hardcoded one
            const can_rule_t *rules = has_can_rules ? ov_can->can_rules : NULL;
            uint8_t rule_count = has_can_rules ? ov_can->can_rule_count : 0;

            // If there are no override rules but can_broadcast_mode=true, use the legacy inline parsing (compat)
            if (!has_can_rules) {
                // Legacy ZC/N6 hardcoded parsing (kept for compatibility)
                char *p140 = strstr(buf, "140 ");
                if (p140) {
                    unsigned id=0,b0,b1,b2,b3,b4,b5,b6,b7;
                    int vals = sscanf(p140, "%x %x %x %x %x %x %x %x %x", &id,&b0,&b1,&b2,&b3,&b4,&b5,&b6,&b7);
                    if (vals >= 9 && id == 0x140) {
                        uint16_t can_rpm = (uint16_t)(b2 | ((b3 & 0x3F) << 8));
                        if (s_cbs.on_parsed_rpm) s_cbs.on_parsed_rpm(can_rpm);
                        uint8_t tps_pct = (uint8_t)((uint32_t)b6 * 100 / 255);
                        if (s_cbs.on_parsed_throttle_position) s_cbs.on_parsed_throttle_position(tps_pct);
                    }
                }
                char *p360 = strstr(buf, "360 ");
                if (p360) {
                    unsigned id=0,b0,b1,b2,b3,b4,b5,b6,b7;
                    int vals = sscanf(p360, "%x %x %x %x %x %x %x %x %x", &id,&b0,&b1,&b2,&b3,&b4,&b5,&b6,&b7);
                    if (vals >= 9 && id == 0x360) {
                        int32_t oil_c = (int32_t)b2 - 40;
                        int32_t clt_c = (int32_t)b3 - 40;
                        if (oil_c >= -40 && oil_c <= 215 && s_cbs.on_parsed_oil_temp)
                            s_cbs.on_parsed_oil_temp((uint32_t)oil_c);
                        if (clt_c >= -40 && clt_c <= 215 && s_cbs.on_parsed_coolant_temp)
                            s_cbs.on_parsed_coolant_temp((uint32_t)clt_c);
                    }
                }
                char *p0d1 = strstr(buf, "0D1 ");
                if (p0d1) {
                    unsigned id=0,b0,b1,b2,b3;
                    int vals = sscanf(p0d1, "%x %x %x %x %x", &id,&b0,&b1,&b2,&b3);
                    if (vals >= 4 && id == 0x0D1) {
                        uint8_t speed_kmh = (uint8_t)(((b0 | (b1 << 8)) * 157 + 5000) / 10000);
                        if (s_cbs.on_parsed_speed_kmh) s_cbs.on_parsed_speed_kmh(speed_kmh);
                    }
                }
                mark_obd_data_valid();
                goto can_parse_done;
            }

            // ---- Generic CAN rule parsing ----
            // Collect unique CAN IDs
            for (uint8_t i = 0; i < rule_count; i++) {
                bool found = false;
                for (uint8_t j = 0; j < seen_count; j++) {
                    if (seen_ids[j] == rules[i].can_id) { found = true; break; }
                }
                if (!found && seen_count < 8) seen_ids[seen_count++] = rules[i].can_id;
            }
            // For each unique CAN ID, find and parse it in buf (line-start matches only, to avoid coincidental hits inside data bytes)
            for (uint8_t si = 0; si < seen_count; si++) {
                const char *p = find_can_id_token(buf, seen_ids[si]);
                if (!p) continue;
                // Parse the hex bytes
                uint8_t data[8] = {0};
                uint32_t parsed_id = 0;
                int vals = sscanf(p, "%x %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                                  &parsed_id, &data[0],&data[1],&data[2],&data[3],
                                  &data[4],&data[5],&data[6],&data[7]);
                if (vals < 2 || parsed_id != seen_ids[si]) continue;

                // Apply the rules
                float channels[CH_COUNT];
                for (int c = 0; c < CH_COUNT; c++) channels[c] = -32768.0f;
                can_apply_rules(rules, rule_count, seen_ids[si], data, channels);

                // Write into obd_data_cache
                if (channels[CH_RPM] >= 0 && s_cbs.on_parsed_rpm)
                    s_cbs.on_parsed_rpm((uint16_t)channels[CH_RPM]);
                if (channels[CH_SPEED] >= 0 && s_cbs.on_parsed_speed_kmh)
                    s_cbs.on_parsed_speed_kmh((uint8_t)channels[CH_SPEED]);
                if (channels[CH_OIL_TEMP] > -40 && channels[CH_OIL_TEMP] <= 215 && s_cbs.on_parsed_oil_temp)
                    s_cbs.on_parsed_oil_temp((uint32_t)(int16_t)channels[CH_OIL_TEMP]);
                if (channels[CH_COOLANT] > -40 && channels[CH_COOLANT] <= 215 && s_cbs.on_parsed_coolant_temp)
                    s_cbs.on_parsed_coolant_temp((uint32_t)(int16_t)channels[CH_COOLANT]);
                if (channels[CH_TPS] >= 0 && s_cbs.on_parsed_throttle_position)
                    s_cbs.on_parsed_throttle_position((uint32_t)channels[CH_TPS]);
                if (channels[CH_GEAR] > 0 && channels[CH_GEAR] < 127 && s_cbs.on_parsed_gear)
                    s_cbs.on_parsed_gear((int8_t)channels[CH_GEAR]);
                if (channels[CH_LOAD] >= 0 && s_cbs.on_parsed_load_pct)
                    s_cbs.on_parsed_load_pct((int16_t)channels[CH_LOAD]);

                mark_obd_data_valid();
            }
            can_parse_done: ;
        }
        // Any valid data frame header received → refresh the "valid data" timestamp and set the flag
        if (p41 || p62 || p61) mark_obd_data_valid();

        if (p61 != NULL && s_expect_mode21) {
            s_expect_mode21 = false;
            uint32_t d[64] = {0};
            int count = parse_mode21_data(buf, d, 64);
            int32_t oil_c = 0;
            if (extract_mode21_oil_temp(d, count, &oil_c)) {
                record_oil_temp_success(OIL_TEMP_MODE_TOYOTA_21_01);
                if (s_cbs.on_parsed_oil_temp) s_cbs.on_parsed_oil_temp((uint32_t)oil_c);
            } else {
                record_oil_temp_failure(OIL_TEMP_MODE_TOYOTA_21_01);
            }
        } else if (p41 != NULL && !s_expect_mode21) {
            uint32_t d[6] = {0};
            uint32_t mode = 0, pid = 0;
            int values = sscanf(p41, "%x %x %x %x %x %x %x %x",
                &mode, &pid, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5]);
            if (values >= 3 && mode == 0x41) {
                int dc = values - 2;

                // During protocol detection, only handle RPM (0x0C)
                if (s_protocol_detect_idx >= 0 && pid != 0x0C) {
                    break;  // skip non-target PIDs
                }

                switch (pid) {
                    case 0x40: /* Supported standard PIDs 41..60; 5E is D bit2. */
                        if(atomic_exchange(&s_fuel_probe_pending,false)){
                            uint32_t fuel_bytes[4];
                            bool valid=gauge_fuel_line_bytes(p41,0x40,fuel_bytes,4);
                            atomic_store(&s_fuel_supported,valid&&gauge_fuel_supports_5e(fuel_bytes,4));
                        }
                        break;
                    case 0x5E: {
                        uint16_t raw;
                        if(atomic_load(&s_fuel_supported)){
                            uint32_t fuel_bytes[2];
                            if(gauge_fuel_line_bytes(p41,0x5E,fuel_bytes,2)&&gauge_fuel_decode_5e(fuel_bytes,2,&raw)){
                                atomic_store(&s_fuel_attempts,0);
                                gv5_data_observed(GV5_FUEL_RATE,raw);
                            }
                            else gv5_data_invalidated(GV5_FUEL_RATE);
                        }
                        break;
                    }
                    case 0x04: // Engine load (0~100%)
                        if (dc >= 1 && s_cbs.on_parsed_load_pct && s_protocol_detect_idx < 0)
                            s_cbs.on_parsed_load_pct((uint32_t)d[0] * 100 / 255);
                        break;
                    case 0x05: // Coolant temp
                        if (dc >= 1 && s_cbs.on_parsed_coolant_temp && s_protocol_detect_idx < 0)
                            {elm_diag_first_pid(0x05);s_cbs.on_parsed_coolant_temp((uint32_t)((int32_t)d[0] - 40));}
                        break;
                    case 0x0C: // RPM
                        if (dc >= 2) {
                            uint16_t rpm_val = (uint16_t)(((d[0] << 8) | d[1]) / 4);

                            if (s_protocol_detect_idx >= 0) {
                                // Protocol detection mode
                                s_protocol_detect_rpm = (int32_t)rpm_val;
                                s_protocol_detect_got_response = true;
                            } else {
                                // Normal mode
                                if (s_cbs.on_parsed_rpm)
                                    {elm_diag_first_pid(0x0C);s_cbs.on_parsed_rpm(rpm_val);}
                            }
                        }
                        break;
                    case 0x0D: // Vehicle speed
                        if (dc >= 1 && s_cbs.on_parsed_speed_kmh && s_protocol_detect_idx < 0)
                            s_cbs.on_parsed_speed_kmh((uint8_t)d[0]);
                        break;
                    case 0x0F: // Intake air temp
                        if (dc >= 1 && s_cbs.on_parsed_intake_temp && s_protocol_detect_idx < 0)
                            s_cbs.on_parsed_intake_temp((uint32_t)((int32_t)d[0] - 40));
                        break;
                    case 0x11: // Throttle position TPS (0~100%)
                        if (dc >= 1 && s_cbs.on_parsed_throttle_position && s_protocol_detect_idx < 0)
                            s_cbs.on_parsed_throttle_position((uint32_t)d[0] * 100 / 255);
                        break;
                    case 0x0B: // Intake manifold absolute pressure MAP (kPa) → boost gauge pressure
                        if (dc >= 1 && s_cbs.on_parsed_manifold_pressure && s_protocol_detect_idx < 0)
                            s_cbs.on_parsed_manifold_pressure((uint32_t)d[0]);
                        break;
                    case 0x5C: // Oil temp PID (standard, used for ZD8)
                        if (dc >= 1 && s_cbs.on_parsed_oil_temp && s_protocol_detect_idx < 0) {
                            int32_t oil_temp = (int32_t)d[0] - 40;
                            // Validate range: -40 to 215°C
                            if (oil_temp >= -40 && oil_temp <= 215) {
                                record_oil_temp_success(OIL_TEMP_MODE_PID_5C);
                                s_cbs.on_parsed_oil_temp((uint32_t)oil_temp);
                            } else {
                                record_oil_temp_failure(OIL_TEMP_MODE_PID_5C);
                            }
                        }
                        break;
                    case 0x42: // Battery voltage (mV)
                        if (dc >= 2 && s_cbs.on_parsed_control_module_voltage && s_protocol_detect_idx < 0)
                            {elm_diag_first_pid(0x42);s_cbs.on_parsed_control_module_voltage((d[0] << 8) | d[1]);}
                        break;
                    case 0x44: // Air-fuel ratio AFR - Commanded Equivalence Ratio (λ)
                        // λ = (A*256+B)/32768, range 0~<2
                        // AFR = λ × 14.7, stored ×100: 1470 = 14.70:1
                        if (dc >= 2 && s_cbs.on_parsed_afr && s_protocol_detect_idx < 0) {
                            uint32_t raw = (d[0] << 8) | d[1];
                            // λ = raw / 32768, AFR×100 = λ × 1470
                            uint32_t afr_x100 = (raw * 1470UL) / 32768UL;
                            if (afr_x100 >= 800 && afr_x100 <= 2200) {
                                s_cbs.on_parsed_afr(afr_x100);
                            }
                        }
                        break;
                    default:
                        break;
                }
            }
        } else if (p62 != NULL) {
            // Mode 22 response: "62 HH LL D0 D1 ..."  (d0=A, d1=B)
            // If Mode21 was expected but Mode22 arrived, clear the expect flag and record a failure
            if (s_expect_mode21) {
                record_oil_temp_failure(OIL_TEMP_MODE_TOYOTA_21_01);
                s_expect_mode21 = false;
            }
            uint32_t mode22 = 0, ph = 0, pl = 0, d0 = 0, d1 = 0;
            int values = sscanf(p62, "%x %x %x %x %x", &mode22, &ph, &pl, &d0, &d1);
            if (values >= 4 && mode22 == 0x62 && s_cbs.on_parsed_oil_temp) {
                uint32_t pid16 = (ph << 8) | pl;

                // ---- Engine oil pressure (Mode 22 DID, per-profile: 4436=B58, 586F=N55; absolute hPa, 16-bit) ----
                {
                    const vehicle_profile_t *vp = vehicle_profile_get_active();
                    if (vp && vp->obd_oil_pressure_did != 0 && pid16 == vp->obd_oil_pressure_did) {
                        if (values >= 5 && s_cbs.on_parsed_oil_pressure) {
                            uint32_t hpa = (d0 << 8) | d1;
                            s_cbs.on_parsed_oil_pressure(hpa);
                        }
                        goto oil_temp_done;
                    }
                }

                // ---- Transmission gear (Mode 22 DID, per-profile: D031=BMW ZF 8HP; single byte 0=N,1..8=forward) ----
                {
                    const vehicle_profile_t *vp = vehicle_profile_get_active();
                    if (vp && vp->obd_gear_did != 0 && pid16 == vp->obd_gear_did) {
                        if (values >= 4 && s_cbs.on_parsed_obd_gear) {
                            s_cbs.on_parsed_obd_gear((uint8_t)d0);
                        }
                        goto oil_temp_done;
                    }
                }

                // ---- Data-driven override parsing ----
                if (s_oil_use_override) {
                    const oil_formula_t *oil_f = (s_oil_override_idx == 0) ? s_oil_formula_pri : s_oil_formula_sec;
                    if (oil_f && oil_f->type == OIL_UDS_22) {
                        uint32_t expect_pid = (oil_f->pid[0] << 8) | oil_f->pid[1];
                        if (pid16 == expect_pid) {
                            uint32_t resp_data[4] = {d0, d1, 0, 0};
                            int16_t temp = oil_formula_parse_resp(oil_f, resp_data, (uint8_t)(values - 3));
                            if (temp != -32768) {
                                s_oil_override_fail = 0;
                                record_oil_temp_success(s_oil_mode_priority[0]);
                                s_cbs.on_parsed_oil_temp((uint32_t)temp);
                            } else {
                                s_oil_override_fail++;
                                record_oil_temp_failure(s_oil_mode_priority[0]);
                                if (s_oil_override_fail >= OIL_OVERRIDE_FAIL_MAX && s_oil_formula_sec) {
                                    s_oil_override_idx = 1;
                                    s_oil_override_fail = 0;
                                    ESP_LOGW(TAG, "[Override] primary failed %d times, switch to secondary", OIL_OVERRIDE_FAIL_MAX);
                                }
                            }
                            goto oil_temp_done;
                        }
                    }
                }

                // ---- Legacy PID switch/case (when there's no override or the PID doesn't match) ----
                if (pid16 == 0x1310) {
                    // Mazda Skyactiv oil temp PID 1310: (A*256+B)/100 - 40 (°C), requires 2 data bytes
                    if (values >= 5) {
                        int32_t mazda_oil = (int32_t)(((d0 * 256) + d1) / 100) - 40;
                        if (mazda_oil >= -40 && mazda_oil <= 215) {
                            record_oil_temp_success(OIL_TEMP_MODE_MAZDA_22_1310);
                            s_cbs.on_parsed_oil_temp((uint32_t)mazda_oil);
                        } else {
                            record_oil_temp_failure(OIL_TEMP_MODE_MAZDA_22_1310);
                        }
                    } else {
                        record_oil_temp_failure(OIL_TEMP_MODE_MAZDA_22_1310);
                    }
                } else if (pid16 == 0x111F) {
                    // PID 111F: °C = A - 50 (both Mazda Skyactiv and BMW use this formula)
                    // Decide which mode's stats to record based on the current strategy
                    bool is_bmw_111f = (s_oil_mode_priority[0] == OIL_TEMP_MODE_BMW_22_111F ||
                                        s_oil_mode_priority[1] == OIL_TEMP_MODE_BMW_22_111F ||
                                        s_oil_mode_priority[2] == OIL_TEMP_MODE_BMW_22_111F ||
                                        s_oil_mode_priority[3] == OIL_TEMP_MODE_BMW_22_111F);
                    oil_temp_query_mode_t mode_111f = is_bmw_111f ? OIL_TEMP_MODE_BMW_22_111F
                                                                  : OIL_TEMP_MODE_MAZDA_22_111F;
                    int32_t oil_111f = (int32_t)d0 - 50;
                    if (oil_111f >= -40 && oil_111f <= 215) {
                        record_oil_temp_success(mode_111f);
                        s_cbs.on_parsed_oil_temp((uint32_t)oil_111f);
                    } else {
                        record_oil_temp_failure(mode_111f);
                    }
                } else if (pid16 == 0x5822) {
                    // MINI/BMW oil temp PID 5822: °C = A - 60 (°F = A*9/5 - 76)
                    int32_t mini_oil = (int32_t)d0 - 60;
                    if (mini_oil >= -40 && mini_oil <= 215) {
                        record_oil_temp_success(OIL_TEMP_MODE_MINI_22_5822);
                        s_cbs.on_parsed_oil_temp((uint32_t)mini_oil);
                    } else {
                        record_oil_temp_failure(OIL_TEMP_MODE_MINI_22_5822);
                    }
                } else if (pid16 == 0x4402) {
                    if (s_oil_mode_priority[0] == OIL_TEMP_MODE_BMW_G_22_4402 ||
                        s_oil_mode_priority[1] == OIL_TEMP_MODE_BMW_G_22_4402 ||
                        s_oil_mode_priority[2] == OIL_TEMP_MODE_BMW_G_22_4402) {
                        // BMW G-series oil temp PID 4402: °C = (A*256+B)*191.25/255-48 (two bytes)
                        if (values >= 5) {
                            int32_t raw = (int32_t)d0 * 256 + (int32_t)d1;
                            int32_t bmw_g_oil = (int32_t)(raw * 191.25f / 255.0f - 48.0f);
                            if (bmw_g_oil >= -48 && bmw_g_oil <= 143) {
                                record_oil_temp_success(OIL_TEMP_MODE_BMW_G_22_4402);
                                s_cbs.on_parsed_oil_temp((uint32_t)bmw_g_oil);
                            } else {
                                record_oil_temp_failure(OIL_TEMP_MODE_BMW_G_22_4402);
                            }
                        } else {
                            record_oil_temp_failure(OIL_TEMP_MODE_BMW_G_22_4402);
                        }
                    } else {
                        // BMW F-series oil temp PID 4402: °C = B - 64 (second data byte d1 of the response)
                        if (values >= 5) {
                            int32_t bmw_oil = (int32_t)d1 - 64;
                            if (bmw_oil >= -40 && bmw_oil <= 215) {
                                record_oil_temp_success(OIL_TEMP_MODE_BMW_22_4402);
                                s_cbs.on_parsed_oil_temp((uint32_t)bmw_oil);
                            } else {
                                record_oil_temp_failure(OIL_TEMP_MODE_BMW_22_4402);
                            }
                        } else {
                            record_oil_temp_failure(OIL_TEMP_MODE_BMW_22_4402);
                        }
                    }
                } else if (pid16 == 0xD002) {
                    // BMW G-series oil-pan oil temp PID D002: °C = (A*256+B)*191.25/255-48 (two bytes)
                    if (values >= 5) {
                        int32_t raw = (int32_t)d0 * 256 + (int32_t)d1;
                        int32_t bmw_g_oil = (int32_t)(raw * 191.25f / 255.0f - 48.0f);
                        if (bmw_g_oil >= -48 && bmw_g_oil <= 143) {
                            record_oil_temp_success(OIL_TEMP_MODE_BMW_22_D002);
                            s_cbs.on_parsed_oil_temp((uint32_t)bmw_g_oil);
                        } else {
                            record_oil_temp_failure(OIL_TEMP_MODE_BMW_22_D002);
                        }
                    } else {
                        record_oil_temp_failure(OIL_TEMP_MODE_BMW_22_D002);
                    }
                } else if (pid16 == 0x03F3) {
                    // BMW G-series oil temp PID 03F3: °C = A - 40 (Header 7E0 physical addressing)
                    int32_t bmw_g_oil = (int32_t)d0 - 40;
                    if (bmw_g_oil >= -40 && bmw_g_oil <= 215) {
                        record_oil_temp_success(OIL_TEMP_MODE_BMW_22_03F3);
                        s_cbs.on_parsed_oil_temp((uint32_t)bmw_g_oil);
                    } else {
                        record_oil_temp_failure(OIL_TEMP_MODE_BMW_22_03F3);
                    }
                } else if (pid16 == 0x1017 || pid16 == 0x0011 || pid16 == 0x1C00) {
                    record_oil_temp_success(OIL_TEMP_MODE_UDS_22_10_17);
                    s_cbs.on_parsed_oil_temp((uint32_t)((int32_t)d0 - 40));
                } else {
                    record_oil_temp_failure(OIL_TEMP_MODE_UDS_22_10_17);
                }
            }
            oil_temp_done: ;
        } else {
            // If Mode21 was expected but an unrelated response arrived, record a failure too
            if (s_expect_mode21) {
                record_oil_temp_failure(OIL_TEMP_MODE_TOYOTA_21_01);
                s_expect_mode21 = false;
            }
        }

        // Clear the accumulation buffer after a full response
        s_accum_len = 0;
        s_accum_buf[0] = '\0';
        if(fast_poll)obd_fast_complete();
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT: {
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "Write failed status=%d", param->write.status);
            s_elm_ready = true; // release on write failure too, to prevent the poll task from getting stuck forever
        }
        break;
    }
    default:
        break;
    }
}


static void ensure_obd_poll_task(void)
{
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_poll_task_started, &expected, true)) return;
    if (xTaskCreate(obd_poll_task, "obd_poll", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "OBD poll task creation failed; retry on next explicit connection");
        atomic_store(&s_poll_task_started, false);
    }
}

void elm327_ble_start_default(const char *target_name, const uint8_t mac[6]) {

    const elm327_ble_callbacks_t cbs = {
        .on_connected = default_on_connected,
        .on_disconnected = default_on_disconnected,
        .on_raw_notify = default_on_raw_notify,
        .on_parsed_rpm = default_on_parsed_rpm,
        .on_parsed_speed_kmh = default_on_parsed_speed,
        .on_parsed_coolant_temp = default_on_parsed_coolant_temp,
        .on_parsed_intake_temp = default_on_parsed_intake_temp,
        .on_parsed_oil_temp = default_on_parsed_oil_temp,
        .on_parsed_load_pct = default_on_parsed_load_pct,
        .on_parsed_control_module_voltage = default_on_parsed_control_module_voltage,
        .on_parsed_throttle_position = default_on_parsed_throttle_position,
        .on_parsed_gear = default_on_parsed_gear,
        .on_parsed_manifold_pressure = default_on_parsed_manifold_pressure,
        .on_parsed_afr = default_on_parsed_afr,
        .on_parsed_oil_pressure = default_on_parsed_oil_pressure,
        .on_parsed_obd_gear = default_on_parsed_obd_gear,
    };
    s_scan_only_mode = false;
    bool mac_set = mac && (mac[0]|mac[1]|mac[2]|mac[3]|mac[4]|mac[5]) != 0;
    if (mac_set) {
        memcpy(s_target_bda, mac, sizeof(esp_bd_addr_t));
        s_target_bda_valid = true;
    } else {
        s_target_bda_valid = false;
    }
    elm327_ble_init_and_start(target_name, &cbs);
    s_scan_duration_s = 10;
    start_scan(); // target installed after early stack init: never depend on a second REG event
    ensure_obd_poll_task();
}

// ---- Scan-mode implementation ----

static void ble_ensure_init(void) {
    if (s_ble_inited) return;
    // Init the BLE stack (no target name; init only)
    elm327_ble_init_and_start(NULL, NULL);
}

// For callers that only need peripheral BLE advertising (RaceChrono DIY / SkyGauge pairing) without connecting an OBD device right now:
// idempotently bring up the controller + Bluedroid + GAP/GATTC callbacks, without starting any ELM327 scan/connect.
void elm327_ble_ensure_stack_init(void) {
    ble_ensure_init();
}

void elm327_ble_scan_only_start(int duration_s, ble_scan_found_cb_t cb) {
    ble_ensure_init();
    s_scan_only_mode = true;
    s_scan_cb = cb;
    s_scan_count = 0;
    memset(s_scan_list, 0, sizeof(s_scan_list));
    ESP_LOGD(TAG, "Starting scan-only mode (%ds)...", duration_s);
    s_scan_duration_s = duration_s > 0 ? duration_s : 10;
    start_scan();
}

void elm327_ble_scan_only_stop(void) {
    scan_stop();
    s_scan_only_mode = false;
    ESP_LOGD(TAG, "Scan-only stopped. Found %d devices.", s_scan_count);
}

void elm327_ble_connect_by_addr(const uint8_t mac[6], const char *name) {
    if (!mac || !(mac[0]|mac[1]|mac[2]|mac[3]|mac[4]|mac[5])) return;
    scan_stop();
    s_scan_only_mode = false;
    if (name && name[0]) {
        strncpy(s_target_name, name, sizeof(s_target_name) - 1);
        s_target_name[sizeof(s_target_name) - 1] = '\0';
    }
    memcpy(s_target_bda, mac, sizeof(esp_bd_addr_t));
    s_target_bda_valid = true;

    atomic_fetch_add(&s_obd_target_generation,1);
    obd_scan_reset_session();
    atomic_store(&s_fuel_probe_pending,false);atomic_store(&s_fuel_supported,false);
    atomic_store(&s_fuel_attempts,0);
    gv5_data_reset_session();
    s_scan_retry_pending=false;s_scan_retry_after_us=0;atomic_store(&s_scan_skip_verified,false);
    retarget_close_explicit();
    s_notify_ready=false;
    if(s_poll_task_handle)xTaskNotify(s_poll_task_handle,0,eNoAction);
    ESP_LOGD(TAG, "Connect by addr: %02X:%02X:%02X:%02X:%02X:%02X (%s)",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], name ? name : "");

    // Set default callbacks (if not already set)
    if (!s_cbs.on_connected) {
        s_cbs.on_connected = default_on_connected;
        s_cbs.on_disconnected = default_on_disconnected;
        s_cbs.on_raw_notify = default_on_raw_notify;
        s_cbs.on_parsed_rpm = default_on_parsed_rpm;
        s_cbs.on_parsed_speed_kmh = default_on_parsed_speed;
        s_cbs.on_parsed_coolant_temp = default_on_parsed_coolant_temp;
        s_cbs.on_parsed_intake_temp = default_on_parsed_intake_temp;
        s_cbs.on_parsed_oil_temp = default_on_parsed_oil_temp;
        s_cbs.on_parsed_load_pct = default_on_parsed_load_pct;
        s_cbs.on_parsed_control_module_voltage = default_on_parsed_control_module_voltage;
        s_cbs.on_parsed_throttle_position = default_on_parsed_throttle_position;
        s_cbs.on_parsed_gear = default_on_parsed_gear;
        s_cbs.on_parsed_manifold_pressure = default_on_parsed_manifold_pressure;
        s_cbs.on_parsed_afr = default_on_parsed_afr;
        s_cbs.on_parsed_oil_pressure = default_on_parsed_oil_pressure;
        s_cbs.on_parsed_obd_gear = default_on_parsed_obd_gear;
    }
    // Keep the request pending while an old scan/native connection retires.
    s_scan_duration_s = 10;
    start_scan();
    // Create the poll task (if not already created)
    ensure_obd_poll_task();
}

bool elm327_ble_is_connected(void) {
    return s_connected;
}

bool elm327_ble_restart_current_session(void) {
    if (!s_target_bda_valid) return false;
    uint8_t mac[6];
    char name[sizeof s_target_name];
    memcpy(mac, s_target_bda, sizeof mac);
    memcpy(name, s_target_name, sizeof name);
    name[sizeof name - 1] = '\0';
    elm327_ble_connect_by_addr(mac, name);
    return true;
}

void elm327_ble_disconnect(void) {
    if (s_connected && s_gattc_if != ESP_GATT_IF_NONE && s_conn_id != 0xFFFF) {
        ESP_LOGD(TAG, "Disconnecting from BLE device...");
        esp_ble_gattc_close(s_gattc_if, s_conn_id);
    }
}

// Called when entering WiFi OTA mode: drop the ELM327 link and suppress the
// auto-reconnect + poll loop, so the single 2.4GHz radio is fully available to
// the SoftAP. This fixes the upload stalling mid-transfer while the ELM327
// GATT client keeps re-scanning/reconnecting in the background.
void elm327_ble_pause_for_ota(void) {
    s_ota_paused = true;
    scan_stop();
    if (s_connected && s_gattc_if != ESP_GATT_IF_NONE && s_conn_id != 0xFFFF) {
        esp_ble_gattc_close(s_gattc_if, s_conn_id);
    }
    if (s_poll_task_handle) xTaskNotify(s_poll_task_handle, 0, eNoAction);
    ESP_LOGI(TAG, "BLE paused for WiFi OTA");
}

// Called when leaving WiFi OTA mode: re-arm auto-reconnect if a target is bound.
void elm327_ble_resume_after_ota(void) {
    s_ota_paused = false;
    if (s_target_bda_valid && !s_connected) {
        start_scan();
    }
    ESP_LOGI(TAG, "BLE resumed after WiFi OTA");
}

const char *elm327_ble_get_connected_name(void) {
    return s_target_name;
}

// ---- Oil-temp calibration API implementation ----
void elm327_oil_temp_set_offset(int8_t offset_c) {
    s_oil_temp_offset = offset_c;
    ESP_LOGI(TAG, "OIL temp offset set to %d°C", offset_c);
}

int8_t elm327_oil_temp_get_offset(void) {
    return s_oil_temp_offset;
}

void elm327_oil_temp_get_diag(elm327_oil_diag_t *out) {
    if (!out) return;
    out->mode0_ok = s_oil_diag.mode0_ok;
    out->mode1_ok = s_oil_diag.mode1_ok;
    out->mode2_ok = s_oil_diag.mode2_ok;
    out->mode0_fail = s_oil_diag.mode0_fail;
    out->mode1_fail = s_oil_diag.mode1_fail;
    out->mode2_fail = s_oil_diag.mode2_fail;
    out->last_raw = s_oil_diag.last_raw_temp;
    out->last_filtered = s_oil_diag.last_filtered_temp;
    out->current_mode = s_oil_query_mode;
    
    ESP_LOGI(TAG, "OIL DIAG: Mode0(01 5C)=%u/%u, Mode1(22 10 17)=%u/%u, Mode2(21 01)=%u/%u, Mode3(22 11 1F Mz)=%u/%u, Mode4(22 13 10)=%u/%u, Mode6(22 58 22)=%u/%u, Mode7(22 44 02 F)=%u/%u, Mode8(22 03 F3)=%u/%u, Mode9(22 44 02 G)=%u/%u, Mode10(22 D0 02)=%u/%u, Mode11(22 11 1F BM)=%u/%u",
             out->mode0_ok, out->mode0_fail, out->mode1_ok, out->mode1_fail,
             out->mode2_ok, out->mode2_fail, s_oil_diag.mode3_ok, s_oil_diag.mode3_fail,
             s_oil_diag.mode4_ok, s_oil_diag.mode4_fail,
             s_oil_diag.mode6_ok, s_oil_diag.mode6_fail, s_oil_diag.mode7_ok, s_oil_diag.mode7_fail,
             s_oil_diag.mode8_ok, s_oil_diag.mode8_fail,
             s_oil_diag.mode9_ok, s_oil_diag.mode9_fail,
             s_oil_diag.mode10_ok, s_oil_diag.mode10_fail,
             s_oil_diag.mode11_ok, s_oil_diag.mode11_fail);
}
