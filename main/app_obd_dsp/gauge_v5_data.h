#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { GV5_ZH=0, GV5_EN=1, GV5_JA=2, GV5_ZH_TW=3 } gv5_lang_t;
typedef enum {
    GV5_NONE=0, GV5_RPM, GV5_COOLANT, GV5_IAT, GV5_OIL, GV5_THROTTLE,
    GV5_LOAD, GV5_SPEED, GV5_VOLTAGE, GV5_BOOST, GV5_OIL_PRESSURE,
    GV5_BRAKE_TEMP, GV5_AFR, GV5_GEAR, GV5_PEDAL, GV5_FUEL_RATE,
    GV5_FUEL_CONSUMPTION, GV5_TRIP_FUEL, GV5_BATTERY_POWER,
    GV5_EV_CONSUMPTION, GV5_TRIP_ENERGY, GV5_MAP_ABS, GV5_BARO, GV5_FIELD_COUNT
} gv5_field_t;
typedef enum { GV5_UNSUPPORTED=0, GV5_UNSEEN, GV5_OFFLINE, GV5_STALE,
               GV5_INVALID, GV5_CONFIRMED } gv5_state_t;
typedef struct {
    const char *key;
    const char *label[4];
    const char *short_label[4];
    const char *unit;
    float min, max;
    uint8_t precision;
    bool implemented, continuous, trend;
    uint32_t ttl_ms;
} gv5_meta_t;
typedef struct {
    float value;
    bool valid;
    gv5_state_t state;
    int64_t updated_us;
    uint32_t age_ms;
} gv5_reading_t;
typedef struct {
    bool online;
    bool slave;
    uint32_t epoch;
    int64_t captured_us;
    gv5_reading_t readings[GV5_FIELD_COUNT];
} gv5_frame_t;

const gv5_meta_t *gv5_meta(gv5_field_t field);
const char *gv5_label(gv5_field_t field, gv5_lang_t lang, bool abbreviated);
bool gv5_continuous(gv5_field_t field);
bool gv5_trend_eligible(gv5_field_t field);
/* Traction-energy fields, including hybrid drivetrains; not the 12 V voltage. */
bool gv5_is_electric(gv5_field_t field);
void gv5_capture(gv5_frame_t *frame);
gv5_reading_t gv5_read(const gv5_frame_t *frame, gv5_field_t field);
/* Confirmed means a finite accepted cache setter sample within TTL on this link.
 * It is not a standard supported-PID bitmap or vehicle certification. */
bool gv5_usable(const gv5_frame_t *frame, gv5_field_t field);
/* Cache setter hook: raw integer cache units, excluding simulated overrides. */
void gv5_data_observed(gv5_field_t field, int32_t raw);
void gv5_data_invalidated(gv5_field_t field);
/* Existing cache reset hooks also mark an OBD session boundary. */
void gv5_data_reset_session(void);
/* Latest 30 one-second samples, oldest first; NAN means a missing sample. */
size_t gv5_history(gv5_field_t field, float *out, size_t capacity);
