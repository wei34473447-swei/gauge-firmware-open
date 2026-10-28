#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Fixed-memory integration of measured 01 5E fuel rate + measured speed.
 * Trip is valid only for a continuous, trustworthy paired-sample interval. */
typedef struct {
    double litres, kilometres;
    int64_t stamp_us;
    float previous_lph, previous_kmh;
    bool started, complete;
} gauge_fuel_t;
void gauge_fuel_reset(gauge_fuel_t *state);
void gauge_fuel_missing(gauge_fuel_t *state);
void gauge_fuel_step(gauge_fuel_t *state, float litres_per_hour, float speed_kmh, int64_t now_us, bool speed_fresh);
bool gauge_fuel_average(const gauge_fuel_t *state, float *litres_per_100km);
/* Strict mode-01 bytes: supported range 41..60, PID 5E is D bit 2. */
bool gauge_fuel_supports_5e(const uint32_t *bytes, size_t count);
bool gauge_fuel_decode_5e(const uint32_t *bytes, size_t count, uint16_t *raw);
/* Read only this ELM response line; never consume the next ECU's payload. */
bool gauge_fuel_line_bytes(const char *response, uint8_t pid, uint32_t *bytes, size_t expected_count);
