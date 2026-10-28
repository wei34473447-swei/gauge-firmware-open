#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "gauge_imu_peak.h"

#define GAUGE_IMU_POLL_MS 20u
#define GAUGE_IMU_STALE_MS 250u

/* Positive sensor axis used as vehicle-forward hint; AUTO prefers +Z for a
 * screen facing the driver, then +Y/+X when the screen is flat. Installation
 * up comes from a gravity reference; forward/right are fixed sensor vectors. */
typedef enum {
    GAUGE_IMU_MOUNT_AUTO=0,
    GAUGE_IMU_MOUNT_X=1,
    GAUGE_IMU_MOUNT_Y=2,
    GAUGE_IMU_MOUNT_Z=3,
} gauge_imu_mount_axis_t;

/* Result of installation capture within the last accepted ZERO action.
 * Centering is accepted independently: untrusted/parallel keeps the previous
 * installation and quality flags; no raw-motion gravity/bias calibration. */
typedef enum {
    GAUGE_IMU_ZERO_MOUNT_NOT_REQUESTED=0,
    GAUGE_IMU_ZERO_MOUNT_CAPTURED=1,
    GAUGE_IMU_ZERO_MOUNT_UNTRUSTED=2,
    GAUGE_IMU_ZERO_MOUNT_PARALLEL=3,
} gauge_imu_zero_mount_capture_t;

typedef struct {
    /* calibrated = gravity/reference gyro DC established from a stable
     * sensor window assuming actual startup rest, NOT certified vehicle rest.
     * Constant startup yaw/force vs DC/gravity remain six-axis ambiguous.
     * Later automatic DC adaptation observes perpendicular-to-g only.
     * stationary is corrected-rate auto-feedback eligibility; explicit park
     * CAL can use a raw stable window. ZERO never promotes pose/history. */
    bool ready, valid, calibrated, stationary;
    bool projection_ready, reverse_lr, reverse_fb;
    uint8_t address, who_am_i;
    int64_t timestamp_us;
    uint32_t age_ms;
    esp_err_t error;
    /* Sensor-frame axes, g including gravity; NOT certified vehicle axes. */
    float accel_g[3];
    float raw_gyro_dps[3], gyro_dps[3];
    gauge_imu_mount_axis_t mount_axis, effective_forward_axis;
    gauge_imu_zero_mount_capture_t zero_mount_capture;
    bool gravity_initialized, gravity_quiet_validated, gravity_tracking;
    /* Estimated signed rest acceleration in sensor frame, g. Gyro propagates
     * it using actual fresh-sample dt; finite, bounded quiet feedback only.
     * Initialization assumes a gravity-dominated sample until first quiet.
     * Six axes cannot identify arbitrary startup/sustained acceleration. */
    float gravity_g[3], linear_accel_g[3];
    uint32_t gravity_dt_us;
    /* [vehicle-right, vehicle-forward] estimated linear G, using all XYZ.
     * Fixed installation basis; reverse flags affect screen signs only.
     * Valid raw data does not imply usable orientation: projection_ready and
     * gravity_tracking must both be true for finite measured plane values. */
    float plane_g[2];
    gm_peak_snapshot_t peak;
} gauge_imu_frame_t;

/* Root calls once after LCD/I2C startup. A task retries missing hardware;
 * ESP_OK means the sampling task started, not that physical readings passed. */
esp_err_t gauge_imu_start(i2c_master_bus_handle_t bus);
/* Cache-only, no I2C; peak clipping occurs on a private copy outside lock. */
void gauge_imu_get(gauge_imu_frame_t *out);
/* Acquisition freezes one fixed diagnostic snapshot per 10-second interval.
 * Consumer never performs I2C: nonwaiting cache copy, once per snapshot.
 * Lock busy/no new snapshot/null -> false; nonnull out remains untouched.
 * Diagnostics do not alter measurement, attitude, installation or history. */
typedef struct {
    uint32_t sequence, loops, fresh, nr, dup;
    uint32_t read_max_us, acquire_max_us, loop_gap_max_us, publish_max_us;
    uint32_t cache_wait_max_us, cache_hold_max_us, peak_samples;
    int32_t raw_mg[3], plane_mg[2]; /* 99999 = unavailable, NOT a sample. */
    int32_t gravity_mg[3], linear_mg[3], gyro_mdps[3], install_up_milli[3];
    /* gyro_mdps = original sensor angular rate; 999999 unavailable/overflow. */
    int32_t stable_acc_mean_mg[3], stable_gyro_mean_mdps[3], learned_bias_mdps[3];
    /* Boot-monotonic microseconds: captured at cache-lock acquisition; window
     * start/end cover the current acquisition-owned raw sample window, NOT a
     * 10-second average. Empty/reset window -> count=0 and both times=0.
     * Stable means are valid only for the current accepted, fresh 100-sample
     * window; stale/rejected/nonfinite -> sentinels, never valid zero means.
     * Bias is the current cache value at capture, independently valid after
     * first-quiet learning even if this raw window is rejected/reset/stale.
     * These six-axis observations do not certify actual vehicle rest. */
    int64_t captured_us, window_start_us, window_end_us;
    uint8_t window_count, quiet_reject;
    bool stable_mean_valid, learned_bias_valid;
    /* quiet_reject is the latest existing gate mask: 1=count, 2=span,
     * 4=raw norm/rate OR corrected-rate feedback, 8=variance/span, 16=drift.
     * A valid raw mean can coexist with reject=4 / stationary=false. */
    bool valid, tracking, quiet_validated;
    gauge_imu_mount_axis_t mount_axis, effective_forward_axis;
} gauge_imu_diagnostics_t;
bool gauge_imu_get_diagnostics(gauge_imu_diagnostics_t *out);
/* Call from the existing main diagnostic cycle, never the acquisition task.
 * All seven short integer log lines execute after releasing the cache mutex.
 * d f=fresh/not-ready; u=read/acquire/gap/publish/cache-wait/cache-hold maxima
 * in microseconds. w t=captured/start/end; n=count, v=stable-mean validity,
 * b=bias validity, r=existing rejection mask; m a/w=raw stable accel/gyro
 * means; b w=learned bias. Hold maximum includes snapshot assembly/cache copy
 * through its timer sample, excluding the final metric write and unlock.
 * No allocation, task, timer, queue or physical gate change. */
void gauge_imu_log_diagnostics(void);
/* Explicit ZERO: fresh finite real sample, even in motion, no quiet wait.
 * Clear history and center this cached frame immediately. Preserve
 * gravity and gyro bias. Capture current tracked installation-up once using
 * the configured forward axis when attitude has trusted first-quiet quality.
 * Unavailable/parallel capture keeps old mounting and quality, but centering
 * is still accepted. zero_mount_capture states which outcome occurred.
 * NEXT fresh sample resumes absolute linear G in the fixed captured basis, so
 * moving acceleration is never stored as a permanent subtractive offset.
 * Invalid/stale/nonfinite/lock failures preserve state. */
esp_err_t gauge_imu_zero(void);
/* Same command plus an atomic accepted cache snapshot. The returned frame is
 * the ZERO action's actual centered/cleared state even if acquisition resumes
 * before the UI paints. No I2C; out may be NULL. On failure out is untouched. */
esp_err_t gauge_imu_zero_capture(gauge_imu_frame_t *out);
/* Clears only the current power-session envelope, preserves calibration. */
void gauge_imu_clear_peak(void);

/* In-memory signs; changes clear history and preserve gravity/mounting/bias. */
esp_err_t gauge_imu_set_orientation(bool reverse_lr, bool reverse_fb);
/* Explicit installation capture, including applying the SAME selection again:
 * use current fresh finite tracked gravity after first-quiet validation to
 * establish a NEW fixed installation-up/basis. No current quiet requirement;
 * never use a moving raw acceleration as up. Clear history only on success.
 * Parallel axis/stale/untrusted/invalid/lock failures preserve the entire old
 * installation and history. Before gravity exists, store selection only.
 * No I2C/wait/NVS. AUTO records its effective fallback. */
esp_err_t gauge_imu_set_mount_axis(gauge_imu_mount_axis_t axis);
/* Explicit parking/rest alignment after an unobserved long gap. Requires the
 * current fresh finite sample's validated raw stable window and an existing fixed
 * installation basis. Realign gravity/bias, clear history; preserve mounting.
 * No I2C/wait/NVS. Motion/stale/invalid/lock failures preserve all state.
 * The raw stable window tolerates bounded initial gyro DC; a high remaining
 * DC may suppress automatic feedback without blocking this parking action.
 * Quiet IMU data alone cannot certify a stationary car; caller labels this as
 * a user-requested rest calibration, never automatically as a motion ZERO. */
esp_err_t gauge_imu_align_at_rest(void);
