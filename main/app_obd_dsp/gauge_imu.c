#include "gauge_imu.h"
#include "../gauge_memory_diag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <stdatomic.h>
#include <string.h>
#include <limits.h>

/* QST QMI8658A Rev A: explicit little endian, +/-4g, +/-512dps,
 * both ODR setting 6 (112.1 Hz in 6DOF), LPF mode 3 (~15 Hz). */
#define IMU_IO_TIMEOUT_MS 25
#define IMU_RETRY_US 2000000LL
#define IMU_STILL_SAMPLES 100u
/* A 100-sample 50Hz window spans 99 intervals, approximately two seconds. */
#define IMU_STILL_MIN_SPAN_US ((IMU_STILL_SAMPLES-1u)*GAUGE_IMU_POLL_MS*1000LL)
#define IMU_WHO_AM_I 0x00
#define IMU_STATUS0 0x2e
#define IMU_TIMESTAMP 0x30
#define IMU_ACCEL 0x35
/* Engineering error budget for quiet feedback: preserve .01g real forces;
 * reject even a .3g/600s ramp using separated 25-sample means. Faster feedback
 * arrests a small gyro-bias drift without a large accelerometer capture gate.
 * Below the .002g noise budget, tilt/force/bias remain six-axis ambiguous.
 * Gyro-bias tracking below is independent of sustained linear acceleration. */
/* QST board-level initial gyro offset tolerance is +/-10dps per axis.
 * Bootstrap/rest intent tolerates DC offset, NEVER variance or direction drift.
 * Startup assumes the stable window is actual rest; yaw/constant force are
 * six-axis ambiguous and are NOT certified by this initialization. */
#define IMU_INITIAL_GYRO_DC_LIMIT_DPS 12.0f
#define IMU_CORRECTED_QUIET_RATE_DPS 3.0f
#define IMU_QUIET_QUARTER_DRIFT_G .0005f
#define IMU_GRAVITY_CORRECTION_MAX_G .002f
#define IMU_GRAVITY_CORRECTION_RATE 1.0f
static const char *TAG="gauge_imu";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static TaskHandle_t s_task;
static SemaphoreHandle_t s_lock;
static gauge_imu_frame_t s_frame;
static gm_peak_t s_peak;
static float s_baseline[3],s_bias[3],s_mean_acc[3],s_mean_gyro[3];
/* Cache-mutex protected fusion/mounting state; no heap or second task. */
static float s_forward[3],s_right[3],s_install_up[3],s_last_gyro[3];
static float s_gravity_norm;
static bool s_mount_ready;
/* Cache protected. Raw stable window can serve an explicit parking CAL even
 * if unobservable yaw DC exceeds the conservative auto-feedback rate gate. */
static bool s_rest_window_ready;
static int64_t s_attitude_last_us,s_publish_last_us;
static uint32_t s_gravity_corrections,s_attitude_gaps;
static atomic_bool s_clear_requested;
/* Sample window belongs only to the acquisition task. */
static float s_window[IMU_STILL_SAMPLES][6];
static int64_t s_window_times[IMU_STILL_SAMPLES];
static unsigned s_window_count,s_window_next;
static int64_t s_window_last_us;
static int64_t s_quiet_run_start_us;
static uint32_t s_sensor_counter;
static bool s_counter_seen;
static uint32_t s_soft_timestamp_skips,s_window_gap_resets,s_invalid_clears;
/* Acquisition-task-owned fixed counters. Wall times include scheduling delays. */
static struct {uint32_t loops,fresh,nr,dup,reads,read_max,acq_max,gap_max,window_max;
    uint32_t publish_max,cache_wait_max,cache_hold_max;
    uint64_t read_us,acq_us;} s_diag;
/* Only the acquisition task writes s_diag. Consumer reads only this cache
 * protected frozen snapshot; no shared unsynchronized counter reads. */
static gauge_imu_diagnostics_t s_diag_cached;
static bool s_diag_pending;
static uint32_t s_diag_sequence;
static int64_t s_diag_at,s_diag_last_loop;
static uint8_t s_quiet_reject; /* 1=count 2=span 4=norm/gyro 8=variance/span 16=direction drift */
static void failure(esp_err_t err,bool ready);
/* Timing covers acquisition-side cache locks, including waiting behind a UI
 * command/get. Counters have one writer and never format or print under lock. */
static uint32_t elapsed_us(int64_t since)
{
    int64_t dt=esp_timer_get_time()-since;
    return dt<=0?0:dt>UINT32_MAX?UINT32_MAX:(uint32_t)dt;
}
static int64_t acquisition_cache_take(void)
{
    int64_t before=esp_timer_get_time();xSemaphoreTake(s_lock,portMAX_DELAY);
    uint32_t dt=elapsed_us(before);if(dt>s_diag.cache_wait_max)s_diag.cache_wait_max=dt;
    return esp_timer_get_time();
}
static void acquisition_cache_give(int64_t held_since)
{
    uint32_t dt=elapsed_us(held_since);if(dt>s_diag.cache_hold_max)s_diag.cache_hold_max=dt;
    xSemaphoreGive(s_lock);
}
static int32_t diagnostic_mg(float g)
{
    /* This sentinel is telemetry only, never an input to fusion/history.
     * Bound before float->integer conversion so NaN/overflow cannot be UB. */
    if(!isfinite(g)||fabsf(g)>=99.999f)return 99999;
    return (int32_t)lroundf(g*1000.0f);
}
static int32_t diagnostic_mdps(float dps)
{
    if(!isfinite(dps)||fabsf(dps)>=999.999f)return 999999;
    return (int32_t)lroundf(dps*1000.0f);
}
static void freeze_diagnostics(void)
{
    int64_t held=acquisition_cache_take();
    gauge_imu_diagnostics_t d={0};
    d.sequence=++s_diag_sequence;d.loops=s_diag.loops;d.fresh=s_diag.fresh;
    d.nr=s_diag.nr;d.dup=s_diag.dup;d.read_max_us=s_diag.read_max;
    d.acquire_max_us=s_diag.acq_max;d.loop_gap_max_us=s_diag.gap_max;
    d.publish_max_us=s_diag.publish_max;d.cache_wait_max_us=s_diag.cache_wait_max;
    d.peak_samples=s_peak.sample_count;d.valid=s_frame.valid;
    d.tracking=s_frame.gravity_tracking;d.quiet_validated=s_frame.gravity_quiet_validated;
    d.mount_axis=s_frame.mount_axis;d.effective_forward_axis=s_frame.effective_forward_axis;
    /* Only acquisition calls this function and owns window/count/timestamps;
     * cached means, bias and frame are copied together under the cache mutex. */
    d.captured_us=held;d.window_count=(uint8_t)s_window_count;d.quiet_reject=s_quiet_reject;
    if(s_window_count){
        unsigned oldest=s_window_count==IMU_STILL_SAMPLES?s_window_next:0;
        d.window_start_us=s_window_times[oldest];d.window_end_us=s_window_last_us;
    }
    d.stable_mean_valid=s_rest_window_ready&&s_window_count==IMU_STILL_SAMPLES&&
        s_frame.valid&&s_frame.timestamp_us==d.window_end_us&&d.window_start_us>0&&
        d.window_end_us>=d.window_start_us&&
        held>=d.window_end_us&&held-d.window_end_us<=(int64_t)GAUGE_IMU_STALE_MS*1000;
    d.learned_bias_valid=s_frame.gravity_quiet_validated;
    for(unsigned i=0;i<3;i++){
        d.raw_mg[i]=diagnostic_mg(s_frame.accel_g[i]);
        d.gravity_mg[i]=diagnostic_mg(s_frame.gravity_g[i]);
        d.linear_mg[i]=diagnostic_mg(s_frame.linear_accel_g[i]);
        d.gyro_mdps[i]=diagnostic_mdps(s_frame.raw_gyro_dps[i]);
        d.install_up_milli[i]=s_mount_ready?diagnostic_mg(s_install_up[i]):99999;
        d.stable_acc_mean_mg[i]=diagnostic_mg(s_mean_acc[i]);
        d.stable_gyro_mean_mdps[i]=diagnostic_mdps(s_mean_gyro[i]);
        d.learned_bias_mdps[i]=diagnostic_mdps(s_bias[i]);
        if(d.stable_acc_mean_mg[i]==99999||d.stable_gyro_mean_mdps[i]==999999)
            d.stable_mean_valid=false;
        if(d.learned_bias_mdps[i]==999999)d.learned_bias_valid=false;
    }
    for(unsigned i=0;i<2;i++)d.plane_mg[i]=diagnostic_mg(s_frame.plane_g[i]);
    for(unsigned i=0;i<3;i++){
        if(!d.stable_mean_valid){d.stable_acc_mean_mg[i]=99999;d.stable_gyro_mean_mdps[i]=999999;}
        if(!d.learned_bias_valid)d.learned_bias_mdps[i]=999999;
    }
    s_diag_cached=d;s_diag_pending=true;
    /* Include assembly and the expanded cache copy up to this timer sample;
     * the final metric write and unlock itself are outside this measurement. */
    uint32_t hold=elapsed_us(held);
    s_diag_cached.cache_hold_max_us=hold>s_diag.cache_hold_max?hold:s_diag.cache_hold_max;
    acquisition_cache_give(held);
    memset(&s_diag,0,sizeof(s_diag));s_diag_at=esp_timer_get_time();
}

static esp_err_t read_reg(uint8_t reg,uint8_t *out,size_t n)
{
 int64_t before=esp_timer_get_time();
 esp_err_t err=i2c_master_transmit_receive(s_dev,&reg,1,out,n,IMU_IO_TIMEOUT_MS);
 uint32_t elapsed=(uint32_t)(esp_timer_get_time()-before);
 s_diag.reads++;s_diag.read_us+=elapsed;if(elapsed>s_diag.read_max)s_diag.read_max=elapsed;
 return err;
}
static esp_err_t write_reg(uint8_t reg,uint8_t value)
{ const uint8_t data[2]={reg,value};return i2c_master_transmit(s_dev,data,2,IMU_IO_TIMEOUT_MS); }
static uint32_t counter(const uint8_t v[3])
{ return (uint32_t)v[0]|((uint32_t)v[1]<<8)|((uint32_t)v[2]<<16); }
static float decode_le(const uint8_t *p,float scale)
{ return (float)(int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8))*scale; }
static uint32_t age_ms(int64_t now,int64_t stamp)
{ int64_t age=now-stamp;if(age<0)return UINT32_MAX;return age/1000>UINT32_MAX?UINT32_MAX:(uint32_t)(age/1000); }

static esp_err_t configure(void)
{
    esp_err_t err=ESP_ERR_NOT_FOUND;
    if(!s_dev){
        const uint8_t addresses[]={0x6b,0x6a};
        for(unsigned i=0;i<2;i++){
            if(i2c_master_probe(s_bus,addresses[i],IMU_IO_TIMEOUT_MS)!=ESP_OK)continue;
            i2c_device_config_t config={.dev_addr_length=I2C_ADDR_BIT_7,
                .device_address=addresses[i],.scl_speed_hz=400000};
            err=i2c_master_bus_add_device(s_bus,&config,&s_dev);
            if(err!=ESP_OK){s_dev=NULL;continue;}
            uint8_t id=0;err=read_reg(IMU_WHO_AM_I,&id,1);
            if(err==ESP_OK&&id==0x05){
                int64_t held=acquisition_cache_take();
                s_frame.address=addresses[i];s_frame.who_am_i=id;
                acquisition_cache_give(held);break;
            }
            i2c_master_bus_rm_device(s_dev);s_dev=NULL;
        }
        if(!s_dev)return err==ESP_OK?ESP_ERR_NOT_FOUND:err;
    }
    uint8_t id=0;err=read_reg(IMU_WHO_AM_I,&id,1);
    if(err!=ESP_OK)return err;
    if(id!=0x05)return ESP_ERR_NOT_FOUND;
    const uint8_t regs[][2]={{0x08,0x00},{0x02,0x40},{0x03,0x16},
                            {0x04,0x56},{0x06,0x77},{0x07,0x00},{0x08,0x03}};
    for(unsigned i=0;i<sizeof(regs)/sizeof(regs[0]);i++){
        err=write_reg(regs[i][0],regs[i][1]);if(err!=ESP_OK)return err;
    }
    /* Verify the conversion configuration rather than guessing scale. */
    const uint8_t verify[][2]={{0x02,0x40},{0x03,0x16},{0x04,0x56},{0x06,0x77},{0x08,0x03}};
    for(unsigned i=0;i<sizeof(verify)/sizeof(verify[0]);i++){
        uint8_t value=0;err=read_reg(verify[i][0],&value,1);
        if(err!=ESP_OK)return err;
        if(value!=verify[i][1])return ESP_ERR_INVALID_RESPONSE;
    }
    s_counter_seen=false;s_window_count=0;s_window_next=0;s_window_last_us=0;s_quiet_run_start_us=0;
    ESP_LOGI(TAG,"QMI8658 id=%u range_g=4 range_dps=512 ODR_mHz=112100",(unsigned)id);
    return ESP_OK;
}

static esp_err_t acquire(float accel[3],float gyro[3],bool *fresh)
{
    *fresh=false;uint8_t status=0;esp_err_t err=read_reg(IMU_STATUS0,&status,1);
    if(err!=ESP_OK)return err;
    if((status&0x03)!=0x03){s_diag.nr++;return ESP_OK;}
    for(unsigned attempt=0;attempt<2;attempt++){
        uint8_t before[3],after[3],data[12];
        if((err=read_reg(IMU_TIMESTAMP,before,3))!=ESP_OK)return err;
        if((err=read_reg(IMU_ACCEL,data,12))!=ESP_OK)return err;
        if((err=read_reg(IMU_TIMESTAMP,after,3))!=ESP_OK)return err;
        if(counter(before)!=counter(after))continue;
        uint32_t stamp=counter(after);
        if(s_counter_seen&&stamp==s_sensor_counter){s_diag.dup++;return ESP_OK;}
        /* Hardware timestamps wrap at 24 bits. A backward/out-of-order stamp
         * is not a fresh sample; half-range comparison permits normal wrap. */
        if(s_counter_seen&&((stamp-s_sensor_counter)&0xffffffu)>=0x800000u)
            return ESP_ERR_INVALID_RESPONSE;
        for(unsigned i=0;i<3;i++){
            accel[i]=decode_le(data+2*i,4.0f/32768.0f);
            gyro[i]=decode_le(data+6+2*i,512.0f/32768.0f);
            /* Saturation cannot produce a trustworthy new peak. */
            if(!isfinite(accel[i])||!isfinite(gyro[i])||fabsf(accel[i])>=3.9998f||fabsf(gyro[i])>=511.98f)
                return ESP_ERR_INVALID_RESPONSE;
        }
        s_sensor_counter=stamp;s_counter_seen=true;*fresh=true;return ESP_OK;
    }
    /* An ODR update during both bounded attempts is a skipped frame, not a
     * physical-motion or transport-error observation. Never publish it. */
    s_soft_timestamp_skips++;
    return ESP_OK;
}

static bool stationary_window(const float accel[3],const float gyro[3],int64_t now,
                              float mean_acc[3],float mean_gyro[3])
{
    if(s_window_last_us&&now-s_window_last_us>(int64_t)GAUGE_IMU_STALE_MS*1000){
        s_window_count=0;s_window_next=0;s_quiet_run_start_us=0;s_window_gap_resets++;}
    s_window_last_us=now;
    memcpy(s_window[s_window_next],accel,3*sizeof(float));
    memcpy(s_window[s_window_next]+3,gyro,3*sizeof(float));
    s_window_times[s_window_next]=now;
    s_window_next=(s_window_next+1)%IMU_STILL_SAMPLES;
    if(s_window_count<IMU_STILL_SAMPLES)s_window_count++;
    if(s_window_count>s_diag.window_max)s_diag.window_max=s_window_count;
    s_quiet_reject=0;
    if(s_window_count<IMU_STILL_SAMPLES){s_quiet_run_start_us=0;s_quiet_reject=1;return false;}
    double sum[6]={0},sq[6]={0};float lo[3]={INFINITY,INFINITY,INFINITY},hi[3]={-INFINITY,-INFINITY,-INFINITY};
    for(unsigned i=0;i<IMU_STILL_SAMPLES;i++){
        float norm=0;
        for(unsigned a=0;a<6;a++){double v=s_window[i][a];sum[a]+=v;sq[a]+=v*v;}
        for(unsigned a=0;a<3;a++){
            float v=s_window[i][a];norm+=v*v;
            if(fabsf(s_window[i][a+3])>IMU_INITIAL_GYRO_DC_LIMIT_DPS){
                s_quiet_run_start_us=0;s_quiet_reject=4;return false;
            }
            if(v<lo[a])lo[a]=v;
            if(v>hi[a])hi[a]=v;
        }
        if(norm<0.90f*0.90f||norm>1.10f*1.10f){s_quiet_run_start_us=0;s_quiet_reject=4;return false;}
    }
    for(unsigned a=0;a<3;a++){
        double mean=sum[a]/IMU_STILL_SAMPLES,var=sq[a]/IMU_STILL_SAMPLES-mean*mean;
        double gm=sum[a+3]/IMU_STILL_SAMPLES,gv=sq[a+3]/IMU_STILL_SAMPLES-gm*gm;
        if(var>0.015*0.015||gv>0.15*0.15||hi[a]-lo[a]>0.08f){s_quiet_run_start_us=0;s_quiet_reject=8;return false;}
        mean_acc[a]=(float)mean;mean_gyro[a]=(float)gm;
    }
    /* The old variance gate accepted a smooth 1dps tilt as gyro bias, or a
     * slow acceleration ramp as rest. Compare oldest/newest quarter MEANS,
     * not noisy endpoints. Keep the actual g magnitude (e.g. .944g), never
     * normalise moving acceleration to 1g. About 1.5s separates their centres. */
    float drift2=0;
    for(unsigned a=0;a<3;a++){
        double oldest=0,newest=0;
        for(unsigned i=0;i<IMU_STILL_SAMPLES/4;i++){
            oldest+=s_window[(s_window_next+i)%IMU_STILL_SAMPLES][a];
            newest+=s_window[(s_window_next+3*IMU_STILL_SAMPLES/4+i)%IMU_STILL_SAMPLES][a];
        }
        float d=(float)((newest-oldest)/(IMU_STILL_SAMPLES/4));drift2+=d*d;
    }
    if(drift2>IMU_QUIET_QUARTER_DRIFT_G*IMU_QUIET_QUARTER_DRIFT_G){s_quiet_run_start_us=0;s_quiet_reject=16;return false;}
    /* Count the duration of consecutively validated physical windows, not
     * only the newest 99 sample intervals: slightly faster scheduling must
     * not make a genuinely quiet sensor wait forever. Keep both the minimum
     * sample count and the full elapsed duration; any motion/error/gap clears
     * the run. This does not re-zero an already valid board reference. */
    if(!s_quiet_run_start_us)s_quiet_run_start_us=s_window_times[s_window_next];
    if(now-s_quiet_run_start_us<IMU_STILL_MIN_SPAN_US){s_quiet_reject=2;return false;}
    return true; /* IMU quiet detection, not proof the vehicle is stationary. */
}
static float dot3(const float a[3],const float b[3])
{return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static float norm3(const float v[3]) {return sqrtf(dot3(v,v));}
static void cross3(const float a[3],const float b[3],float out[3])
{out[0]=a[1]*b[2]-a[2]*b[1];out[1]=a[2]*b[0]-a[0]*b[2];out[2]=a[0]*b[1]-a[1]*b[0];}
static bool mounting_basis(gauge_imu_mount_axis_t configured,const float up[3],
                           float forward[3],float right[3],gauge_imu_mount_axis_t *effective)
{
    const gauge_imu_mount_axis_t choices[]={GAUGE_IMU_MOUNT_Z,GAUGE_IMU_MOUNT_Y,GAUGE_IMU_MOUNT_X};
    unsigned attempts=configured==GAUGE_IMU_MOUNT_AUTO?3:1;
    for(unsigned i=0;i<attempts;i++){
        gauge_imu_mount_axis_t axis=configured==GAUGE_IMU_MOUNT_AUTO?choices[i]:configured;
        float hint[3]={0};hint[(unsigned)axis-1u]=1;
        float vertical=dot3(hint,up);
        for(unsigned a=0;a<3;a++)forward[a]=hint[a]-vertical*up[a];
        float n=norm3(forward);if(n<.25f)continue;
        for(unsigned a=0;a<3;a++)forward[a]/=n;
        /* Vehicle right = forward cross up. V1 front view USB down: +X up,
         * +Y right, +Z away from driver, so default maps Y/Z without losing Z. */
        cross3(forward,up,right);*effective=axis;return true;
    }
    return false;
}
/* Shared atomic installation transaction for FWD and explicit ZERO. Only
 * estimated trusted gravity supplies up; moving raw acceleration NEVER does.
 * Compute every vector locally before committing, including SAME selection.
 * Ordinary sampling and parking CAL never call this capture transaction. */
static gauge_imu_zero_mount_capture_t capture_installation_locked(gauge_imu_mount_axis_t axis)
{
    float up[3],forward[3],right[3];gauge_imu_mount_axis_t effective=GAUGE_IMU_MOUNT_AUTO;
    bool finite=axis>=GAUGE_IMU_MOUNT_AUTO&&axis<=GAUGE_IMU_MOUNT_Z&&
        s_frame.gravity_initialized&&s_frame.valid&&s_frame.gravity_tracking&&s_frame.gravity_quiet_validated&&
        s_frame.timestamp_us>0&&age_ms(esp_timer_get_time(),s_frame.timestamp_us)<=GAUGE_IMU_STALE_MS;
    for(unsigned a=0;a<3;a++)finite=finite&&isfinite(s_frame.gravity_g[a])&&
        isfinite(s_frame.accel_g[a])&&isfinite(s_frame.raw_gyro_dps[a]);
    float n=norm3(s_frame.gravity_g);
    if(!finite||!isfinite(n)||n<=0)return GAUGE_IMU_ZERO_MOUNT_UNTRUSTED;
    for(unsigned a=0;a<3;a++)up[a]=s_frame.gravity_g[a]/n;
    if(!mounting_basis(axis,up,forward,right,&effective))return GAUGE_IMU_ZERO_MOUNT_PARALLEL;
    memcpy(s_install_up,up,sizeof up);memcpy(s_forward,forward,sizeof forward);memcpy(s_right,right,sizeof right);
    s_frame.mount_axis=axis;s_frame.effective_forward_axis=effective;s_mount_ready=true;
    return GAUGE_IMU_ZERO_MOUNT_CAPTURED;
}
static void establish_mount_locked(const float gravity[3])
{
    float n=norm3(gravity);
    for(unsigned a=0;a<3;a++)s_install_up[a]=gravity[a]/n;
    s_mount_ready=mounting_basis(s_frame.mount_axis,s_install_up,s_forward,s_right,
                                 &s_frame.effective_forward_axis);
}
/* Only attitude gravity rotates. Installation basis above stays fixed in the
 * board frame: following current gravity with vehicle axes would erase pitch
 * forces and invent a moving mounting direction. */
static void project_locked(void)
{
    for(unsigned i=0;i<3;i++)s_frame.linear_accel_g[i]=s_frame.gravity_initialized&&s_frame.gravity_tracking?
        s_frame.accel_g[i]-s_frame.gravity_g[i]:NAN;
    s_frame.projection_ready=s_mount_ready&&s_frame.gravity_tracking;
    if(!s_frame.projection_ready){s_frame.plane_g[0]=s_frame.plane_g[1]=NAN;return;}
    s_frame.plane_g[0]=dot3(s_frame.linear_accel_g,s_right)*(s_frame.reverse_lr?-1:1);
    s_frame.plane_g[1]=dot3(s_frame.linear_accel_g,s_forward)*(s_frame.reverse_fb?-1:1);
}
static bool corrected_quiet_rate_locked(void)
{
    /* Compare the entire raw window to ONE current cache-protected bias.
     * UI parking CAL can change bias; never read it outside this mutex. */
    for(unsigned i=0;i<IMU_STILL_SAMPLES;i++){
        float rate2=0;
        for(unsigned a=0;a<3;a++){
            float rate=s_window[i][a+3]-s_bias[a];
            if(!isfinite(rate))return false;
            rate2+=rate*rate;
        }
        if(rate2>IMU_CORRECTED_QUIET_RATE_DPS*IMU_CORRECTED_QUIET_RATE_DPS)return false;
    }
    return true;
}
static void calibrate_locked(void)
{
    memcpy(s_baseline,s_mean_acc,sizeof(s_baseline));
    /* Retain the existing practical startup-rest assumption: average all DC
     * gyro axes only after the full stable raw window passes. True constant
     * yaw/acceleration at startup remain unobservable, not silently solved.
     * Perpendicular-only bootstrap regresses later pitch/roll when an unknown
     * fixed body-yaw DC becomes transverse; explicit rest CAL uses all axes. */
    memcpy(s_bias,s_mean_gyro,sizeof s_bias);
    memcpy(s_frame.gravity_g,s_mean_acc,sizeof(s_frame.gravity_g));
    s_gravity_norm=norm3(s_frame.gravity_g);
    s_frame.gravity_initialized=true;s_frame.gravity_quiet_validated=true;s_frame.gravity_tracking=true;
    establish_mount_locked(s_frame.gravity_g);
    s_frame.calibrated=true;
}
static void propagate_gravity_locked(const float gyro[3],float dt)
{
    /* Exact Rodrigues update for body-frame g'=-omega cross g. Trapezoidal
     * rates reduce error on a changing real gyro; timestamps supply dt. */
    float rotation[3];
    for(unsigned a=0;a<3;a++)rotation[a]=-(gyro[a]+s_last_gyro[a])*.5f*dt*.017453292519943295f;
    float angle=norm3(rotation);
    if(angle>1e-8f){
        float axis[3],c=cosf(angle),s=sinf(angle),cross[3],old[3];
        memcpy(old,s_frame.gravity_g,sizeof old);
        for(unsigned a=0;a<3;a++)axis[a]=rotation[a]/angle;
        cross3(axis,old,cross);float d=dot3(axis,old);
        for(unsigned a=0;a<3;a++)s_frame.gravity_g[a]=old[a]*c+cross[a]*s+axis[a]*d*(1-c);
        float n=norm3(s_frame.gravity_g);
        if(n>0)for(unsigned a=0;a<3;a++)s_frame.gravity_g[a]*=s_gravity_norm/n;
    }
}
static void update_gravity_locked(bool still,bool just_validated,int64_t now)
{
    float corrected[3];
    s_frame.gravity_dt_us=0;
    if(s_attitude_last_us>0&&now>s_attitude_last_us&&now-s_attitude_last_us<=(int64_t)GAUGE_IMU_STALE_MS*1000)
        s_frame.gravity_dt_us=(uint32_t)(now-s_attitude_last_us);
    if(s_frame.gravity_quiet_validated&&s_frame.gravity_tracking&&still&&!just_validated){
        /* Constant vehicle force does NOT imply rotation or change gyro bias.
         * Trend-validated quiet can gently track the perpendicular gyro bias
         * without learning accelerometer force as gravity. Parallel-to-gravity
         * yaw is unobservable here; leave it alone. No motion ZERO invokes this. */
        float up[3],error[3];for(unsigned a=0;a<3;a++){
            up[a]=s_frame.gravity_g[a]/s_gravity_norm;error[a]=s_mean_gyro[a]-s_bias[a];}
        float yaw=dot3(error,up);
        float alpha=1-expf(-1.0f*(float)s_frame.gravity_dt_us*.000001f);
        for(unsigned a=0;a<3;a++)s_bias[a]+=alpha*(error[a]-yaw*up[a]);
    }
    for(unsigned a=0;a<3;a++)corrected[a]=s_frame.raw_gyro_dps[a]-s_bias[a];
    if(!s_frame.gravity_initialized){
        float n=norm3(s_frame.accel_g);
        if(n>=.75f&&n<=1.25f){
            /* Provisional bootstrap, explicitly not quiet-validated. Starting
             * under an unknown force is unobservable without an outside aid. */
            s_gravity_norm=n>=.9f&&n<=1.1f?n:1.0f;
            for(unsigned a=0;a<3;a++)s_frame.gravity_g[a]=s_frame.accel_g[a]*s_gravity_norm/n;
            s_frame.gravity_initialized=true;s_frame.gravity_tracking=true;
            establish_mount_locked(s_frame.gravity_g);
        }
    }else if(!just_validated&&s_attitude_last_us>0){
        int64_t elapsed=now-s_attitude_last_us;
        if(elapsed>0&&elapsed<=(int64_t)GAUGE_IMU_STALE_MS*1000){
            s_frame.gravity_dt_us=(uint32_t)elapsed;
            if(s_frame.gravity_tracking)propagate_gravity_locked(corrected,(float)elapsed*.000001f);
        }else{s_frame.gravity_tracking=false;s_attitude_gaps++;}
    }
    /* Conservative recovery after unobserved rotation: require quiet and a
     * small residual to the retained gravity. Do not overwrite it with .1g
     * sustained force. A large rotation across a long gap needs re-init; no
     * guessed interpolation, pose or peak is emitted meanwhile. */
    if(!s_frame.gravity_tracking&&s_frame.gravity_initialized&&still){
        float residual[3];for(unsigned a=0;a<3;a++)residual[a]=s_mean_acc[a]-s_frame.gravity_g[a];
        if(norm3(residual)<=IMU_GRAVITY_CORRECTION_MAX_G){s_frame.gravity_tracking=true;}
    }
    if(s_frame.gravity_tracking&&s_frame.gravity_quiet_validated&&still&&!just_validated){
        float residual[3];for(unsigned a=0;a<3;a++)residual[a]=s_mean_acc[a]-s_frame.gravity_g[a];
        if(fabsf(norm3(s_mean_acc)-s_gravity_norm)<=.015f&&norm3(residual)<=IMU_GRAVITY_CORRECTION_MAX_G){
            float alpha=1-expf(-IMU_GRAVITY_CORRECTION_RATE*(float)s_frame.gravity_dt_us*.000001f);
            float next[3];for(unsigned a=0;a<3;a++)next[a]=s_frame.gravity_g[a]+alpha*residual[a];
            float n=norm3(next);
            if(n>0){for(unsigned a=0;a<3;a++)s_frame.gravity_g[a]=next[a]*s_gravity_norm/n;s_gravity_corrections++;}
        }
    }
    memcpy(s_frame.gyro_dps,corrected,sizeof corrected);memcpy(s_last_gyro,corrected,sizeof corrected);
    s_attitude_last_us=now;
}
static void publish(const float accel[3],const float gyro[3],int64_t now)
{
    int64_t publish_before=esp_timer_get_time();
    /* Defensive ingress for coherent driver data and host-time ordering.
     * Invalid input must not poison the window, pose or maximum envelope. */
    bool finite=now>0&&(!s_publish_last_us||now>s_publish_last_us);
    for(unsigned a=0;a<3;a++)finite=finite&&isfinite(accel[a])&&isfinite(gyro[a])&&
        fabsf(accel[a])<3.9998f&&fabsf(gyro[a])<511.98f;
    if(!finite){failure(ESP_ERR_INVALID_RESPONSE,true);return;}
    s_publish_last_us=now;
    float mean_acc[3],mean_gyro[3];bool rest_window=stationary_window(accel,gyro,now,mean_acc,mean_gyro);
    int64_t held=acquisition_cache_take();
    s_rest_window_ready=rest_window;
    bool still=rest_window;
    if(still&&s_frame.gravity_quiet_validated&&!corrected_quiet_rate_locked()){
        still=false;s_quiet_reject=4;
    }
    if(rest_window){memcpy(s_mean_acc,mean_acc,sizeof s_mean_acc);memcpy(s_mean_gyro,mean_gyro,sizeof s_mean_gyro);}
    if(atomic_exchange_explicit(&s_clear_requested,false,memory_order_relaxed))gm_peak_clear(&s_peak);
    memcpy(s_frame.accel_g,accel,3*sizeof(float));memcpy(s_frame.raw_gyro_dps,gyro,3*sizeof(float));
    s_frame.timestamp_us=now;s_frame.valid=true;s_frame.ready=true;s_frame.stationary=still;s_frame.error=ESP_OK;
    bool just_validated=false;
    if(still){
        /* ZERO does not certify attitude/bias or enable provisional history.
         * First quiet establishes the final fixed installation basis, then
         * initializes history in exactly that coordinate system. */
        if(!s_frame.gravity_quiet_validated){
            calibrate_locked();just_validated=true;gm_peak_clear(&s_peak);}}
    update_gravity_locked(still,just_validated,now);
    project_locked();
    if(s_frame.calibrated&&s_frame.projection_ready)gm_peak_add(&s_peak,s_frame.plane_g[0],s_frame.plane_g[1]);
    acquisition_cache_give(held);
    uint32_t elapsed=elapsed_us(publish_before);
    if(elapsed>s_diag.publish_max)s_diag.publish_max=elapsed;
}
static void failure(esp_err_t err,bool ready)
{
    s_invalid_clears++;
    s_window_count=0;s_window_next=0;s_window_last_us=0;s_quiet_run_start_us=0;
    int64_t held=acquisition_cache_take();s_frame.valid=false;s_frame.stationary=false;s_rest_window_ready=false;
    s_frame.ready=ready;s_frame.error=err;acquisition_cache_give(held);
}
static void acquisition_task(void *arg)
{
    (void)arg;bool configured=false;int64_t retry_at=0;unsigned failures=0;
    gauge_stack_diag_record_self(GAUGE_STACK_IMU);
    TickType_t wake=xTaskGetTickCount();
    for(;;){
        int64_t now=esp_timer_get_time();s_diag.loops++;
        if(!s_diag_at)s_diag_at=now;
        if(s_diag_last_loop){uint32_t gap=(uint32_t)(now-s_diag_last_loop);if(gap>s_diag.gap_max)s_diag.gap_max=gap;}
        s_diag_last_loop=now;
        if(!configured&&now>=retry_at){esp_err_t err=configure();configured=err==ESP_OK;
            if(!configured){failure(err,false);retry_at=now+IMU_RETRY_US;}else failures=0;}
        if(configured){float a[3],g[3];bool fresh;int64_t before=esp_timer_get_time();esp_err_t err=acquire(a,g,&fresh);
            uint32_t elapsed=(uint32_t)(esp_timer_get_time()-before);s_diag.acq_us+=elapsed;
            if(elapsed>s_diag.acq_max)s_diag.acq_max=elapsed;
            if(err==ESP_OK){failures=0;if(fresh){s_diag.fresh++;publish(a,g,esp_timer_get_time());}}
            else{failure(err,true);if(++failures>=10){configured=false;retry_at=now+IMU_RETRY_US;}}
        }
        if(atomic_load_explicit(&s_clear_requested,memory_order_relaxed)){
            int64_t held=acquisition_cache_take();
            if(atomic_exchange_explicit(&s_clear_requested,false,memory_order_relaxed))gm_peak_clear(&s_peak);
            acquisition_cache_give(held);
        }
        if(esp_timer_get_time()-s_diag_at>=10000000){
            freeze_diagnostics();
            gauge_stack_diag_record_self(GAUGE_STACK_IMU);
        }
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(GAUGE_IMU_POLL_MS));
    }
}
esp_err_t gauge_imu_start(i2c_master_bus_handle_t bus)
{
    if(!bus)return ESP_ERR_INVALID_ARG;
    if(s_task)return bus==s_bus?ESP_OK:ESP_ERR_INVALID_STATE;
    s_bus=bus;s_lock=xSemaphoreCreateMutex();if(!s_lock)return ESP_ERR_NO_MEM;
    memset(&s_frame,0,sizeof(s_frame));s_frame.error=ESP_ERR_NOT_FOUND;s_frame.age_ms=UINT32_MAX;
    memset(s_bias,0,sizeof s_bias);memset(s_baseline,0,sizeof s_baseline);s_rest_window_ready=false;
    memset(s_last_gyro,0,sizeof s_last_gyro);s_gravity_norm=0;s_mount_ready=false;
    s_attitude_last_us=s_publish_last_us=0;s_gravity_corrections=s_attitude_gaps=0;
    s_frame.plane_g[0]=s_frame.plane_g[1]=NAN;gm_peak_init(&s_peak);
    memset(&s_diag,0,sizeof s_diag);memset(&s_diag_cached,0,sizeof s_diag_cached);
    s_diag_pending=false;s_diag_sequence=0;s_diag_at=s_diag_last_loop=0;
    /* Float/FPU use otherwise pins this task to its first execution core.
     * Isolate acquisition from the observed core0 radio workload; retain
     * priority, stack, timing and physical validity rules unchanged. */
#if CONFIG_FREERTOS_UNICORE
    const BaseType_t imu_core=0;
#else
    const BaseType_t imu_core=1;
#endif
    if(xTaskCreatePinnedToCore(acquisition_task,"gauge_imu",4096,NULL,2,&s_task,imu_core)!=pdPASS){
        vSemaphoreDelete(s_lock);s_lock=NULL;s_task=NULL;return ESP_ERR_NO_MEM;}
    return ESP_OK;
}
void gauge_imu_get(gauge_imu_frame_t *out)
{
    if(!out)return;
    memset(out,0,sizeof(*out));out->age_ms=UINT32_MAX;out->error=ESP_ERR_INVALID_STATE;
    out->plane_g[0]=out->plane_g[1]=NAN;
    if(!s_lock)return;
    if(xSemaphoreTake(s_lock,0)!=pdTRUE){out->error=ESP_ERR_TIMEOUT;return;}
    *out=s_frame;gm_peak_t history=s_peak;xSemaphoreGive(s_lock);
    out->age_ms=out->timestamp_us>0?age_ms(esp_timer_get_time(),out->timestamp_us):UINT32_MAX;
    if(out->age_ms>GAUGE_IMU_STALE_MS){out->valid=false;out->stationary=false;
        if(out->error==ESP_OK)out->error=ESP_ERR_TIMEOUT;}
    gm_peak_snapshot(&history,&out->peak);
}
bool gauge_imu_get_diagnostics(gauge_imu_diagnostics_t *out)
{
    if(!out||!s_lock||xSemaphoreTake(s_lock,0)!=pdTRUE)return false;
    bool available=s_diag_pending;
    if(available){*out=s_diag_cached;s_diag_pending=false;}
    xSemaphoreGive(s_lock);return available;
}
void gauge_imu_log_diagnostics(void)
{
    gauge_imu_diagnostics_t d;
    if(!gauge_imu_get_diagnostics(&d))return;
    /* Complete payloads <=93 bytes even for maximum uint32 counters; no
     * floats and no cache lock held while the console potentially blocks. */
    ESP_LOGI(TAG,"s a=%d,%d,%d p=%d,%d v=%u t=%u q=%u m=%u/%u pk=%lu",
        (int)d.raw_mg[0],(int)d.raw_mg[1],(int)d.raw_mg[2],
        (int)d.plane_mg[0],(int)d.plane_mg[1],(unsigned)d.valid,
        (unsigned)d.tracking,(unsigned)d.quiet_validated,
        (unsigned)d.mount_axis,(unsigned)d.effective_forward_axis,(unsigned long)d.peak_samples);
    ESP_LOGI(TAG,"d f=%lu/%lu u=%lu/%lu/%lu/%lu/%lu/%lu",
        (unsigned long)d.fresh,(unsigned long)d.nr,(unsigned long)d.read_max_us,
        (unsigned long)d.acquire_max_us,(unsigned long)d.loop_gap_max_us,
        (unsigned long)d.publish_max_us,(unsigned long)d.cache_wait_max_us,(unsigned long)d.cache_hold_max_us);
    ESP_LOGI(TAG,"g e=%d,%d,%d l=%d,%d,%d",
        (int)d.gravity_mg[0],(int)d.gravity_mg[1],(int)d.gravity_mg[2],
        (int)d.linear_mg[0],(int)d.linear_mg[1],(int)d.linear_mg[2]);
    ESP_LOGI(TAG,"r w=%d,%d,%d i=%d,%d,%d",
        (int)d.gyro_mdps[0],(int)d.gyro_mdps[1],(int)d.gyro_mdps[2],
        (int)d.install_up_milli[0],(int)d.install_up_milli[1],(int)d.install_up_milli[2]);
    ESP_LOGI(TAG,"w t=%lld/%lld/%lld n=%u v=%u b=%u r=%u",
        (long long)d.captured_us,(long long)d.window_start_us,(long long)d.window_end_us,
        (unsigned)d.window_count,(unsigned)d.stable_mean_valid,
        (unsigned)d.learned_bias_valid,(unsigned)d.quiet_reject);
    ESP_LOGI(TAG,"m a=%d,%d,%d w=%d,%d,%d",
        (int)d.stable_acc_mean_mg[0],(int)d.stable_acc_mean_mg[1],(int)d.stable_acc_mean_mg[2],
        (int)d.stable_gyro_mean_mdps[0],(int)d.stable_gyro_mean_mdps[1],(int)d.stable_gyro_mean_mdps[2]);
    ESP_LOGI(TAG,"b w=%d,%d,%d",
        (int)d.learned_bias_mdps[0],(int)d.learned_bias_mdps[1],(int)d.learned_bias_mdps[2]);
}
esp_err_t gauge_imu_zero_capture(gauge_imu_frame_t *out)
{
    if(!s_lock)return ESP_ERR_INVALID_STATE;
    if(xSemaphoreTake(s_lock,0)!=pdTRUE)return ESP_ERR_TIMEOUT;
    esp_err_t err=ESP_ERR_INVALID_STATE;
    gauge_imu_zero_mount_capture_t captured=GAUGE_IMU_ZERO_MOUNT_NOT_REQUESTED;
    gauge_imu_mount_axis_t effective=GAUGE_IMU_MOUNT_AUTO;
    if(s_frame.valid&&s_frame.timestamp_us>0&&
       age_ms(esp_timer_get_time(),s_frame.timestamp_us)<=GAUGE_IMU_STALE_MS){
        /* Explicit ZERO also confirms the current installation angle, using
         * only already tracked trusted gravity. It does not certify quiet or
         * seed gravity/bias from current motion. The NEXT sample is absolute G. */
        bool finite=true;
        for(unsigned i=0;i<3;i++)finite=finite&&isfinite(s_frame.accel_g[i])&&
            isfinite(s_frame.raw_gyro_dps[i]);
        if(finite){
            /* Provisional display is allowed, but only a physical quiet
             * initialization establishes the final basis and enables history.
             * A user action cannot mix peaks from provisional/final axes. */
            captured=capture_installation_locked(s_frame.mount_axis);
            if(captured==GAUGE_IMU_ZERO_MOUNT_CAPTURED)project_locked();
            s_frame.zero_mount_capture=captured;effective=s_frame.effective_forward_axis;
            gm_peak_clear(&s_peak);s_frame.plane_g[0]=s_frame.plane_g[1]=0;
            if(out){*out=s_frame;memset(&out->peak,0,sizeof out->peak);out->age_ms=age_ms(esp_timer_get_time(),out->timestamp_us);}
            err=ESP_OK;
        }
    }
    xSemaphoreGive(s_lock);
    /* Once per explicit action, outside the cache lock; never per frame. */
    if(err==ESP_OK)ESP_LOGI(TAG,"manual-zero accepted capture=%u why=%u fwd=%u plane_mg=0,0",
        (unsigned)(captured==GAUGE_IMU_ZERO_MOUNT_CAPTURED),(unsigned)captured,(unsigned)effective);
    else ESP_LOGI(TAG,"manual-zero rejected: fresh finite sample required");
    return err;
}
esp_err_t gauge_imu_zero(void) {return gauge_imu_zero_capture(NULL);}

esp_err_t gauge_imu_align_at_rest(void)
{
    if(!s_lock)return ESP_ERR_INVALID_STATE;
    if(xSemaphoreTake(s_lock,0)!=pdTRUE)return ESP_ERR_TIMEOUT;
    esp_err_t err=ESP_ERR_INVALID_STATE;
    bool finite=s_frame.valid&&s_rest_window_ready&&s_mount_ready&&s_frame.timestamp_us>0&&
        age_ms(esp_timer_get_time(),s_frame.timestamp_us)<=GAUGE_IMU_STALE_MS;
    for(unsigned a=0;a<3;a++)finite=finite&&isfinite(s_frame.accel_g[a])&&
        isfinite(s_frame.raw_gyro_dps[a])&&isfinite(s_mean_acc[a])&&isfinite(s_mean_gyro[a]);
    float n=norm3(s_mean_acc);
    if(finite&&n>=.9f&&n<=1.1f){
        /* Explicit rest action resolves orientation lost in an unobserved
         * interval. The physical installation vectors NEVER follow this new
         * gravity direction; only the signed gravity/bias estimate realigns. */
        memcpy(s_frame.gravity_g,s_mean_acc,sizeof s_frame.gravity_g);s_gravity_norm=n;
        memcpy(s_baseline,s_mean_acc,sizeof s_baseline);memcpy(s_bias,s_mean_gyro,sizeof s_bias);
        s_frame.gravity_initialized=s_frame.gravity_quiet_validated=s_frame.gravity_tracking=s_frame.calibrated=true;
        for(unsigned a=0;a<3;a++)s_frame.gyro_dps[a]=s_frame.raw_gyro_dps[a]-s_bias[a];
        memcpy(s_last_gyro,s_frame.gyro_dps,sizeof s_last_gyro);s_attitude_last_us=s_frame.timestamp_us;
        s_frame.gravity_dt_us=0;gm_peak_clear(&s_peak);project_locked();err=ESP_OK;
    }
    xSemaphoreGive(s_lock);
    if(err==ESP_OK)ESP_LOGI(TAG,"rest-align accepted: gravity and gyro bias aligned, fixed mount preserved, history cleared");
    else ESP_LOGI(TAG,"rest-align rejected: fresh validated quiet sample and established mount required; state preserved");
    return err;
}

esp_err_t gauge_imu_set_mount_axis(gauge_imu_mount_axis_t axis)
{
    if(axis<GAUGE_IMU_MOUNT_AUTO||axis>GAUGE_IMU_MOUNT_Z)return ESP_ERR_INVALID_ARG;
    if(!s_lock)return ESP_ERR_INVALID_STATE;
    if(xSemaphoreTake(s_lock,0)!=pdTRUE)return ESP_ERR_TIMEOUT;
    esp_err_t err=ESP_OK;
    if(!s_frame.gravity_initialized){
        s_frame.mount_axis=axis;gm_peak_clear(&s_peak);project_locked();
    }else{
        /* A board may boot lying on a table, then be installed upright without
         * losing power. This EXPLICIT configuration action captures current
         * tracked up, even for the same selected axis; ordinary samples and
         * parking CAL never rotate the stored physical installation basis. */
        if(capture_installation_locked(axis)!=GAUGE_IMU_ZERO_MOUNT_CAPTURED)err=ESP_ERR_INVALID_STATE;
        else{gm_peak_clear(&s_peak);project_locked();}
    }
    xSemaphoreGive(s_lock);return err;
}
void gauge_imu_clear_peak(void)
{
    if(!s_lock)return;
    atomic_store_explicit(&s_clear_requested,true,memory_order_relaxed);
    if(xSemaphoreTake(s_lock,0)==pdTRUE){
        if(atomic_exchange_explicit(&s_clear_requested,false,memory_order_relaxed))gm_peak_clear(&s_peak);
        xSemaphoreGive(s_lock);
    }
}

esp_err_t gauge_imu_set_orientation(bool reverse_lr,bool reverse_fb)
{
    if(!s_lock)return ESP_ERR_INVALID_STATE;
    if(xSemaphoreTake(s_lock,0)!=pdTRUE)return ESP_ERR_TIMEOUT;
    if(s_frame.reverse_lr!=reverse_lr||s_frame.reverse_fb!=reverse_fb){
        s_frame.reverse_lr=reverse_lr;s_frame.reverse_fb=reverse_fb;
        gm_peak_clear(&s_peak);project_locked();
    }
    xSemaphoreGive(s_lock);return ESP_OK;
}
