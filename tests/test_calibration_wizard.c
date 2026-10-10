#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "calibration.h"
#include "driver/gpio.h"
#include "nvs.h"

static tracker_calibration_t committed, pending;
static tracker_calibration_v1_t legacy;
static bool have_v2, have_v1, fail_commit, fail_set, overflow, moving, wrong_pose;
static bool approximate_corner, wrong_corner;
static bool face_motion, bad_face, cancel_retry, retry_waiting, sensor_failure;
static unsigned retries;
static tracker_calibration_stage_t retried_stage;
static int64_t now;
static tracker_calibration_stage_t current_stage, cancel_stage;
static int mag_sample_count, write_count, led;
static const int8_t imu_map[3] = {2,-1,3}, mag_map[3] = {-1,-2,3};
static const float accel_bias[3] = {0.01f,-0.03f,0.02f}, accel_scale[3] = {1.02f,0.99f,1.01f};
static const float gyro_bias[3] = {0.5f,-0.3f,0.2f}, mag_bias[3] = {8,-7,11};

static void held_corner(unsigned corner, float body[3])
{
    calibration_corner_direction(corner,body);
    if (approximate_corner) {
        /* About 29 degrees from the ideal diagonal, outside the old tolerance. */
        const float components[3]={0.9f,0.3082207f,0.3082207f};
        for (int i=0;i<3;++i) body[i]=(corner & (1U<<i)) ? -components[i]:components[i];
    }
    if (wrong_corner) body[0]=-body[0];
}

static void unmap(const float body[3], const int8_t map[3], float native[3])
{
    for (int i=0;i<3;++i) {
        int axis=map[i]<0 ? -map[i]:map[i];
        native[axis-1]=body[i]*(map[i]<0 ? -1:1);
    }
}

int64_t esp_timer_get_time(void) { return now; }
void vTaskDelay(uint32_t ticks) { now += (int64_t)ticks*1000; }
esp_err_t gpio_config(const gpio_config_t *config) { (void)config; return ESP_OK; }
esp_err_t gpio_set_level(int pin, int level) { (void)pin; led=level; return ESP_OK; }
esp_err_t nvs_flash_init(void) { return ESP_OK; }
esp_err_t nvs_flash_erase(void) { return ESP_OK; }
esp_err_t nvs_open(const char *name, int mode, nvs_handle_t *handle)
{ assert(strcmp(name,"sensor_cal")==0); (void)mode; *handle=1; return ESP_OK; }
void nvs_close(nvs_handle_t handle) { (void)handle; }
esp_err_t nvs_get_blob(nvs_handle_t handle, const char *key, void *data, size_t *size)
{
    (void)handle;
    const void *blob;
    size_t length;
    if (strcmp(key,"v2")==0 && have_v2) { blob=&committed; length=sizeof(committed); }
    else if (strcmp(key,"v1")==0 && have_v1) { blob=&legacy; length=sizeof(legacy); }
    else return ESP_FAIL;
    if (*size<length) { *size=length; return ESP_FAIL; }
    memcpy(data,blob,length); *size=length; return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t handle, const char *key, const void *data, size_t size)
{
    (void)handle;
    assert(strcmp(key,"v2")==0 && size==sizeof(pending));
    ++write_count;
    if (fail_set) return ESP_FAIL;
    memcpy(&pending,data,size); return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t handle)
{
    (void)handle;
    if (fail_commit) return ESP_FAIL;
    committed=pending; have_v2=true; return ESP_OK;
}

esp_err_t sensors_read_mpu(sensors_t *sensors, mpu6050_sample_t *sample)
{
    (void)sensors;
    if (sensor_failure && current_stage>=TRACKER_CAL_STAGE_CAPTURE_CORNER_0) return ESP_FAIL;
    float body[3]={0,0,1}, native[3];
    if (current_stage>=TRACKER_CAL_STAGE_CAPTURE_POS_X && current_stage<=TRACKER_CAL_STAGE_CAPTURE_NEG_Z) {
        int face=(current_stage-TRACKER_CAL_STAGE_CAPTURE_POS_X)/2;
        memset(body,0,sizeof(body)); body[face/2]=face%2 ? -1:1;
        if (bad_face && face==0) { body[0]=0; body[1]=1; }
    } else if (current_stage>=TRACKER_CAL_STAGE_CAPTURE_CORNER_0) {
        unsigned corner=(unsigned)(current_stage-TRACKER_CAL_STAGE_CAPTURE_CORNER_0)/2;
        held_corner(corner,body);
        if (wrong_pose) { body[0]=0;body[1]=0;body[2]=1; }
    }
    unmap(body,imu_map,native);
    for (int i=0;i<3;++i) {
        sample->accel_g[i]=native[i]/accel_scale[i]+accel_bias[i];
        sample->gyro_dps[i]=gyro_bias[i];
    }
    if (moving && current_stage>=TRACKER_CAL_STAGE_CAPTURE_CORNER_0) sample->gyro_dps[0]+=10;
    if (face_motion && current_stage==TRACKER_CAL_STAGE_CAPTURE_POS_Y) sample->gyro_dps[0]+=10;
    return ESP_OK;
}

esp_err_t sensors_read_mag(sensors_t *sensors, qmc5883p_sample_t *sample, bool *ready)
{
    (void)sensors;
    *ready=true; sample->overflow=overflow && current_stage>=TRACKER_CAL_STAGE_CAPTURE_CORNER_0;
    float body[3]={0}, native[3];
    if (current_stage==TRACKER_CAL_STAGE_CAPTURE_MAG) {
        int n=mag_sample_count++%6;
        body[n/2]=n%2 ? -50:50;
    } else {
        unsigned corner=(unsigned)(current_stage-TRACKER_CAL_STAGE_CAPTURE_CORNER_0)/2;
        float a[3], t[3], u[3];
        const float yaw[8]={0.2f,1.7f,-0.8f,2.8f,-2.2f,0.9f,-1.6f,2.3f};
        held_corner(corner,a);
        float length=sqrtf(a[0]*a[0]+a[1]*a[1]);
        t[0]=a[1]/length;t[1]=-a[0]/length;t[2]=0;
        u[0]=a[1]*t[2]-a[2]*t[1];u[1]=a[2]*t[0]-a[0]*t[2];u[2]=a[0]*t[1]-a[1]*t[0];
        for (int i=0;i<3;++i)
            body[i]=50*(0.4f*a[i]+sqrtf(0.84f)*(cosf(yaw[corner])*t[i]+sinf(yaw[corner])*u[i]));
    }
    unmap(body,mag_map,native);
    for (int i=0;i<3;++i) sample->magnetic_ut[i]=native[i]+mag_bias[i];
    return ESP_OK;
}

static void notify(void *context, tracker_calibration_stage_t stage,
    tracker_calibration_result_t result, uint8_t seconds, uint16_t count, bool can_finish)
{
    (void)context;(void)seconds;(void)count;(void)can_finish;
    current_stage=stage;
    retry_waiting=result==TRACKER_CAL_RESULT_MOTION || result==TRACKER_CAL_RESULT_WRONG_FACE ||
                  result==TRACKER_CAL_RESULT_MAP_INVALID;
    if (retry_waiting) {
        ++retries; retried_stage=stage;
        if (!cancel_retry) {
            moving=false;wrong_pose=false;wrong_corner=false;face_motion=false;bad_face=false;
        }
    }
}
static calibration_control_t control(void *context, tracker_calibration_stage_t stage, uint32_t timeout)
{
    (void)context;
    if (retry_waiting && cancel_retry) return CALIBRATION_CONTROL_CANCEL;
    if (stage==cancel_stage) return CALIBRATION_CONTROL_CANCEL;
    if (timeout!=0) return CALIBRATION_CONTROL_CONFIRM;
    if (stage==TRACKER_CAL_STAGE_CAPTURE_MAG && now>=10000000) return CALIBRATION_CONTROL_FINISH;
    return CALIBRATION_CONTROL_NONE;
}
static esp_err_t wizard(tracker_calibration_t *output, tracker_calibration_result_t *failure)
{
    sensors_t sensors={0};
    calibration_remote_io_t io={.notify=notify,.wait_control=control};
    now=0; current_stage=TRACKER_CAL_STAGE_IDLE; mag_sample_count=0;
    return calibration_run_wizard(&sensors,&io,output,failure);
}

int main(void)
{
    assert(calibration_storage_init()==ESP_OK);
    tracker_calibration_t output, loaded, previous;
    calibration_defaults(&output);
    previous=output;
    assert(!calibration_load(&loaded));
    legacy=(tracker_calibration_v1_t){.magic=CALIBRATION_MAGIC,.version=1,.size=sizeof(legacy),
        .accel_scale={1,1,1},.mag_scale={1,1,1},.gyro_bias_dps={1,2,3}};
    have_v1=true;
    assert(calibration_load(&loaded) && loaded.flags==0 && loaded.gyro_bias_dps[2]==3);
    assert(!have_v2 && write_count==0);
    tracker_calibration_result_t failure;
    assert(wizard(&output,&failure)==ESP_OK && failure==TRACKER_CAL_RESULT_OK);
    assert(output.flags==CALIBRATION_HAS_MAPS && memcmp(output.imu_axis_map,imu_map,3)==0);
    assert(memcmp(output.mag_axis_map,mag_map,3)==0 && led==0);
    assert(calibration_load(&loaded) && memcmp(&loaded,&output,sizeof(output))==0);
    approximate_corner=true;
    assert(wizard(&output,&failure)==ESP_OK && failure==TRACKER_CAL_RESULT_OK);
    assert(memcmp(output.mag_axis_map,mag_map,3)==0);
    approximate_corner=false;
    for (int i=0;i<3;++i) {
        assert(fabsf(output.accel_bias_g[i]-accel_bias[i])<1e-5f);
        assert(fabsf(output.gyro_bias_dps[i]-gyro_bias[i])<1e-5f);
        assert(fabsf(output.mag_bias_ut[i]-mag_bias[i])<1e-5f);
    }
    mpu6050_sample_t sample={.accel_g={0.01f,-0.03f+1/0.99f,0.02f},
                              .gyro_dps={0.5f,-0.3f+20,0.2f}};
    float a[3],g[3],body_a[3],body_g[3];
    calibration_apply_mpu(&output,&sample,a,g);
    calibration_map_vector(a,output.imu_axis_map,body_a);
    calibration_map_vector(g,output.imu_axis_map,body_g);
    assert(fabsf(body_a[0]-1)<1e-5f && fabsf(body_g[0]-20)<1e-5f);
    previous=committed;
    output=previous;
    cancel_stage=TRACKER_CAL_STAGE_WAIT_CORNER_3;
    int old_writes=write_count;
    assert(wizard(&output,&failure)!=ESP_OK && failure==TRACKER_CAL_RESULT_CANCELLED);
    assert(write_count==old_writes && memcmp(&output,&previous,sizeof(output))==0);
    cancel_stage=TRACKER_CAL_STAGE_IDLE;
    moving=true;
    unsigned old_retries=retries;
    assert(wizard(&output,&failure)==ESP_OK && failure==TRACKER_CAL_RESULT_OK);
    assert(retries==old_retries+1 && retried_stage==TRACKER_CAL_STAGE_WAIT_CORNER_0);
    moving=false; wrong_pose=true;
    assert(wizard(&output,&failure)==ESP_OK && failure==TRACKER_CAL_RESULT_OK);
    wrong_pose=false;
    wrong_corner=true;
    assert(wizard(&output,&failure)==ESP_OK && failure==TRACKER_CAL_RESULT_OK);
    wrong_corner=false;
    face_motion=true;
    assert(wizard(&output,&failure)==ESP_OK && failure==TRACKER_CAL_RESULT_OK);
    assert(retried_stage==TRACKER_CAL_STAGE_WAIT_POS_Y);
    bad_face=true;
    assert(wizard(&output,&failure)==ESP_OK && failure==TRACKER_CAL_RESULT_OK);
    assert(retried_stage==TRACKER_CAL_STAGE_WAIT_POS_X);
    old_writes=write_count;
    wrong_pose=true;cancel_retry=true;
    assert(wizard(&output,&failure)!=ESP_OK && failure==TRACKER_CAL_RESULT_CANCELLED);
    wrong_pose=false;cancel_retry=false;
    assert(write_count==old_writes && memcmp(&output,&previous,sizeof(output))==0);
    sensor_failure=true;
    assert(wizard(&output,&failure)!=ESP_OK && failure==TRACKER_CAL_RESULT_SENSOR);
    sensor_failure=false;
    /* Overflow during the alignment holds must time out without writing NVS. */
    overflow=true;
    assert(wizard(&output,&failure)!=ESP_OK && failure==TRACKER_CAL_RESULT_MAG_SAMPLES && led==0);
    overflow=false;
    fail_commit=true;
    assert(wizard(&output,&failure)!=ESP_OK && failure==TRACKER_CAL_RESULT_SAVE);
    assert(memcmp(&committed,&previous,sizeof(committed))==0 && memcmp(&output,&previous,sizeof(output))==0);
    fail_commit=false;fail_set=true;
    assert(wizard(&output,&failure)!=ESP_OK && failure==TRACKER_CAL_RESULT_SAVE);
    assert(memcmp(&committed,&previous,sizeof(committed))==0);
    fail_set=false;
    committed.reserved=1;
    assert(calibration_load(&loaded) && loaded.flags==0 && loaded.gyro_bias_dps[2]==3);
    have_v1=false;
    assert(!calibration_load(&loaded));
    puts("calibration wizard/storage tests passed");
}
