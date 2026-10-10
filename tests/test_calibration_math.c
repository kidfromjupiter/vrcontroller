#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "calibration_math.h"

static float length(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1]*b[2]-a[2]*b[1];
    out[1] = a[2]*b[0]-a[0]*b[2];
    out[2] = a[0]*b[1]-a[1]*b[0];
}

static void unmap(const float body[3], const int8_t map[3], float native[3])
{
    for (int i = 0; i < 3; ++i) {
        int axis = map[i] < 0 ? -map[i] : map[i];
        native[axis-1] = body[i] * (map[i] < 0 ? -1 : 1);
    }
}

static void magnetic_observations(float a[8][3], float m[8][3], float dip)
{
    const float yaw[8] = {0.2f, 1.7f, -0.8f, 2.8f, -2.2f, 0.9f, -1.6f, 2.3f};
    const float z[3] = {0,0,1};
    for (unsigned n = 0; n < 8; ++n) {
        float tangent[3], perpendicular[3];
        calibration_corner_direction(n, a[n]);
        cross(a[n], z, tangent);
        float norm = length(tangent);
        for (int i = 0; i < 3; ++i) tangent[i] /= norm;
        cross(a[n], tangent, perpendicular);
        for (int i = 0; i < 3; ++i)
            m[n][i] = 50 * (dip*a[n][i] + sqrtf(1-dip*dip) *
                (cosf(yaw[n])*tangent[i] + sinf(yaw[n])*perpendicular[i]));
    }
}

static void test_mountings(void)
{
    const int perm[6][3] = {{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    const float bias[3] = {0.02f,-0.03f,0.05f}, scale[3] = {0.9f,1.1f,1.05f};
    const float mag_bias[3] = {6,-8,11}, mag_scale[3] = {0.8f,1.1f,1.2f};
    int tested = 0;
    for (int p = 0; p < 6; ++p) for (unsigned signs = 0; signs < 8; ++signs) {
        int8_t map[3];
        for (int i = 0; i < 3; ++i) map[i] = (int8_t)(perm[p][i] * (signs & (1U << i) ? -1 : 1));
        if (!tracker_axis_map_valid(map)) continue;
        ++tested;
        tracker_calibration_t c;
        calibration_defaults(&c);
        float faces[6][3];
        for (int face = 0; face < 6; ++face) {
            float body[3] = {0}, native[3];
            body[face/2] = face%2 ? -1 : 1;
            unmap(body, map, native);
            for (int i = 0; i < 3; ++i) faces[face][i] = native[i] / scale[i] + bias[i];
        }
        assert(calibration_fit_accelerometer(faces, &c) == TRACKER_CAL_RESULT_OK);
        assert(memcmp(map, c.imu_axis_map, 3) == 0);
        for (int i = 0; i < 3; ++i) {
            assert(fabsf(c.accel_bias_g[i]-bias[i]) < 1e-6f);
            assert(fabsf(c.accel_scale[i]-scale[i]) < 1e-6f);
        }
        for (int dip_index = 0; dip_index < 3; ++dip_index) {
            float body_a[8][3], body_m[8][3], native_m[8][3];
            magnetic_observations(body_a, body_m, dip_index * 0.4f);
            for (int n = 0; n < 8; ++n) {
                float native[3], raw[3];
                unmap(body_m[n], map, native);
                for (int i = 0; i < 3; ++i) raw[i] = native[i]/mag_scale[i] + mag_bias[i];
                calibration_correct_vector(raw, mag_bias, mag_scale, native_m[n]);
                assert(fabsf(length(native_m[n])-50) < 1e-4f);
            }
            int8_t fitted[3] = {0};
            float best, second;
            assert(calibration_fit_mag_map(body_a, native_m, 8, fitted, &best, &second) == TRACKER_CAL_RESULT_OK);
            assert(memcmp(map, fitted, 3) == 0 && best < 1e-6f && second > 0.03f);
            for (int n = 0; n < 8; ++n) {
                float corrected_body[3];
                calibration_map_vector(native_m[n], fitted, corrected_body);
                for (int i = 0; i < 3; ++i) assert(fabsf(corrected_body[i]-body_m[n][i]) < 1e-4f);
            }
        }
    }
    assert(tested == 24);
}

static void test_rejections(void)
{
    tracker_calibration_t c, before;
    calibration_defaults(&c); before=c;
    float faces[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    faces[2][0] = 1; faces[2][1] = 0; faces[3][0] = -1; faces[3][1] = 0;
    assert(calibration_fit_accelerometer(faces, &c) == TRACKER_CAL_RESULT_MAP_INVALID);
    assert(memcmp(&before,&c,sizeof(c)) == 0);
    float reflection[6][3] = {{-1,0,0},{1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    assert(calibration_fit_accelerometer(reflection, &c) == TRACKER_CAL_RESULT_MAP_INVALID);
    reflection[0][0] = NAN;
    assert(calibration_fit_accelerometer(reflection, &c) == TRACKER_CAL_RESULT_WRONG_FACE);
    float tilted[6][3] = {{0.8f,0.6f,0},{-0.8f,-0.6f,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    assert(calibration_fit_accelerometer(tilted, &c) == TRACKER_CAL_RESULT_WRONG_FACE);
    float a[8][3], m[8][3]; int8_t map[3] = {1,2,3};
    magnetic_observations(a,m,0.4f);
    assert(calibration_fit_mag_map(a,m,7,map,NULL,NULL) == TRACKER_CAL_RESULT_MAG_SAMPLES);
    m[0][0] = NAN;
    assert(calibration_fit_mag_map(a,m,8,map,NULL,NULL) == TRACKER_CAL_RESULT_MAP_ALIGNMENT);
    for (int n = 0; n < 8; ++n) { a[n][0]=0; a[n][1]=0; a[n][2]=1; m[n][0]=30; m[n][1]=0; m[n][2]=40; }
    assert(calibration_fit_mag_map(a,m,8,map,NULL,NULL) == TRACKER_CAL_RESULT_MAP_AMBIGUOUS);
    magnetic_observations(a,m,0.4f);
    for (int n = 0; n < 8; ++n) {
        m[n][0] = 50*cosf(n*1.3f); m[n][1] = 50*sinf(n*0.7f); m[n][2] = 50*cosf(n*2.2f);
    }
    assert(calibration_fit_mag_map(a,m,8,map,NULL,NULL) == TRACKER_CAL_RESULT_MAP_ALIGNMENT);
}

static void test_storage_layout(void)
{
    tracker_calibration_v1_t old = {.magic=CALIBRATION_MAGIC,.version=1,.size=sizeof(old),
        .gyro_bias_dps={1,2,3},.accel_bias_g={0.01f,0.02f,0.03f},.accel_scale={1,1.1f,0.9f},
        .mag_bias_ut={10,20,30},.mag_scale={1,1,1}};
    tracker_calibration_t c;
    assert(calibration_decode_blob(&old,sizeof(old),&c));
    assert(c.flags == 0 && c.version == 2 && c.gyro_bias_dps[2] == 3 && c.mag_bias_ut[1] == 20);
    c.flags=CALIBRATION_HAS_MAPS;
    c.imu_axis_map[0]=2; c.imu_axis_map[1]=-1; c.imu_axis_map[2]=3;
    tracker_calibration_t loaded, before;
    calibration_defaults(&loaded); before=loaded;
    assert(calibration_decode_blob(&c,sizeof(c),&loaded));
    assert(memcmp(&loaded,&c,sizeof(c)) == 0);
    assert(!calibration_decode_blob(&c,sizeof(c)-1,&loaded));
    c.mag_scale[1]=NAN;
    assert(!calibration_decode_blob(&c,sizeof(c),&loaded));
    old.accel_scale[0]=0;
    assert(!calibration_decode_blob(&old,sizeof(old),&loaded));
    c=before; c.imu_axis_map[0]=0; assert(!calibration_valid(&c));
    c=before; c.reserved=1; assert(!calibration_valid(&c));
    c=before; c.flags=0x80; assert(!calibration_valid(&c));
    c=before; c.version=1; assert(!calibration_valid(&c));
}

int main(void)
{
    test_mountings(); test_rejections(); test_storage_layout();
    puts("calibration math tests passed (24 maps, three magnetic inclinations)");
}
