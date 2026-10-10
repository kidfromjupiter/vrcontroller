#include "calibration_math.h"

#include <float.h>
#include <math.h>
#include <string.h>
#include "tracker_config.h"

_Static_assert(sizeof(tracker_calibration_v1_t) == 68, "NVS v1 layout changed");
_Static_assert(sizeof(tracker_calibration_t) == 76, "NVS v2 layout changed");

static float norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static bool normalize(const float input[3], float output[3])
{
    float length = norm(input);
    if (!isfinite(length) || length < 1.0e-6f) return false;
    for (int i = 0; i < 3; ++i) output[i] = input[i] / length;
    return true;
}

void calibration_defaults(tracker_calibration_t *c)
{
    *c = (tracker_calibration_t){
        .magic = CALIBRATION_MAGIC, .version = CALIBRATION_VERSION, .size = sizeof(*c),
        .accel_scale = {1, 1, 1}, .mag_scale = {1, 1, 1},
        .imu_axis_map = TRACKER_IMU_AXIS_MAP, .mag_axis_map = TRACKER_MAG_AXIS_MAP,
    };
}

bool calibration_valid(const tracker_calibration_t *c)
{
    if (!c || c->magic != CALIBRATION_MAGIC || c->version != CALIBRATION_VERSION ||
        c->size != sizeof(*c) || c->reserved != 0 || (c->flags & ~CALIBRATION_HAS_MAPS) ||
        !tracker_axis_map_valid(c->imu_axis_map) || !tracker_axis_map_valid(c->mag_axis_map))
        return false;
    const float *arrays[] = {c->gyro_bias_dps, c->accel_bias_g, c->accel_scale,
                             c->mag_bias_ut, c->mag_scale};
    for (size_t a = 0; a < sizeof(arrays)/sizeof(arrays[0]); ++a)
        for (int i = 0; i < 3; ++i)
            if (!isfinite(arrays[a][i])) return false;
    for (int i = 0; i < 3; ++i)
        if (c->accel_scale[i] < 0.25f || c->accel_scale[i] > 4.0f ||
            c->mag_scale[i] < 0.1f || c->mag_scale[i] > 10.0f) return false;
    return true;
}

bool calibration_decode_blob(const void *data, size_t size, tracker_calibration_t *c)
{
    if (!data || !c) return false;
    tracker_calibration_t candidate;
    if (size == sizeof(candidate)) {
        memcpy(&candidate, data, size);
    } else if (size == sizeof(tracker_calibration_v1_t)) {
        tracker_calibration_v1_t legacy;
        memcpy(&legacy, data, size);
        if (legacy.magic != CALIBRATION_MAGIC || legacy.version != 1 || legacy.size != size)
            return false;
        calibration_defaults(&candidate);
        memcpy(candidate.gyro_bias_dps, legacy.gyro_bias_dps, sizeof(legacy.gyro_bias_dps));
        memcpy(candidate.accel_bias_g, legacy.accel_bias_g, sizeof(legacy.accel_bias_g));
        memcpy(candidate.accel_scale, legacy.accel_scale, sizeof(legacy.accel_scale));
        memcpy(candidate.mag_bias_ut, legacy.mag_bias_ut, sizeof(legacy.mag_bias_ut));
        memcpy(candidate.mag_scale, legacy.mag_scale, sizeof(legacy.mag_scale));
    } else return false;
    if (!calibration_valid(&candidate)) return false;
    *c = candidate;
    return true;
}

void calibration_correct_vector(const float input[3], const float bias[3],
                                const float scale[3], float output[3])
{
    for (int i = 0; i < 3; ++i) output[i] = (input[i] - bias[i]) * scale[i];
}

void calibration_map_vector(const float input[3], const int8_t map[3], float output[3])
{
    float mapped[3];
    for (int i = 0; i < 3; ++i) {
        int axis = map[i] < 0 ? -map[i] : map[i];
        mapped[i] = input[axis - 1] * (map[i] < 0 ? -1.0f : 1.0f);
    }
    memcpy(output, mapped, sizeof(mapped));
}

void calibration_corner_direction(unsigned corner, float direction[3])
{
    for (unsigned i = 0; i < 3; ++i)
        direction[i] = (corner & (1U << i)) ? -0.577350269f : 0.577350269f;
}

tracker_calibration_result_t calibration_fit_accelerometer(
    const float faces[6][3], tracker_calibration_t *c)
{
    tracker_calibration_t candidate = *c;
    for (int body = 0; body < 3; ++body) {
        float delta[3];
        int native = 0;
        for (int i = 0; i < 3; ++i) {
            if (!isfinite(faces[2*body][i]) || !isfinite(faces[2*body+1][i]))
                return TRACKER_CAL_RESULT_WRONG_FACE;
            delta[i] = faces[2*body][i] - faces[2*body+1][i];
            if (fabsf(delta[i]) > fabsf(delta[native])) native = i;
        }
        float length = norm(delta), span = fabsf(delta[native]);
        if (!isfinite(length) || span < 0.5f || span / length < TRACKER_CAL_FACE_DOMINANCE)
            return TRACKER_CAL_RESULT_WRONG_FACE;
        candidate.imu_axis_map[body] = (int8_t)((native + 1) * (delta[native] < 0 ? -1 : 1));
        candidate.accel_bias_g[native] = (faces[2*body][native] + faces[2*body+1][native]) * 0.5f;
        candidate.accel_scale[native] = 2.0f / span;
    }
    if (!tracker_axis_map_valid(candidate.imu_axis_map)) return TRACKER_CAL_RESULT_MAP_INVALID;
    for (int i = 0; i < 3; ++i)
        if (candidate.accel_scale[i] < 0.25f || candidate.accel_scale[i] > 4.0f)
            return TRACKER_CAL_RESULT_WRONG_FACE;
    for (int face = 0; face < 6; ++face) {
        float native[3], body[3];
        calibration_correct_vector(faces[face], candidate.accel_bias_g, candidate.accel_scale, native);
        calibration_map_vector(native, candidate.imu_axis_map, body);
        float length = norm(body), expected = face % 2 ? -1.0f : 1.0f;
        if (!isfinite(length) || length < 0.9f || length > 1.1f ||
            expected * body[face / 2] / length < TRACKER_CAL_POSE_COSINE)
            return TRACKER_CAL_RESULT_WRONG_FACE;
    }
    *c = candidate;
    return TRACKER_CAL_RESULT_OK;
}

tracker_calibration_result_t calibration_fit_mag_map(
    const float acceleration[][3], const float magnetic[][3], size_t count,
    int8_t map[3], float *best_score, float *runner_up_score)
{
    if (count < CALIBRATION_CORNER_COUNT) return TRACKER_CAL_RESULT_MAG_SAMPLES;
    for (size_t n = 0; n < count; ++n) {
        float a[3], m[3];
        if (!normalize(acceleration[n], a) || !normalize(magnetic[n], m))
            return TRACKER_CAL_RESULT_MAP_ALIGNMENT;
    }
    const int permutations[6][3] = {{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    float best = FLT_MAX, second = FLT_MAX;
    int8_t best_map[3] = {0};
    for (int p = 0; p < 6; ++p) {
        for (unsigned signs = 0; signs < 8; ++signs) {
            int8_t candidate[3];
            for (int i = 0; i < 3; ++i)
                candidate[i] = (int8_t)(permutations[p][i] * ((signs & (1U << i)) ? -1 : 1));
            if (!tracker_axis_map_valid(candidate)) continue;
            float mean = 0, m2 = 0;
            for (size_t n = 0; n < count; ++n) {
                float a[3], m[3], mapped[3];
                (void)normalize(acceleration[n], a);
                (void)normalize(magnetic[n], m);
                calibration_map_vector(m, candidate, mapped);
                float dot = a[0]*mapped[0] + a[1]*mapped[1] + a[2]*mapped[2];
                float delta = dot - mean;
                mean += delta / (float)(n + 1);
                m2 += delta * (dot - mean);
            }
            float score = sqrtf(fmaxf(0, m2 / (float)count));
            if (score < best) {
                second = best; best = score;
                memcpy(best_map, candidate, sizeof(best_map));
            } else if (score < second) second = score;
        }
    }
    if (best_score) *best_score = best;
    if (runner_up_score) *runner_up_score = second;
    if (best > TRACKER_CAL_MAG_MAP_MAX_SCORE) return TRACKER_CAL_RESULT_MAP_ALIGNMENT;
    if (second < fmaxf(best * TRACKER_CAL_MAG_MAP_RATIO, best + TRACKER_CAL_MAG_MAP_MARGIN))
        return TRACKER_CAL_RESULT_MAP_AMBIGUOUS;
    memcpy(map, best_map, sizeof(best_map));
    return TRACKER_CAL_RESULT_OK;
}
