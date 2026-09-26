#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "madgwick.h"

int main(void)
{
    madgwick_t filter;
    madgwick_init(&filter, 0.05f);
    float gyro[3] = {0, 0, (float)M_PI / 2.0f};
    for (int i = 0; i < 100; ++i)
        assert(madgwick_update_gyro(&filter, gyro, 0.01f));
    float norm = sqrtf(filter.q[0]*filter.q[0]+filter.q[1]*filter.q[1]
                     +filter.q[2]*filter.q[2]+filter.q[3]*filter.q[3]);
    assert(fabsf(norm - 1.0f) < 1e-5f);
    assert(fabsf(fabsf(filter.q[3]) - 0.7071f) < 0.01f);

    float zero[3] = {0,0,0};
    assert(madgwick_update(&filter, zero, zero, zero, true, 0.01f));
    assert(isfinite(filter.q[0]));

    float gravity[3] = {0,0,1}, north[3] = {1,0,0}, still[3] = {0,0,0};
    madgwick_init(&filter, 0.05f);
    for (int i = 0; i < 100; ++i)
        assert(madgwick_update(&filter, still, gravity, north, true, 0.01f));
    assert(isfinite(filter.q[0]));
    puts("tracker tests passed");
    return 0;
}
