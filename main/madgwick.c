#include "madgwick.h"

#include <math.h>

static bool normalize4(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!isfinite(n) || n < 1.0e-9f) {
        return false;
    }
    for (int i = 0; i < 4; ++i) q[i] /= n;
    return true;
}

static bool normalize3(float v[3])
{
    float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!isfinite(n) || n < 1.0e-9f) return false;
    for (int i = 0; i < 3; ++i) v[i] /= n;
    return true;
}

void madgwick_init(madgwick_t *filter, float beta)
{
    filter->q[0] = 1.0f;
    filter->q[1] = filter->q[2] = filter->q[3] = 0.0f;
    filter->beta = beta;
}

void madgwick_seed(madgwick_t *filter, const float accel_in[3],
                   const float mag_in[3], bool use_mag)
{
    float a[3] = {accel_in[0], accel_in[1], accel_in[2]};
    if (!normalize3(a)) return;
    float roll = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    float yaw = 0.0f;
    if (use_mag) {
        float m[3] = {mag_in[0], mag_in[1], mag_in[2]};
        if (normalize3(m)) {
            float cr = cosf(roll), sr = sinf(roll);
            float cp = cosf(pitch), sp = sinf(pitch);
            float mxh = m[0] * cp + m[2] * sp;
            float myh = m[0] * sr * sp + m[1] * cr - m[2] * sr * cp;
            yaw = atan2f(-myh, mxh);
        }
    }
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    filter->q[0] = cr * cp * cy + sr * sp * sy;
    filter->q[1] = sr * cp * cy - cr * sp * sy;
    filter->q[2] = cr * sp * cy + sr * cp * sy;
    filter->q[3] = cr * cp * sy - sr * sp * cy;
    (void)normalize4(filter->q);
}

bool madgwick_update_gyro(madgwick_t *f, const float g[3], float dt)
{
    if (!(dt > 0.0f) || !isfinite(dt)) return false;
    float q0 = f->q[0], q1 = f->q[1], q2 = f->q[2], q3 = f->q[3];
    f->q[0] += 0.5f * (-q1 * g[0] - q2 * g[1] - q3 * g[2]) * dt;
    f->q[1] += 0.5f * ( q0 * g[0] + q2 * g[2] - q3 * g[1]) * dt;
    f->q[2] += 0.5f * ( q0 * g[1] - q1 * g[2] + q3 * g[0]) * dt;
    f->q[3] += 0.5f * ( q0 * g[2] + q1 * g[1] - q2 * g[0]) * dt;
    return normalize4(f->q);
}

static bool update_imu(madgwick_t *f, const float g[3], const float ain[3],
                       float dt)
{
    float a[3] = {ain[0], ain[1], ain[2]};
    if (!normalize3(a)) return madgwick_update_gyro(f, g, dt);
    float q0=f->q[0], q1=f->q[1], q2=f->q[2], q3=f->q[3];
    float s0 = 4*q0*q2*q2 + 2*q2*a[0] + 4*q0*q1*q1 - 2*q1*a[1];
    float s1 = 4*q1*q3*q3 - 2*q3*a[0] + 4*q0*q0*q1 - 2*q0*a[1]
             - 4*q1 + 8*q1*q1*q1 + 8*q1*q2*q2 + 4*q1*a[2];
    float s2 = 4*q0*q0*q2 + 2*q0*a[0] + 4*q2*q3*q3 - 2*q3*a[1]
             - 4*q2 + 8*q2*q1*q1 + 8*q2*q2*q2 + 4*q2*a[2];
    float s3 = 4*q1*q1*q3 - 2*q1*a[0] + 4*q2*q2*q3 - 2*q2*a[1];
    float sn = sqrtf(s0*s0+s1*s1+s2*s2+s3*s3);
    if (sn > 1.0e-9f && isfinite(sn)) { s0/=sn; s1/=sn; s2/=sn; s3/=sn; }
    else { s0=s1=s2=s3=0.0f; }
    float qd0=0.5f*(-q1*g[0]-q2*g[1]-q3*g[2])-f->beta*s0;
    float qd1=0.5f*( q0*g[0]+q2*g[2]-q3*g[1])-f->beta*s1;
    float qd2=0.5f*( q0*g[1]-q1*g[2]+q3*g[0])-f->beta*s2;
    float qd3=0.5f*( q0*g[2]+q1*g[1]-q2*g[0])-f->beta*s3;
    f->q[0]+=qd0*dt; f->q[1]+=qd1*dt; f->q[2]+=qd2*dt; f->q[3]+=qd3*dt;
    return normalize4(f->q);
}

bool madgwick_update(madgwick_t *f, const float g[3], const float ain[3],
                     const float min[3], bool use_mag, float dt)
{
    if (!use_mag) return update_imu(f, g, ain, dt);
    float a[3]={ain[0],ain[1],ain[2]}, m[3]={min[0],min[1],min[2]};
    if (!normalize3(a) || !normalize3(m)) return update_imu(f, g, ain, dt);
    float q0=f->q[0],q1=f->q[1],q2=f->q[2],q3=f->q[3];
    float _2q0mx=2*q0*m[0], _2q0my=2*q0*m[1], _2q0mz=2*q0*m[2];
    float _2q1mx=2*q1*m[0], _2q0=2*q0, _2q1=2*q1, _2q2=2*q2, _2q3=2*q3;
    float q0q0=q0*q0,q0q1=q0*q1,q0q2=q0*q2,q0q3=q0*q3,q1q1=q1*q1;
    float q1q2=q1*q2,q1q3=q1*q3,q2q2=q2*q2,q2q3=q2*q3,q3q3=q3*q3;
    float _2q0q2 = 2.0f * q0q2;
    float hx=m[0]*q0q0-_2q0my*q3+_2q0mz*q2+m[0]*q1q1+_2q1*m[1]*q2
             +_2q1*m[2]*q3-m[0]*q2q2-m[0]*q3q3;
    float hy=_2q0mx*q3+m[1]*q0q0-_2q0mz*q1+_2q1mx*q2-m[1]*q1q1
             +m[1]*q2q2+_2q2*m[2]*q3-m[1]*q3q3;
    float _2bx=sqrtf(hx*hx+hy*hy);
    float _2bz=-_2q0mx*q2+_2q0my*q1+m[2]*q0q0+_2q1mx*q3-m[2]*q1q1
              +_2q2*m[1]*q3-m[2]*q2q2+m[2]*q3q3;
    float _4bx=2*_2bx, _4bz=2*_2bz;
    float s0=-_2q2*(2*q1q3-_2q0q2-a[0])+_2q1*(2*q0q1+_2q2*q3-a[1])
      -_2bz*q2*(_2bx*(0.5f-q2q2-q3q3)+_2bz*(q1q3-q0q2)-m[0])
      +(-_2bx*q3+_2bz*q1)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-m[1])
      +_2bx*q2*(_2bx*(q0q2+q1q3)+_2bz*(0.5f-q1q1-q2q2)-m[2]);
    float s1=_2q3*(2*q1q3-_2q0q2-a[0])+_2q0*(2*q0q1+_2q2*q3-a[1])
      -4*q1*(1-2*q1q1-2*q2q2-a[2])
      +_2bz*q3*(_2bx*(0.5f-q2q2-q3q3)+_2bz*(q1q3-q0q2)-m[0])
      +(_2bx*q2+_2bz*q0)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-m[1])
      +(_2bx*q3-_4bz*q1)*(_2bx*(q0q2+q1q3)+_2bz*(0.5f-q1q1-q2q2)-m[2]);
    float s2=-_2q0*(2*q1q3-_2q0q2-a[0])+_2q3*(2*q0q1+_2q2*q3-a[1])
      -4*q2*(1-2*q1q1-2*q2q2-a[2])
      +(-_4bx*q2-_2bz*q0)*(_2bx*(0.5f-q2q2-q3q3)+_2bz*(q1q3-q0q2)-m[0])
      +(_2bx*q1+_2bz*q3)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-m[1])
      +(_2bx*q0-_4bz*q2)*(_2bx*(q0q2+q1q3)+_2bz*(0.5f-q1q1-q2q2)-m[2]);
    float s3=_2q1*(2*q1q3-_2q0q2-a[0])+_2q2*(2*q0q1+_2q2*q3-a[1])
      +(-_4bx*q3+_2bz*q1)*(_2bx*(0.5f-q2q2-q3q3)+_2bz*(q1q3-q0q2)-m[0])
      +(-_2bx*q0+_2bz*q2)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-m[1])
      +_2bx*q1*(_2bx*(q0q2+q1q3)+_2bz*(0.5f-q1q1-q2q2)-m[2]);
    float sn=sqrtf(s0*s0+s1*s1+s2*s2+s3*s3);
    if (sn>1e-9f && isfinite(sn)) {s0/=sn;s1/=sn;s2/=sn;s3/=sn;}
    else {s0=s1=s2=s3=0;}
    float qd0=.5f*(-q1*g[0]-q2*g[1]-q3*g[2])-f->beta*s0;
    float qd1=.5f*(q0*g[0]+q2*g[2]-q3*g[1])-f->beta*s1;
    float qd2=.5f*(q0*g[1]-q1*g[2]+q3*g[0])-f->beta*s2;
    float qd3=.5f*(q0*g[2]+q1*g[1]-q2*g[0])-f->beta*s3;
    f->q[0]+=qd0*dt;f->q[1]+=qd1*dt;f->q[2]+=qd2*dt;f->q[3]+=qd3*dt;
    return normalize4(f->q);
}
