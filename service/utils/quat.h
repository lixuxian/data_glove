#ifndef _QUAT_H
#define _QUAT_H

#include <math.h>

#include "vec.h"
#include "mat.h"

// -----------------------------------------------------------------------
// -----------------------------------------------------------------------

template <typename TYPE>
mat<TYPE, 3, 3> quatToMatrix(const vec<TYPE, 4>& q) {
    mat<TYPE, 3, 3> R;
    TYPE q0(q.w);
    TYPE q1(q.x);
    TYPE q2(q.y);
    TYPE q3(q.z);
    TYPE sq_q1 = 2 * q1 * q1;
    TYPE sq_q2 = 2 * q2 * q2;
    TYPE sq_q3 = 2 * q3 * q3;
    TYPE q1_q2 = 2 * q1 * q2;
    TYPE q3_q0 = 2 * q3 * q0;
    TYPE q1_q3 = 2 * q1 * q3;
    TYPE q2_q0 = 2 * q2 * q0;
    TYPE q2_q3 = 2 * q2 * q3;
    TYPE q1_q0 = 2 * q1 * q0;
    R[0][0] = 1 - sq_q2 - sq_q3;
    R[0][1] = q1_q2 - q3_q0;
    R[0][2] = q1_q3 + q2_q0;
    R[1][0] = q1_q2 + q3_q0;
    R[1][1] = 1 - sq_q1 - sq_q3;
    R[1][2] = q2_q3 - q1_q0;
    R[2][0] = q1_q3 - q2_q0;
    R[2][1] = q2_q3 + q1_q0;
    R[2][2] = 1 - sq_q1 - sq_q2;
    return R;
}

template <typename TYPE>
vec<TYPE, 4> matrixToQuat(const mat<TYPE, 3, 3>& R) {
    // matrix to quaternion

    struct {
        inline TYPE operator()(TYPE v) {
            return v < 0 ? 0 : v;
        }
    } clamp;

    vec<TYPE, 4> q;
    const float Hx = R[0].x;
    const float My = R[1].y;
    const float Az = R[2].z;
    q.x = sqrtf( clamp( Hx - My - Az + 1) * 0.25f );
    q.y = sqrtf( clamp(-Hx + My - Az + 1) * 0.25f );
    q.z = sqrtf( clamp(-Hx - My + Az + 1) * 0.25f );
    q.w = sqrtf( clamp( Hx + My + Az + 1) * 0.25f );
    q.x = copysignf(q.x, R[2].y - R[1].z);
    q.y = copysignf(q.y, R[0].z - R[2].x);
    q.z = copysignf(q.z, R[1].x - R[0].y);
    // guaranteed to be unit-quaternion
    return q;
}

template <typename TYPE>
vec<TYPE, 4> normalize_quat(const vec<TYPE, 4>& q) {
    vec<TYPE, 4> r(q);
    if (r.w < 0) {
        r = -r;
    }
    return normalize(r);
}

// -----------------------------------------------------------------------

typedef vec4_t quat_t;

// -----------------------------------------------------------------------

#endif /* _QUAT_H */
