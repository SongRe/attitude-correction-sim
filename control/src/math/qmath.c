#include "math/qmath.h"
#include "math/vmath.h"
#include <math.h>
#include <assert.h>

quat quat_init(const double w, const double x, const double y, const double z)
{
    return (quat){
        .w = w,
        .v = vec3_init(x, y, z)};
}

// note that the result of this operation will always be 1
quat quat_from_axis(const vec3 *v, const double ang)
{
    const vec3 v_norm = vec3_norm(v);
    const double imag_const = sin(0.5 * ang);

    return (quat){
        .w = cos(0.5 * ang),
        .v = vec3_smul(&v_norm, imag_const)};
}

quat quat_add(const quat *q1, const quat *q2)
{
    return (quat){
        .w = q1->w + q2->w,
        .v = vec3_add(&q1->v, &q2->v)};
}

quat quat_mul(const quat *q1, const quat *q2)
{
    const vec3 w1_v2 = vec3_smul(&q2->v, q1->w);
    const vec3 w2_v1 = vec3_smul(&q1->v, q2->w);
    const vec3 v1_v2_cross = vec3_cross(&q1->v, &q2->v);

    vec3 v_sum = vec3_add(&w1_v2, &w2_v1);
    v_sum = vec3_add(&v_sum, &v1_v2_cross);

    return (quat){
        .w = q1->w * q2->w - vec3_dot(&q1->v, &q2->v),
        .v = v_sum};
}

quat quat_smul(const quat *q, const double s)
{
    return (quat){
        .w = q->w * s,
        .v = vec3_smul(&q->v, s)};
}

double quat_mag(const quat *q)
{
    const double v_mag = vec3_mag(&q->v);
    return sqrt(q->w * q->w + v_mag * v_mag);
}

quat quat_norm(const quat *q)
{
    const double mag = quat_mag(q);
    assert(mag > 0);

    return (quat){
        .w = q->w / mag,
        .v = vec3_smul(&q->v, 1 / mag)};
}

double quat_dot(const quat *q1, const quat *q2)
{
    return q1->w * q2->w + vec3_dot(&q1->v, &q2->v);
}

quat quat_conj(const quat *q)
{
    return (quat){
        .w = q->w,
        .v = vec3_smul(&q->v, -1)};
}

quat quat_inv(const quat *q)
{
    const double q_mag = quat_mag(q);
    assert(q_mag > 0);

    const quat q_conj = quat_conj(q);
    return quat_smul(&q_conj, 1 / (q_mag * q_mag));
}

vec3 quat_rot_vec(const quat *q, const vec3 *v)
{
    const quat q_v = quat_init(0, v->x, v->y, v->z);
    const quat q_inv = quat_inv(q);

    const quat prod_rot_v = quat_mul(q, &q_v);
    const quat prod_rot_v_norm = quat_mul(&prod_rot_v, &q_inv);

    return prod_rot_v.v;
}

vec4 quat_to_vec(const quat *q)
{
    return (vec4){
        .x = q->v.x,
        .y = q->v.y,
        .z = q->v.z,
        .w = q->w};
}

quat quat_from_vec(const vec4 *v)
{
    return (quat){
        .w = v->w,
        .v = vec3_init(v->x, v->y, v->z)};
}