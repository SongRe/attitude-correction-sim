#ifndef QMATH_H
#define QMATH_H

#include "vmath.h"

typedef struct
{
    double w;
    vec3 v;
} quat;

quat quat_init(const double w, const double x, const double y, const double z);
quat quat_from_axis(const vec3 *v, const double ang);

quat quat_add(const quat *q1, const quat *q2);
quat quat_mul(const quat *q1, const quat *q2);
quat quat_smul(const quat *q, const double s);

double quat_mag(const quat *q);
quat quat_norm(const quat *q);

double quat_dot(const quat* q1, const quat* q2);

quat quat_conj(const quat *q);
quat quat_inv(const quat *q);

vec3 quat_rot_vec(const quat *q, const vec3 *v);

vec4 quat_to_vec(const quat *q);
quat quat_from_vec(const vec4* v);

#endif