#ifndef MMATH_H
#define MMATH_H

#include "vmath.h"

typedef struct
{
    double m1[2], m2[2];
} mat2;

mat2 mat2_init(const double m1[2], const double m2[2]);
mat2 mat2_add(const mat2 *mt1, const mat2 *mt2);
mat2 mat2_mul(const mat2 *mt1, const mat2 *mt2);
mat2 mat2_smul(const mat2 *mt, const double s);

double mat2_det(const mat2 *mt);
mat2 mat2_inv(const mat2* mt);

vec2 mat2_vmul(const mat2 *mt, const vec2 *v);

typedef struct
{
    double m1[3], m2[3], m3[3];
} mat3;

mat3 mat3_init(const double m1[3], const double m2[3], const double m3[3]);
mat3 mat3_add(const mat3 *mt1, const mat3 *mt2);
mat3 mat3_mul(const mat3 *mt1, const mat3 *mt2);
mat3 mat3_smul(const mat3 *mt, const double s);

double mat3_det(const mat3 *mt);

mat3 mat3_transp(const mat3* mt);
mat3 mat3_adj(const mat3* mt);
mat3 mat3_inv(const mat3* mt);

vec3 mat3_vmul(const mat3 *mt, const vec3 *v);

typedef struct
{
    double m1[4], m2[4], m3[4], m4[4];
} mat4;

mat4 mat4_init(const double m1[4], const double m2[4], const double m3[4], const double m4[4]);
mat4 mat4_add(const mat4 *mt1, const mat4 *mt2);
mat4 mat4_mul(const mat4 *mt1, const mat4 *mt2);
mat4 mat4_smul(const mat4 *mt, const double s);

// double mat4_det(const mat4 *mt); -> not needed right now and is more work

vec4 mat4_vmul(const mat4 *mt, const vec4 *v);

#endif