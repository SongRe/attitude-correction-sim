/**
 * @file vmath.c
 * @author Samuel Street (planetaryeclipse@gmail.com)
 * @brief Simple 3-space double-precision math library for control logic
 * @version 0.1
 * @date 2022-02-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "vmath.h"
#include "math.h"

double Vec3_dot(Vec3 *v1, Vec3 *v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

double Vec3_mag(Vec3 *v)
{
    return sqrt(
        v->x * v->x +
        v->y * v->y +
        v->z * v->z);
}

Vec3 Vec3_add(Vec3 *v1, Vec3 *v2)
{
    return (Vec3){
        v1->x + v2->x,
        v1->y + v2->y,
        v1->z + v2->z
    };
}

Vec3 Vec3_scalar_prod(Vec3 *v, double s)
{
    return (Vec3){.x = v->x * s, .y = v->y * s, .z = v->z * s};
}

Vec3 Vec3_norm(Vec3 *v)
{
    double mag = Vec3_mag(v);
    if(mag == 0)
        return *v;
    else
        return Vec3_scalar_prod(v, 1 / Vec3_mag(v));
}

Vec3 Vec3_cross(Vec3 *v1, Vec3 *v2)
{
    return (Vec3){
        {v1->y * v2->z - v1->z * v2->y},
        {v1->z * v2->x - v1->x * v2->z},
        {v1->x * v2->y - v1->y * v2->x}};
}

// do not need right now
//  Mat3 mat3_mul(Mat3 *m1, Mat3 *m2);

Vec3 mat3_vec3_mul(Mat3 *m, Vec3 *v)
{
    return (Vec3){
        {m->row_1[0] * v->x + m->row_1[1] * v->y + m->row_1[2] * v->z},
        {m->row_2[0] * v->x + m->row_2[1] * v->y + m->row_2[2] * v->z},
        {m->row_3[0] * v->x + m->row_3[1] * v->y + m->row_3[2] * v->z}};
}