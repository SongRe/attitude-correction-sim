/**
 * @file vmath.h
 * @author Samuel Street (planetaryeclipse@gmail.com)
 * @brief Simple 3-space double-precision math library for control logic
 * @version 0.1
 * @date 2022-02-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef VMATH_H
#define VMATH_H

typedef struct Vec3
{
    double x, y, z;
} Vec3;

typedef struct Mat3
{
    double row_1[3];
    double row_2[3];
    double row_3[3];
} Mat3;

double Vec3_dot(Vec3 *v1, Vec3 *v2);
double Vec3_mag(Vec3 *v);

Vec3 Vec3_add(Vec3 *v1, Vec3 *v2);
Vec3 Vec3_scalar_prod(Vec3 *v, double s);
Vec3 Vec3_norm(Vec3 *v);
Vec3 Vec3_cross(Vec3 *v1, Vec3 *v2);

// do not need right now
//  Mat3 mat3_mul(Mat3 *m1, Mat3 *m2);

Vec3 mat3_vec3_mul(Mat3 *m, Vec3 *v);

#endif