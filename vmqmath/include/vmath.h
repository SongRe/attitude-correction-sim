#ifndef VMATH_H
#define VMATH_H

typedef struct
{
    double x, y;
} vec2;

vec2 vec2_init(const double x, const double y);
vec2 vec2_add(const vec2 *v1, const vec2 *v2);
vec2 vec2_smul(const vec2 *v, const double s);

double vec2_mag(const vec2 *v);
vec2 vec2_norm(const vec2 *v);

double vec2_dot(const vec2 *v1, const vec2 *v2);

typedef struct
{
    double x, y, z;
} vec3;

vec3 vec3_init(const double x, const double y, const double z);
vec3 vec3_add(const vec3 *v1, const vec3 *v2);
vec3 vec3_smul(const vec3 *v, const double s);

double vec3_mag(const vec3 *v);
vec3 vec3_norm(const vec3 *v);

double vec3_dot(const vec3 *v1, const vec3 *v2);
vec3 vec3_cross(const vec3 *v1, const vec3 *v2);

typedef struct
{
    double x, y, z, w;
} vec4;

vec4 vec4_init(const double x, double y, double z, double w);
vec4 vec4_add(const vec4 *v1, const vec4 *v2);
vec4 vec4_smul(const vec4 *v, const double s);

double vec4_mag(const vec4 *v);
vec4 vec4_norm(const vec4 *v);

double vec4_dot(const vec4 *v1, const vec4 *v2);

#endif