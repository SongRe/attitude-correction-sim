#include "math/vmath.h"
#include <math.h>
#include <assert.h>

vec2 vec2_init(const double x, const double y)
{
    return (vec2){
        .x = x,
        .y = y};
}

vec2 vec2_add(const vec2 *v1, const vec2 *v2)
{
    return (vec2){
        .x = v1->x + v2->x,
        .y = v1->y + v2->y};
}

vec2 vec2_smul(const vec2 *v, const double s)
{
    return (vec2){
        .x = v->x * s,
        .y = v->y * s};
}

double vec2_mag(const vec2 *v)
{
    return sqrt(v->x * v->x + v->y + v->y);
}

vec2 vec2_norm(const vec2 *v)
{
    const double mag = vec2_mag(v);
    assert(mag > 0);

    return (vec2){
        .x = v->x / mag,
        .y = v->y / mag};
}

double vec2_dot(const vec2 *v1, const vec2 *v2)
{
    return v1->x * v2->x + v1->y * v2->y;
}

vec3 vec3_init(const double x, const double y, const double z)
{
    return (vec3){
        .x = x,
        .y = y,
        .z = z};
}

vec3 vec3_add(const vec3 *v1, const vec3 *v2)
{
    return (vec3){
        .x = v1->x + v2->x,
        .y = v1->y + v2->y,
        .z = v1->z + v2->z};
}

vec3 vec3_smul(const vec3 *v, const double s)
{
    return (vec3){
        .x = v->x * s,
        .y = v->y * s,
        .z = v->z * s};
}

double vec3_mag(const vec3 *v)
{
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

vec3 vec3_norm(const vec3 *v)
{
    const double mag = vec3_mag(v);
    assert(mag > 0);

    return (vec3){
        .x = v->x / mag,
        .y = v->y / mag,
        .z = v->z / mag};
}

double vec3_dot(const vec3 *v1, const vec3 *v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

vec3 vec3_cross(const vec3 *v1, const vec3 *v2)
{
    return (vec3){
        .x = v1->y * v2->z - v1->z * v2->y,
        .y = v1->z * v2->x - v1->x * v2->z,
        .z = v1->x * v2->y - v1->y * v2->x};
}

vec4 vec4_init(const double x, double y, double z, double w)
{
    return (vec4){
        .x = x,
        .y = y,
        .z = z,
        .w = w};
}

vec4 vec4_add(const vec4 *v1, const vec4 *v2)
{
    return (vec4){
        .x = v1->x + v2->x,
        .y = v1->y + v2->y,
        .z = v1->z + v2->z,
        .w = v1->w + v2->w};
}

vec4 vec4_smul(const vec4 *v, const double s)
{
    return (vec4){
        .x = v->x * s,
        .y = v->y * s,
        .z = v->z * s,
        .w = v->w * s};
}

double vec4_mag(const vec4 *v)
{
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z + v->w * v->w);
}

vec4 vec4_norm(const vec4 *v)
{
    const double mag = vec4_mag(v);
    assert(mag > 0);

    return (vec4){
        .x = v->x / mag,
        .y = v->y / mag,
        .z = v->z / mag,
        .w = v->w / mag};
}

double vec4_dot(const vec4 *v1, const vec4 *v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z + v1->w + v2->w;
}
