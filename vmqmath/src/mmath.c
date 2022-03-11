#include "mmath.h"
#include "vmath.h"
#include <assert.h>

mat2 mat2_init(const double m1[2], const double m2[2])
{
    return (mat2){
        .m1 = {m1[0], m1[1]},
        .m2 = {m2[0], m2[1]}};
}

mat2 mat2_add(const mat2 *mt1, const mat2 *mt2)
{
    return (mat2){
        .m1 = {mt1->m1[0] + mt2->m1[0], mt1->m1[1] + mt2->m1[1]},
        .m2 = {mt1->m2[0] + mt2->m2[0], mt1->m2[1] + mt2->m2[1]}};
}

mat2 mat2_mul(const mat2 *mt1, const mat2 *mt2)
{
    return (mat2){
        .m1 = {mt1->m1[0] * mt2->m1[0] + mt1->m1[1] * mt2->m2[0],
               mt1->m1[0] * mt2->m1[1] + mt1->m1[1] * mt2->m2[1]},
        .m2 = {mt1->m2[0] * mt2->m1[0] + mt1->m2[1] * mt2->m2[0],
               mt1->m2[0] * mt2->m1[1] + mt1->m2[1] * mt2->m2[1]}};
}

mat2 mat2_smul(const mat2 *mt, const double s)
{
    return (mat2){
        .m1 = {mt->m1[0] * s, mt->m1[1] * s},
        .m2 = {mt->m2[0] * s, mt->m2[1] * s}};
}

double mat2_det(const mat2 *mt)
{
    return mt->m1[0] * mt->m2[1] - mt->m2[0] * mt->m1[1];
}

mat2 mat2_inv(const mat2 *mt)
{
    const double det = mat2_det(mt);
    assert(det != 0);

    const mat2 adjoint = (mat2){
        .m1 = {mt->m2[1], -mt->m1[1]},
        .m2 = {-mt->m2[0], mt->m1[0]}};

    return mat2_smul(&adjoint, 1 / det);
}

vec2 mat2_vmul(const mat2 *mt, const vec2 *v)
{
    return (vec2){
        .x = mt->m1[0] * v->x + mt->m1[1] * v->y,
        .y = mt->m2[0] * v->x + mt->m2[1] * v->y};
}

mat3 mat3_init(const double m1[3], const double m2[3], const double m3[3])
{
    return (mat3){
        .m1 = {m1[0], m1[1], m1[2]},
        .m2 = {m2[0], m2[1], m2[2]},
        .m3 = {m3[0], m3[1], m3[2]}};
}

mat3 mat3_add(const mat3 *mt1, const mat3 *mt2)
{
    return (mat3){
        .m1 = {mt1->m1[0] + mt2->m1[0], mt1->m1[1] + mt2->m1[1], mt1->m1[2] + mt2->m1[2]},
        .m2 = {mt1->m2[0] + mt2->m2[0], mt1->m2[1] + mt2->m2[1], mt1->m2[2] + mt2->m2[2]},
        .m3 = {mt1->m3[0] + mt2->m3[0], mt1->m3[1] + mt2->m3[1], mt1->m3[2] + mt2->m3[2]}};
}

mat3 mat3_mul(const mat3 *mt1, const mat3 *mt2)
{
    return (mat3){
        .m1 = {mt1->m1[0] * mt2->m1[0] + mt1->m1[1] * mt2->m2[0] + mt1->m1[2] * mt2->m3[0],
               mt1->m1[0] * mt2->m1[1] + mt1->m1[1] * mt2->m2[1] + mt1->m1[2] * mt2->m3[1],
               mt1->m1[0] * mt2->m1[2] + mt1->m1[1] * mt2->m2[2] + mt1->m1[2] * mt2->m3[2]},
        .m2 = {mt1->m2[0] * mt2->m1[0] + mt1->m2[1] * mt2->m2[0] + mt1->m2[2] * mt2->m3[0],
               mt1->m2[0] * mt2->m1[1] + mt1->m2[1] * mt2->m2[1] + mt1->m2[2] * mt2->m3[1],
               mt1->m2[0] * mt2->m1[2] + mt1->m2[1] * mt2->m2[2] + mt1->m2[2] * mt2->m3[2]},
        .m3 = {mt1->m3[0] * mt2->m1[0] + mt1->m3[1] * mt2->m2[0] + mt1->m3[2] * mt2->m3[0],
               mt1->m3[0] * mt2->m1[1] + mt1->m3[1] * mt2->m2[1] + mt1->m3[2] * mt2->m3[1],
               mt1->m3[0] * mt2->m1[2] + mt1->m3[1] * mt2->m2[2] + mt1->m3[2] * mt2->m3[2]}};
}

mat3 mat3_smul(const mat3 *mt, const double s)
{
    return (mat3){
        .m1 = {mt->m1[0] * s, mt->m1[1] * s, mt->m1[2] * s},
        .m2 = {mt->m2[0] * s, mt->m2[1] * s, mt->m2[2] * s},
        .m3 = {mt->m3[0] * s, mt->m3[1] * s, mt->m3[2] * s}};
}

double mat3_det(const mat3 *mt)
{
    const mat2 sub_mt_lright = (mat2){
        .m1 = {mt->m2[1], mt->m2[2]},
        .m2 = {mt->m3[1], mt->m3[2]}};

    const mat2 sub_mt_lcenter = (mat2){
        .m1 = {mt->m2[0], mt->m2[2]},
        .m2 = {mt->m3[0], mt->m3[2]}};

    const mat2 sub_mt_lleft = (mat2){
        .m1 = {mt->m2[0], mt->m2[1]},
        .m2 = {mt->m3[0], mt->m3[1]}};

    return mt->m1[0] * mat2_det(&sub_mt_lright) - mt->m1[1] * mat2_det(&sub_mt_lcenter) + mt->m1[2] * mat2_det(&sub_mt_lleft);
}

mat3 mat3_transp(const mat3 *mt)
{
    return (mat3){
        .m1 = {mt->m1[0], mt->m2[0], mt->m3[0]},
        .m2 = {mt->m1[1], mt->m2[1], mt->m3[1]},
        .m3 = {mt->m1[2], mt->m2[2], mt->m3[2]}};
}

mat3 mat3_adj(const mat3 *mt)
{
    const mat2 cofctr_mt_1_1 = (mat2){
        .m1 = {mt->m2[1], mt->m2[2]},
        .m2 = {mt->m3[1], mt->m3[2]}};
    const mat2 cofctr_mt_1_2 = (mat2){
        .m1 = {mt->m2[0], mt->m2[2]},
        .m2 = {mt->m3[0], mt->m3[2]}};
    const mat2 cofctr_mt_1_3 = (mat2){
        .m1 = {mt->m2[0], mt->m2[1]},
        .m2 = {mt->m3[0], mt->m3[1]}};

    const mat2 cofctr_mt_2_1 = (mat2){
        .m1 = {mt->m1[1], mt->m1[2]},
        .m2 = {mt->m3[1], mt->m3[2]}};
    const mat2 cofctr_mt_2_2 = (mat2){
        .m1 = {mt->m1[0], mt->m1[2]},
        .m2 = {mt->m3[0], mt->m3[2]}};
    const mat2 cofctr_mt_2_3 = (mat2){
        .m1 = {mt->m1[0], mt->m1[1]},
        .m2 = {mt->m3[0], mt->m3[1]}};

    const mat2 cofctr_mt_3_1 = (mat2){
        .m1 = {mt->m1[1], mt->m1[2]},
        .m2 = {mt->m2[1], mt->m2[2]}};
    const mat2 cofctr_mt_3_2 = (mat2){
        .m1 = {mt->m1[0], mt->m1[2]},
        .m2 = {mt->m2[0], mt->m2[2]}};
    const mat2 cofctr_mt_3_3 = (mat2){
        .m1 = {mt->m1[0], mt->m1[1]},
        .m2 = {mt->m2[0], mt->m2[1]}};

    const mat3 cofctr_mt = (mat3){
        .m1 = {mat2_det(&cofctr_mt_1_1), -mat2_det(&cofctr_mt_1_2), mat2_det(&cofctr_mt_1_3)},
        .m2 = {-mat2_det(&cofctr_mt_2_1), mat2_det(&cofctr_mt_2_2), -mat2_det(&cofctr_mt_2_3)},
        .m3 = {mat2_det(&cofctr_mt_3_1), -mat2_det(&cofctr_mt_3_2), mat2_det(&cofctr_mt_3_3)}};

    return mat3_transp(&cofctr_mt);
}

mat3 mat3_inv(const mat3 *mt)
{
    const double det = mat3_det(mt);
    const mat3 adj = mat3_adj(mt);

    assert(det != 0);

    return mat3_smul(&adj, 1 / det);
}

vec3 mat3_vmul(const mat3 *mt, const vec3 *v)
{
    return (vec3){
        .x = mt->m1[0] * v->x + mt->m1[1] * v->y + mt->m1[2] * v->z,
        .y = mt->m2[0] * v->x + mt->m2[1] * v->y + mt->m2[2] * v->z,
        .z = mt->m3[0] * v->x + mt->m3[1] * v->y + mt->m3[2] * v->z};
}

mat4 mat4_init(const double m1[4], const double m2[4], const double m3[4], const double m4[4])
{
    return (mat4){
        .m1 = {m1[0], m1[1], m1[2], m1[3]},
        .m2 = {m2[0], m2[1], m2[2], m2[3]},
        .m3 = {m3[0], m3[1], m3[2], m3[3]},
        .m4 = {m4[0], m4[1], m4[2], m4[3]}};
}
mat4 mat4_add(const mat4 *mt1, const mat4 *mt2)
{
    return (mat4){
        .m1 = {mt1->m1[0] + mt2->m1[0], mt1->m1[1] + mt2->m1[1], mt1->m1[2] + mt2->m1[2], mt1->m1[3] + mt2->m1[3]},
        .m2 = {mt1->m2[0] + mt2->m2[0], mt1->m2[1] + mt2->m2[1], mt1->m2[2] + mt2->m2[2], mt1->m2[3] + mt2->m2[3]},
        .m3 = {mt1->m3[0] + mt2->m3[0], mt1->m3[1] + mt2->m3[1], mt1->m3[2] + mt2->m3[2], mt1->m3[3] + mt2->m3[3]},
        .m4 = {mt1->m4[0] + mt2->m4[0], mt1->m4[1] + mt2->m4[1], mt1->m4[2] + mt2->m4[2], mt1->m4[3] + mt2->m4[3]}};
}
mat4 mat4_mul(const mat4 *mt1, const mat4 *mt2)
{
    return (mat4){
        .m1 = {mt1->m1[0] * mt2->m1[0] + mt1->m1[1] * mt2->m2[0] + mt1->m1[2] * mt2->m3[0] + mt1->m1[3] * mt2->m4[0],
               mt1->m1[0] * mt2->m1[1] + mt1->m1[1] * mt2->m2[1] + mt1->m1[2] * mt2->m3[1] + mt1->m1[3] * mt2->m4[1],
               mt1->m1[0] * mt2->m1[2] + mt1->m1[1] * mt2->m2[2] + mt1->m1[2] * mt2->m3[2] + mt1->m1[3] * mt2->m4[2],
               mt1->m1[0] * mt2->m1[3] + mt1->m1[1] * mt2->m2[3] + mt1->m1[2] * mt2->m3[3] + mt1->m1[3] * mt2->m4[3]},
        .m2 = {mt1->m2[0] * mt2->m1[0] + mt1->m2[1] * mt2->m2[0] + mt1->m2[2] * mt2->m3[0] + mt1->m2[3] * mt2->m4[0],
               mt1->m2[0] * mt2->m1[1] + mt1->m2[1] * mt2->m2[1] + mt1->m2[2] * mt2->m3[1] + mt1->m2[3] * mt2->m4[1],
               mt1->m2[0] * mt2->m1[2] + mt1->m2[1] * mt2->m2[2] + mt1->m2[2] * mt2->m3[2] + mt1->m2[3] * mt2->m4[2],
               mt1->m2[0] * mt2->m1[3] + mt1->m2[1] * mt2->m2[3] + mt1->m2[2] * mt2->m3[3] + mt1->m2[3] * mt2->m4[3]},
        .m3 = {mt1->m3[0] * mt2->m1[0] + mt1->m3[1] * mt2->m2[0] + mt1->m3[2] * mt2->m3[0] + mt1->m3[3] * mt2->m4[0],
               mt1->m3[0] * mt2->m1[1] + mt1->m3[1] * mt2->m2[1] + mt1->m3[2] * mt2->m3[1] + mt1->m3[3] * mt2->m4[1],
               mt1->m3[0] * mt2->m1[2] + mt1->m3[1] * mt2->m2[2] + mt1->m3[2] * mt2->m3[2] + mt1->m3[3] * mt2->m4[2],
               mt1->m3[0] * mt2->m1[3] + mt1->m3[1] * mt2->m2[3] + mt1->m3[2] * mt2->m3[3] + mt1->m3[3] * mt2->m4[3]},
        .m4 = {mt1->m4[0] * mt2->m1[0] + mt1->m4[1] * mt2->m2[0] + mt1->m4[2] * mt2->m3[0] + mt1->m4[3] * mt2->m4[0],
               mt1->m4[0] * mt2->m1[1] + mt1->m4[1] * mt2->m2[1] + mt1->m4[2] * mt2->m3[1] + mt1->m4[3] * mt2->m4[1],
               mt1->m4[0] * mt2->m1[2] + mt1->m4[1] * mt2->m2[2] + mt1->m4[2] * mt2->m3[2] + mt1->m4[3] * mt2->m4[2],
               mt1->m4[0] * mt2->m1[3] + mt1->m4[1] * mt2->m2[3] + mt1->m4[2] * mt2->m3[3] + mt1->m4[3] * mt2->m4[3]}};
}

mat4 mat4_smul(const mat4 *mt, const double s)
{
    return (mat4){
        .m1 = {mt->m1[0] * s, mt->m1[1] * s, mt->m1[2] * s, mt->m1[3] * s},
        .m2 = {mt->m2[0] * s, mt->m2[1] * s, mt->m2[2] * s, mt->m2[3] * s},
        .m3 = {mt->m3[0] * s, mt->m3[1] * s, mt->m3[2] * s, mt->m3[3] * s},
        .m4 = {mt->m4[0] * s, mt->m4[1] * s, mt->m4[2] * s, mt->m4[3] * s}};
}

vec4 mat4_vmul(const mat4 *mt, const vec4 *v)
{
    return (vec4){
        .x = mt->m1[0] * v->x + mt->m1[1] * v->y + mt->m1[2] * v->z + mt->m1[3] * v->w,
        .y = mt->m2[0] * v->x + mt->m2[1] * v->y + mt->m2[2] * v->z + mt->m2[3] * v->w,
        .z = mt->m3[0] * v->x + mt->m3[1] * v->y + mt->m3[2] * v->z + mt->m3[3] * v->w,
        .w = mt->m4[0] * v->x + mt->m4[1] * v->y + mt->m4[2] * v->z + mt->m4[3] * v->w};
}