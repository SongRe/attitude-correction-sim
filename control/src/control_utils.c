#include "mmath.h"
#include "qmath.h"
#include "vmath.h"

#include "control_utils.h"

#include <stdio.h>

quat calc_error_quat(quat *curr, quat *desir)
{
    /*
    vec4 v_curr = quat_to_vec(curr);
    mat4 m_desir = (mat4){
        .m1 = {desir->w, desir->v.z, -desir->v.y, -desir->v.x},
        .m2 = {-desir->v.z, desir->w, desir->v.x, -desir->v.y},
        .m3 = {desir->v.y, -desir->v.x, desir->w, -desir->v.z},
        .m4 = {desir->v.x, desir->v.y, desir->v.z, desir->w}};

    vec4 v_err = mat4_vmul(&m_desir, &v_curr);
    return quat_from_vec(&v_err);
    */

    // from the article for design/implementation of 6U CubeSat mission
    quat desir_inv = quat_inv(desir);
    return quat_mul(&desir_inv, curr);
}

void quat_print(quat *q)
{
    printf("Quat:\tw:%f\tx:%f\ty:%f\tz:%f\n", q->w, q->v.x, q->v.y, q->v.z);
}

void vec3_print(vec3 *v)
{
    printf("Vec3:\tx:%f\ty:%f\tz:%f\n", v->x, v->y, v->z);
}

void mat3_print(mat3 *mt)
{
    printf("Mat3:\n");
    printf("[\t%f\t%f\t%f\n", mt->m1[0], mt->m1[1], mt->m1[2]);
    printf("\t%f\t%f\t%f\n", mt->m2[0], mt->m2[1], mt->m2[2]);
    printf("\t%f\t%f\t%f\t]\n", mt->m3[0], mt->m3[1], mt->m3[2]);
}
