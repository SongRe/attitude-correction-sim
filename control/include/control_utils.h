#ifndef CONTROL_UTILS_H
#define CONTROL_UTILS_H

#include "mmath.h"
#include "qmath.h"
#include "vmath.h"

quat calc_error_quat(quat* curr, quat* desir);

void quat_print(quat* q);
void vec3_print(vec3* v);

#endif