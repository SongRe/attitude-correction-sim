#ifndef CONTROL_EIGENAXIS_H
#define CONTROL_EIGENAXIS_H

#include "control_proxy.h"

#include "vmath.h"
#include "mmath.h"

typedef struct
{
    double gain_d, gain_p;
    mat3 mt_gain_d, mt_gain_p;
} cntrl_eigenaxis;

vec3 calc_cntrl_mom_with_gyro_decoupl(mat3* iner_tensor, vec3* ang_vel, mat3* gain_d, mat3* gain_p, vec3 *err_ang_vel, vec3 *err_attit_v);
vec3 calc_cntrl_mom(mat3* gain_d, mat3* gain_p, vec3 *err_ang_vel, vec3 *err_attit_v);

void cntrl_eigenaxis_init(cntrl_proxy *proxy, void **data, double timestep);
void cntrl_eigenaxis_update(cntrl_proxy *proxy, void **data, double timestep);
void cntrl_eigenaxis_reset(cntrl_proxy *proxy, void **data, double timestep);
void cntrl_eigenaxis_teardown(cntrl_proxy *proxy, void **data, double timestep);
void* cntrl_eigenaxis_output(cntrl_proxy *proxy, void **data, double timestep);

#endif