#ifndef CONTROL_EIGENAXIS_INT_H
#define CONTROL_EIGENAXIS_INT_H

#include "control_proxy.h"

#include "vmath.h"
#include "mmath.h"

typedef struct
{
    vec3 min, max; // to prevent integrator windup
    vec3 val;
} vec3_integral;

typedef struct
{
    double gain_d, gain_p, gain_i;
    mat3 mt_gain_d, mt_gain_p, mt_gain_i;
    double time_sample_inter, time_filter;
    vec3_integral integral;
    vec3 prev_attitude_error, prev_i_error, prev_ang_vel_error;

} cntrl_eigenaxis;

vec3 calc_cntrl_mom_with_gyro_decoupl(mat3* iner_tensor, vec3* ang_vel, cntrl_eigenaxis* control, vec3 *err_ang_vel, vec3 *err_attit_v);
vec3 calc_cntrl_mom(cntrl_eigenaxis *control_data, vec3 *err_ang_vel, vec3 *err_attit_v);

void cntrl_eigenaxis_init(cntrl_proxy *proxy, void **data);
void cntrl_eigenaxis_update(cntrl_proxy *proxy, void **data);
void cntrl_eigenaxis_reset(cntrl_proxy *proxy, void **data);
void cntrl_eigenaxis_teardown(cntrl_proxy *proxy, void **data);
void* cntrl_eigenaxis_output(cntrl_proxy *proxy, void **data);

#endif
