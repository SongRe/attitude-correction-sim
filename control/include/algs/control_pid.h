#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include "vmath.h"
#include "mmath.h"
#include "control_proxy.h"

typedef struct
{
    vec3 min, max; // to prevent integrator windup
    vec3 val;
} vec3_integr;

// would be stored by void* cntrl_data
typedef struct
{
    double gain_p, gain_i, gain_d;

    vec3_integr integr;
    vec3 prev_err;
} cntrl_pid;

vec3 calc_pid_output(cntrl_pid *pid, mat3 *iner_tensor, vec3 *err_v, vec3 *curr_attit_v, double timestep);

void cntrl_pid_init(cntrl_proxy *proxy, void **data, double timestep);
void cntrl_pid_update(cntrl_proxy *proxy, void **data, double timestep);
void cntrl_pid_reset(cntrl_proxy *proxy, void **data, double timestep);
void cntrl_pid_teardown(cntrl_proxy *proxy, void **data, double timestep);
void* cntrl_pid_output(cntrl_proxy *proxy, void **data, double timestep);

#endif