#include "algs/control_pid.h"
#include "vmath.h"
#include "mmath.h"

#include "control_proxy.h"
#include "control_utils.h"

#include <stdio.h>
#include <stdlib.h>

vec3 calc_pid_output(cntrl_pid *pid, mat3 *iner_tensor, vec3 *err_v, vec3 *curr_attit_v, double timestep)
{
    // proportional control
    vec3 p_out = vec3_smul(err_v, pid->gain_p);

    // integral control
    vec3 integr_incr = vec3_smul(err_v, pid->gain_i * timestep);
    vec3 integr = vec3_add(&pid->integr.val, &integr_incr);

    // prevent integral windup
    if ((integr.x > pid->integr.min.x &&
         integr.y > pid->integr.min.y &&
         integr.z > pid->integr.min.z) && // checks to ensure the new integral doesn't exceed minimum
        (integr.x < pid->integr.max.x &&
         integr.y < pid->integr.max.y &&
         integr.z < pid->integr.max.z)) // checks to ensure the new integral doesn't exceed maximum
    {
        // printf("Integrator within limits!\n");
        // vec3_print(&integr);
        pid->integr.val = integr;
    }
    else
    {
        // printf("Integrator outside of limts!\n");
        // vec3_print(&integr);
    }
    // printf("Integrator:\n");
    // vec3_print(&pid->integr.val);

    vec3 i_out = pid->integr.val;

    // derivative control (unfiltered)
    vec3 diff_err = vec3_subtract(err_v, &pid->prev_err);
    vec3 deriv = vec3_smul(&diff_err, 1.0 / timestep);
    vec3 d_out = vec3_smul(&deriv, pid->gain_d);
    pid->prev_err = *err_v; // updates for the next frame

    // multiplies by the inertia tensor
    vec3 out_sum = vec3_add(&p_out, &i_out);
    out_sum = vec3_add(&out_sum, &d_out);

    // negates the result
    vec3 prod_with_iner_tensor = mat3_vmul(iner_tensor, &out_sum);
    return vec3_smul(&prod_with_iner_tensor, -1);
}

void cntrl_pid_init(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("PID control: init\n");

    *data = malloc(sizeof(cntrl_pid));
    cntrl_pid *pid_data = (cntrl_pid *)*data;

    const double gain_p = 1.0;
    const double gain_i = 0.25;
    const double gain_d = 2.0;

    pid_data->gain_p = gain_p;
    pid_data->gain_i = gain_i;
    pid_data->gain_d = gain_d;

    pid_data->integr = (vec3_integr){
        .min = vec3_init(-1, -1, -1), // vec3_init(-0.2, -0.2, -0.2),
        .max = vec3_init(1, 1, 1), // vec3_init(0.2, 0.2, 0.2),
        .val = vec3_init(0, 0, 0)};

    rbody_data comm_data, curr_data;
    cntrl_proxy_pull_comm_rbody(proxy, &comm_data);
    cntrl_proxy_pull_curr_rbody(proxy, &curr_data);

    // sets this initially but this is handled by calc_pid_output
    quat err_attit = calc_error_quat(&curr_data.attit, &comm_data.attit);
    pid_data->prev_err = err_attit.v;
}

void cntrl_pid_update(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("PID control: update\n");

    cntrl_pid *pid_data = (cntrl_pid *)*data;

    rbody_data comm_data, curr_data;
    cntrl_proxy_pull_comm_rbody(proxy, &comm_data);
    cntrl_proxy_pull_curr_rbody(proxy, &curr_data);

    // printf("Current angular velocity:\n");
    // vec3_print(&curr_data.ang_vel);

    quat err_attit = calc_error_quat(&curr_data.attit, &comm_data.attit);

    vec3 moment = calc_pid_output(pid_data, &curr_data.iner_tensor, &err_attit.v, &curr_data.attit.v, timestep);
    // printf("Calculated commanded moment:\n");
    // vec3_print(&moment);
    cntrl_proxy_push_cntrl_mom(proxy, &moment);

    // printf("Integrator:\n");
    // vec3_print(&pid_data->integr.val);
}

void cntrl_pid_reset(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("PID control: reset\n");

    cntrl_pid *pid_data = (cntrl_pid *)*data;
    pid_data->integr.val = vec3_init(0, 0, 0);
}

void cntrl_pid_teardown(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("PID control: teardown\n");
    free(*data);
}

void *cntrl_pid_output(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("PID control: output\n");
}
