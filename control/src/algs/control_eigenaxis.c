#include "vmath.h"
#include "mmath.h"
#include "qmath.h"

#include "algs/control_eigenaxis.h"
#include "control_utils.h"

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

vec3 calc_cntrl_mom_with_gyro_decoupl(mat3 *iner_tensor, vec3 *ang_vel, mat3 *gain_d, mat3 *gain_p, vec3 *err_ang_vel, vec3 *err_attit_v)
{
    // M_cmd = -w x Jw - K_d w_e - K_p q_ev

    // first term
    vec3 tensor_vel_prod = mat3_vmul(iner_tensor, ang_vel);
    vec3 vel_cross_prod = vec3_cross(ang_vel, &tensor_vel_prod);
    vec3 vel_cross_prod_neg = vec3_smul(&vel_cross_prod, -1);

    // second and third terms
    vec3 gain_prods = calc_cntrl_mom(gain_d, gain_p, err_ang_vel, err_attit_v);

    return vec3_add(&vel_cross_prod_neg, &gain_prods);
}

vec3 calc_cntrl_mom(mat3 *gain_d, mat3 *gain_p, vec3 *err_ang_vel, vec3 *err_attit_v)
{
    // M_cmd = - K_d w_e - K_p q_ev

    vec3 gain_d_prod = mat3_vmul(gain_d, err_ang_vel);
    vec3 gain_p_prod = mat3_vmul(gain_p, err_attit_v);
    vec3 sum = vec3_add(&gain_d_prod, &gain_p_prod);

    return vec3_smul(&sum, -1);
}

void cntrl_eigenaxis_init(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("cntrl_eigenaxis: init\n");

    *data = malloc(sizeof(cntrl_eigenaxis));
    cntrl_eigenaxis *cntrl_data = (cntrl_eigenaxis *)*data;

    const double gain_d = 1.0;
    const double gain_p = 0.5;

    // NOTE: J is inertia matrix w.r.t. body-fixed frame

    rbody_data curr_data;
    cntrl_proxy_pull_curr_rbody(proxy, &curr_data);

    // printf("Inertia tensor:\n");
    // mat3_print(&curr_data.iner_tensor);

    *cntrl_data = (cntrl_eigenaxis){
        .gain_d = gain_d,
        .gain_p = gain_p,
        .mt_gain_d = mat3_smul(&curr_data.iner_tensor, gain_d),
        .mt_gain_p = mat3_smul(&curr_data.iner_tensor, gain_p)};
}

void cntrl_eigenaxis_update(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("cntrl_eigenaxis: update\n");

    cntrl_eigenaxis *cntrl_data = (cntrl_eigenaxis *)*data;

    // pulls the most recent commanded values and current values
    rbody_data comm_data, curr_data;
    cntrl_proxy_pull_comm_rbody(proxy, &comm_data);
    cntrl_proxy_pull_curr_rbody(proxy, &curr_data);

    // printf("Current angular velocity:\n");
    // vec3_print(&curr_data.ang_vel);

    // calculates the error values required for controller calculation
    vec3 comm_ang_vel_neg = vec3_smul(&comm_data.ang_vel, -1);
    vec3 err_ang_vel = vec3_add(&curr_data.ang_vel, &comm_ang_vel_neg);

    quat err_attit = calc_error_quat(&curr_data.attit, &comm_data.attit);

    // calculates the required moment and delegates the proxy to push the value
    vec3 moment = calc_cntrl_mom(&cntrl_data->mt_gain_d, &cntrl_data->mt_gain_p, &err_ang_vel, &err_attit.v);
    // printf("Calculated commanded moment:\n");
    // vec3_print(&moment);

    cntrl_proxy_push_cntrl_mom(proxy, &moment);
}

void cntrl_eigenaxis_reset(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("cntrl_eigenaxis: reset\n");
    // there is nothing to reset with this controller
}

void cntrl_eigenaxis_teardown(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("cntrl_eigenaxis: teardown\n");
    free(*data);
}

void *cntrl_eigenaxis_output(cntrl_proxy *proxy, void **data, double timestep)
{
    // printf("cntrl_eigenaxis: output\n");
}
