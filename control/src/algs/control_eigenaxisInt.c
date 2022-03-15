#include "vmath.h"
#include "mmath.h"
#include "qmath.h"

#include "algs/control_eigenaxisInt.h"
#include "control_utils.h"

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

vec3 calc_cntrl_mom_with_gyro_decoupl(mat3 *iner_tensor, vec3 *ang_vel, cntrl_eigenaxis *control_data, vec3 *err_ang_vel, vec3 *err_attit_v)
{
    // M_cmd = -J * (dw_e - pq_ev - iq_ev)

    // the second part isn't exactly the same as calc_cntrl_mom, so we have to re-do everything here
    double time_ratio = control_data->time_sample_inter / control_data->time_filter;


    vec3 *err_proportional = &err_attit_v;

    vec3 error_i_prod = vec3_smul(&control_data->prev_attitude_error, control_data->time_sample_inter);
    vec3* p = &error_i_prod;
    vec3 err_integral = vec3_add(p, &control_data->prev_i_error);
    vec3 *err_i_p = &err_integral;


    // vec3 error_f_left = vec3_smul(&err_ang_vel, time_ratio);
    // vec3 error_f_right = vec3_smul(&err_ang_vel, 1 - time_ratio);
    // vec3 *err_l_p = &error_f_left;
    // vec3 *err_r_p = &error_f_right;
    // vec3 err_filtered = vec3_add(err_l_p, err_r_p);
    // vec3 *err_filtered_p = &err_filtered;
    

    mat3 *gain_d = &control_data->gain_d;
    mat3 *gain_p = &control_data->gain_p;
    mat3 *gain_i = &control_data->gain_i;

    vec3 gain_d_prod = mat3_vmul(gain_d, err_ang_vel);
   // vec3 gain_d_prod = mat3_vmul(gain_d, err_filtered_p);  
    vec3 gain_p_prod = mat3_vmul(gain_p, err_attit_v);
    vec3 gain_i_prod = mat3_vmul(gain_i, err_i_p);
    vec3 sum = vec3_add(&gain_i_prod, &gain_p_prod);
    sum = vec3_smul(&sum, -1);
    vec3 *sum_p = &sum;
    vec3 *gain_p_p = &gain_d_prod;
    sum = vec3_add(gain_d, sum_p);

    return mat3_vmul(iner_tensor, sum_p);
}

vec3 calc_cntrl_mom(cntrl_eigenaxis *control_data, vec3 *err_ang_vel, vec3 *err_attit_v)
{
    // M_cmd = - K_d w_e - K_p q_ev - K_i * integral(q_ev)
    // in discrete time, this equation becomes:
    // M_cmd(k) = k_p * e(k) + k_i * e_i(k) + k_d * e_f(k);
    /*
        where k is the kth iteration 
        e(k) is just e(k) - attit_v
        e_i(k) = e_i(k - 1) + T_s * e(k-1)
        e_f(k) = a * e(k) + (1 - a) e_f(k - 1); - e(k) for this is the angular velocity

    */
    double time_ratio = control_data->time_sample_inter / control_data->time_filter;


    vec3 *err_proportional = &err_attit_v;

    vec3 error_i_prod = vec3_smul(&control_data->prev_attitude_error, control_data->time_sample_inter);
    vec3* p = &error_i_prod;
    vec3 err_integral = vec3_add(p, &control_data->prev_i_error);
    vec3 *err_i_p = &err_integral;


    vec3 error_f_left = vec3_smul(&err_ang_vel, time_ratio);
    vec3 error_f_right = vec3_smul(&err_ang_vel, 1 - time_ratio);
    vec3 *err_l_p = &error_f_left;
    vec3 *err_r_p = &error_f_right;
    vec3 err_filtered = vec3_add(err_l_p, err_r_p);
    vec3 *err_filtered_p = &err_filtered;

    mat3 *gain_d = &control_data->gain_d;
    mat3 *gain_p = &control_data->gain_p;
    mat3 *gain_i = &control_data->gain_i;

    vec3 gain_d_prod = mat3_vmul(gain_d, err_filtered_p);  
    vec3 gain_p_prod = mat3_vmul(gain_p, err_attit_v);
    vec3 gain_i_prod = mat3_vmul(gain_i, err_i_p);
    vec3 sum = vec3_add(&gain_d_prod, &gain_p_prod);
    sum = vec3_add(&sum, &gain_i_prod);

    return vec3_smul(&sum, -1);
}

void cntrl_eigenaxis_init(cntrl_proxy *proxy, void **data)
{
    printf("cntrl_eigenaxis: init\n");

    *data = malloc(sizeof(cntrl_eigenaxis));
    cntrl_eigenaxis *cntrl_data = (cntrl_eigenaxis *)*data;

    const double gain_d = 1.0;
    const double gain_p = 0.5;
    const double gain_i = 0.5;
    const double timeStep = 100;
    const double timeFilter = 100;

    // NOTE: J is inertia matrix w.r.t. body-fixed frame

    rbody_data curr_data;
    cntrl_proxy_pull_curr_rbody(proxy, &curr_data);

    printf("Inertia tensor:\n");
    mat3_print(&curr_data.iner_tensor);

    cntrl_data->integral = (vec3_integral){
        .min = vec3_init(-1, -1, -1), // vec3_init(-0.2, -0.2, -0.2),
        .max = vec3_init(1, 1, 1), // vec3_init(0.2, 0.2, 0.2),
        .val = vec3_init(0, 0, 0)};

    *cntrl_data = (cntrl_eigenaxis){
        .gain_d = gain_d,
        .gain_p = gain_p,
        .gain_i = gain_i,
        .mt_gain_d = mat3_smul(&curr_data.iner_tensor, gain_d),
        .mt_gain_p = mat3_smul(&curr_data.iner_tensor, gain_p),
        .mt_gain_i = mat3_smul(&curr_data.iner_tensor, gain_i),
        .time_sample_inter = timeStep,
        .time_filter = timeFilter};

    rbody_data comm_data, curr_data;
    cntrl_proxy_pull_comm_rbody(proxy, &comm_data);
    cntrl_proxy_pull_curr_rbody(proxy, &curr_data);

    // setting initial attiude error
    quat err_attit = calc_error_quat(&curr_data.attit, &comm_data.attit);
    cntrl_data->prev_attitude_error = err_attit.v;
    cntrl_data->prev_i_error = err_attit.v;

    //setting initial angular velocity error
    vec3 comm_ang_vel_neg = vec3_smul(&comm_data.ang_vel, -1);
    vec3 err_ang_vel = vec3_add(&curr_data.ang_vel, &comm_ang_vel_neg); //
    cntrl_data->prev_ang_vel_error = err_ang_vel;

}

void cntrl_eigenaxis_update(cntrl_proxy *proxy, void **data)
{
    /*
        u(k) = k_p * e(k) + k_i * e_i(k) + k_d * e_f(k);
        where k is the kth iteration 
        e(k) is just e(k) - attit_v
        e_i(k) = e_i(k - 1) + T_s * e(k-1)
        e_f(k) = a * e(k) + (1 - a) e_f(k - 1); - e(k) will be for angular velocity
        a is T_s / T_f, where T_f is the filtering constant

    */
    printf("cntrl_eigenaxis: update\n");

    cntrl_eigenaxis *cntrl_data = (cntrl_eigenaxis *)*data;

    // pulls the most recent commanded values and current values
    rbody_data comm_data, curr_data;
    cntrl_proxy_pull_comm_rbody(proxy, &comm_data);
    cntrl_proxy_pull_curr_rbody(proxy, &curr_data);

    printf("Current angular velocity:\n");
    vec3_print(&curr_data.ang_vel);

    // calculates the error values required for controller calculation
    vec3 comm_ang_vel_neg = vec3_smul(&comm_data.ang_vel, -1);
    vec3 err_ang_vel = vec3_add(&curr_data.ang_vel, &comm_ang_vel_neg); //
    quat err_attit = calc_error_quat(&curr_data.attit, &comm_data.attit); // this is e(k)

    // calculates the required moment and delegates the proxy to push the value

    vec3 moment = calc_cntrl_mom(&cntrl_data, &err_ang_vel, &err_attit.v);
    printf("Calculated commanded moment:\n");
    vec3_print(&moment);

    cntrl_proxy_push_cntrl_mom(proxy, &moment);
}

void cntrl_eigenaxis_reset(cntrl_proxy *proxy, void **data)
{
    printf("cntrl_eigenaxis: reset\n");
    // there is nothing to reset with this controller
}

void cntrl_eigenaxis_teardown(cntrl_proxy *proxy, void **data)
{
    printf("cntrl_eigenaxis: teardown\n");
    free(*data);
}

void *cntrl_eigenaxis_output(cntrl_proxy *proxy, void **data)
{
    printf("cntrl_eigenaxis: output\n");
}
