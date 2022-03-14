#ifndef CONTROL_PROXY_H
#define CONTROL_PROXY_H

#include "qmath.h"
#include "vmath.h"
#include "mmath.h"

#include <pthread.h>

typedef struct
{
    quat *attit;
    vec3 *ang_vel;
    mat3* iner_tensor;
} rbody_data_ref;

typedef struct
{
    quat attit;
    vec3 ang_vel;
    mat3 iner_tensor;
} rbody_data;

typedef struct
{
    rbody_data_ref *comm_rbody;
    rbody_data_ref *curr_rbody;

    vec3 *cntrl_mom;

    // mutex protections for proxy interface
    pthread_mutex_t *comm_rbody_protect;
    pthread_mutex_t *curr_rbody_protect;
    pthread_mutex_t *cntrl_mom_protect;
} cntrl_proxy;

cntrl_proxy cntrl_proxy_init(rbody_data_ref *comm_rbody, rbody_data_ref *curr_rbody, vec3 *cntrl_mom);
void cntrl_proxy_add_protect(cntrl_proxy* proxy, pthread_mutex_t *comm_rbody_protect, pthread_mutex_t *curr_rbody_protect, pthread_mutex_t *cntrl_mom_protect);

void cntrl_proxy_pull_comm_rbody(cntrl_proxy *proxy, rbody_data *repl);
void cntrl_proxy_pull_curr_rbody(cntrl_proxy *proxy, rbody_data *repl);

void cntrl_proxy_push_comm_rbody(cntrl_proxy *proxy, rbody_data* data);
void cntrl_proxy_push_cntrl_mom(cntrl_proxy *proxy, vec3 *mom);

#endif