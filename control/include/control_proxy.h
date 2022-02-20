#ifndef CONTROL_PROXY_H
#define CONTROL_PROXY_H

#include "math/qmath.h"
#include "math/vmath.h"

#include <pthread.h>

typedef struct
{
    quat attit;
    vec3 ang_vel;
} att_cntrl_data;

// might be good at some stage to just create a massive buffer and have custom implementation in controller
// and command/current state updaters interpret -> would make this more generic than current
typedef struct
{
    att_cntrl_data *comm;
    att_cntrl_data *curr;

    pthread_mutex_t *comm_protect;
    pthread_mutex_t *curr_protect;
} cntrl_proxy;

cntrl_proxy cntrl_proxy_init(const att_cntrl_data *comm, const att_cntrl_data *curr);
void cntrl_proxy_init_protect(cntrl_proxy *proxy, const pthread_mutex_t *comm_protect, const pthread_mutex_t *curr_protect);

void cntrl_proxy_pull_comm(const cntrl_proxy *proxy, att_cntrl_data *comm_repl);
void cntrl_proxy_pull_curr(const cntrl_proxy *proxy, att_cntrl_data *curr_repl);

#endif