#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include "control_proxy.h"

typedef struct
{
    double k_p, k_i, k_d;
} pid_gain;

typedef struct
{
    double min_i, max_i;
    double integ;
} integrator;

// would be stored by void* cntrl_data
typedef struct
{
    pid_gain gain;
    integrator integr;

    double comm;
} cntrl_pid;

void pid_init(const cntrl_pid *pid);
void pid_update(const cntrl_pid *pid);

void cntrl_pid_init(cntrl_proxy* proxy, void** data);
void cntrl_pid_update(cntrl_proxy* proxy, void** data);
void cntrl_pid_reset(cntrl_proxy* proxy, void** data);
void cntrl_pid_teardown(cntrl_proxy* proxy, void** data);
void cntrl_pid_output(cntrl_proxy* proxy, void** data);

#endif