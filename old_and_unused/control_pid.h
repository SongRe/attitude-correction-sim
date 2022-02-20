#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include "control_bridge.h"
#include "control_proxy.h"
#include <stdbool.h>

typedef struct
{
    double K_p, K_i, K_d;
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

// utilizes defined controller interface
void cntrl_pid_init(const cntrl_bridge *bridge, void *cntrl_data);
void cntrl_pid_update(const cntrl_bridge *bridge, void *cntrl_data);
void cntrl_pid_reset(const cntrl_bridge *bridge, void *cntrl_data);
void cntrl_pid_teardown(const cntrl_bridge *bridge, void *cntrl_data);
void *cntrl_pid_output(const cntrl_bridge *bridge, void *cntrl_data);

#endif