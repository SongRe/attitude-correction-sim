#ifndef CONTROL_BRIDGE_H
#define CONTROL_BRIDGE_H

#include "control_proxy.h"
#include <stdbool.h>

typedef struct
{
    void *data;

    void (*cntrl_init)(const cntrl_proxy *, void **);
    void (*cntrl_update)(const cntrl_proxy *, void **);
    void (*cntrl_reset)(const cntrl_proxy *, void **);
    void (*cntrl_teardown)(const cntrl_proxy *, void **);
    void *(*cntrl_output)(const cntrl_proxy *, void **);
} cntrl_inf;

typedef struct
{
    double cntrl_upd_freq;
    bool suspended;

    cntrl_proxy *proxy;
    cntrl_inf *inf;

} cntrl_bridge;

// not part of interface definition
cntrl_bridge cntrl_bridge_init(const double cntrl_upd_freq, const bool suspended, const cntrl_proxy *proxy);
void cntrl_bridge_inf_suspend(cntrl_bridge *cntrl_bridge);
void cntrl_bridge_inf_resume(cntrl_bridge *cntrl_bridge);

// defines interface
void cntrl_bridge_inf_delegate_init(cntrl_bridge *bridge);
void cntrl_bridge_inf_delegate_update(cntrl_bridge *bridge);
void cntrl_bridge_inf_delegate_reset(cntrl_bridge *bridge);
void cntrl_bridge_inf_delegate_teardown(cntrl_bridge *bridge);
void *cntrl_bridge_inf_delegate_output(cntrl_bridge *bridge);

#endif