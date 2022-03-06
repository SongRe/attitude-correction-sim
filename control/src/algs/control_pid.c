#include "control_proxy.h"
#include "control_bridge.h"

#include "algs/control_pid.h"

void pid_init(const cntrl_pid *pid) {}

void pid_update(const cntrl_pid *pid) {}

void cntrl_pid_init(cntrl_proxy *proxy, void **data) {}

void cntrl_pid_update(cntrl_proxy *proxy, void **data) {}

void cntrl_pid_reset(cntrl_proxy *proxy, void **data) {}

void cntrl_pid_teardown(cntrl_proxy *proxy, void **data) {}

void cntrl_pid_output(cntrl_proxy *proxy, void **data) {}
