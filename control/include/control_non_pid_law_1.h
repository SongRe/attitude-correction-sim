#ifndef CONTROL_NON_PID_LIN_GAIN_H
#define CONTROL_NON_PID_LIN_GAIN_H

#include "control_proxy.h"

// describes the interface definitions
void cntrl_non_pid_law_1_init(const cntrl_proxy* proxy, void** data);
void cntrl_non_pid_law_1_update(const cntrl_proxy* proxy, void** data);
void cntrl_non_pid_law_1_reset(const cntrl_proxy* proxy, void** data);
void cntrl_non_pid_law_1_teardown(const cntrl_proxy* proxy, void** data);
void* cntrl_non_pid_law_1_output(const cntrl_proxy* proxy, void** data);

#endif