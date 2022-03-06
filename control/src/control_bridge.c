#include "control_bridge.h"
#include <assert.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
cntrl_bridge cntrl_bridge_init(const bool suspended, const cntrl_proxy *proxy)
{
    return (cntrl_bridge){
        .suspended = suspended,
        .proxy = proxy};
}
#pragma GCC diagnostic pop

void cntrl_bridge_inf_suspend(cntrl_bridge *cntrl_bridge)
{
    cntrl_bridge->suspended = true;
}

void cntrl_bridge_inf_resume(cntrl_bridge *cntrl_bridge)
{
    cntrl_bridge->suspended = false;
}

void cntrl_bridge_inf_delegate_init(cntrl_bridge *bridge)
{
    assert(bridge->inf->cntrl_init != NULL);
    bridge->inf->cntrl_init(bridge->proxy, &bridge->inf->data);
}

void cntrl_bridge_inf_delegate_update(cntrl_bridge *bridge)
{
    assert(bridge->inf->cntrl_update != NULL);
    if (bridge->suspended)
        return; // prevents updates if the controller is suspended
    bridge->inf->cntrl_update(bridge->proxy, &bridge->inf->data);
}

void cntrl_bridge_inf_delegate_reset(cntrl_bridge *bridge)
{
    assert(bridge->inf->cntrl_reset != NULL);
    bridge->inf->cntrl_reset(bridge->proxy, &bridge->inf->data);
}

void cntrl_bridge_inf_delegate_teardown(cntrl_bridge *bridge)
{
    assert(bridge->inf->cntrl_teardown != NULL);
    bridge->inf->cntrl_teardown(bridge->proxy, &bridge->inf->data);
}

void *cntrl_bridge_inf_delegate_output(cntrl_bridge *bridge)
{
    assert(bridge->inf->cntrl_output != NULL);
    return bridge->inf->cntrl_output(bridge->proxy, &bridge->inf->data);
}
