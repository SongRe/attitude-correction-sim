#ifndef COMMAND_SIM_HPP
#define COMMAND_SIM_HPP

extern "C"
{
#include "control_bridge.h"
#include "control_proxy.h"
}

#include "physics_sim.hpp"

// used to create a pipeline to control the commanded attitude
void CommandPipeline(PhysicsSimProperties* sim_properties, cntrl_bridge* bridge);

#endif