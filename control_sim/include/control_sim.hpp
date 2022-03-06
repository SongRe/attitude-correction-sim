#ifndef CONTROL_SIM_HPP
#define CONTROL_SIM_HPP

extern "C"
{
#include "control_bridge.h"
#include "control_proxy.h"
}

#include "physics_sim.hpp"

// used to create a pipeline to access the interface
void ControlPipeline(PhysicsSimProperties *sim_properties, cntrl_bridge *bridge, double timestep);

#endif