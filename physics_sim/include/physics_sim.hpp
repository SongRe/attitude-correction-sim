#ifndef PHYSICS_SIM_HPP
#define PHYSICS_SIM_HPP

extern "C"
{
#include "vmath.h"
}

#include "rigidbody.hpp"
#include <thread>
#include <mutex>

struct PhysicsSimProperties
{
    // contains references to shared objects
    Rigidbody &rbody;
    vec3 &appl_mom;

    std::mutex &rbody_mutex;
    std::mutex &appl_mom_mutex;

    bool running;
    std::mutex running_mutex;
};

// used to create a new thread (only allows pointers instead of references)
void PhysicsPipeline(PhysicsSimProperties *sim_properties, double timestep);

#endif