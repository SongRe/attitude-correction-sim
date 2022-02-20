#ifndef RIGIDBODY_SIM_HPP
#define RIGIDBODY_SIM_HPP

extern "C"
{
#include "rigidbody.h"
}

#include <pthread.h>
#include "vmath.h"

struct PhysicsSimProperties{
    Rigidbody rbody;
    pthread_mutex_t rbody_lock;

    Vec3 mom_command;
    pthread_mutex_t mom_command_lock;
};

void physicsPipeline(PhysicsSimProperties* sim_properties, double timestep);

#endif