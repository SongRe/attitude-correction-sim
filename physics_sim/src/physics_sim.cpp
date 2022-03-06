#include "physics_sim.hpp"

extern "C"
{
#include "vmath.h"
}

#include <chrono>
#include <mutex>
#include <thread>

inline bool IsRunning(PhysicsSimProperties *sim_properties)
{
    sim_properties->running_mutex.lock();
    const bool running = sim_properties->running;
    sim_properties->running_mutex.unlock();

    return running;
}

void PhysicsPipeline(PhysicsSimProperties *sim_properties, double timestep)
{
    while (IsRunning(sim_properties))
    {
        auto start = std::chrono::high_resolution_clock::now();

        sim_properties->rbody_mutex.lock();
        sim_properties->appl_mom_mutex.lock();

        // updates the rigidbody
        sim_properties->rbody.UpdateAngVel(sim_properties->appl_mom, timestep);
        sim_properties->rbody.UpdateAttitude(timestep);

        sim_properties->appl_mom_mutex.unlock();
        sim_properties->rbody_mutex.unlock();

        auto stop = std::chrono::high_resolution_clock::now();

        // enforces the timestep
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        auto wait_duration = std::chrono::duration<double>(timestep) - duration;

        std::this_thread::sleep_for(wait_duration);
    }
}