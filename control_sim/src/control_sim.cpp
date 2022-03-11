extern "C"
{
#include "control_bridge.h"
#include "control_proxy.h"
}

#include "physics_sim.hpp"
#include "control_sim.hpp"

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

void ControlPipeline(PhysicsSimProperties *sim_properties, cntrl_bridge *bridge, double timestep)
{
    cntrl_bridge_inf_delegate_init(bridge);

    while (IsRunning(sim_properties))
    {
        auto start = std::chrono::high_resolution_clock::now();

        cntrl_bridge_inf_delegate_update(bridge);

        // the bidirectional nature of the proxy can replace the output functionality
        // cntrl_bridge_inf_delegate_output(bridge);

        auto stop = std::chrono::high_resolution_clock::now();

        // enforces the timestep
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        auto wait_duration = std::chrono::duration<double>(timestep) - duration;

        std::this_thread::sleep_for(wait_duration);
    }

    cntrl_bridge_inf_delegate_teardown(bridge);
}