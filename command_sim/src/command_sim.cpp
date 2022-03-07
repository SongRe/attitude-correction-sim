extern "C"
{
#include "control_bridge.h"
#include "control_proxy.h"
#include "control_utils.h"

#include "vmath.h"
#include "qmath.h"
}

#include "physics_sim.hpp"
#include "command_sim.hpp"

#include <chrono>
#include <mutex>
#include <thread>

#include <cstdlib>
#include <signal.h>

#include <iostream>
#include <cmath>

// globals defined to be used within the interrupt callback
PhysicsSimProperties *int_sim_properties;
cntrl_bridge *int_bridge;

void SignalInterruptCallback(int signum)
{
    // int_sim_properties->running_mutex.lock();
    // int_sim_properties->running = false;
    // int_sim_properties->running_mutex.unlock();

    exit(EXIT_SUCCESS);
}

inline bool IsRunning(PhysicsSimProperties *sim_properties)
{
    sim_properties->running_mutex.lock();
    const bool running = sim_properties->running;
    sim_properties->running_mutex.unlock();

    return running;
}

inline bool ValsClose(double v1, double v2)
{
    const double criterion = 1e-2;
    return (std::abs(v1 - v2) < criterion);
}

inline bool AttitudeMatches(cntrl_proxy *proxy)
{
    rbody_data comm_data, curr_data;
    cntrl_proxy_pull_comm_rbody(proxy, &comm_data);
    cntrl_proxy_pull_curr_rbody(proxy, &curr_data);

    std::cout << "                                                                               \r"
              << "Current attitude:"
              << "\tw: " << curr_data.attit.w
              << "\tx: " << curr_data.attit.v.x
              << "\ty: " << curr_data.attit.v.y
              << "\tz: " << curr_data.attit.v.z << "\r" << std::flush;

    // std::cout << "Comm attitude:" << std::endl;
    // quat_print(&comm_data.attit);
    // std::cout << "Curr attitude:" << std::endl;
    // quat_print(&curr_data.attit);

    bool match = ValsClose(comm_data.attit.w, curr_data.attit.w) &&
                 ValsClose(comm_data.attit.v.x, curr_data.attit.v.x) &&
                 ValsClose(comm_data.attit.v.y, curr_data.attit.v.y) &&
                 ValsClose(comm_data.attit.v.z, curr_data.attit.v.z);
    // std::cout << "Matches: " << (match ? "true" : "false") << std::endl;

    return match;
}

void CommandPipeline(PhysicsSimProperties *sim_properties, cntrl_bridge *bridge)
{
    // changes default interrupt behaviour
    int_sim_properties = sim_properties;
    int_bridge = int_bridge;
    signal(SIGINT, SignalInterruptCallback);

    std::cout << "Controller not set, rigidbody is currently drifting" << std::endl;

    while (IsRunning(sim_properties))
    {
        // auto start = std::chrono::high_resolution_clock::now();

        double x, y, z;
        double angle;

        std::cout << "Enter new commanded attitude:" << std::endl;
        std::cout << "Enter axis of rotation and angle for rigidbody target attitude:\n";
        std::cout << "\tx: ";
        std::cin >> x;
        std::cout << "\ty: ";
        std::cin >> y;
        std::cout << "\tz: ";
        std::cin >> z;
        std::cout << "\tangle (deg): ";
        std::cin >> angle;

        vec3 axis;
        if (x == 0 && y == 0 && z == 0) // prevents an assertion failure
            axis = vec3_init(1, 0, 0);
        else
            axis = vec3_init(x, y, z);
        quat attit = quat_from_axis(&axis, angle * M_PI / 180);

        std::cout << "\tCommanded quaternion: ";
        quat_print(&attit);

        std::cout << "Transitioning to new commanded attitude..." << std::endl;

        rbody_data new_comm = (rbody_data){
            .attit = attit,
            .ang_vel = vec3_init(0, 0, 0)};
        cntrl_proxy_push_comm_rbody(bridge->proxy, &new_comm);

        while (!AttitudeMatches(bridge->proxy))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "\nTransition complete" << std::endl;

        // auto stop = std::chrono::high_resolution_clock::now();

        // enforces the timestep
        // auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        // auto wait_duration = std::chrono::duration<double>(timestep) - duration;

        // std::this_thread::sleep_for(wait_duration);
    }
}