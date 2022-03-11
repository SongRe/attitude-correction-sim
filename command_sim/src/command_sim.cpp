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

#define CLOSE_CRITERION 1e-2
#define NUM_WHITESPACE 30

inline bool IsRunning(PhysicsSimProperties *sim_properties)
{
    sim_properties->running_mutex.lock();
    const bool running = sim_properties->running;
    sim_properties->running_mutex.unlock();

    return running;
}

void CommandPipeline(PhysicsSimProperties *sim_properties, cntrl_bridge *bridge)
{
    std::cout << "Controller not set, rigidbody is currently drifting" << std::endl;

    while (IsRunning(sim_properties))
    {
        double x, y, z;
        double angle;

        std::cout << "Enter new commanded attitude:" << std::endl;
        std::cout << "Enter axis of rotation and angle for rigidbody target attitude:\n";
        std::cout << '\t' << "x: ";
        std::cin >> x;
        std::cout << '\t' << "y: ";
        std::cin >> y;
        std::cout << '\t' << "z: ";
        std::cin >> z;
        std::cout << '\t' << "angle (deg): ";
        std::cin >> angle;

        vec3 axis;
        if (x == 0 && y == 0 && z == 0) // prevents an assertion failure
            axis = vec3_init(1, 0, 0);
        else
            axis = vec3_init(x, y, z);
        quat attit = quat_from_axis(&axis, angle * M_PI / 180);

        std::cout << '\t' << "Commanded quaternion: ";
        quat_print(&attit);

        std::cout << "Transitioning to new commanded attitude..." << std::endl;

        rbody_data new_comm = (rbody_data){
            .attit = attit,
            .ang_vel = vec3_init(0, 0, 0)};
        cntrl_proxy_push_comm_rbody(bridge->proxy, &new_comm);

        while (true)
        {
            rbody_data comm_data, curr_data;
            cntrl_proxy_pull_comm_rbody(bridge->proxy, &comm_data);
            cntrl_proxy_pull_curr_rbody(bridge->proxy, &curr_data);

            for (int i = 0; i < NUM_WHITESPACE; i++)
                std::cout << ' ';
            std::cout << '\r'
                      << "Current attitude:"
                      << '\t' << "w: " << curr_data.attit.w
                      << '\t' << "x: " << curr_data.attit.v.x
                      << '\t' << "y: " << curr_data.attit.v.y
                      << '\t' << "z: " << curr_data.attit.v.z << '\r' << std::flush;

            if ((std::abs(comm_data.attit.w - curr_data.attit.w) < CLOSE_CRITERION) &&
                (std::abs(comm_data.attit.v.x - curr_data.attit.v.x) < CLOSE_CRITERION) &&
                (std::abs(comm_data.attit.v.y - curr_data.attit.v.y) < CLOSE_CRITERION) &&
                (std::abs(comm_data.attit.v.z - curr_data.attit.v.z) < CLOSE_CRITERION))
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << '\n'
                  << "Transition complete" << std::endl;
    }
}