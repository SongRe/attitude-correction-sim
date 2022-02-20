#include "rigidbody_sim.hpp"
#include "attitude_viewer.hpp"

// ensures compatability with C-code
extern "C"
{
#include "rigidbody.h"
#include "vmath.h"
#include <Quaternion/Quaternion.h>
};

#include <thread>
#include <iostream>

#include <chrono>
#include <cmath>

#include <pthread.h>

void physicsPipeline(PhysicsSimProperties* sim_properties, double timestep)
{
    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();

        Vec3 moment = (Vec3){0, 0, 0};

        pthread_mutex_lock(&sim_properties->rbody_lock);

        Rigidbody_update_ang_vel(&sim_properties->rbody, &moment, timestep);
        Rigidbody_update_orient(&sim_properties->rbody, timestep);

        pthread_mutex_unlock(&sim_properties->rbody_lock);

        auto stop = std::chrono::high_resolution_clock::now();

        // enforces the timestep
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        auto wait_duration = std::chrono::duration<double>(timestep) - duration;

        std::this_thread::sleep_for(wait_duration);
    }
}

int main()
{
    // uses C-style mutex lock for controller
    pthread_mutex_t rbody_lock;
    if (pthread_mutex_init(&rbody_lock, NULL) != 0)
    {
        std::cout << "Failed to initialize mutex lock" << std::endl;
        return EXIT_FAILURE;
    }

    PhysicsSimProperties sim_properties = {
        .rbody = (Rigidbody){
        (Quaternion){
            1,          // real component (w)
            {0, 0, 0}}, // complex vector component (v)
        (Vec3){
            0, M_PI, M_PI},
        (Mat3){
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}}},
        .mom_command = {0, 0, 0}
    };

    int rbody_lock_success, mom_command_success;

    if(rbody_lock_success = pthread_mutex_init(&sim_properties.rbody_lock, NULL) != 0)
        std::cout << "Failed to initialize rbody lock" << std::endl;
    if(mom_command_success = pthread_mutex_init(&sim_properties.mom_command_lock, NULL) != 0)
        std::cout << "Failed to initialize moment command lock" << std::endl;

    if(rbody_lock_success != 0 || mom_command_success != 0){
        pthread_mutex_destroy(&sim_properties.rbody_lock);
        pthread_mutex_destroy(&sim_properties.mom_command_lock);
        return EXIT_FAILURE;
    }

    std::thread thread_render(renderPipeline, &sim_properties, 0.01);
    std::thread thread_physics(physicsPipeline, &sim_properties, 0.01);

    thread_render.join();
    thread_physics.join();

    return EXIT_SUCCESS;
}