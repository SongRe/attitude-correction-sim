#include <mutex>
#include <thread>
#include <cmath>

extern "C"
{
#include "vmath.h"
#include "mmath.h"
#include "qmath.h"
}

#include "physics_sim.hpp"
#include "sim_viewer.hpp"

int main()
{
    std::mutex rbody_mutex;
    std::mutex appl_mom_mutex;

    const vec3 axis_rot = {1, 0, 0};
    const quat rot_init = quat_from_axis(&axis_rot, 45.0 / 180 * M_PI);

    Rigidbody rbody = (Rigidbody){
        .attit = rot_init,
        .ang_vel = vec3_init(0, 1, 1),
        .iner_tensor = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}}};
    vec3 appl_mom = vec3_init(0, 0, 0);

    PhysicsSimProperties sim_properties = {
        .rbody = rbody,
        .appl_mom = appl_mom,
        .rbody_mutex = rbody_mutex,
        .appl_mom_mutex = appl_mom_mutex,
        .running = true};

    std::thread thread_physics(PhysicsPipeline, &sim_properties, 0.01);
    std::thread thread_render(RenderPipeline, &sim_properties, 1.0f / 60);

    thread_physics.join();
    thread_render.join();

    return EXIT_SUCCESS;
}
