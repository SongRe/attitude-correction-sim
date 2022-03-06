#include <mutex>
#include <thread>
#include <cmath>

extern "C"
{
#include "vmath.h"
#include "mmath.h"
#include "qmath.h"

#include "control_bridge.h"
#include "control_proxy.h"
    // #include "control_test.h" // for initial testing

#include "algs/control_eigenaxis.h"
}

#include "physics_sim.hpp"
#include "control_sim.hpp"
#include "sim_viewer.hpp"

int main()
{
    std::mutex comm_rbody_mutex;
    std::mutex curr_rbody_mutex;
    std::mutex appl_mom_mutex;

    // prepares the initial rigidbody
    const vec3 axis_rot_init = {1, 1, 0};
    const quat rot_init = quat_from_axis(&axis_rot_init, 45.0 / 180 * M_PI);
    Rigidbody rbody = (Rigidbody){
        .attit = rot_init,
        .ang_vel = vec3_init(0, 5, 0),
        .iner_tensor = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}}};
    vec3 appl_mom = vec3_init(0, 0, 0);

    PhysicsSimProperties sim_properties = {
        .rbody = rbody,
        .appl_mom = appl_mom,
        .rbody_mutex = curr_rbody_mutex,
        .appl_mom_mutex = appl_mom_mutex,
        .running = true};

    // sets up the controller
    quat comm_attit = quat_from_axis(&axis_rot_init, 0);
    vec3 comm_ang_vel = vec3_init(0, 0, 0);

    rbody_data_ref comm_rbody = (rbody_data_ref){
        .attit = &comm_attit,
        .ang_vel = &comm_ang_vel};

    rbody_data_ref curr_rbody = (rbody_data_ref){
        .attit = &rbody.attit,
        .ang_vel = &rbody.ang_vel,
        .iner_tensor = &sim_properties.rbody.iner_tensor};

    cntrl_proxy proxy = cntrl_proxy_init(&comm_rbody, &curr_rbody, &sim_properties.appl_mom);
    cntrl_proxy_add_protect(&proxy, comm_rbody_mutex.native_handle(), curr_rbody_mutex.native_handle(), appl_mom_mutex.native_handle());

    cntrl_bridge bridge = cntrl_bridge_init(false, &proxy);
    cntrl_inf inf = (cntrl_inf){
        .cntrl_init = cntrl_eigenaxis_init,
        .cntrl_update = cntrl_eigenaxis_update,
        .cntrl_teardown = cntrl_eigenaxis_teardown,
        .cntrl_output = cntrl_eigenaxis_output};
    bridge.inf = &inf;

    std::thread thread_physics(PhysicsPipeline, &sim_properties, 0.01);         // update at 100 Hz
    std::thread thread_control(ControlPipeline, &sim_properties, &bridge, 0.1); // update at 10 Hz
    std::thread thread_render(RenderPipeline, &sim_properties, 1.0f / 60);      // update at 60 FPS

    thread_physics.join();
    thread_control.join();
    thread_render.join();

    return EXIT_SUCCESS;
}