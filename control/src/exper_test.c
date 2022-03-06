// Validating the interfaces can work correctly together

#include <stdio.h>

#include "control_bridge.h"
#include "control_proxy.h"
#include "control_test.h"

#include "vmath.h"
#include "qmath.h"

#include <pthread.h>

int main()
{
    // attitude data

    quat comm_attit = quat_init(5, 0, 0, 5);
    vec3 comm_ang_vel = vec3_init(0, 5, 0);

    rbody_data_ref comm_rbody = (rbody_data_ref){
        .attit = &comm_attit,
        .ang_vel = &comm_ang_vel};

    quat curr_attit = quat_init(1, 1, 0, 0);
    vec3 curr_ang_vel = vec3_init(0, 1, 0);

    rbody_data_ref curr_rbody = (rbody_data_ref){
        .attit = &curr_attit,
        .ang_vel = &curr_ang_vel};

    vec3 cntrl_mom = vec3_init(0, 0, 0);

    // protection for the attitude data
    pthread_mutex_t comm_rbody_protect;
    pthread_mutex_t curr_rbody_protect;
    pthread_mutex_t cntrl_mom_protect;

    pthread_mutex_init(&comm_rbody_protect, NULL);
    pthread_mutex_init(&curr_rbody_protect, NULL);
    pthread_mutex_init(&cntrl_mom_protect, NULL);

    // thread safe protection for interface implementations to pull data
    cntrl_proxy proxy = cntrl_proxy_init(&comm_rbody, &curr_rbody, &cntrl_mom);
    cntrl_proxy_add_protect(&proxy, &comm_rbody_protect, &curr_rbody_protect, &cntrl_mom_protect);

    // sets up the interface abstraction to use the test controller
    cntrl_bridge bridge = cntrl_bridge_init(false, &proxy);
    cntrl_inf inf = (cntrl_inf){
        .cntrl_init = cntrl_test_init,
        .cntrl_update = cntrl_test_update,
        .cntrl_teardown = cntrl_test_teardown,
        .cntrl_output = cntrl_test_output};
    bridge.inf = &inf;

    // test out the method implementation (delegation to the interface)
    cntrl_bridge_inf_delegate_init(&bridge);
    cntrl_bridge_inf_delegate_update(&bridge); // this should pull from the observer
    printf("Output data from delegated output:%s\n", (const char *)cntrl_bridge_inf_delegate_output(&bridge));
    cntrl_bridge_inf_delegate_teardown(&bridge);

    // this should have an assertion fail as implementation not assigned to interface
    cntrl_bridge_inf_delegate_reset(&bridge);

    return 0;
}