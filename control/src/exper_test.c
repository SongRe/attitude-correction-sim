// Validating the interfaces can work correctly together

#include <stdio.h>

#include "control_bridge.h"
#include "control_proxy.h"
#include "control_test.h"

#include "math/vmath.h"
#include "math/qmath.h"

#include <pthread.h>

int main()
{
    // attitude data
    att_cntrl_data comm_data = (att_cntrl_data){
        .attit = quat_init(5, 0, 0, 5),
        .ang_vel = vec3_init(0, 5, 0)};
    att_cntrl_data curr_data = (att_cntrl_data){
        .attit = quat_init(1, 1, 0, 0),
        .ang_vel = vec3_init(0, 1, 0)};

    // protection for the attitude data
    pthread_mutex_t comm_protect;
    pthread_mutex_init(&comm_protect, NULL);
    pthread_mutex_t curr_protect;
    pthread_mutex_init(&curr_protect, NULL);

    // thread safe protection for interface implementations to pull data
    cntrl_proxy proxy = cntrl_proxy_init(&comm_data, &curr_data);
    cntrl_proxy_init_protect(&proxy, &comm_protect, &curr_protect);

    // sets up the interface abstraction to use the test controller
    cntrl_bridge bridge = cntrl_bridge_init(10, false, &proxy);
    cntrl_inf inf = (cntrl_inf){
        .cntrl_init = cntrl_test_init,
        .cntrl_update = cntrl_test_update,
        .cntrl_teardown = cntrl_test_teardown,
        .cntrl_output = cntrl_test_output
    };
    bridge.inf = &inf;

    // test out the method implementation (delegation to the interface)
    cntrl_bridge_inf_delegate_init(&bridge);
    cntrl_bridge_inf_delegate_update(&bridge); // this should pull from the observer
    printf("Output data from delegated output:%s\n", (const char*)cntrl_bridge_inf_delegate_output(&bridge));
    cntrl_bridge_inf_delegate_teardown(&bridge);

    // this should have an assertion fail as implementation not assigned to interface
    cntrl_bridge_inf_delegate_reset(&bridge);

    return 0;
}