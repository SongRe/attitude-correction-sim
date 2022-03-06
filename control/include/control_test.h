#ifndef CONTROL_TEST_H
#define CONTROL_TEST_H

// simply a test file to ensure the interfaces are working correctly together

#include "control_proxy.h"
#include "control_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void cntrl_test_init(cntrl_proxy *proxy, void **data)
{
    printf("cntrl_test_init called -> will allocate data\n");

    char msg[27] = "Allocated buffer contents!";
    *data = malloc(27);
    memcpy(*data, msg, 27);
}

void cntrl_test_update(cntrl_proxy *proxy, void **data)
{
    printf("cntrl_test_update called\n");

    rbody_data comm_rbody_data, curr_rbody_data;
    cntrl_proxy_pull_comm_rbody(proxy, &comm_rbody_data);
    cntrl_proxy_pull_curr_rbody(proxy, &curr_rbody_data);

    printf("Commanded quaternion:\n");
    quat_print(&comm_rbody_data.attit);
    printf("Commanded angular velocity:\n");
    vec3_print(&comm_rbody_data.ang_vel);
}

void cntrl_test_teardown(cntrl_proxy *proxy, void **data)
{
    printf("cntrl_test_teardown called -> will free data\n");

    free(*data);
}

void *cntrl_test_output(cntrl_proxy *proxy, void **data)
{
    printf("cntrl_test_output called\n");
    return *(data);
}

#endif