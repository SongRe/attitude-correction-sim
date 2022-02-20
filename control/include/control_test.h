#ifndef CONTROL_TEST_H
#define CONTROL_TEST_H

// simply a test file to ensure the interfaces are working correctly together

#include "control_proxy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void cntrl_test_init(const cntrl_proxy *proxy, void **data)
{
    printf("cntrl_test_init called, will allocate data\n");

    char msg[27] = "Allocated buffer contents!";
    *data = malloc(27);
    memcpy(*data, msg, 27);
}

void cntrl_test_update(const cntrl_proxy *proxy, void **data)
{
    printf("cntrl_test_update called\n");

    att_cntrl_data comm;
    cntrl_proxy_pull_comm(proxy, &comm);

    printf("comm quat: w:%f, x:%f, y:%f, z:%f\n", comm.attit.w, comm.attit.v.x, comm.attit.v.y, comm.attit.v.z);
    printf("comm ang vel: x:%f, y:%f, z:%f\n", comm.ang_vel.x, comm.ang_vel.y, comm.ang_vel.z);
}

void cntrl_test_teardown(const cntrl_proxy *proxy, void **data)
{
    printf("cntrl_test_teardown called, will free data\n");

    free(*data);
}

void *cntrl_test_output(const cntrl_proxy *proxy, void **data)
{
    return *(data);
}

#endif