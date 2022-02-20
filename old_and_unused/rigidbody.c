/**
 * @file rigidbody.c
 * @author Samuel Street (planetaryeclipse@gmail.com)
 * @brief Rigidbody used for purposes of controller algorithms.
 * @version 0.1
 * @date 2022-02-14
 *
 * @copyright Copyright (c) 2022
 *
 * The following code below uses the Euler method for updating values. In future,
 * likely the modified Euler method or Runge-Kutta should be implemented instead.
 */

#include "rigidbody.h"

#include <Quaternion/Quaternion.h>
#include "vmath.h"

#include<stdio.h>
#include<stdlib.h>

void Rigidbody_update_orient(Rigidbody* rbody, double timestep)
{
    Vec3 axis_rotation = Vec3_norm(&rbody->ang_vel);

    Quaternion rotate;
    Quaternion_fromAxisAngle(&axis_rotation.x,
                             Vec3_mag(&rbody->ang_vel) * timestep,
                             &rotate);
    Quaternion_multiply(&rotate, &rbody->orient, &rbody->orient);
}

void Rigidbody_update_ang_vel(Rigidbody* rbody, Vec3* moment, double timestep)
{
    Vec3 impulse = Vec3_scalar_prod(moment, timestep);
    rbody->ang_vel = Vec3_add(&rbody->ang_vel, &impulse);
}