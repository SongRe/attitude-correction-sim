/**
 * @file rigidbody.h
 * @author Samuel Street (planetaryeclipse@gmail.com)
 * @brief Rigidbody used for purposes of controller algorithms.
 * @version 0.1
 * @date 2022-02-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <Quaternion/Quaternion.h>
#include "vmath.h"

typedef struct Rigidbody
{
    Quaternion orient;
    
    Vec3 ang_vel;
    Mat3 iner_tensor;
} Rigidbody;

void Rigidbody_update_orient(Rigidbody* rbody, double timestep);
void Rigidbody_update_ang_vel(Rigidbody* rbody, Vec3* moment, double timestep);

#endif