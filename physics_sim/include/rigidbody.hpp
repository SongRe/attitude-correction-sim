#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

extern "C"
{
#include "vmath.h"
#include "mmath.h"
#include "qmath.h"
}

struct Rigidbody
{
    quat attit;
    vec3 ang_vel;
    mat3 iner_tensor;

    void UpdateAttitude(double timestep);
    void UpdateAngVel(const vec3& moment, double timestep);
};

#endif