#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

extern "C"
{
#include "math/vmath.h"
#include "math/mmath.h"
#include "math/qmath.h"
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