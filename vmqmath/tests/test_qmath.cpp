#define CONFIG_CATCH_MAIN

#include <catch2/catch_all.hpp>
#include <string.h>
#include <iostream>

extern "C"
{
#include "qmath.h"
}

using namespace std;

void quat_print(quat q)
{
    cout << "[ " << q.w
         << "\t" << q.v.x << "\t" << q.v.y << "\t" << q.v.z << " ]" << endl;
}

void vec3_print(vec3 v)
{
    cout << "[ " << v.x << "\t" << v.y << "\t" << v.z << " ]" << endl;
}

TEST_CASE("init", "[quat]")
{
    quat q_test = quat_init(1, 2, 3, 4);
    quat q_expect = (quat){
        .w = 1,
        .v = vec3_init(2, 3, 4)};

    REQUIRE(memcmp(&q_test, &q_expect, sizeof(quat)) == 0);
}

TEST_CASE("add", "[quat]")
{
    quat q1 = quat_init(1, 2, 3, 4);
    quat q2 = quat_init(5, 6, 7, 8);

    quat q_test = quat_add(&q1, &q2);
    quat q_expect = quat_init(6, 8, 10, 12);

    REQUIRE(memcmp(&q_test, &q_expect, sizeof(quat)) == 0);
}

TEST_CASE("multiplication", "[quat]")
{
    quat q1 = quat_init(1, 2, 3, 4);
    quat q2 = quat_init(5, 6, 7, 8);

    quat q_test = quat_mul(&q1, &q2);
    quat q_expect = quat_init(-60, 12, 30, 24);

    // quat_print(q_test);

    REQUIRE(memcmp(&q_test, &q_expect, sizeof(quat)) == 0);
}

TEST_CASE("scalar multiplication", "[quat]")
{
    quat q = quat_init(1, 2, 3, 4);
    double s = 2.5;

    quat q_test = quat_smul(&q, s);
    quat q_expect = quat_init(2.5, 5, 7.5, 10);

    REQUIRE(memcmp(&q_test, &q_expect, sizeof(quat)) == 0);
}

TEST_CASE("magnitude", "[quat]")
{
    quat q = quat_init(1, 2, 3, 4);

    double mag = quat_mag(&q);
    REQUIRE(mag == sqrt(30));
}

TEST_CASE("normalized", "[quat]")
{
    quat q = quat_init(1, 2, 3, 4);

    quat q_test = quat_norm(&q);
    quat q_expect = quat_init(1 / sqrt(30),
                              2 / sqrt(30),
                              3 / sqrt(30),
                              4 / sqrt(30));

    REQUIRE(memcmp(&q_test, &q_expect, sizeof(quat)) == 0);
}

// located after normalization testing
TEST_CASE("from axis", "[quat]")
{
    vec3 axis = vec3_init(1, 2, 3);

    quat q_test_1 = quat_from_axis(&axis, M_PI / 6); // 30 degrees

    vec3 normed_axis = vec3_norm(&axis);
    quat q_test_2 = quat_from_axis(&normed_axis, M_PI / 6);

    const double real_const = cos(0.5 * M_PI / 6);
    const double imag_const = sin(0.5 * M_PI / 6);

    quat q_expect = quat_init(real_const,
                              imag_const * 1 / sqrt(14),
                              imag_const * 2 / sqrt(14),
                              imag_const * 3 / sqrt(14));

    REQUIRE(memcmp(&q_test_1, &q_expect, sizeof(quat)) == 0);
    REQUIRE(memcmp(&q_test_1, &q_test_2, sizeof(quat)) == 0);

    REQUIRE(quat_mag(&q_test_1) == 1); // quat_from_axis produces a quaternion for rotation purposes
    REQUIRE(quat_mag(&q_test_2) == 1);
}

TEST_CASE("dot product", "[quat]")
{
    quat q1 = quat_init(1, 2, 3, 4);
    quat q2 = quat_init(5, 6, 7, 8);

    double dot = quat_dot(&q1, &q2);
    REQUIRE(dot == 70);
}

TEST_CASE("conjugate", "[quat]")
{
    quat q = quat_init(1, 2, 3, 4);

    quat q_test = quat_conj(&q);
    quat q_expect = quat_init(1, -2, -3, -4);

    REQUIRE(memcmp(&q_test, &q_expect, sizeof(quat)) == 0);
}

TEST_CASE("inverse", "[quat]")
{
    quat q = quat_init(1, 2, 3, 4);

    quat q_inv = quat_inv(&q);
    quat q_real_result = quat_mul(&q_inv, &q);

    // needs decimal point to ensure values are considered doubles
    quat q_inv_expect = quat_init(1.0 / 30, -2.0 / 30, -3.0 / 30, -4.0 / 30);
    quat q_real_result_expect = quat_init(1, 0, 0, 0);

    REQUIRE(memcmp(&q_inv, &q_inv_expect, sizeof(quat)) == 0);
    REQUIRE(memcmp(&q_real_result, &q_real_result_expect, sizeof(quat)) == 0);
}

TEST_CASE("rotate vector around axis", "[quat]")
{
    vec3 axis = vec3_init(1, 1, 1);

    quat rot_one_third = quat_from_axis(&axis, 2 * M_PI / 3);
    quat rot_two_third = quat_from_axis(&axis, 4 * M_PI / 3);
    quat rot_three_third = quat_from_axis(&axis, 2 * M_PI);

    quat rot_sequential = quat_mul(&rot_one_third, &rot_one_third);

    vec3 point = vec3_init(2, 0, 0);

    vec3 point_rot_one_third = quat_rot_vec(&rot_one_third, &point);
    vec3 point_rot_two_third = quat_rot_vec(&rot_two_third, &point);
    vec3 point_rot_three_third = quat_rot_vec(&rot_three_third, &point);

    vec3 point_rot_sequential = quat_rot_vec(&rot_sequential, &point);

    // due to floating point roundoff errors it will not become zero

    // checks the first rotation (120 degrees clockwise)
    REQUIRE(abs(point_rot_one_third.x) < 1e-10);
    REQUIRE(point_rot_one_third.y == 2.0);
    REQUIRE(abs(point_rot_one_third.z) < 1e-10);

    // checks the second rotation (240 degrees clockwise)
    REQUIRE(abs(point_rot_two_third.x) < 1e-10);
    REQUIRE(abs(point_rot_two_third.y) < 1e-10);
    REQUIRE(point_rot_two_third.z == 2.0);

    // checks the third rotation (360 degrees clockwise)
    REQUIRE(point_rot_three_third.x == 2.0);
    REQUIRE(abs(point_rot_three_third.y) < 1e-10);
    REQUIRE(abs(point_rot_three_third.z) < 1e-10);

    // checks the sequential rotations (2 chained 120 degree rotations)
    REQUIRE(abs(point_rot_sequential.x) < 1e-10);
    REQUIRE(abs(point_rot_sequential.y) < 1e-10);
    REQUIRE(abs(point_rot_sequential.z - 2.0) < 1e-10); // roundoff error prevented using equivalency
}

TEST_CASE("quat to vec", "[quat]")
{
    quat q = quat_init(1, 2, 3, 4);

    vec4 v_test = quat_to_vec(&q);
    vec4 v_expect = vec4_init(2, 3, 4, 1);

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec4)) == 0);
}

TEST_CASE("quat from vec", "[quat]")
{
    vec4 v = vec4_init(1, 2, 3, 4);

    quat q_test = quat_from_vec(&v);
    quat q_expect = quat_init(4, 1, 2, 3);

    REQUIRE(memcmp(&q_test, &q_expect, sizeof(vec4)) == 0);
}