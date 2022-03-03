#define CONFIG_CATCH_MAIN

#include <catch2/catch_all.hpp>
#include <string.h>
#include <iostream>

extern "C"
{
#include "math/vmath.h"
}

TEST_CASE("init", "[vec2]")
{
    vec2 v_test = vec2_init(1, 2);
    vec2 v_expect = (vec2){
        .x = 1,
        .y = 2};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec2)) == 0);
}

TEST_CASE("add", "[vec2]")
{
    vec2 v1 = (vec2){
        .x = 1,
        .y = 2};
    vec2 v2 = (vec2){
        .x = 3,
        .y = -4};

    vec2 v_test = vec2_add(&v1, &v2);
    vec2 v_expect = (vec2){
        .x = 4,
        .y = -2};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec2)) == 0);
}

TEST_CASE("scalar multiplication", "[vec2]")
{
    vec2 v = (vec2){
        .x = 1,
        .y = 2};

    double s = 2.5;

    vec2 v_test = vec2_smul(&v, s);
    vec2 v_expect = (vec2){
        .x = 2.5,
        .y = 5};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec2)) == 0);
}

TEST_CASE("magnitude", "[vec2]")
{
    vec2 v = (vec2){
        .x = 3,
        .y = 4};

    double mag = vec2_mag(&v);
    REQUIRE(mag == 5);
}

TEST_CASE("normalized", "[vec2]")
{
    vec2 v = (vec2){
        .x = 3,
        .y = 4};

    vec2 v_norm = vec2_norm(&v);
    vec2 v_expect = (vec2){
        .x = 0.6,
        .y = 0.8};

    REQUIRE(memcmp(&v_norm, &v_expect, sizeof(vec2)) == 0);
}

TEST_CASE("dot product", "[vec2]")
{
    vec2 v1 = (vec2){
        .x = 1,
        .y = -1};

    vec2 v2 = (vec2){
        .x = 2,
        .y = 3};

    double dot = vec2_dot(&v1, &v2);
    REQUIRE(dot == -1);
}

TEST_CASE("init", "[vec3]")
{
    vec3 v_test = vec3_init(1, 2, 3);
    vec3 v_expect = (vec3){
        .x = 1,
        .y = 2,
        .z = 3};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec3)) == 0);
}

TEST_CASE("add", "[vec3]")
{
    vec3 v1 = (vec3){
        .x = 1,
        .y = 2,
        .z = 3};
    vec3 v2 = (vec3){
        .x = 3,
        .y = -4,
        .z = 5};

    vec3 v_test = vec3_add(&v1, &v2);
    vec3 v_expect = (vec3){
        .x = 4,
        .y = -2,
        .z = 8};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec3)) == 0);
}

TEST_CASE("scalar multiplication", "[vec3]")
{
    vec3 v = (vec3){
        .x = 1,
        .y = 2,
        .z = 3};

    double s = 2.5;

    vec3 v_test = vec3_smul(&v, s);
    vec3 v_expect = (vec3){
        .x = 2.5,
        .y = 5,
        .z = 7.5};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec3)) == 0);
}

TEST_CASE("magnitude", "[vec3]")
{
    vec3 v = (vec3){
        .x = 1,
        .y = 2,
        .z = 3};

    double mag = vec3_mag(&v);
    REQUIRE(mag == sqrt(14));
}

TEST_CASE("normalized", "[vec3]")
{
    vec3 v = (vec3){
        .x = 1,
        .y = 2,
        .z = 3};

    vec3 v_norm = vec3_norm(&v);
    vec3 v_expect = (vec3){
        .x = 1 / sqrt(14),
        .y = 2 / sqrt(14),
        .z = 3 / sqrt(14)};

    REQUIRE(memcmp(&v_norm, &v_expect, sizeof(vec3)) == 0);
}

TEST_CASE("dot product", "[vec3]")
{
    vec3 v1 = (vec3){
        .x = 1,
        .y = 2,
        .z = 3};

    vec3 v2 = (vec3){
        .x = -3,
        .y = -2,
        .z = -1};

    double dot = vec3_dot(&v1, &v2);
    REQUIRE(dot == -10);
}

TEST_CASE("cross product", "[vec3]")
{
    vec3 v1 = (vec3){
        .x = 1,
        .y = 2,
        .z = 3};

    vec3 v2 = (vec3){
        .x = -3,
        .y = -2,
        .z = -1};

    vec3 v_test = vec3_cross(&v1, &v2);
    vec3 v_expect = (vec3){
        .x = 4,
        .y = -8,
        .z = 4};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec3)) == 0);
}

TEST_CASE("init", "[ve42]")
{
    vec4 v_test = vec4_init(1, 2, 3, 4);
    vec4 v_expect = (vec4){
        .x = 1,
        .y = 2,
        .z = 3,
        .w = 4};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec4)) == 0);
}

TEST_CASE("add", "[vec4]")
{
    vec4 v1 = (vec4){
        .x = 1,
        .y = 2,
        .z = 3,
        .w = 4};
    vec4 v2 = (vec4){
        .x = 3,
        .y = -4,
        .z = 3,
        .w = -4};

    vec4 v_test = vec4_add(&v1, &v2);
    vec4 v_expect = (vec4){
        .x = 4,
        .y = -2,
        .z = 6,
        .w = 0};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec4)) == 0);
}

TEST_CASE("scalar multiplication", "[vec4]")
{
    vec4 v = (vec4){
        .x = 1,
        .y = 2,
        .z = 3,
        .w = 4};

    double s = 2.5;

    vec4 v_test = vec4_smul(&v, s);
    vec4 v_expect = (vec4){
        .x = 2.5,
        .y = 5,
        .z = 7.5,
        .w = 10};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec4)) == 0);
}

TEST_CASE("magnitude", "[vec4]")
{
    vec4 v = (vec4){
        .x = 1,
        .y = 2,
        .z = 3,
        .w = 4};

    double mag = vec4_mag(&v);
    REQUIRE(mag == sqrt(30));
}

TEST_CASE("normalized", "[vec4]")
{
    vec4 v = (vec4){
        .x = 1,
        .y = 2,
        .z = 3,
        .w = 4};

    vec4 v_norm = vec4_norm(&v);
    vec4 v_expect = (vec4){
        .x = 1 / sqrt(30),
        .y = 2 / sqrt(30),
        .z = 3 / sqrt(30),
        .w = 4 / sqrt(30)};

    REQUIRE(memcmp(&v_norm, &v_expect, sizeof(vec4)) == 0);
}

TEST_CASE("dot product", "[vec4]")
{
    vec4 v1 = (vec4){
        .x = 1,
        .y = 2,
        .z = 3,
        .w = 4};

    vec4 v2 = (vec4){
        .x = -4,
        .y = -3,
        .z = -2,
        .w = -1};

    double dot = vec4_dot(&v1, &v2);
    REQUIRE(dot == -20);
}

/*
void mat3_print(mat3 m)
{
    std::cout << "[\t" << m.m1[0] << "\t" << m.m1[1] << "\t" << m.m1[2] << "\n\t"
              << m.m2[0] << "\t" << m.m2[1] << "\t" << m.m2[2] << "\n\t"
              << m.m3[0] << "\t" << m.m3[1] << "\t" << m.m3[2] << " ]" << std::endl;
}

void mat4_print(mat4 m)
{
    std::cout << "[\t" << m.m1[0] << "\t" << m.m1[1] << "\t" << m.m1[2] << "\t" << m.m1[3] << "\n\t"
              << m.m2[0] << "\t" << m.m2[1] << "\t" << m.m2[2] << "\t" << m.m2[3] << "\n\t"
              << m.m3[0] << "\t" << m.m3[1] << "\t" << m.m3[2] << "\t" << m.m3[3] << "\n\t"
              << m.m4[0] << "\t" << m.m4[1] << "\t" << m.m4[2] << "\t" << m.m4[3] << " ]" << std::endl;
}
*/