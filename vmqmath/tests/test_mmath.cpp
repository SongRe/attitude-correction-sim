#define CONFIG_CATCH_MAIN

#include <catch2/catch_all.hpp>
#include <string.h>
#include <iostream>

extern "C"
{
#include "mmath.h"
}

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

TEST_CASE("init", "[mat2]")
{
    double m1[2] = {0, 1};
    double m2[2] = {2, 3};
    mat2 mt_test = mat2_init(m1, m2);

    mat2 mt_expect = (mat2){
        .m1 = {0, 1},
        .m2 = {2, 3}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat2)) == 0);
}

TEST_CASE("add", "[mat2]")
{
    mat2 mt_1 = (mat2){
        .m1 = {0, 1},
        .m2 = {1, 0}};

    mat2 mt_2 = (mat2){
        .m1 = {-1, 2},
        .m2 = {2, -1}};

    mat2 mt_test = mat2_add(&mt_1, &mt_2);

    mat2 mt_expect = (mat2){
        .m1 = {-1, 3},
        .m2 = {3, -1}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat2)) == 0);
}

TEST_CASE("multiplication", "[mat2]")
{
    mat2 mt_1 = (mat2){
        .m1 = {0, 1},
        .m2 = {2, 3}};

    mat2 mt_2 = (mat2){
        .m1 = {4, 5},
        .m2 = {6, 7}};

    mat2 mt_test = mat2_mul(&mt_1, &mt_2);

    mat2 mt_expect = (mat2){
        .m1 = {6, 7},
        .m2 = {26, 31}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat2)) == 0);
}

TEST_CASE("scalar multiplication", "[mat2]")
{
    mat2 mt = (mat2){
        .m1 = {0, 1},
        .m2 = {2, 3}};

    double s = 2.5;

    mat2 mt_test = mat2_smul(&mt, s);

    mat2 mt_expect = (mat2){
        .m1 = {0, 2.5},
        .m2 = {5, 7.5}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat2)) == 0);
}

TEST_CASE("determinant", "[mat2]")
{
    mat2 mt = (mat2){
        .m1 = {0, 1},
        .m2 = {2, 3}};

    double det = mat2_det(&mt);
    REQUIRE(det == -2);
}

TEST_CASE("inverse", "[mat2]")
{
    mat2 mt = (mat2){
        .m1 = {0, 1},
        .m2 = {2, 3}};

    mat2 mt_inv = mat2_inv(&mt);
    mat2 mt_test = mat2_mul(&mt_inv, &mt);

    mat2 mt_iden = (mat2){
        .m1 = {1, 0},
        .m2 = {0, 1}};

    REQUIRE(memcmp(&mt_test, &mt_iden, sizeof(mat2)) == 0);
}

TEST_CASE("vector multiplication", "[mat2]")
{
    mat2 mt = (mat2){
        .m1 = {0, 1},
        .m2 = {2, 3}};

    vec2 v = (vec2){
        .x = 1,
        .y = 2};

    vec2 v_test = mat2_vmul(&mt, &v);
    vec2 v_expect = (vec2){
        .x = 2,
        .y = 8};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec2)) == 0);
}

TEST_CASE("init", "[mat3]")
{
    double m1[3] = {0, 1, 2};
    double m2[3] = {3, 4, 5};
    double m3[3] = {6, 7, 8};
    mat3 mt_test = mat3_init(m1, m2, m3);

    mat3 mt_expect = (mat3){
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, 8}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat3)) == 0);
}

TEST_CASE("add", "[mat3]")
{
    mat3 mt_1 = (mat3){
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, 8}};

    mat3 mt_2 = (mat3){
        .m1 = {9, 10, 11},
        .m2 = {12, 13, 14},
        .m3 = {15, 16, 17}};

    mat3 mt_test = mat3_add(&mt_1, &mt_2);

    mat3 mt_expect = (mat3){
        .m1 = {9, 11, 13},
        .m2 = {15, 17, 19},
        .m3 = {21, 23, 25}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat3)) == 0);
}

TEST_CASE("multiplication", "[mat3]")
{
    mat3 mt_1 = (mat3){
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, 8}};

    mat3 mt_2 = (mat3){
        .m1 = {9, 10, 11},
        .m2 = {12, 13, 14},
        .m3 = {15, 16, 17}};

    mat3 mt_test = mat3_mul(&mt_1, &mt_2);

    mat3 mt_expect = (mat3){
        .m1 = {42, 45, 48},
        .m2 = {150, 162, 174},
        .m3 = {258, 279, 300}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat3)) == 0);
}

TEST_CASE("scalar multiplication", "[mat3]")
{
    mat3 mt = (mat3){
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, 8}};

    double s = 2.5;

    mat3 mt_test = mat3_smul(&mt, s);

    mat3 mt_expect = (mat3){
        .m1 = {0, 2.5, 5},
        .m2 = {7.5, 10, 12.5},
        .m3 = {15, 17.5, 20}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat3)) == 0);
}

TEST_CASE("determinant", "[mat3]")
{
    mat3 mt = {
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, -8}};

    double det = mat3_det(&mt);

    REQUIRE(det == 48);
}

TEST_CASE("transpose", "[mat3]")
{
    mat3 mt = (mat3){
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, 8}};

    mat3 mt_test = mat3_transp(&mt);
    mat3 mt_expect = (mat3){
        .m1 = {0, 3, 6},
        .m2 = {1, 4, 7},
        .m3 = {2, 5, 8}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat3)) == 0);
}

TEST_CASE("adjoint", "[mat3]")
{
    mat3 mt = {
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, -8}};

    mat3 mt_test = mat3_adj(&mt);
    mat3 mt_expect = (mat3){
        .m1 = {-67, 22, -3},
        .m2 = {54, -12, 6},
        .m3 = {-3, 6, -3}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat3)) == 0);
}

TEST_CASE("inverse", "[mat3]")
{
    mat3 mt = (mat3){
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, -8}};

    mat3 mt_inv = mat3_inv(&mt);
    mat3 mt_test = mat3_mul(&mt_inv, &mt);

    mat3 mt_iden = (mat3){
        .m1 = {1, 0, 0},
        .m2 = {0, 1, 0},
        .m3 = {0, 0, 1}};

    // mat3_print(mt_iden);

    REQUIRE(memcmp(&mt_test, &mt_iden, sizeof(mat3)) == 0);
}

TEST_CASE("vector multiplication", "[mat3]")
{
    mat3 mt = (mat3){
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, 8}};

    vec3 v = (vec3){
        .x = 1,
        .y = -2,
        .z = 3};

    vec3 v_test = mat3_vmul(&mt, &v);

    vec3 v_expect = (vec3){
        .x = 4,
        .y = 10,
        .z = 16};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec3)) == 0);
}

TEST_CASE("init", "[mat4]")
{
    double m1[4] = {0, 1, 2, 3};
    double m2[4] = {4, 5, 6, 7};
    double m3[4] = {8, 9, 10, 11};
    double m4[4] = {12, 13, 14, 15};
    mat4 mt_test = mat4_init(m1, m2, m3, m4);

    mat4 mt_expect = (mat4){
        .m1 = {0, 1, 2, 3},
        .m2 = {4, 5, 6, 7},
        .m3 = {8, 9, 10, 11},
        .m4 = {12, 13, 14, 15}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat4)) == 0);
}

TEST_CASE("add", "[mat4]")
{
    mat4 m1 = (mat4){
        .m1 = {0, 1, 2, 3},
        .m2 = {4, 5, 6, 7},
        .m3 = {8, 9, 10, 11},
        .m4 = {12, 13, 14, 15}};
    mat4 m2 = (mat4){
        .m1 = {-16, 17, 18, 19},
        .m2 = {20, -21, 22, 23},
        .m3 = {24, 25, -26, 27},
        .m4 = {28, 29, 30, -31}};

    mat4 mt_test = mat4_add(&m1, &m2);
    mat4 mt_expect = (mat4){
        .m1 = {-16, 18, 20, 22},
        .m2 = {24, -16, 28, 30},
        .m3 = {32, 34, -16, 38},
        .m4 = {40, 42, 44, -16}};

    // mat4_print(mt_test);

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat4)) == 0);
}

TEST_CASE("multiplication", "[mat4]")
{
    mat4 m1 = (mat4){
        .m1 = {0, 1, 2, 3},
        .m2 = {4, 5, 6, 7},
        .m3 = {8, 9, 10, 11},
        .m4 = {12, 13, 14, 15}};
    mat4 m2 = (mat4){
        .m1 = {-16, 17, 18, 19},
        .m2 = {20, -21, 22, 23},
        .m3 = {24, 25, -26, 27},
        .m4 = {28, 29, 30, -31}};

    mat4 mt_test = mat4_mul(&m1, &m2);
    mat4 mt_expect = (mat4){
        .m1 = {152, 116, 60, -16},
        .m2 = {376, 316, 236, 136},
        .m3 = {600, 516, 412, 288},
        .m4 = {824, 716, 588, 440}};

    // mat4_print(mt_test);

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat4)) == 0);
}

TEST_CASE("scalar multiplication", "[mat4]")
{
    mat4 m = (mat4){
        .m1 = {-1, 2, 3, 4},
        .m2 = {5, -6, 7, 8},
        .m3 = {9, 10, -11, 12},
        .m4 = {13, 14, 15, -16}};
    double s = 2.5;

    mat4 mt_test = mat4_smul(&m, s);
    mat4 mt_expect = (mat4){
        .m1 = {-2.5, 5, 7.5, 10},
        .m2 = {12.5, -15, 17.5, 20},
        .m3 = {22.5, 25, -27.5, 30},
        .m4 = {32.5, 35, 37.5, -40}};

    REQUIRE(memcmp(&mt_test, &mt_expect, sizeof(mat4)) == 0);
}

TEST_CASE("vector multiplication", "[mat4]")
{
    mat4 m = (mat4){
        .m1 = {-1, 2, 3, 4},
        .m2 = {5, -6, 7, 8},
        .m3 = {9, 10, -11, 12},
        .m4 = {13, 14, 15, -16}};

    vec4 v = (vec4){
        .x = 0,
        .y = 1,
        .z = 2,
        .w = 3};

    vec4 v_test = mat4_vmul(&m, &v);
    vec4 v_expect = (vec4){
        .x = 20,
        .y = 32,
        .z = 24,
        .w = -4};

    REQUIRE(memcmp(&v_test, &v_expect, sizeof(vec4)) == 0);
}
