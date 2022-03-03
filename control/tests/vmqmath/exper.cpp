#include <stdlib.h>
#include <iostream>

extern "C"
{
#include "math/mmath.h"
}

using namespace std;

void mat3_print(mat3 m)
{
    std::cout << "[\t" << m.m1[0] << "\t" << m.m1[1] << "\t" << m.m1[2] << "\n\t"
              << m.m2[0] << "\t" << m.m2[1] << "\t" << m.m2[2] << "\n\t"
              << m.m3[0] << "\t" << m.m3[1] << "\t" << m.m3[2] << " ]" << std::endl;
}

/*
[       -1.39583        0.458333        -0.0625
        1.125   -0.25   0.125
        -0.0625 0.125   -0.0625 ]
*/

int main()
{
    mat3 mt = (mat3){
        .m1 = {0, 1, 2},
        .m2 = {3, 4, 5},
        .m3 = {6, 7, -8}};

    cout << "Original matrix:" << endl;
    mat3_print(mt);

    mat3 adjoint = mat3_adj(&mt);
    cout << "Adjoint matrix:" << endl;
    mat3_print(adjoint);

    double det = mat3_det(&mt);
    mat3 inv = mat3_smul(&adjoint, 1 / det);

    cout << "Inverse matrix:" << endl;
    mat3_print(inv);

    mat3 iden = mat3_mul(&inv, &mt);
    cout << "Identity matrix:" << endl;
    mat3_print(iden);

    cout << "----------" << endl;

    mat3 calc_inv = mat3_inv(&mt);
    mat3_print(calc_inv);

    return EXIT_SUCCESS;
}