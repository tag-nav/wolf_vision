/**
 * \file test_loc_param_hmg.cpp
 *
 *  Created on: Nov 4, 2016
 *      \author: jsola
 */




#include "wolf.h"
#include "local_parametrization_homogeneous.h"

#include <iostream>

int main()
{
    using namespace wolf;
    using namespace Eigen;
    
    LocalParametrizationHomogeneous lh;

    Scalar dx = 1e-6;
    VectorXs h(4);
    VectorXs h0(4);
    VectorXs v(3), v0(3), dv(3);
    MatrixXs J_num(4,3), J_anal(4,3);
//    Vector4s h(4);
//    Vector4s h0(4);
//    Vector3s v(3), v0(3), dv(3);
//    Matrix<Scalar,4,3> J_num, J_anal;

//    h0.setRandom().normalized();
    h0.setRandom();

    std::cout << "h0: " << h0.transpose() << std::endl;

    Map<const VectorXs> _h0(h0.data(), 4);
    Map<const VectorXs> _dv(dv.data(), 3);
    Map<VectorXs> _h(h.data(), 4);
    Map<MatrixXs> _J_anal(J_anal.data(),4,3);

    for (int i = 0; i< 3; i++)
    {
        dv.setZero();
        dv(i) = dx;
        lh.plus(_h0, _dv, _h);
        J_num.col(i) = (h-h0)/dx;
    }

    lh.computeJacobian(_h0, _J_anal);

    std::cout << "\nJ=dh/dv(h0) analytic:\n " << J_anal << std::endl;
    std::cout << "\nJ=dh/dv(h0) numeric :\n " << J_num << std::endl;

    return 0;
}
