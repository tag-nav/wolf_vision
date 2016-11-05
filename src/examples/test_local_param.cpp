/*
 * \file test_local_param_quat.cpp
 *
 *  Created on: Feb 19, 2016
 *      author: jsola
 */


#include "local_parametrization_quaternion.h"
#include "local_parametrization_homogeneous.h"

#include "wolf.h"

#include <iostream>

#define JAC_NUMERIC(T, _x0, _J, dx) \
{    Eigen::VectorXs dv(3); \
    Eigen::Map<const Eigen::VectorXs> _dv (dv.data(), 3); \
    Eigen::VectorXs xo(3); \
    Eigen::Map<Eigen::VectorXs> _xo (xo.data(), 4); \
    for (int i = 0; i< 3; i++) {\
        dv.setZero();\
        dv(i) = dx;\
        T.plus(_x0, _dv, _xo);\
        _J.col(i) = (_xo - _x0)/dx;  }}





int main(){

    using namespace Eigen;
    using namespace std;
    using namespace wolf;

    // Storage
    VectorXs x(22);
    MatrixXs M(1,12); // matrix dimensions do not matter, just storage size.

    x.setRandom();
    M.setZero();

    // QUATERNION ------------------------------------------
    Map<VectorXs> q(&x(0),4);
    q.normalize();

    Map<VectorXs> da(&x(4),3);
    da /= 10.0;
    Map<VectorXs> qo(&x(7),4);
    Map<MatrixXs> J(&M(0,0),4,3);

    cout << "Initial values:" << endl;
    cout << "q  = " << q.transpose() << "   with norm = " << q.norm() << "\nda = " << da.transpose() << endl;
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    LocalParametrizationQuaternion<DQ_GLOBAL> Qpar;
    LocalParametrizationQuaternion<DQ_LOCAL> Qpar_loc;
    bool pass;

    cout << "\nGLOBAL D_QUAT plus()" << endl;
    Map<const VectorXs> q_m(q.data(),4);
    Map<const VectorXs> da_m(da.data(),3);
    Qpar.plus(q_m,da_m,qo);
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    Qpar.computeJacobian(q_m,J);
    cout << " J = \n" << J << endl << endl;

    MatrixXs J_num(4,3);
    JAC_NUMERIC(Qpar, q_m, J_num, 1e-9)
    cout << " J_num = \n" << J_num;

    pass = (J-J_num).isMuchSmallerThan(1,1e-6);
    std::cout << "Jacobians test " << (pass ? "PASSED" : "FAIL") << std::endl;

    cout << "\nLOCAL D_QUAT plus()" << endl;
    Qpar_loc.plus(q_m,da_m,qo);
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    Qpar_loc.computeJacobian(q_m,J);
    cout << " J = " << J << endl;

    JAC_NUMERIC(Qpar_loc, q_m, J_num, 1e-9)
    cout << " J_num = \n" << J_num << endl;

    pass = (J-J_num).isMuchSmallerThan(1,1e-6);
    std::cout << "Jacobians test " << (pass ? "PASSED" : "FAIL") << std::endl;


    // HOMOGENEOUS ----------------------------------------
    cout << "\nHOMOGENEOUS plus()" << endl;
    Map<VectorXs> h(&x(11),4);
    h.setRandom();
    Map<VectorXs> d(&x(15),3);
    d << .1,.2,.3;
    Map<VectorXs> h_out(&x(18),4);

    cout << "Initial values:" << endl;
    cout << "h  = " << h.transpose() << "   with norm: " << h.norm() << endl;
    cout << "d  = " << d.transpose() << endl;

    LocalParametrizationHomogeneous Hpar;
    Map<const VectorXs> h_const(h.data(),4);
    Map<const VectorXs> d_const(d.data(),3);

    Hpar.plus(h_const,d_const,h_out);
    cout << "\nh_out = " << h_out.transpose() << "   with norm: " << h_out.norm() << endl;

    Hpar.computeJacobian(h_const,J);
    cout << " J = " << J << "\n" << endl;

    JAC_NUMERIC(Hpar, q_m, J_num, 1e-9)
    cout << " J_num = \n" << J_num << endl;

    pass = (J-J_num).isMuchSmallerThan(1,1e-6);
    std::cout << "Jacobians test " << (pass ? "PASSED" : "FAIL") << std::endl;


    return 0;
}
