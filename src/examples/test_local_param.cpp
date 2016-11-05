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
{    VectorXs dv(3); \
    Map<const VectorXs> _dv (dv.data(), 3); \
    VectorXs xo(3); \
    Map<VectorXs> _xo (xo.data(), 4); \
    for (int i = 0; i< 3; i++) {\
        dv.setZero();\
        dv(i) = dx;\
        T.plus(_x0, _dv, _xo);\
        _J.col(i) = (_xo - _x0)/dx;  }}





int main(){

    using namespace Eigen;
    using namespace std;
    using namespace wolf;

    cout << endl << endl;
    cout << "=========== Test Local Parametrization ==========" << endl;
    cout << "====   Quaternion and Homogeneous 3D vector  ====" << endl << endl;

    // test result
    bool all_tests_passed = true;
    bool pass;

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
    MatrixXs J_num(4,3);

    cout << "\n--------------- QUATERNION plus() --------------- " << endl;
    cout << "Initial values:" << endl;
    cout << "q  = " << q.transpose() << "   with norm = " << q.norm() << "\nda = " << da.transpose() << endl;
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;

    LocalParametrizationQuaternion<DQ_GLOBAL> Qpar;
    LocalParametrizationQuaternion<DQ_LOCAL> Qpar_loc;

    // Global --------------------
    cout << "\nGLOBAL D_QUAT" << endl;
    cout << "Results:" << endl;
    Map<const VectorXs> q_m(q.data(),4);
    Map<const VectorXs> da_m(da.data(),3);
    Qpar.plus(q_m,da_m,qo);
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;
    pass = (q_m.norm()-qo.norm()) < 1e-9;
    all_tests_passed = all_tests_passed && pass;
    cout << "-------------------- Norm test " << (pass ? "PASSED" : "FAIL") << endl;

    Qpar.computeJacobian(q_m,J);
    cout << " J = \n" << J << endl;

    JAC_NUMERIC(Qpar, q_m, J_num, 1e-9)
    cout << " J_num = \n" << J_num << endl;

    pass = (J-J_num).isMuchSmallerThan(1,1e-6);
    all_tests_passed = all_tests_passed && pass;
    cout << "-------------------- Jacobians test " << (pass ? "PASSED" : "FAIL") << endl;

    // Local -------------------------
    cout << "\nLOCAL D_QUAT" << endl;
    cout << "Results:" << endl;
    Qpar_loc.plus(q_m,da_m,qo);
    cout << "qo = " << qo.transpose() << "   with norm = " << qo.norm() << endl;
    pass = (q_m.norm()-qo.norm()) < 1e-9;
    all_tests_passed = all_tests_passed && pass;
    cout << "-------------------- Norm test " << (pass ? "PASSED" : "FAIL") << endl;

    Qpar_loc.computeJacobian(q_m,J);
    cout << " J = \n" << J << endl;

    JAC_NUMERIC(Qpar_loc, q_m, J_num, 1e-9)
    cout << " J_num = \n" << J_num << endl;

    pass = (J-J_num).isMuchSmallerThan(1,1e-6);
    all_tests_passed = all_tests_passed && pass;
    cout << "-------------------- Jacobians test " << (pass ? "PASSED" : "FAIL") << endl;


    // HOMOGENEOUS ----------------------------------------
    cout << "\n--------------- HOMOGENEOUS plus() --------------- " << endl;
    Map<VectorXs> h(&x(11),4);
    h.setRandom();
    Map<VectorXs> d(&x(15),3);
    d << .1,.2,.3;
    Map<VectorXs> ho(&x(18),4);

    cout << "Initial values:" << endl;
    cout << "h  = " << h.transpose() << "   with norm: " << h.norm() << endl;
    cout << "d  = " << d.transpose() << endl;

    LocalParametrizationHomogeneous Hpar;
    Map<const VectorXs> h_m(h.data(),4);
    Map<const VectorXs> d_m(d.data(),3);

    cout << "\nResults:" << endl;
    Hpar.plus(h_m,d_m,ho);
    cout << "ho = " << ho.transpose() << "   with norm: " << ho.norm() << endl;
    pass = (h_m.norm()-ho.norm()) < 1e-9;
    all_tests_passed = all_tests_passed && pass;
    cout << "-------------------- Norm test " << (pass ? "PASSED" : "FAIL") << endl;

    Hpar.computeJacobian(h_m,J);
    cout << " J = \n" << J << endl;

    JAC_NUMERIC(Hpar, h_m, J_num, 1e-9)
    cout << " J_num = \n" << J_num << endl;

    pass = (J-J_num).isMuchSmallerThan(1,1e-6);
    all_tests_passed = all_tests_passed && pass;
    cout << "-------------------- Jacobians test " << (pass ? "PASSED" : "FAIL") << endl;

    cout << endl << "-------------------- All tests " << (all_tests_passed ? "PASSED" : "FAIL") << endl;

    if (!all_tests_passed)
        return -1;
    return 0;
}
