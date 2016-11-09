/*
 * \file test_local_param_quat.cpp
 *
 *  Created on: Feb 19, 2016
 *      author: jsola
 */

#include "utils_gtest.h"


#include "../src/local_parametrization_quaternion.h"
#include "../src/local_parametrization_homogeneous.h"
#include "../src/rotations.h"

#include "../src/wolf.h"

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


using namespace Eigen;
using namespace std;
using namespace wolf;

TEST(TestLocalParametrization, QuaternionLocal)
{
    // Storage
    VectorXs x_storage(22);
    MatrixXs M_storage(1,12); // matrix dimensions do not matter, just storage size.
    x_storage.setRandom();
    M_storage.setZero();

    // QUATERNION ------------------------------------------
    Map<VectorXs> q(&x_storage(0),4);
    q.normalize();

    Map<VectorXs> da(&x_storage(4),3);
    da /= 10.0;
    Map<VectorXs> qo_m(&x_storage(7),4);
    Map<MatrixXs> J(&M_storage(0,0),4,3);
    MatrixXs J_num(4,3);

    LocalParametrizationQuaternion<DQ_LOCAL> Qpar_loc;

    Map<const VectorXs> q_m(q.data(),4);
    Map<const VectorXs> da_m(da.data(),3);

    Qpar_loc.plus(q_m,da_m,qo_m);

    ASSERT_DOUBLE_EQ(qo_m.norm(), 1);

    Quaternions qref = Map<Quaternions>(q.data()) * wolf::v2q(da);
    ASSERT_TRUE(qo_m.isApprox( qref.coeffs() ) );

    Qpar_loc.computeJacobian(q_m,J);

    JAC_NUMERIC(Qpar_loc, q_m, J_num, 1e-9)

    ASSERT_NEAR((J-J_num).norm(), 0, 1e-6);

}

TEST(TestLocalParametrization, QuaternionGlobal)
{
    // Storage
    VectorXs x_storage(22);
    MatrixXs M_storage(1,12); // matrix dimensions do not matter, just storage size.
    x_storage.setRandom();
    M_storage.setZero();

    // QUATERNION ------------------------------------------
    Map<VectorXs> q(&x_storage(0),4);
    q.normalize();

    Map<VectorXs> da(&x_storage(4),3);
    da /= 10.0;
    Map<VectorXs> qo_m(&x_storage(7),4);
    Map<MatrixXs> J(&M_storage(0,0),4,3);
    MatrixXs J_num(4,3);

    LocalParametrizationQuaternion<DQ_GLOBAL> Qpar_glob;

    Map<const VectorXs> q_m(q.data(),4);
    Map<const VectorXs> da_m(da.data(),3);

    Qpar_glob.plus(q_m,da_m,qo_m);

    ASSERT_DOUBLE_EQ(qo_m.norm(), 1);

    Quaternions qref =  wolf::v2q(da) * Map<Quaternions>(q.data());
    ASSERT_TRUE(qo_m.isApprox( qref.coeffs() ) );

    Qpar_glob.computeJacobian(q_m,J);

    JAC_NUMERIC(Qpar_glob, q_m, J_num, 1e-9)

    ASSERT_NEAR((J-J_num).norm(), 0, 1e-6);

}

TEST(TestLocalParametrization, Homogeneous)
{
    // Storage
    VectorXs x_storage(22);
    MatrixXs M_storage(1,12); // matrix dimensions do not matter, just storage size.

    // HOMOGENEOUS ----------------------------------------
    Map<VectorXs> h(&x_storage(11),4);
    h.setRandom();
    Map<VectorXs> d(&x_storage(15),3);
    d << .1,.2,.3;
    Map<VectorXs> ho_m(&x_storage(18),4);
    Map<MatrixXs> J(&M_storage(0,0),4,3);
    MatrixXs J_num(4,3);

    LocalParametrizationHomogeneous Hpar;
    Map<const VectorXs> h_m(h.data(),4);
    Map<const VectorXs> d_m(d.data(),3);

    Hpar.plus(h_m,d_m,ho_m);

    ASSERT_DOUBLE_EQ(ho_m.norm(), h.norm());

    Hpar.computeJacobian(h_m,J);

    JAC_NUMERIC(Hpar, h_m, J_num, 1e-9)

    ASSERT_NEAR((J-J_num).norm(), 0, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

