/*
 * gtest_imu_tools.cpp
 *
 *  Created on: Jul 29, 2017
 *      Author: jsola
 */

#include "imu_tools.h"

#include "utils_gtest.h"


using namespace Eigen;
using namespace wolf;

TEST(IMU_tools, identity)
{
    VectorXs id_true;
    id_true.setZero(10);
    id_true(6) = 1.0;

    VectorXs id = imu::identity<Scalar>();
    ASSERT_MATRIX_APPROX(id, id_true, 1e-10);
}

TEST(IMU_tools, inverse)
{
    VectorXs d(10), id(10), iid(10), iiid(10);
    Vector4s qv;
    Scalar dt = 0.1;

    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d << 0, 1, 2, qv, 7, 8, 9;

    id   = imu::inverse(d, dt);
    imu::inverse(id, -dt, iid); // Observe -dt is reversed !!
    iiid = imu::inverse(iid, dt);

    ASSERT_MATRIX_APPROX( d,  iid, 1e-10);
    ASSERT_MATRIX_APPROX(id, iiid, 1e-10);
}

TEST(IMU_tools, compose_between)
{
    VectorXs dx1(10), dx2(10), dx3(10);
    Matrix<Scalar, 10, 1> d1, d2, d3;
    Vector4s qv;
    Scalar dt = 0.1;

    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    dx1 << 0, 1, 2, qv, 7, 8, 9;
    d1 = dx1;
    qv = (Vector4s() << 6, 5, 4, 3).finished().normalized();
    dx2 << 9, 8, 7, qv, 2, 1, 0;
    d2 = dx2;

    // combinations of templates for sum()
    imu::compose(dx1, dx2, dt, dx3);
    imu::compose(d1, d2, dt, d3);
    ASSERT_MATRIX_APPROX(d3, dx3, 1e-10);

    imu::compose(d1, dx2, dt, dx3);
    imu::compose(dx1, d2, dt, d3);
    ASSERT_MATRIX_APPROX(d3, dx3, 1e-10);

    // minus(d1, d3) should recover delta_2
    VectorXs diffx(10);
    Matrix<Scalar,10,1> diff;
    imu::between(d1, d3, dt, diff);
    ASSERT_MATRIX_APPROX(diff, d2, 1e-10);
}

TEST(IMU_tools, lift_retract)
{
    VectorXs d_min(9); d_min << 0, 1, 2, .3, .4, .5, 6, 7, 8; // use angles in the ball theta < pi

    // transform to delta
    VectorXs delta = imu::retract(d_min);

    // expected delta
    Vector3s dp = d_min.head(3);
    Quaternions dq = v2q(d_min.segment(3,3));
    Vector3s dv = d_min.tail(3);
    VectorXs delta_true(10); delta_true << dp, dq.coeffs(), dv;
    ASSERT_MATRIX_APPROX(delta, delta_true, 1e-10);

    // transform to a new tangent -- should be the original tangent
    VectorXs d_from_delta = imu::lift(delta);
    ASSERT_MATRIX_APPROX(d_from_delta, d_min, 1e-10);

    // transform to a new delta -- should be the previous delta
    VectorXs delta_from_d = imu::retract(d_from_delta);
    ASSERT_MATRIX_APPROX(delta_from_d, delta, 1e-10);
}

TEST(IMU_tools, compare)
{
    VectorXs d1(10), d2(10);
    Vector4s qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d1 << 0, 1, 2, qv, 7, 8, 9;
    d2 = d1;

    VectorXs err(9);
    imu::compare(d1, d2, err);
    ASSERT_MATRIX_APPROX(err, VectorXs::Zero(9), 1e-10);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

