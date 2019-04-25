/**
 * \file gtest_SE3.cpp
 *
 *  Created on: Feb 2, 2019
 *      \author: jsola
 */


#include "base/math/SE3.h"
#include "utils_gtest.h"



using namespace Eigen;
using namespace wolf;
using namespace three_D;

TEST(SE3, exp_0)
{
    Vector6s tau = Vector6s::Zero();
    Vector7s pose; pose << 0,0,0, 0,0,0,1;

    ASSERT_MATRIX_APPROX(pose, exp_SE3(tau), 1e-8);
}

TEST(SE3, log_I)
{
    Vector7s pose; pose << 0,0,0, 0,0,0,1;
    Vector6s tau = Vector6s::Zero();

    ASSERT_MATRIX_APPROX(tau, log_SE3(pose), 1e-8);
}

TEST(SE3, exp_log)
{
    Vector6s tau = Vector6s::Random() / 5.0;
    Vector7s pose = exp_SE3(tau);

    ASSERT_MATRIX_APPROX(tau, log_SE3(exp_SE3(tau)), 1e-8);
    ASSERT_MATRIX_APPROX(pose, exp_SE3(log_SE3(pose)), 1e-8);
}

TEST(SE3, exp_of_multiple)
{
    Vector6s tau, tau2, tau3;
    Vector7s pose, pose2, pose3;
    tau = Vector6s::Random() / 5;
    pose << exp_SE3(tau);
    tau2  = 2*tau;
    tau3  = 3*tau;
    pose2 = compose(pose, pose);
    pose3 = compose(pose2, pose);

    // 1
    ASSERT_MATRIX_APPROX(exp_SE3(tau), pose, 1e-8);

    // 2
    ASSERT_MATRIX_APPROX(exp_SE3(tau2), pose2, 1e-8);

    // 3
    ASSERT_MATRIX_APPROX(exp_SE3(tau3), pose3, 1e-8);
}

TEST(SE3, log_of_power)
{
    Vector6s tau, tau2, tau3;
    Vector7s pose, pose2, pose3;
    tau = Vector6s::Random() / 5;
    pose << exp_SE3(tau);
    tau2 = 2*tau;
    tau3 = 3*tau;
    pose2 = compose(pose, pose);
    pose3 = compose(pose2, pose);

    // 1
    ASSERT_MATRIX_APPROX(tau, log_SE3(pose), 1e-8);

    // 2
    ASSERT_MATRIX_APPROX(tau2, log_SE3(pose2), 1e-8);

    // 3
    ASSERT_MATRIX_APPROX(tau3, log_SE3(pose3), 1e-8);
}

TEST(SE3, interpolate_I_xyz)
{
    Vector7s p1, p2, p_pre;

    p1 << 0,0,0, 0,0,0,1;
    p2 << 1,2,3, 0,0,0,1;
    Scalar t = 0.2;

    interpolate(p1, p2, t, p_pre);

    Vector7s p_gt; p_gt << 0.2,0.4,0.6, 0,0,0,1;

    ASSERT_MATRIX_APPROX(p_pre, p_gt, 1e-8);
}

TEST(SE3, interpolate_xyz_xyz)
{
    Vector7s p1, p2, p_pre;

    p1 << 1,2,3, 0,0,0,1;
    p2 << 2,4,6, 0,0,0,1;
    Scalar t = 0.2;

    interpolate(p1, p2, t, p_pre);

    Vector7s p_gt; p_gt << 1.2,2.4,3.6, 0,0,0,1;

    ASSERT_MATRIX_APPROX(p_pre, p_gt, 1e-8);
}

TEST(SE3, interpolate_I_qx)
{
    Vector7s p1, p2, p_pre;

    p1 << 0,0,0, 0,0,0,1;
    p2 << 0,0,0, 1,0,0,0;
    Scalar t = 0.5;

    interpolate(p1, p2, t, p_pre);

    Vector7s p_gt; p_gt << 0,0,0, sqrt(2)/2,0,0,sqrt(2)/2;

    ASSERT_MATRIX_APPROX(p_pre, p_gt, 1e-8);
}

TEST(SE3, interpolate_I_qy)
{
    Vector7s p1, p2, p_pre;

    p1 << 0,0,0, 0,0,0,1;
    p2 << 0,0,0, 0,1,0,0;
    Scalar t = 0.5;

    interpolate(p1, p2, t, p_pre);

    Vector7s p_gt; p_gt << 0,0,0, 0,sqrt(2)/2,0,sqrt(2)/2;

    ASSERT_MATRIX_APPROX(p_pre, p_gt, 1e-8);
}

TEST(SE3, interpolate_I_qz)
{
    Vector7s p1, p2, p_pre;

    p1 << 0,0,0, 0,0,0,1;
    p2 << 0,0,0, 0,0,1,0;
    Scalar t = 0.5;

    interpolate(p1, p2, t, p_pre);

    Vector7s p_gt; p_gt << 0,0,0, 0,0,sqrt(2)/2,sqrt(2)/2;

    ASSERT_MATRIX_APPROX(p_pre, p_gt, 1e-8);
}

TEST(SE3, interpolate_I_qxyz)
{
    Vector7s p1, p2, dp, p_pre, p_gt;
    Vector3s da;

    da.setRandom(); da /= 5;

    p1 << 0,0,0, 0,0,0,1;
    dp << 0,0,0, exp_q(da).coeffs();
    p_gt = compose(p1, dp);
    p2   = compose(p_gt, dp);
    Scalar t = 0.5;

    interpolate(p1, p2, t, p_pre);

    ASSERT_MATRIX_APPROX(p_pre, p_gt, 1e-8);
}

TEST(SE3, interpolate_half)
{
    Vector7s p1, p2, dp, p_pre, p_gt;
    Vector6s da;

    p1.setRandom(); p1.tail(4).normalize();

    da.setRandom(); da /= 5; // small rotation
    dp  << exp_SE3(da);

    // compose double, then interpolate 1/2

    p_gt = compose(p1, dp);
    p2   = compose(p_gt, dp);

    Scalar t = 0.5;
    interpolate(p1, p2, t, p_pre);

    ASSERT_MATRIX_APPROX(p_pre, p_gt, 1e-8);
}

TEST(SE3, interpolate_third)
{
    Vector7s p1, p2, dp, dp2, dp3, p_pre, p_gt;
    Vector6s da;

    p1.setRandom(); p1.tail(4).normalize();

    da.setRandom(); da /= 5; // small rotation
    dp  << exp_SE3(da);
    dp2 = compose(dp, dp);
    dp3 = compose(dp2, dp);

    // compose triple, then interpolate 1/3

    p_gt = compose(p1, dp);
    p2   = compose(p1, dp3);

    Scalar t = 1.0/3.0;
    interpolate(p1, p2, t, p_pre);

    ASSERT_MATRIX_APPROX(p_pre, p_gt, 1e-8);
}

TEST(SE3, toSE3_I)
{
    Vector7s pose; pose << 0,0,0, 0,0,0,1;
    Matrix4s R;
    toSE3(pose, R);

    ASSERT_MATRIX_APPROX(R, Matrix4s::Identity(), 1e-8);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

