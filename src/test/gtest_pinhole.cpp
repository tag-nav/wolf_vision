/*
 * gtest_pinhole.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: jsola
 */

#include "utils_gtest.h"

#include "../pinholeTools.h"

using namespace Eigen;
using namespace wolf;
using namespace pinhole;
using Constants::EPS;
using Constants::EPS_SMALL;

TEST(Pinhole, EigenTypes)
{
    Vector3s vs; vs << 1,2,4;
    Vector2s ps;
    Map<Vector3s> vm(vs.data());
    Map<Vector2s> pm(ps.data());
    Map<const Vector3s> cvm(vs.data());
    VectorXs vd(3); vd = vs;
    VectorXs pd(2);

    Vector2s pe; pe << 0.25, 0.5; // expected result

    // static size
    projectPointToNormalizedPlane(vs,ps);
    ASSERT_TRUE((ps - pe).isMuchSmallerThan(1, EPS_SMALL));

    // dynamic size
    projectPointToNormalizedPlane(vd,pd);
    ASSERT_TRUE((pd - pe).isMuchSmallerThan(1, EPS_SMALL));

    // Map
    projectPointToNormalizedPlane(vm,pm);
    ASSERT_TRUE((pm - pe).isMuchSmallerThan(1, EPS_SMALL));

    // Map const
    projectPointToNormalizedPlane(cvm,pm);
    ASSERT_TRUE((pm - pe).isMuchSmallerThan(1, EPS_SMALL));
}

TEST(Pinhole, pix_pnt_pix)
{
    Vector4s k; k << 516.686, 355.129, 991.852, 995.269; // From Joan Sola thesis example
    Vector2s d; d << -0.301701, 0.0963189;
    Vector2s u0;
    Vector3s p;
    Vector2s u1;
    Vector2s c; // should be close to (0.297923, 0.216263)
    Scalar depth = 10;
    Scalar pix_error_allowed = 0.1;

    computeCorrectionModel(k, d, c);

    WOLF_INFO("correction: ", c.transpose(), " // should be close to (0.297923, 0.216263)");

    // choose some random points
    u0 << 123, 321;
    p  = backprojectPoint(k, c, u0, depth);
    u1 = projectPoint(k, d, p);
    WOLF_INFO("error: ", (u1-u0).transpose());
    ASSERT_LT((u1 - u0).norm(), pix_error_allowed) << "u1: "<< u1.transpose() << "\nu0: " << u0.transpose();

    u0 << 1, 1;
    p  = backprojectPoint(k, c, u0, depth);
    u1 = projectPoint(k, d, p);
    WOLF_INFO("error: ", (u1-u0).transpose());
    ASSERT_LT((u1 - u0).norm(), pix_error_allowed) << "u1: "<< u1.transpose() << "\nu0: " << u0.transpose();

    u0 << 320, 240;
    p  = backprojectPoint(k, c, u0, depth);
    u1 = projectPoint(k, d, p);
    WOLF_INFO("error: ", (u1-u0).transpose());
    ASSERT_LT((u1 - u0).norm(), pix_error_allowed) << "u1: "<< u1.transpose() << "\nu0: " << u0.transpose();

    u0 << 640, 480;
    p  = backprojectPoint(k, c, u0, depth);
    u1 = projectPoint(k, d, p);
    WOLF_INFO("error: ", (u1-u0).transpose());
    ASSERT_LT((u1 - u0).norm(), pix_error_allowed) << "u1: "<< u1.transpose() << "\nu0: " << u0.transpose();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


