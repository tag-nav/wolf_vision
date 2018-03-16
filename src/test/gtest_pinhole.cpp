/*
 * gtest_pinhole.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: jsola
 */

#include "pinhole_tools.h"
#include "utils_gtest.h"


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

    WOLF_DEBUG("correction: ", c.transpose(), " // should be close to (0.297923, 0.216263)");

    // choose some random points
    u0 << 123, 321;
    p  = backprojectPoint(k, c, u0, depth);
    u1 = projectPoint(k, d, p);
    WOLF_DEBUG("error: ", (u1-u0).transpose());
    ASSERT_DOUBLE_EQ(p(2), depth);
    ASSERT_LT((u1 - u0).norm(), pix_error_allowed) << "u1: "<< u1.transpose() << "\nu0: " << u0.transpose();

    u0 << 1, 1;
    p  = backprojectPoint(k, c, u0, depth);
    u1 = projectPoint(k, d, p);
    WOLF_DEBUG("error: ", (u1-u0).transpose());
    ASSERT_DOUBLE_EQ(p(2), depth);
    ASSERT_LT((u1 - u0).norm(), pix_error_allowed) << "u1: "<< u1.transpose() << "\nu0: " << u0.transpose();

    u0 << 320, 240;
    p  = backprojectPoint(k, c, u0, depth);
    u1 = projectPoint(k, d, p);
    WOLF_DEBUG("error: ", (u1-u0).transpose());
    ASSERT_DOUBLE_EQ(p(2), depth);
    ASSERT_LT((u1 - u0).norm(), pix_error_allowed) << "u1: "<< u1.transpose() << "\nu0: " << u0.transpose();

    u0 << 640, 480;
    p  = backprojectPoint(k, c, u0, depth);
    u1 = projectPoint(k, d, p);
    WOLF_DEBUG("error: ", (u1-u0).transpose());
    ASSERT_DOUBLE_EQ(p(2), depth);
    ASSERT_LT((u1 - u0).norm(), pix_error_allowed) << "u1: "<< u1.transpose() << "\nu0: " << u0.transpose();
}

TEST(Pinhole, Jacobians)
{
    Vector4s k; k << 516.686, 355.129, 991.852, 995.269; // From Joan Sola thesis example
    Vector2s d; d << -0.301701, 0.0963189;
    Vector3s v;
    Vector2s u;
    MatrixXs U_v(2, 3);

    v << 1,2,4;
    projectPoint(k, d, v, u, U_v);

    Vector2s c;
    computeCorrectionModel(k, d, c);

    Vector3s p;
    MatrixXs P_u(3,2), P_depth(3,1);
    backprojectPoint(k, c, u, 4, p, P_u, P_depth);

    // check that reprojected point is close to original
    ASSERT_LT((p-v).norm(), 1e-3) << "p: " << p.transpose() << "\nv: " << v.transpose();

    // Check that both Jacobians are inverse one another (in the smallest dimension)
    ASSERT_TRUE((U_v*P_u - Matrix2s::Identity()).isMuchSmallerThan(1, 1e-3)) << "U_v*P_u: " << U_v*P_u;

    WOLF_DEBUG("U_v*P_u: \n", U_v*P_u);
    WOLF_DEBUG("P_u*U_v: \n", P_u*U_v);
}

TEST(Pinhole, JacobiansDist)
{
    Vector4s k; k << 516.686, 355.129, 991.852, 995.269; // From Joan Sola thesis example
    Vector2s d; d << -0.301701, 0.0963189;
    Vector3s v;
    Vector2s u;
    MatrixXs U_v(2, 3);
    Scalar dist;

    v << 1,2,4;
    projectPoint(k, d, v, u, dist, U_v);

    // check that the recovered distance is indeed the distance to v
    ASSERT_DOUBLE_EQ(dist, v.norm());

    Vector2s c;
    computeCorrectionModel(k, d, c);

    Vector3s p;
    MatrixXs P_u(3,2), P_depth(3,1);
    backprojectPoint(k, c, u, 4, p, P_u, P_depth);

    // check that reprojected point is close to original
    ASSERT_LT((p-v).norm(), 1e-3) << "p: " << p.transpose() << "\nv: " << v.transpose();

    // Check that both Jacobians are inverse one another (in the smallest dimension)
    ASSERT_TRUE((U_v*P_u - Matrix2s::Identity()).isMuchSmallerThan(1, 1e-3)) << "U_v*P_u: " << U_v*P_u;
}

TEST(Pinhole, isInRoi)
{
    Vector2s p;
    p << 15, 15;

    ASSERT_TRUE (isInRoi(p, 10, 10, 10, 10));
    ASSERT_FALSE(isInRoi(p,  0,  0, 10, 10));
    ASSERT_FALSE(isInRoi(p,  0, 10, 10, 10));
    ASSERT_FALSE(isInRoi(p,  0, 20, 10, 10));
    ASSERT_FALSE(isInRoi(p, 10,  0, 10, 10));
    ASSERT_FALSE(isInRoi(p, 10, 20, 10, 10));
    ASSERT_FALSE(isInRoi(p, 20,  0, 10, 10));
    ASSERT_FALSE(isInRoi(p, 20, 10, 10, 10));
    ASSERT_FALSE(isInRoi(p, 20, 20, 10, 10));
}

TEST(Pinhole, isInImage)
{
    Vector2s p;
    p << 15, 15;
    ASSERT_TRUE (isInImage(p, 640, 480));

    p << -15, -15;
    ASSERT_FALSE (isInImage(p, 640, 480));

    p << -15, 15;
    ASSERT_FALSE (isInImage(p, 640, 480));

    p << -15, 500;
    ASSERT_FALSE (isInImage(p, 640, 480));

    p <<  15, -15;
    ASSERT_FALSE (isInImage(p, 640, 480));

    p <<  15, 500;
    ASSERT_FALSE (isInImage(p, 640, 480));

    p << 700, -15;
    ASSERT_FALSE (isInImage(p, 640, 480));

    p << 700, 15;
    ASSERT_FALSE (isInImage(p, 640, 480));

    p << 700, 500;
    ASSERT_FALSE (isInImage(p, 640, 480));

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


