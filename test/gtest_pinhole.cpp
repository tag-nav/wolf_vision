//--------LICENSE_START--------
//
// Copyright (C) 2020,2021 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
/*
 * gtest_pinhole.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: jsola
 */

#include "vision/math/pinhole_tools.h"

#include <core/utils/utils_gtest.h>

using namespace Eigen;
using namespace wolf;
using namespace pinhole;
using Constants::EPS;
using Constants::EPS_SMALL;

TEST(Pinhole, EigenTypes)
{
    Vector3d vs; vs << 1,2,4;
    Vector2d ps;
    Map<Vector3d> vm(vs.data());
    Map<Vector2d> pm(ps.data());
    Map<const Vector3d> cvm(vs.data());
    VectorXd vd(3); vd = vs;
    VectorXd pd(2);

    Vector2d pe; pe << 0.25, 0.5; // expected result

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
    Vector4d k; k << 516.686, 355.129, 991.852, 995.269; // From Joan Sola thesis example
    Vector2d d; d << -0.301701, 0.0963189;
    Vector2d u0;
    Vector3d p;
    Vector2d u1;
    Vector2d c; // should be close to (0.297923, 0.216263)
    double depth = 10;
    double pix_error_allowed = 0.1;

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
    Vector4d k; k << 516.686, 355.129, 991.852, 995.269; // From Joan Sola thesis example
    Vector2d d; d << -0.301701, 0.0963189;
    Vector3d v;
    Vector2d u;
    MatrixXd U_v(2, 3);

    v << 1,2,4;
    projectPoint(k, d, v, u, U_v);

    Vector2d c;
    computeCorrectionModel(k, d, c);

    Vector3d p;
    MatrixXd P_u(3,2), P_depth(3,1);
    backprojectPoint(k, c, u, 4, p, P_u, P_depth);

    // check that reprojected point is close to original
    ASSERT_LT((p-v).norm(), 1e-3) << "p: " << p.transpose() << "\nv: " << v.transpose();

    // Check that both Jacobians are inverse one another (in the smallest dimension)
    ASSERT_TRUE((U_v*P_u - Matrix2d::Identity()).isMuchSmallerThan(1, 1e-3)) << "U_v*P_u: " << U_v*P_u;

    WOLF_DEBUG("U_v*P_u: \n", U_v*P_u);
    WOLF_DEBUG("P_u*U_v: \n", P_u*U_v);
}

TEST(Pinhole, JacobiansDist)
{
    Vector4d k; k << 516.686, 355.129, 991.852, 995.269; // From Joan Sola thesis example
    Vector2d d; d << -0.301701, 0.0963189;
    Vector3d v;
    Vector2d u;
    MatrixXd U_v(2, 3);
    double dist;

    v << 1,2,4;
    projectPoint(k, d, v, u, dist, U_v);

    // check that the recovered distance is indeed the distance to v
    ASSERT_DOUBLE_EQ(dist, v.norm());

    Vector2d c;
    computeCorrectionModel(k, d, c);

    Vector3d p;
    MatrixXd P_u(3,2), P_depth(3,1);
    backprojectPoint(k, c, u, 4, p, P_u, P_depth);

    // check that reprojected point is close to original
    ASSERT_LT((p-v).norm(), 1e-3) << "p: " << p.transpose() << "\nv: " << v.transpose();

    // Check that both Jacobians are inverse one another (in the smallest dimension)
    ASSERT_TRUE((U_v*P_u - Matrix2d::Identity()).isMuchSmallerThan(1, 1e-3)) << "U_v*P_u: " << U_v*P_u;
}

TEST(Pinhole, isInRoi)
{
    Vector2d p;
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
    Vector2d p;
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

