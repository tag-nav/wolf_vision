//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
/**
 * \file gtest_factor_epipolar.cpp
 *
 *  Created on: Aug 25, 2019
 *      \author: jsola
 */


#include "vision/factor/factor_epipolar.h"
#include "vision/capture/capture_image.h"
#include "vision/feature/feature_point_image.h"

#include "vision/internal/config.h"

#include <core/frame/frame_base.h>
#include <core/utils/utils_gtest.h>
#include <core/problem/problem.h>

using namespace wolf;
using namespace Eigen;

TEST(FactorEpipolar, exemple)
{
    Vector7d pose0, pose1, posecam;
    pose0   << 0,0,0, 0,0,0,1;
    pose1   << 1,0,0, 0,0,0,1;
    posecam << 0,1,0, 0,0,0,1;

    Vector2d pix0, pix1;
    pix0 << 320,240; // central line, central point
    pix1 << 300,240; // same line but more on the left than pix0.

    auto P  = Problem::create("PO", 3);

    // Install sensor and processor
    ParamsSensorCameraPtr intr      = std::make_shared<ParamsSensorCamera>();
    intr->pinhole_model_raw       = Eigen::Vector4d(320,240,320,320);
    intr->pinhole_model_rectified = Eigen::Vector4d(320,240,320,320);
    intr->width  = 640;
    intr->height = 480;
    auto S      = P->installSensor("SensorCamera", "camera", posecam, intr);
    auto camera = std::static_pointer_cast<SensorCamera>(S);

    auto F0 = P             ->emplaceFrame(0.0, pose0);
    auto F1 = P             ->emplaceFrame(1.0, pose1);
    auto C0 = CaptureBase   ::emplace<CaptureImage>(F0, F0->getTimeStamp(), camera, cv::Mat());
    auto C1 = CaptureBase   ::emplace<CaptureImage>(F1, F1->getTimeStamp(), camera, cv::Mat());
    auto f0 = FeatureBase   ::emplace<FeaturePointImage>(C0, pix0, 0, cv::Mat(), Matrix2d::Identity());

    double residual_0, residual_1, residual_2, residual_n1;

    // same line
    auto f1 = FeatureBase   ::emplace<FeaturePointImage>(C1, pix1, 0, cv::Mat(), Matrix2d::Identity());
    auto c0 = FactorBase    ::emplace<FactorEpipolar>(f0, f0, f1, nullptr, false);
    c0->operator()(F0->getP()->getState().data(),
                   F0->getO()->getState().data(),
                   F1->getP()->getState().data(),
                   F1->getO()->getState().data(),
                   camera->getP()->getState().data(),
                   camera->getO()->getState().data(),
                   &residual_0);

    WOLF_TRACE("residual @  0 pix : ", residual_0);


    // lines 1 pix difference
    auto f2 = FeatureBase   ::emplace<FeaturePointImage>(C1, Vector2d(300, 241), 0, cv::Mat(), Matrix2d::Identity());
    auto c1 = FactorBase    ::emplace<FactorEpipolar>(f0, f0, f2, nullptr, false);
    //f1->setMeasurement(Vector2d(300, 241));
    c1->operator()(F0->getP()->getState().data(),
                   F0->getO()->getState().data(),
                   F1->getP()->getState().data(),
                   F1->getO()->getState().data(),
                   camera->getP()->getState().data(),
                   camera->getO()->getState().data(),
                   &residual_1);

    WOLF_TRACE("residual @  1 pix : ", residual_1);

    // lines 2 pixels difference
    auto f3 = FeatureBase   ::emplace<FeaturePointImage>(C1, Vector2d(300, 242), 0, cv::Mat(), Matrix2d::Identity());
    auto c2 = FactorBase    ::emplace<FactorEpipolar>(f0, f0, f3, nullptr, false);
    //f1->setMeasurement(Vector2d(300, 242));
    c2->operator()(F0->getP()->getState().data(),
                   F0->getO()->getState().data(),
                   F1->getP()->getState().data(),
                   F1->getO()->getState().data(),
                   camera->getP()->getState().data(),
                   camera->getO()->getState().data(),
                   &residual_2);

    WOLF_TRACE("residual @  2 pix : ", residual_2);


    // lines 1 pix difference in the other direction
    auto f4 = FeatureBase   ::emplace<FeaturePointImage>(C1, Vector2d(300, 239), 0, cv::Mat(), Matrix2d::Identity());
    auto c3 = FactorBase    ::emplace<FactorEpipolar>(f0, f0, f4, nullptr, false);
    //f1->setMeasurement(Vector2d(300, 239));
    c3->operator()(F0->getP()->getState().data(),
                   F0->getO()->getState().data(),
                   F1->getP()->getState().data(),
                   F1->getO()->getState().data(),
                   camera->getP()->getState().data(),
                   camera->getO()->getState().data(),
                   &residual_n1);

    WOLF_TRACE("residual @ -1 pix : ", residual_n1);


    // all asserts down here:

    // residual zero when nominal
    ASSERT_NEAR(residual_0, 0.0, 1e-6);

    // residual positive when camera up
    ASSERT_GT(residual_1, 0.0);

    // residual double when camera doubly up
    ASSERT_NEAR(residual_2, 2.0 * residual_1, 1e-6);

    // residual opposite when cam down
    ASSERT_NEAR(residual_1, -residual_n1, 1e-6);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
//::testing::GTEST_FLAG(filter) = "TestGroup.DummyTestExample";
  return RUN_ALL_TESTS();
}

