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
    IntrinsicsCameraPtr intr      = std::make_shared<IntrinsicsCamera>();
    intr->pinhole_model_raw       = Eigen::Vector4d(320,240,320,320);
    intr->pinhole_model_rectified = Eigen::Vector4d(320,240,320,320);
    intr->width  = 640;
    intr->height = 480;
    auto S      = P->installSensor("CAMERA", "camera", posecam, intr);
    auto camera = std::static_pointer_cast<SensorCamera>(S);

    auto F0 = P             ->emplaceFrame(KEY, pose0, 0.0);
    auto F1 = P             ->emplaceFrame(KEY, pose1, 1.0);
    auto C0 = CaptureBase   ::emplace<CaptureImage>(F0, F0->getTimeStamp(), camera, cv::Mat());
    auto C1 = CaptureBase   ::emplace<CaptureImage>(F1, F1->getTimeStamp(), camera, cv::Mat());
    auto f0 = FeatureBase   ::emplace<FeaturePointImage>(C0, pix0, 0, cv::Mat(), Matrix2d::Identity());
    auto f1 = FeatureBase   ::emplace<FeaturePointImage>(C1, pix1, 0, cv::Mat(), Matrix2d::Identity());
    auto c  = FactorBase    ::emplace<FactorEpipolar>(f0, f0, f1, nullptr, false);

    double residual_0, residual_1, residual_2, residual_n1;

    // same line
    c->operator()(F0->getP()->getState().data(),
                  F0->getO()->getState().data(),
                  F1->getP()->getState().data(),
                  F1->getO()->getState().data(),
                  camera->getP()->getState().data(),
                  camera->getO()->getState().data(),
                  &residual_0);

    WOLF_TRACE("residual @  0 pix: ", residual_0);

    ASSERT_NEAR(residual_0, 0.0, 1e-6);

    // lines 1 pix difference
    f1->setMeasurement(Vector2d(300, 241));
    c->operator()(F0->getP()->getState().data(),
                  F0->getO()->getState().data(),
                  F1->getP()->getState().data(),
                  F1->getO()->getState().data(),
                  camera->getP()->getState().data(),
                  camera->getO()->getState().data(),
                  &residual_1);

    WOLF_TRACE("residual @  1 pix : ", residual_1);

    // lines 2 pixels difference
    f1->setMeasurement(Vector2d(300, 242));
    c->operator()(F0->getP()->getState().data(),
                  F0->getO()->getState().data(),
                  F1->getP()->getState().data(),
                  F1->getO()->getState().data(),
                  camera->getP()->getState().data(),
                  camera->getO()->getState().data(),
                  &residual_2);

    WOLF_TRACE("residual @  2 pix : ", residual_2);

    ASSERT_NEAR(residual_2, 2.0 * residual_1, 1e-6);

    // lines 1 pix difference in the other direction
    f1->setMeasurement(Vector2d(300, 239));
    c->operator()(F0->getP()->getState().data(),
                  F0->getO()->getState().data(),
                  F1->getP()->getState().data(),
                  F1->getO()->getState().data(),
                  camera->getP()->getState().data(),
                  camera->getO()->getState().data(),
                  &residual_n1);

    WOLF_TRACE("residual @ -1 pix : ", residual_n1);

    ASSERT_NEAR(residual_1, -residual_n1, 1e-6);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
//::testing::GTEST_FLAG(filter) = "TestGroup.DummyTestExample";
  return RUN_ALL_TESTS();
}

