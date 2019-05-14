/**
 * \file gtest_sensor_camera.cpp
 *
 *  Created on: Feb 7, 2019
 *      \author: jsola
 */


#include "utils_gtest.h"

#include "vision/sensor/sensor_camera.h"
#include "core/sensor/sensor_factory.h"

using namespace wolf;

TEST(SensorCamera, Img_size)
{
    Eigen::VectorXs extrinsics(7); extrinsics << 0,0,0, 0,0,0,1;
    IntrinsicsCamera params;
    params.width  = 640;
    params.height = 480;
    SensorCameraPtr cam = std::make_shared<SensorCamera>(extrinsics, params);

    ASSERT_EQ(cam->getImgWidth() , 640);
    ASSERT_EQ(cam->getImgHeight(), 480);

    cam->setImgWidth(100);
    ASSERT_EQ(cam->getImgWidth() , 100);

    cam->setImgHeight(100);
    ASSERT_EQ(cam->getImgHeight(), 100);
}

TEST(SensorCamera, Intrinsics_Raw_Rectified)
{
    Eigen::VectorXs extrinsics(7); extrinsics << 0,0,0, 0,0,0,1;
    IntrinsicsCamera params;
    params.pinhole_model_raw       << 321, 241, 321, 321;
    params.pinhole_model_rectified << 320, 240, 320, 320;
    SensorCameraPtr cam = std::make_shared<SensorCamera>(extrinsics, params);

    Eigen::Matrix3s K_raw, K_rectified;
    K_raw       << 321, 0, 321,    0, 321, 241,    0, 0, 1;
    K_rectified << 320, 0, 320,    0, 320, 240,    0, 0, 1;
    Eigen::Vector4s k_raw(321,241,321,321);
    Eigen::Vector4s k_rectified(320,240,320,320);

    // default is raw
    ASSERT_TRUE(cam->isUsingRawImages());
    ASSERT_MATRIX_APPROX(K_raw, cam->getIntrinsicMatrix(), 1e-8);
    ASSERT_MATRIX_APPROX(k_raw, cam->getIntrinsic()->getState(), 1e-8);

    cam->useRectifiedImages();
    ASSERT_FALSE(cam->isUsingRawImages());
    ASSERT_MATRIX_APPROX(K_rectified, cam->getIntrinsicMatrix(), 1e-8);
    ASSERT_MATRIX_APPROX(k_rectified, cam->getIntrinsic()->getState(), 1e-8);

    cam->useRawImages();
    ASSERT_TRUE(cam->isUsingRawImages());
    ASSERT_MATRIX_APPROX(K_raw, cam->getIntrinsicMatrix(), 1e-8);
    ASSERT_MATRIX_APPROX(k_raw, cam->getIntrinsic()->getState(), 1e-8);
}

TEST(SensorCamera, Distortion)
{
    Eigen::VectorXs extrinsics(7); extrinsics << 0,0,0, 0,0,0,1;
    IntrinsicsCamera params;
    params.width  = 640;
    params.height = 480;
    params.pinhole_model_raw       << 321, 241, 321, 321;
    params.pinhole_model_rectified << 320, 240, 320, 320;
    params.distortion = Eigen::Vector3s( -0.3, 0.096, 0 );
    SensorCameraPtr cam = std::make_shared<SensorCamera>(extrinsics, params);

    Eigen::Vector3s d(-0.3, 0.096, 0);

    ASSERT_MATRIX_APPROX(d, cam->getDistortionVector(), 1e-8);
}

TEST(SensorCamera, Correction_zero)
{
    Eigen::VectorXs extrinsics(7); extrinsics << 0,0,0, 0,0,0,1;
    IntrinsicsCamera params;
    params.width  = 640;
    params.height = 480;
    params.pinhole_model_raw       << 321, 241, 321, 321;
    params.pinhole_model_rectified << 320, 240, 320, 320;
    params.distortion = Eigen::Vector3s( 0, 0, 0 );
    SensorCameraPtr cam = std::make_shared<SensorCamera>(extrinsics, params);

    Eigen::MatrixXs c(cam->getCorrectionVector().size(), 1);
    c.setZero();

    ASSERT_GE(cam->getCorrectionVector().size(), cam->getDistortionVector().size());
    ASSERT_MATRIX_APPROX(c, cam->getCorrectionVector(), 1e-8);
}

TEST(SensorCamera, create)
{
    Eigen::VectorXs extrinsics(7); extrinsics << 0,0,0, 0,0,0,1;
    IntrinsicsCameraPtr params = std::make_shared<IntrinsicsCamera>();
    params->width  = 640;
    params->height = 480;
    params->pinhole_model_raw       << 321, 241, 321, 321;
    params->pinhole_model_rectified << 320, 240, 320, 320;
    params->distortion = Eigen::Vector3s( 0, 0, 0 );

    SensorBasePtr   sen = SensorCamera::create("camera", extrinsics, params);

    ASSERT_NE(sen, nullptr);

    SensorCameraPtr cam = std::static_pointer_cast<SensorCamera>(sen);

    ASSERT_NE(cam, nullptr);
    ASSERT_EQ(cam->getImgWidth(), 640);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

