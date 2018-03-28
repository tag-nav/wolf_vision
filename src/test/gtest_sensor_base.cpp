/**
 * \file gtest_sensor_base.cpp
 *
 *  Created on: Mar 27, 2018
 *      \author: jsola
 */

#include "sensor_base.h"

#include "utils_gtest.h"


using namespace wolf;

TEST(SensorBase, setNoiseStd)
{
    SensorBasePtr S (std::make_shared<SensorBase>("BASE", nullptr, nullptr, nullptr, 2)); // noise size 2

    Eigen::Vector2s noise_std = Eigen::Vector2s::Ones()     * 0.1;
    Eigen::Matrix2s noise_cov = Eigen::Matrix2s::Identity() * 0.01;

    S->setNoiseStd(noise_std);

    ASSERT_MATRIX_APPROX(noise_std, S->getNoiseStd(), 1e-8);
    ASSERT_MATRIX_APPROX(noise_cov, S->getNoiseCov(), 1e-8);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

