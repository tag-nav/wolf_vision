/*
 * gtest_frame_imu.cpp
 *
 *  Created on: Jan 05, 2017
 *      Author: Dinesh Atchuthan
 */

 #include "utils_gtest.h"
#include "../logging.h"

#include "../frame_imu.h"
#include "../capture_motion.h"

#include <iostream>

using namespace Eigen;
using namespace std;
using namespace wolf;

TEST(FrameIMU, Getters)
{
    TimeStamp ts(0.1);
    Eigen::VectorXs state0(16);
    state0 << 0,0,0,  0,0,0,  0,0,0,1,  0,0,.001,  0,0,.002;
    FrameIMUPtr F = make_shared<FrameIMU>(KEY_FRAME, ts, state0);

    Eigen::Vector3s acc_b = F->getAccBiasPtr()->getState();
    Eigen::Vector3s gyro_b = F->getGyroBiasPtr()->getState();

    ASSERT_TRUE((state0.segment(10,3) - acc_b).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
    ASSERT_TRUE((state0.tail(3) - gyro_b).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
