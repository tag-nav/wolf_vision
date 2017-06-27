/**
 * \file gtest_ceres.cpp
 *
 *  Created on: Jan 18, 2017
 *      \author: Dinesh Atchuthan
 */


#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "sensor_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "wolf.h"

#include <iostream>

using namespace Eigen;
using namespace std;
using namespace wolf;

TEST(SensorIMU, Constructors)
{
    //defining needed parameters
    Eigen::Vector7s pos_ori;
    pos_ori << 0,0,0 ,0,0,0,1;
    StateBlockPtr p_ptr = std::make_shared<StateBlock>(pos_ori.head<3>());
    StateBlockPtr q_ptr = std::make_shared<StateQuaternion>(pos_ori.tail<4>());
    
    SensorIMUPtr sen0 = std::make_shared<SensorIMU>(p_ptr, q_ptr);
    Eigen::Vector3s get_p = sen0->getPPtr()->getState();
    Eigen::Vector4s get_o = sen0->getOPtr()->getState();

    ASSERT_TRUE((pos_ori.head<3>() - get_p).isMuchSmallerThan(1.0, Constants::EPS_SMALL));
    ASSERT_TRUE((pos_ori.tail<4>() - get_o).isMuchSmallerThan(1.0, Constants::EPS_SMALL));
    //StateBlock _a_w_bias should be empty

    //parameter containing accelerometer and gyroscope biases

    IntrinsicsIMUPtr params = std::make_shared<IntrinsicsIMU>();
    SensorIMUPtr sen2 = std::make_shared<SensorIMU>(p_ptr, q_ptr, params);

    ASSERT_EQ(params->w_noise, sen2->getGyroNoise());
    ASSERT_EQ(params->a_noise, sen2->getAccelNoise());
    ASSERT_EQ(params->ab_stdev, sen2->getAbStdev());
    ASSERT_EQ(params->wb_stdev, sen2->getWbStdev());
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
