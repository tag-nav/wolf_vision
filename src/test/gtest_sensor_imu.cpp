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
    Eigen::Vector3s get_p = sen0->getPPtr()->getVector();
    Eigen::Vector4s get_o = sen0->getOPtr()->getVector();

    ASSERT_TRUE((pos_ori.head<3>() - get_p).isMuchSmallerThan(1.0, Constants::EPS_SMALL));
    ASSERT_TRUE((pos_ori.tail<4>() - get_o).isMuchSmallerThan(1.0, Constants::EPS_SMALL));
    //StateBlock _a_w_bias should be empty

    //parameter containing accelerometer and gyroscope biases
    Eigen::Vector6s bias; //ab_x, ab_y, ab_z, wb_x, wb_y, wb_z
    bias.setRandom();
    StateBlockPtr bias_ptr = std::make_shared<StateBlock>(bias);
    SensorIMUPtr sen1 = std::make_shared<SensorIMU>(p_ptr, q_ptr, bias_ptr);

    Eigen::Vector6s get_bias(sen1->getIntrinsicPtr()->getVector());

    ASSERT_TRUE((bias - get_bias).isMuchSmallerThan(1.0, Constants::EPS_SMALL));

    IntrinsicsIMUPtr params = std::make_shared<IntrinsicsIMU>();
    SensorIMUPtr sen2 = std::make_shared<SensorIMU>(p_ptr, q_ptr, params, bias_ptr);

    ASSERT_EQ(params->gyro_noise, sen2->getGyroNoise());
    ASSERT_EQ(params->accel_noise, sen2->getAccelNoise());
    ASSERT_EQ(params->ab_constr, sen2->getAbConstr());
    ASSERT_EQ(params->wb_constr, sen2->getWbConstr());

                                                        //FACTORY SENSOR CONSTRUCTOR using YAML
    std::string wolf_root = _WOLF_ROOT_DIR;
    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs IMU_extrinsics(7);
    IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBasePtr sen4 = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, wolf_root + "/src/examples/sensor_imu.yaml");
    SensorIMUPtr sensor_ptr = std::static_pointer_cast<SensorIMU>(sen4);
    ASSERT_EQ(0.02, sensor_ptr->getGyroNoise()) << "please check gyro_noise value in yaml, sensor got " << sensor_ptr->getGyroNoise() <<std::endl;
    ASSERT_EQ(0.02, sensor_ptr->getAccelNoise()) << "please check accel_noise value in yaml, sensor got " << sensor_ptr->getAccelNoise() <<std::endl;
    ASSERT_EQ(0.01, sensor_ptr->getAbConstr()) << "please check ab_constr value in yaml, sensor got " << sensor_ptr->getAbConstr() <<std::endl;
    ASSERT_EQ(0.01, sensor_ptr->getWbConstr()) << "please check wb_constr value in yaml, sensor got " << sensor_ptr->getWbConstr() <<std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
