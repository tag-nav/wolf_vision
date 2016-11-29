/**
 * \file gtest_imu_preintegration_jacobians.cpp
 *
 *  Created on: Nov 29, 2016
 *      \author: AtDinesh
 */

 //Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu_UnitTester.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

//google test
#include "utils_gtest.h"

//#define DEBUG_RESULTS
//#define WRITE_RESULTS

using namespace wolf;

// A new one of these is created for each test
class ProcessorIMU_jacobians : public testing::Test
{
    public:
    TimeStamp t;
    Eigen::Vector6s data_;

    virtual void SetUp()
    {
        wolf::Scalar deg_to_rad = M_PI/180.0;
        data_ << 10,0.5,3, 100*deg_to_rad,110*deg_to_rad,30*deg_to_rad;

        // Wolf problem
        ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs IMU_extrinsics(7);
        IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
        //SensorBase* sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, nullptr);
        //wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");
    }

    virtual void TearDown(){}
};

TEST_F(ProcessorIMU_jacobians, Dummy)
{
    ASSERT_TRUE(data_.size() == 6);
}

int main(int argc, char **argv)
{
    using namespace wolf;

     testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
}