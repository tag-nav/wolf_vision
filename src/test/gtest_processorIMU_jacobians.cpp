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
    struct IMU_jac_bias bias_jac;

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

        // Set the origin
        t.set(0.0001); // clock in 0,1 ms ticks
        Eigen::VectorXs x0(16);
        x0 << 0,1,0,  0,0,0,1,  1,0,0,  0,0,.000,  0,0,.000; // P Q V B B

        //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);
        //CaptureIMU* imu_ptr;

        ProcessorIMU_UnitTester processor_imu;
        //processor_imu.setOrigin(x0, t);
        wolf::Scalar ddelta_bias = 0.00000001;
        wolf::Scalar dt = 0.001;

        //defining a random Delta to begin with (not to use Origin point)
        Eigen::Matrix<wolf::Scalar,10,1> Delta0;
        Delta0 = Eigen::Matrix<wolf::Scalar,10,1>::Random();
        Delta0.head<3>() = Delta0.head<3>()*100;
        Delta0.segment<3>(3) = Delta0.segment<3>(3)*10;
        Eigen::Vector3s ang0, ang;
        ang0 << 120.08*deg_to_rad, 12.36*deg_to_rad, 54.32*deg_to_rad; 
        //Delta0 << 0,0,0, 1,0,0,0 ,0,0,0; 
        Eigen::Map<Eigen::Quaternions> Delta0_quat(Delta0.data()+6);
        Delta0_quat = v2q(ang0);
        Delta0_quat.normalize();
        ang = q2v(Delta0_quat);

        std::cout << "\ninput Delta0 : " << Delta0 << std::endl;
        std::cout << "\n rotation vector we start with :\n" << ang << std::endl;

        struct IMU_jac_bias bias_jac_c = processor_imu.finite_diff_ab(dt, data_, ddelta_bias, Delta0);
        bias_jac = bias_jac_c;
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