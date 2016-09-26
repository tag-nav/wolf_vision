/**
 * \file test_processor_imu_jacobians.cpp
 *
 *  Created on: Sep 26, 2016
 *      \author: AtDinesh
 */

//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

//#define DEBUG_RESULTS

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor IMU : Checking Jacobians ======================" << std::endl;

    TimeStamp t;
    Eigen::Vector6s data_;

    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PVQBB_3D);
    Eigen::VectorXs IMU_extrinsics(7);
    IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBase* sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, nullptr);
    wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Set the origin
    t.set(0.0001); // clock in 0,1 ms ticks
    Eigen::VectorXs x0(16);
    x0 << 0,1,0,  1,0,0,  0,0,0,1,  0,0,.000,  0,0,.000;

    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    CaptureIMU* imu_ptr;

    delete wolf_problem_ptr_;

    return 0;
}