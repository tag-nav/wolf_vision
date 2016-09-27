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
    data_ << 0,0,0, 0,0,0;

    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PVQBB_3D);
    Eigen::VectorXs IMU_extrinsics(7);
    IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    //SensorBase* sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, nullptr);
    //wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Set the origin
    t.set(0.0001); // clock in 0,1 ms ticks
    Eigen::VectorXs x0(16);
    x0 << 0,1,0,  1,0,0,  0,0,0,1,  0,0,.000,  0,0,.000;

    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    //CaptureIMU* imu_ptr;

    ProcessorIMU processor_imu;
    processor_imu.setOrigin(x0, t);
    wolf::Scalar ddelta_bias = 0.0001;
    struct IMU_jac_deltas bias_jac = processor_imu.finite_diff_ab(0.001, data_, ddelta_bias);

    /* IMU_jac_deltas struct form :
    contains vectors of size 7 :
    Elements at place 0 are those not affected by the bias noise that we add (da_bx,..., dw_bx,... ).
              place 1 : added da_bx in data
              place 2 : added da_by in data
              place 3 : added da_bz in data
              place 4 : added dw_bx in data
              place 5 : added dw_by in data
              place 6 : added dw_bz in data
    */

    Eigen::Matrix3s dDp_dabx_;
    Eigen::Matrix3s dDp_daby_;
    Eigen::Matrix3s dDp_dabz_;
    Eigen::Matrix3s dDv_dabx_;
    Eigen::Matrix3s dDv_daby_;
    Eigen::Matrix3s dDv_dabz_;

    delete wolf_problem_ptr_;

    return 0;
}