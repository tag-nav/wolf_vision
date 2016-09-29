/**
 * \file test_clean.cpp
 *
 *  Created on: Sep 30, 2016
 *      \author: jsola
 */



#include "wolf.h"
#include "processor_imu.h"

int main()
{
    using namespace wolf;


    Problem problem(FRM_PVQBB_3D);

    Eigen::VectorXs extrinsics(7);
    extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBasePtr sensor_ptr = problem.installSensor("IMU", "Main IMU", extrinsics, nullptr);
    problem.installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Time and data variables
    TimeStamp t;
    Scalar mti_clock, tmp;
    Eigen::Vector6s data;

    t.set(0);
    data << 0,0,0, 0,0,0;

    // Set the origin
    Eigen::VectorXs x0(16);
    x0 << 0,1,0,  1,0,0,  0,0,0,1,  0,0,.001,  0,0,.002; // Try some non-zero biases
    problem.getProcessorMotionPtr()->setOrigin(x0, t);



    return 0;
}

