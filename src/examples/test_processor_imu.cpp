/**
 * \file test_processor_imu.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: dtsbourg
 */

//std
#include <iostream>

//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"

int main()
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor IMU test ======================" << std::endl;

    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PQVBB_3D);
    SensorBase* sensor_ptr_ = new SensorIMU( new StateBlock(Eigen::VectorXs::Zero(3)),
                                             new StateQuaternion(),
                                             new StateBlock(Eigen::VectorXs::Zero(6)));

    ProcessorIMU* processor_ptr_ = new ProcessorIMU();

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(processor_ptr_);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    // TODO : Main loop

    delete wolf_problem_ptr_;

    return 0;
}
