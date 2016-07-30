/**
 * \file test_processor_tracker_landmark.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: jvallve
 */

//std
#include <iostream>

//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_base.h"
#include "state_block.h"
#include "processor_tracker_landmark_dummy.h"
#include "capture_void.h"

int main()
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor tracker landmark test ======================" << std::endl;

    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PO_2D);
    SensorBase* sensor_ptr_ = new SensorBase(SEN_ODOM_2D, "ODOM 2D", new StateBlock(Eigen::VectorXs::Zero(2)),
                                             new StateBlock(Eigen::VectorXs::Zero(1)),
                                             new StateBlock(Eigen::VectorXs::Zero(2)), 2);

    ProcessorTrackerLandmarkDummy* processor_ptr_ = new ProcessorTrackerLandmarkDummy(5);

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(processor_ptr_);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    for (auto i = 0; i < 10; i++)
        processor_ptr_->process(new CaptureVoid(TimeStamp(0), sensor_ptr_));

    delete wolf_problem_ptr_;

    return 0;
}



