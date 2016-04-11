/**
 * \file test_processor_tracker_feature.cpp
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

//std
#include <iostream>

//Wolf
#include "wolf.h"
#include "wolf_problem.h"
#include "sensor_base.h"
#include "state_block.h"
#include "processor_tracker_feature_dummy.h"
#include "capture_void.h"

int main()
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor tracker feature test ======================" << std::endl;

    // Wolf problem
    WolfProblem* wolf_problem_ptr_ = new WolfProblem(FRM_PO_2D);
    SensorBase* sensor_ptr_ = new SensorBase(SEN_ODOM_2D, new StateBlock(Eigen::VectorXs::Zero(2)),
                                             new StateBlock(Eigen::VectorXs::Zero(1)),
                                             new StateBlock(Eigen::VectorXs::Zero(2)), 2);

    ProcessorTrackerFeatureDummy* processor_ptr_ = new ProcessorTrackerFeatureDummy();

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(processor_ptr_);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

//    wolf_problem_ptr_->createFrame(KEY_FRAME, Eigen::Vector3s::Zero(), TimeStamp(0));
//
//    std::cout << "first frame created" << std::endl;
//
//    CaptureVoid* new_capture = new CaptureVoid(TimeStamp(0), sensor_ptr_);
//    wolf_problem_ptr_->getLastFramePtr()->addCapture(new_capture);
//
//    std::cout << "first capture created and added to first frame" << std::endl;
//
//    processor_ptr_->process(new_capture);
//
//    std::cout << "processor initialized" << std::endl;

    for (auto i = 0; i < 20; i++)
    {
        CaptureVoid* new_capture = new CaptureVoid(TimeStamp(0), sensor_ptr_);

        processor_ptr_->process(new_capture);
    }

    wolf_problem_ptr_->destruct();

    return 0;
}

