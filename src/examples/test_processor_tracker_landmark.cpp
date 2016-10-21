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

void print_problem_pointers(wolf::ProblemPtr wolf_problem_ptr_)
{
    using namespace wolf;
    std::cout << "\n-----------\nWolf tree begin" << std::endl;
    std::cout << "Hrd: " << wolf_problem_ptr_->getHardwarePtr()->getProblem() << std::endl;
    for (SensorBasePtr sen : wolf_problem_ptr_->getHardwarePtr()->getSensorList())
    {
        std::cout << "\tSen: " << sen->getProblem() << std::endl;
        for (ProcessorBasePtr prc : sen->getProcessorList())
        {
            std::cout << "\t\tPrc: " << prc->getProblem() << std::endl;
        }
    }
    std::cout << "Trj: " << wolf_problem_ptr_->getTrajectoryPtr()->getProblem() << std::endl;
    for (FrameBasePtr frm : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList())
    {
        std::cout << "\tFrm: " << frm->getProblem() << std::endl;
        for (CaptureBasePtr cap : frm->getCaptureList())
        {
            std::cout << "\t\tCap: " << cap->getProblem() << std::endl;
            for (FeatureBasePtr ftr : cap->getFeatureList())
            {
                std::cout << "\t\t\tFtr: " << ftr->getProblem() << std::endl;
                for (ConstraintBasePtr ctr : ftr->getConstraintList())
                {
                    std::cout << "\t\t\t\tCtr: " << ctr->getProblem() << std::endl;
                }
            }
        }
    }
    std::cout << "Map: " << wolf_problem_ptr_->getMapPtr()->getProblem() << std::endl;
    for (LandmarkBasePtr lmk : wolf_problem_ptr_->getMapPtr()->getLandmarkList())
    {
        std::cout << "\tLmk: " << lmk->getProblem() << std::endl;
    }
    std::cout << "Wolf tree end\n-----------\n" << std::endl;
}

int main()
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor tracker landmark test ======================" << std::endl;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_2D);
    SensorBasePtr sensor_ptr_ = std::make_shared< SensorBase>(SEN_ODOM_2D, "ODOM 2D", new StateBlock(Eigen::VectorXs::Zero(2)),
                                             new StateBlock(Eigen::VectorXs::Zero(1)),
                                             new StateBlock(Eigen::VectorXs::Zero(2)), 2);

    std::shared_ptr<ProcessorTrackerLandmarkDummy> processor_ptr_ = std::make_shared< ProcessorTrackerLandmarkDummy>(5);

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(processor_ptr_);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    for (auto i = 0; i < 10; i++)
    {
        processor_ptr_->process(std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr_));
    }

    print_problem_pointers(wolf_problem_ptr_);

    wolf_problem_ptr_->check();


    return 0;
}



