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
#include "processor_image_landmark.h"
#include "capture_void.h"

int main()
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor image landmark test ======================" << std::endl;

    unsigned int img_width = 640;
    unsigned int img_height = 480;


    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PO_3D);
    SensorBase* sensor_ptr_ = new SensorBase(SEN_ODOM_2D, new StateBlock(Eigen::VectorXs::Zero(2)),
                                             new StateBlock(Eigen::VectorXs::Zero(1)),
                                             new StateBlock(Eigen::VectorXs::Zero(2)), 2);


    //wolf_problem_->getHardwarePtr()->addSensor(sen_cam_);

    // PROCESSOR
    ProcessorImageParameters tracker_params;
    tracker_params.image = {img_width,  img_height};
    tracker_params.matcher.min_normalized_score = 0.75;
    tracker_params.matcher.similarity_norm = cv::NORM_HAMMING;
    tracker_params.matcher.roi_width = 30;
    tracker_params.matcher.roi_height = 30;
    tracker_params.active_search.grid_width = 12;
    tracker_params.active_search.grid_height = 8;
    tracker_params.active_search.separation = 1;
    tracker_params.algorithm.max_new_features =0;
    tracker_params.algorithm.min_features_for_keyframe = 20;

    DetectorDescriptorParamsOrb orb_params;
    orb_params.type = DD_ORB;

    DetectorDescriptorParamsBrisk brisk_params;
    brisk_params.type = DD_BRISK;

    // select the kind of detector-descriptor parameters
    tracker_params.detector_descriptor_params_ptr = &orb_params; // choose ORB

    ProcessorImageLandmark* prc_image_ldmk = new ProcessorImageLandmark(tracker_params);

    //sen_cam_->addProcessor(prc_image_ldmk);


//    //=====================================================
//    // Method 2: Use factory to create sensor and processor
//    //=====================================================

//    Problem* wolf_problem_ptr_ = new Problem(FRM_PO_3D);

//    // SENSOR
//    // one-liner API
//    SensorBase* sensor_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", Eigen::VectorXs::Zero(7), "/home/jtarraso/dev/Wolf/src/examples/camera.yaml");
//    SensorCamera* camera_ptr = (SensorCamera*)sensor_ptr;

//    // PROCESSOR
//    // one-liner API
//    wolf_problem_ptr_->installProcessor("IMAGE", "ORB", "PinHole", "/home/jtarraso/dev/Wolf/src/examples/processor_image_ORB.yaml");
//    //=====================================================

    //ProcessorImageLandmark* processor_ptr_ = new ProcessorImageLandmark(5);

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(prc_image_ldmk);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    for (auto i = 0; i < 10; i++)
        prc_image_ldmk->process(new CaptureVoid(TimeStamp(0), sensor_ptr_));

    delete wolf_problem_ptr_;

    return 0;
}

