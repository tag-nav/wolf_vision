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
#include "ceres_wrapper/ceres_manager.h"

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor image landmark test ======================" << std::endl;

    cv::VideoCapture capture;
    const char * filename;
    if (argc == 1)
    {
        //filename = "/home/jtarraso/Vídeos/House interior.mp4";
        filename = "/home/jtarraso/Vídeos/gray.mp4";
        capture.open(filename);
    }
    else if (std::string(argv[1]) == "0")
    {
        //camera
        filename = "0";
        capture.open(0);
    }
    else
    {
        filename = argv[1];
        capture.open(filename);
    }
    std::cout << "Input video file: " << filename << std::endl;
    if(!capture.isOpened()) std::cout << "failed" << std::endl; else std::cout << "succeded" << std::endl;
    capture.set(CV_CAP_PROP_POS_MSEC, 3000);

    unsigned int img_width  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int img_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    unsigned int buffer_size = 20;
    std::vector<cv::Mat> frame(buffer_size);

    TimeStamp t = 1;



    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PO_3D);

    //=====================================================
    // Method 1: Use data generated here for sensor and processor
    //=====================================================

    //    // SENSOR
    //    Eigen::Vector4s k = {320,240,320,320};
    //    SensorCamera* sensor_ptr_ = new SensorCamera(new StateBlock(Eigen::Vector3s::Zero()),
    //                                                 new StateBlock(Eigen::Vector3s::Zero()),
    //                                                 new StateBlock(k,false),img_width,img_height);

    //    wolf_problem_ptr_->getHardwarePtr()->addSensor(sensor_ptr_);

    //    // PROCESSOR
    //    ProcessorImageParameters tracker_params;
    //    tracker_params.image = {img_width,  img_height};
    //    tracker_params.matcher.min_normalized_score = 0.75;
    //    tracker_params.matcher.similarity_norm = cv::NORM_HAMMING;
    //    tracker_params.matcher.roi_width = 30;
    //    tracker_params.matcher.roi_height = 30;
    //    tracker_params.active_search.grid_width = 12;
    //    tracker_params.active_search.grid_height = 8;
    //    tracker_params.active_search.separation = 1;
    //    tracker_params.algorithm.max_new_features =0;
    //    tracker_params.algorithm.min_features_for_keyframe = 20;

    //    DetectorDescriptorParamsOrb orb_params;
    //    orb_params.type = DD_ORB;

    //    DetectorDescriptorParamsBrisk brisk_params;
    //    brisk_params.type = DD_BRISK;

    //    // select the kind of detector-descriptor parameters
    //    tracker_params.detector_descriptor_params_ptr = &orb_params; // choose ORB

    //    ProcessorImageLandmark* prc_image_ldmk = new ProcessorImageLandmark(tracker_params);

    //    sensor_ptr_->addProcessor(prc_image_ldmk);
    //=====================================================


    //=====================================================
    // Method 2: Use factory to create sensor and processor
    //=====================================================

    // SENSOR
    // one-liner API

    /* Do this while there aren't extrinsic parameters on the yaml */
    Eigen::Vector7s extrinsic_cam;
    extrinsic_cam[0] = 0; //px
    extrinsic_cam[1] = 0; //py
    extrinsic_cam[2] = 0; //pz
    extrinsic_cam[3] = 0; //qx
    extrinsic_cam[4] = 0; //qy
    extrinsic_cam[5] = 0; //qz
    extrinsic_cam[6] = 1; //qw
    std::cout << "========extrinsic_cam: " << extrinsic_cam.transpose() << std::endl;
    const Eigen::VectorXs extr = extrinsic_cam;
    /* Do this while there aren't extrinsic parameters on the yaml */

    SensorBase* camera_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", extr, "/home/jtarraso/dev/Wolf/src/examples/camera_params.yml");
    SensorCamera* sensor_ptr_ = (SensorCamera*)camera_ptr;


    // PROCESSOR
    // one-liner API
    wolf_problem_ptr_->installProcessor("IMAGE LANDMARK", "ORB", "PinHole", "/home/jtarraso/dev/Wolf/src/examples/processor_image_ORB.yaml");
    //=====================================================

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;




    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    google::InitGoogleLogging(argv[0]);

    CeresManager ceres_manager(&(*wolf_problem_ptr_), ceres_options);







    // CAPTURES
    CaptureImage* image_ptr;

    unsigned int f  = 1;
    capture >> frame[f % buffer_size];

    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);


    while(!(frame[f % buffer_size].empty()))
    {
        t.setToNow();

        clock_t t1 = clock();

        // Preferred method with factory objects:
        image_ptr = new CaptureImage(t, sensor_ptr_, frame[f % buffer_size]);

        /* process */
        image_ptr->process();

        std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
        cv::waitKey(5);

        if((f%buffer_size) == 1)
        {
            ceres::Solver::Summary summary = ceres_manager.solve();
            std::cout << summary.FullReport() << std::endl;
        }

        f++;
        capture >> frame[f % buffer_size];
    }

    delete wolf_problem_ptr_;

    return 0;
}

