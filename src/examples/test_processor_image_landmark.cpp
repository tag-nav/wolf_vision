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

    unsigned int buffer_size = 8;
    std::vector<cv::Mat> frame(buffer_size);

    TimeStamp t = 1;





    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PO_3D);
//    SensorBase* sensor_ptr_ = new SensorBase(SEN_ODOM_2D, new StateBlock(Eigen::VectorXs::Zero(2)),
//                                             new StateBlock(Eigen::VectorXs::Zero(1)),
//                                             new StateBlock(Eigen::VectorXs::Zero(2)), 2);

//    Eigen::Vector4s k = {320,240,320,320};
//    SensorCamera* sensor_ptr_ = new SensorCamera(new StateBlock(Eigen::Vector3s::Zero()),
//                                              new StateBlock(Eigen::Vector3s::Zero()),
//                                              new StateBlock(k,false),img_width,img_height);

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

//    tracker_params.pinhole_params.k_parameters = {830.748734, 831.18208, 327.219132,234.720244};
//    tracker_params.pinhole_params.distortion = {0.0006579999999999999, 0.023847};
    tracker_params.pinhole_params.k_parameters = {872.791604, 883.154343, 407.599166, 270.343971};
    tracker_params.pinhole_params.distortion = {-0.284384, -0.030014};



    DetectorDescriptorParamsOrb orb_params;
    orb_params.type = DD_ORB;

    DetectorDescriptorParamsBrisk brisk_params;
    brisk_params.type = DD_BRISK;

    // select the kind of detector-descriptor parameters
    tracker_params.detector_descriptor_params_ptr = &orb_params; // choose ORB

    //ProcessorImageLandmark* prc_image_ldmk = new ProcessorImageLandmark(tracker_params);


    //=====================================================
    // Method 2: Use factory to create sensor and processor
    //=====================================================

    //Problem* wolf_problem_ptr_ = new Problem(FRM_PO_3D);

    // SENSOR
    // one-liner API



    SensorBase* camera_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", Eigen::VectorXs::Zero(7), "/home/jtarraso/dev/Wolf/src/examples/camera_params.yml");
    SensorCamera* sensor_ptr_ = (SensorCamera*)camera_ptr;

    std::cout << "========intrinsics: " << sensor_ptr_->getIntrinsicPtr()->getVector().transpose() << std::endl;
    std::cout << "========camera position: " << sensor_ptr_->getPPtr()->getVector().transpose() << std::endl;
    std::cout << "========camera orientation: " << sensor_ptr_->getOPtr()->getVector().transpose() << std::endl;
    std::cout << "========distortion: " << sensor_ptr_->getDistortionVector().transpose() << std::endl;
    std::cout << "========correction: " << sensor_ptr_->getCorrectionVector().transpose() << std::endl;
    std::cout << "========height: " << sensor_ptr_->getImgHeight() << std::endl;
    std::cout << "========width: " << sensor_ptr_->getImgWidth() << std::endl;

    IntrinsicsCamera* intrinsic_cam;
    Eigen::Vector4s intr = sensor_ptr_->getIntrinsicPtr()->getVector();
    intrinsic_cam->pinhole_model = {intr(0),intr(1),intr(2), intr(3)};
    intrinsic_cam->distortion = sensor_ptr_->getDistortionVector();
    intrinsic_cam->height = 480;
    intrinsic_cam->width = 640;

    Eigen::VectorXs p = sensor_ptr_->getPPtr()->getVector();
    Eigen::VectorXs o = sensor_ptr_->getOPtr()->getVector();
    Eigen::Vector7s extrinsic_cam;
    extrinsic_cam(0) = p(0);
    extrinsic_cam[1] = p[1];
    extrinsic_cam[2] = p[2];
    extrinsic_cam[3] = o[0];
    extrinsic_cam[4] = o[1];
    extrinsic_cam[5] = o[2];
    extrinsic_cam[6] = o[3];
    std::cout << "========extrinsic_cam: " << extrinsic_cam.transpose() << std::endl;
    const Eigen::VectorXs extr = extrinsic_cam;

    SensorCamera* sensor_ptr_2 = new SensorCamera(extr,intrinsic_cam);




    /* test */
//    std::string name;
//    for(SensorBaseList::iterator sb = wolf_problem_ptr_->getHardwarePtr()->getSensorListPtr()->begin();
//        sb != wolf_problem_ptr_->getHardwarePtr()->getSensorListPtr()->end(); ++sb)
//                               {
//                                    std::cout << "NAMES" << std::endl;
//                                    SensorBase* sen_base = (SensorBase*)*sb;
//                                    name = sen_base->getName();
//                                    std::cout << name << std::endl;
//                               };
//    std::cout << "obtained: " << wolf_problem_ptr_->getSensorPtr(name)->getIntrinsicPtr()->getVector() << std::endl;

//    tracker_params.pinhole_params.k_parameters = wolf_problem_ptr_->getSensorPtr(name)->getIntrinsicPtr()->getVector();
//    std::cout << "manual: " << tracker_params.pinhole_params.k_parameters << std::endl;

    /* end test */

    // PROCESSOR
    // one-liner API
    wolf_problem_ptr_->installProcessor("IMAGE LANDMARK", "ORB", "PinHole", "/home/jtarraso/dev/Wolf/src/examples/processor_image_ORB.yaml");
    //=====================================================


    ProcessorImageLandmark* prc_image_ldmk = new ProcessorImageLandmark(tracker_params);




    // CAPTURES
    CaptureImage* image_ptr;


//    wolf_problem_ptr_->addSensor(sensor_ptr_2);
//    sensor_ptr_2->addProcessor(prc_image_ldmk);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;




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
        //prc_image_ldmk->process(new CaptureVoid(TimeStamp(0), sensor_ptr_));
        image_ptr->process();

        std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
        cv::waitKey(5);

        f++;
        capture >> frame[f % buffer_size];
    }

    delete wolf_problem_ptr_;

    return 0;
}

