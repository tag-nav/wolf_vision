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
//#include "sensor_base.h"
#include "state_block.h"
#include "processor_image_landmark.h"
//#include "capture_void.h"
#include "processor_odom_3D.h"
#include "sensor_odom_3D.h"
#include "ceres_wrapper/ceres_manager.h"


using Eigen::Vector3s;
using Eigen::Vector4s;
using Eigen::Vector6s;
using Eigen::Vector7s;

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor image landmark test ======================" << std::endl;

    cv::VideoCapture capture;
    const char * filename;
    if (argc == 1)
    {
//        filename = "/home/jtarraso/Videos/House_interior.mp4";
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

    TimeStamp t = 99;

    char const* tmp = std::getenv( "WOLF_ROOT" );
    if ( tmp == nullptr )
        throw std::runtime_error("WOLF_ROOT environment not loaded.");
    std::string wolf_path( tmp );
    std::cout << "Wolf path: " << wolf_path << std::endl;


    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = new Problem(FRM_PO_3D);

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
    //    ProcessorParamsImage tracker_params;
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

    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", extr, wolf_path + "/src/examples/camera_params.yaml");
    SensorCamera* camera_ptr = (SensorCamera*)sensor_ptr;
    camera_ptr->setImgWidth(img_width);
    camera_ptr->setImgHeight(img_height);


    // PROCESSOR
    // one-liner API
    ProcessorImageLandmark* prc_img_ptr = (ProcessorImageLandmark*) wolf_problem_ptr_->installProcessor("IMAGE LANDMARK", "ORB", "PinHole", wolf_path + "/src/examples/processor_image_ORB.yaml");
    prc_img_ptr->setup(camera_ptr);
    std::cout << "sensor & processor created and added to wolf problem" << std::endl;
    //=====================================================

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    SensorBase* sen_odo_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(),"");
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", "odom", "");
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), t);

    Vector6s data(Vector6s::Zero()); // will integrate this data repeatedly
    CaptureMotion* cap_odo = new CaptureMotion(TimeStamp(0), sen_odo_ptr, data);

    std::cout << "t: " << 0 << "  \t\t\t x = ( " << wolf_problem_ptr_->getCurrentState().transpose() << ")" << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;

    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    //    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    //    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    //    google::InitGoogleLogging(argv[0]);

    CeresManager ceres_manager(wolf_problem_ptr_, ceres_options);







    // CAPTURES
    CaptureImage* image_ptr;

    unsigned int f  = 1;
    capture >> frame[f % buffer_size];

    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);

    Scalar dt = 0.04;


    while(!(frame[f % buffer_size].empty()))
    {
        t += dt;

        // Odometry
        cap_odo->setTimeStamp(t);

        // previous state and TS
        Eigen::VectorXs x_prev(7);
        TimeStamp t_prev;
        wolf_problem_ptr_->getCurrentState(x_prev, t_prev);

        // before the previous state
        FrameBaseList::iterator f_it = wolf_problem_ptr_->getTrajectoryPtr()->getFrameListPtr()->begin();
        std::cout << (*f_it)->getTimeStamp().get() << std::endl;
        f_it--;

        // compute delta state, and odometry data
        if (f_it != wolf_problem_ptr_->getTrajectoryPtr()->getFrameListPtr()->end())
        {
            // we have two states
            std::cout << (*f_it)->getTimeStamp().get() << std::endl;
            Vector7s x_prev_prev = (*f_it)->getState();

            // some maps to avoid local variables
            Eigen::Map<Eigen::Vector3s>     p1(x_prev_prev.data());
            Eigen::Map<Eigen::Quaternions>  q1(x_prev_prev.data() + 3);
            Eigen::Map<Eigen::Vector3s>     p2(x_prev.data());
            Eigen::Map<Eigen::Quaternions>  q2(x_prev.data() + 3);

            // delta state PQ
            Eigen::Vector3s dp = q1.conjugate() * (p2 - p1);
            Eigen::Quaternions dq = q1.conjugate() * q2;
            Eigen::Vector3s dtheta = q2v(dq);

            // odometry data
            data.head<3>() = dp;
            data.tail<3>() = dtheta;

        }
        else
        {
            // we have just one state --> odometry data is zero
            data.setZero();
        }
        cap_odo->setData(data);

        cap_odo->process();

        wolf_problem_ptr_->print();



        // Image

        clock_t t1 = clock();

        // Preferred method with factory objects:
        image_ptr = new CaptureImage(t, camera_ptr, frame[f % buffer_size]);

        /* process */
        image_ptr->process();

        wolf_problem_ptr_->print();


        std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
        cv::waitKey(20);

        if((f%buffer_size) == 10)
        {
            ceres::Solver::Summary summary = ceres_manager.solve();
            std::cout << summary.FullReport() << std::endl;


            std::cout << "Last key frame pose: "
                      << wolf_problem_ptr_->getLastKeyFramePtr()->getPPtr()->getVector().transpose() << std::endl;
            std::cout << "Last key frame orientation: "
                      << wolf_problem_ptr_->getLastKeyFramePtr()->getOPtr()->getVector().transpose() << std::endl;

            //cv::waitKey(0);
        }

        std::cout << "END OF ITERATION\n=================================" << std::endl;

        f++;
        capture >> frame[f % buffer_size];
    }

    delete wolf_problem_ptr_;

    return 0;
}

