// Testing things for the 3D image odometry

//Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "processor_image_feature.h"
#include "ceres_wrapper/ceres_manager.h"

//#include "feature_point_image.h"
//#include "state_block.h"
//#include "state_quaternion.h"
//#include "factory.h"

// opencv
//#include "opencv2/calib3d/calib3d.hpp"

// general includes
//#include "unistd.h"
//#include <time.h>
//#include <uEye.h> //used to use the camera?

//std includes
#include <ctime>
#include <iostream>


int main(int argc, char** argv)
{
    using namespace wolf;

    //ProcessorImageFeature test
    std::cout << std::endl << " ========= ProcessorImageFeature test ===========" << std::endl << std::endl;

    cv::VideoCapture capture;
    const char * filename;
    if (argc == 1)
    {
        filename = "/home/jtarraso/Videos/House_interior.mp4";
        //filename = "/home/jtarraso/Vídeos/gray.mp4";
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

    char const* tmp = std::getenv( "WOLF_ROOT" );
    if ( tmp == nullptr )
        throw std::runtime_error("WOLF_ROOT environment not loaded.");

    std::string wolf_path( tmp );
    std::cout << "Wolf path: " << wolf_path << std::endl;

    ProblemPtr wolf_problem_ptr_ = new Problem(FRM_PO_3D);

    //=====================================================
    // Method 1: Use data generated here for sensor and processor
    //=====================================================

    //    // SENSOR
    //    Eigen::Vector4s k = {320,240,320,320};
    //    SensorCamera* sen_cam_ = new SensorCamera(new StateBlock(Eigen::Vector3s::Zero()),
    //                                              new StateBlock(Eigen::Vector3s::Zero()),
    //                                              new StateBlock(k,false),img_width,img_height);
    //
    //    wolf_problem_->getHardwarePtr()->addSensor(sen_cam_);
    //
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
    //
    //    DetectorDescriptorParamsOrb orb_params;
    //    orb_params.type = DD_ORB;
    //
    //    DetectorDescriptorParamsBrisk brisk_params;
    //    brisk_params.type = DD_BRISK;
    //
    //    // select the kind of detector-descriptor parameters
    //    tracker_params.detector_descriptor_params_ptr = &orb_params; // choose ORB
    //
    //    ProcessorImageFeature* prc_image = new ProcessorImageFeature(tracker_params);
    //
    //    sen_cam_->addProcessor(prc_image);
    //=====================================================


    //=====================================================
    // Method 2: Use factory to create sensor and processor
    //=====================================================

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

    // SENSOR
    // one-liner API
    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", extr, wolf_path + "/src/examples/camera_params.yaml");
    SensorCamera* camera_ptr = (SensorCamera*)sensor_ptr;

    // PROCESSOR
    // one-liner API
    wolf_problem_ptr_->installProcessor("IMAGE FEATURE", "ORB", "PinHole", wolf_path + "/src/examples/processor_image_ORB.yaml");
    std::cout << "sensor & processor created and added to wolf problem" << std::endl;
    //=====================================================




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
        std::cout << "\n=============== Frame #: " << f << " in buffer: " << f%buffer_size << " ===============" << std::endl;

        t.setToNow();
        clock_t t1 = clock();

        // Old method with non-factory objects
        //        capture_image_ptr = new CaptureImage(t, sen_cam_,frame[f % buffer_size]);
        //        prc_image->process(capture_image_ptr);

        // Preferred method with factory objects:
        image_ptr = new CaptureImage(t, camera_ptr, frame[f % buffer_size]);
        image_ptr->process();


        std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
        cv::waitKey(1);

//        if((f%buffer_size) == 4)
//        {
//            ceres::Solver::Summary summary = ceres_manager.solve();
//            std::cout << summary.FullReport() << std::endl;


//            std::cout << "Last key frame pose: "
//                      << wolf_problem_ptr_->getLastKeyFramePtr()->getPPtr()->getVector().transpose() << std::endl;
//            std::cout << "Last key frame orientation: "
//                      << wolf_problem_ptr_->getLastKeyFramePtr()->getOPtr()->getVector().transpose() << std::endl;

//            cv::waitKey(0);
//        }

        f++;
        capture >> frame[f % buffer_size];
    }

    wolf_problem_ptr_->destruct();
}
