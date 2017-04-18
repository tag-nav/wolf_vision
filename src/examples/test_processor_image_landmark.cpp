/**
 * \file test_processor_tracker_image_landmark.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: jtarraso
 */

//std
#include <iostream>

//Wolf
#include "wolf.h"
#include "problem.h"
#include "state_block.h"
#include "processor_image_landmark.h"
#include "capture_fix.h"
#include "processor_odom_3D.h"
#include "sensor_odom_3D.h"
#include "ceres_wrapper/ceres_manager.h"

using Eigen::Vector3s;
using Eigen::Vector4s;
using Eigen::Vector6s;
using Eigen::Vector7s;

using namespace wolf;

void cleanupMap(const ProblemPtr& _problem, const TimeStamp& _t, Scalar _dt_max,
                                      Size _min_constraints)
{
    std::list<LandmarkBasePtr> lmks_to_remove;
    for (auto lmk : _problem->getMapPtr()->getLandmarkList())
    {
        TimeStamp t0 = std::static_pointer_cast<LandmarkAHP>(lmk)->getAnchorFrame()->getTimeStamp();
        if (_t - t0 > _dt_max)
            if (lmk->getConstrainedByList().size() <= _min_constraints)
                lmks_to_remove.push_back(lmk);
    }

    for (auto lmk : lmks_to_remove)
    {
        WOLF_DEBUG("Clean up L" , lmk->id() );
        lmk->remove();
    }
}

int main(int argc, char** argv)
{
    std::cout << std::endl << "==================== processor image landmark test ======================" << std::endl;

    //=====================================================
    // Parse arguments
    cv::VideoCapture capture;
    const char * filename;
    if (argc == 1)
    {
//        filename = "/home/jtarraso/Videos/House_interior.mp4";
//        filename = "/home/jtarraso/Vídeos/gray1.mp4";
        filename = "/home/jtarraso/Escritorio/video_test2/sim_video.mpg";
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
    capture.set(CV_CAP_PROP_POS_MSEC, 6000);
    //=====================================================


    //=====================================================
    // Properties of video sequence
    unsigned int img_width  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int img_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;
    unsigned int buffer_size = 10;
    std::vector<cv::Mat> frame(buffer_size);
    //=====================================================


    //=====================================================
    // Environment variable for configuration files
    std::string wolf_root = _WOLF_ROOT_DIR;
    std::cout << wolf_root << std::endl;
    //=====================================================



    //=====================================================
    // Wolf problem
    ProblemPtr problem = Problem::create(FRM_PO_3D);

    // ODOM SENSOR AND PROCESSOR
    SensorBasePtr sensor_base        = problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    SensorOdom3DPtr sensor_odom      = std::static_pointer_cast<SensorOdom3D>(sensor_base);
    ProcessorBasePtr prcocessor_base = problem->installProcessor("ODOM 3D", "odometry integrator", "odom",               wolf_root + "/src/examples/processor_odom_3D.yaml");

    // CAMERA SENSOR AND PROCESSOR
    sensor_base            = problem->installSensor("CAMERA", "PinHole", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sensor_base);
    camera->setImgWidth(img_width);
    camera->setImgHeight(img_height);
    problem->installProcessor("IMAGE LANDMARK", "ORB", "PinHole", wolf_root + "/src/examples/processor_image_ORB.yaml");

    //=====================================================


    //=====================================================
    // Origin Key Frame is fixed
    TimeStamp t = 0;
    FrameBasePtr origin_frame = problem->emplaceFrame(KEY_FRAME, (Vector7s()<<1,0,0,0,0,0,1).finished(), t);
    problem->getProcessorMotionPtr()->setOrigin(origin_frame);
    origin_frame->fix();

    std::cout << "t: " << 0 << "  \t\t\t x = ( " << problem->getCurrentState().transpose() << ")" << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;
    //=====================================================


    //=====================================================
    // running CAPTURES preallocated
    CaptureImagePtr image;
    Vector6s data(Vector6s::Zero()); // will integrate this data repeatedly
    CaptureMotionPtr cap_odo = std::make_shared<CaptureMotion>(t, sensor_odom, data, 7, 6);
    //=====================================================



    //=====================================================
    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    //    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    //    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    ceres_options.max_num_iterations = 5;
    google::InitGoogleLogging(argv[0]);

    CeresManager ceres_manager(problem, ceres_options);
    //=====================================================


    //=====================================================
    // graphics
    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);
    //=====================================================


    //=====================================================
    // main loop
    unsigned int frame_count  = 1;
    capture >> frame[frame_count % buffer_size];
    unsigned int number_of_KFs = 0;

    Scalar dt = 0.04;

    while(!(frame[frame_count % buffer_size].empty()))
    {

        t += dt;

        // Image ---------------------------------------------

        // Preferred method with factory objects:
        image = std::make_shared<CaptureImage>(t, camera, frame[frame_count % buffer_size]);

        /* process */
        camera->process(image);



        // Odometry --------------------------------------------

        cap_odo->setTimeStamp(t);

        // previous state and TS
        Eigen::Vector7s x_prev = problem->getCurrentState();
        Vector7s x_prev_prev;
        Vector7s dx;

        // before the previous state
        FrameBasePtr prev_key_fr_ptr = problem->getLastKeyFramePtr();
        FrameBasePtr prev_prev_key_fr_ptr = nullptr;
        for (auto f_it = problem->getTrajectoryPtr()->getFrameList().rbegin(); f_it != problem->getTrajectoryPtr()->getFrameList().rend(); f_it++)
            if ((*f_it) == prev_key_fr_ptr)
            {
                f_it++;
                if (f_it != problem->getTrajectoryPtr()->getFrameList().rend())
                {
                    prev_prev_key_fr_ptr = (*f_it);
                }
                break;
            }

        // compute delta state, and odometry data
        if (!prev_prev_key_fr_ptr)
        {
            // we have just one state --> odometry data is zero
            data.setZero();
        }
        else
        {
            x_prev_prev = prev_prev_key_fr_ptr->getState();

            // define local variables on top of existing vectors to avoid memory allocation
            Eigen::Vector3s     p_prev_prev(x_prev_prev.data());
            Eigen::Quaternions  q_prev_prev(x_prev_prev.data() + 3);
            Eigen::Vector3s     p_prev(x_prev.data());
            Eigen::Quaternions  q_prev(x_prev.data() + 3);
            Eigen::Vector3s     dp(dx.data());
            Eigen::Quaternions  dq(dx.data() + 3);

            // delta state PQ
            dp = q_prev_prev.conjugate() * (p_prev - p_prev_prev);
            dq = q_prev_prev.conjugate() * q_prev;

            // odometry data
            data.head<3>() = dp;
            data.tail<3>() = q2v(dq);
        }


        cap_odo->setData(data);

        sensor_odom->process(cap_odo);

//        problem->print(2,1,0,0);

//        std::cout << "prev prev ts: " << t_prev_prev.get() << "; x: " << x_prev_prev.transpose() << std::endl;
//        std::cout << "prev      ts: " << t_prev.get() << "; x: " << x_prev.transpose() << std::endl;
//        std::cout << "current   ts: " << t.get() << std::endl;
//        std::cout << "          dt: " << t_prev - t_prev_prev << "; dx: " << dx.transpose() << std::endl;


        // Cleanup map ---------------------------------------

        cleanupMap(problem, t, 2, 5); // dt, min_ctr


        // Solve -----------------------------------------------
        // solve only when new KFs are added
        if (problem->getTrajectoryPtr()->getFrameList().size() > number_of_KFs)
        {
            number_of_KFs = problem->getTrajectoryPtr()->getFrameList().size();
            ceres::Solver::Summary summary = ceres_manager.solve();
            std::cout << summary.BriefReport() << std::endl;
        }


        // Finish loop -----------------------------------------

        cv::waitKey(10);

        std::cout << "=================================================================================================" << std::endl;

        frame_count++;
        capture >> frame[frame_count % buffer_size];
    }

    // problem->print(2);
    problem.reset();

    return 0;
}

