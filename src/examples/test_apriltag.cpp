/**
 * \file test_apriltag.cpp
 *
 *  Created on: Dec 14, 2018
 *      \author: Dinesh Atchtuhan
 */

//std
#include <iostream>
#include <stdlib.h>

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

//Wolf
#include "wolf.h"
#include "ceres_wrapper/ceres_manager.h"
#include "problem.h"
#include "sensor_camera.h"
#include "state_block.h"
#include "processors/processor_tracker_landmark_apriltag.h"
#include "capture_image.h"

#include "rotations.h"

//#define IMAGE_OUTPUT

bool str_ends_with (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char *argv[])
{
    /*
     * HOW TO USE ?
     * For now, just call the executable and append the list of images to be processed
     * For example, if you want to process only one image located at wolf/bin/images/frame1.jpg
     * and if wolf_root is correctly set. then just run (from wolf/bin)
     * ./test_apriltag /bin/images/frame1.jpg
     */

    using namespace wolf;

#ifdef IMAGE_OUTPUT
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
#endif

    WOLF_INFO( "==================== processor apriltag test ======================" )

    std::string wolf_root = _WOLF_ROOT_DIR;
    // Wolf problem
    ProblemPtr problem = Problem::create("PO 3D");
//    SensorBasePtr sen = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_logitech_c300_640_480.yaml");
    SensorBasePtr sen = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_apriltag_params_notangentrect.yaml");
    SensorCameraPtr sen_cam = std::static_pointer_cast<SensorCamera>(sen);
    ProcessorBasePtr prc = problem->installProcessor("TRACKER LANDMARK APRILTAG", "apriltags", "camera", wolf_root + "/src/examples/processor_tracker_landmark_apriltag.yaml");

    // set prior
    Eigen::Matrix6s covariance = Matrix6s::Identity();
    Scalar stdev_m   = 0.00001;  // standard deviation on original translation
    Scalar stdev_deg = 0.00001;  // standard deviation on original rotation
    covariance.topLeftCorner(3,3)       =  stdev_m*stdev_m * covariance.topLeftCorner(3,3);
    covariance.bottomRightCorner(3,3)   = (M_TORAD*stdev_deg)*(M_TORAD*stdev_deg) * covariance.bottomRightCorner(3,3);
    FrameBasePtr F1 = problem->setPrior((Vector7s()<<0,0,0,0,0,0,1).finished(), covariance, 0.0, 0.1);
    ceres::Solver::Options options;
    CeresManagerPtr ceres_manager = std::make_shared<CeresManager>(problem, options);

    WOLF_INFO( "====================       Problem set      ======================" )

    // first argument is the name of the program.
    // following arguments are path to image (from wolf_root)
    const int inputs = argc -1;
    WOLF_DEBUG("nb of images: ", inputs);
    cv::Mat frame;
    Scalar ts(0);

    WOLF_INFO( "====================        Main loop       ======================" )
    Scalar dt = 1;
    for (int input = 1; input <= inputs; input++) {
        std::string path = wolf_root + "/" + argv[input];
        WOLF_DEBUG("path to image ", path);
        frame = cv::imread(path, CV_LOAD_IMAGE_COLOR);

        if( frame.data ) //if imread succeeded
        {
#ifdef IMAGE_OUTPUT
            sleep(0.5); //wait for 0.5 s
            cv::imshow( "Display window", frame );  // display original image.
#endif
            CaptureImagePtr cap = std::make_shared<CaptureImage>(ts, sen_cam, frame);
            cap->setType(argv[input]);
            WOLF_DEBUG("Processing image...");
            sen->process(cap);
            WOLF_DEBUG("Image processed...");
        }
        else
            WOLF_WARN("could not load image ", path);
        ts += dt;
    }

    problem->print(1,1,1,0);
    WOLF_INFO( "====================    Solving problem    ======================" )
    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::FULL); // 0: nothing, 1: BriefReport, 2: FullReport
    WOLF_TRACE(report);
    problem->print(2,0,1,0);



    WOLF_TRACE("============= SOLVED PROBLEM : POS | EULER (DEG) ===============")
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
    {
        if (kf->isKey())
            for (auto cap : kf->getCaptureList())
            {
                if (cap->getType() != "POSE")
                {
                    Vector3s T = kf->getPPtr()->getState();
                    Vector4s qv= kf->getOPtr()->getState();
                    Matrix3s R = q2R(qv);
                    Vector3s e = M_TODEG * R2e(R);
                    WOLF_TRACE(cap->getType(), " => ", T.transpose(), " | ", e.transpose());
                }
            }
    }


    // ===============================================
    // COVARIANCES ===================================
    // ===============================================
    // Print COVARIANCES of all states
    WOLF_TRACE("======== STATE AND COVARIANCES OF SOLVED PROBLEM : POS | QUAT =======")
    ceres_manager->computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
        if (kf->isKey())
        {
            Eigen::MatrixXs cov = kf->getCovariance();
            WOLF_TRACE("KF", kf->id(), "_state        = ", kf->getState().transpose());
            WOLF_TRACE("KF", kf->id(), "_std (sigmas) = ", cov.diagonal().transpose().array().sqrt());
        }
    for (auto lmk : problem->getMapPtr()->getLandmarkList())
    {
        Eigen::MatrixXs cov = lmk->getCovariance();
        WOLF_TRACE("L", lmk->id(), "__state        = ", lmk->getState().transpose());
        WOLF_TRACE("L", lmk->id(), "__std (sigmas) = ", cov.diagonal().transpose().array().sqrt());
    }
    std::cout << std::endl;


    return 0;

}
