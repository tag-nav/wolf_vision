/**
 * \file test_apriltag.cpp
 *
 *  Created on: Dec 14, 2018
 *      \author: Dinesh Atchtuhan
 */

//std
#include <iostream>
#include <stdlib.h>

//Wolf
#include "wolf.h"
#include "ceres_wrapper/ceres_manager.h"
#include "problem.h"
#include "sensor_camera.h"
#include "processors/processor_tracker_landmark_apriltag.h"
#include "capture_image.h"
#include "features/feature_apriltag.h"

#include "rotations.h"

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

#define CONTRAST
//#define IMAGE_OUTPUT
#define USEMAP


int main(int argc, char *argv[])
{
    /*
     * HOW TO USE ?
     * For now, just call the executable and append the list of images to be processed.
     * The images must be placed in the root folder of your wolf project.
     * Ex:
     * ./test_apriltag frame1.jpg frame2.jpg frame3.jpg
     */

    using namespace wolf;


    WOLF_INFO( "==================== processor apriltag test ======================" )

    std::string wolf_root = _WOLF_ROOT_DIR;
    // Wolf problem
    ProblemPtr problem              = Problem::create("PO 3D");
    ceres::Solver::Options options;
    options.function_tolerance = 1e-6;
    options.max_num_iterations = 100;
    CeresManagerPtr ceres_manager   = std::make_shared<CeresManager>(problem, options);


    WOLF_INFO( "====================    Configure Problem      ======================" )

    SensorBasePtr sen       = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_logitech_c300_640_480.yaml");
//    SensorBasePtr sen       = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_apriltag_params_notangentrect.yaml");
    SensorCameraPtr sen_cam = std::static_pointer_cast<SensorCamera>(sen);
    ProcessorBasePtr prc    = problem->installProcessor("TRACKER LANDMARK APRILTAG", "apriltags", "camera", wolf_root + "/src/examples/processor_tracker_landmark_apriltag.yaml");

#ifdef USEMAP
    problem->loadMap(wolf_root + "/src/examples/maps/map_apriltag_logitech_1234.yaml");
    for (auto lmk : problem->getMapPtr()->getLandmarkList()){
        lmk->fix();
    }
#endif

    // set prior
    Eigen::Matrix6s covariance = Matrix6s::Identity();
//    Scalar stdev_m   = 0.00001;  // standard deviation on original translation
//    Scalar stdev_deg = 0.00001;  // standard deviation on original rotation
    Scalar std_m   = 100;  // standard deviation on original translation
    Scalar std_deg = 10;  // standard deviation on original rotation
    covariance.topLeftCorner(3,3)       =  std_m*std_m * covariance.topLeftCorner(3,3);
    covariance.bottomRightCorner(3,3)   = (M_TORAD*std_deg)*(M_TORAD*std_deg) * covariance.bottomRightCorner(3,3);

//    FrameBasePtr F1 = problem->setPrior((Vector7s()<<0,0,0,0,0,0,1).finished(), covariance, 0.0, 0.1);
    FrameBasePtr F1 = problem->setPrior((Vector7s()<<0.08, 0, -0.75, 0, 0, 0, 1).finished(), covariance, 0.0, 0.1);

    // first argument is the name of the program.
    // following arguments are path to image (from wolf_root)
    const int inputs = argc -1;
    WOLF_DEBUG("nb of images: ", inputs);
    cv::Mat frame;
    Scalar ts(0);
    Scalar dt = 1;

    WOLF_INFO( "====================        Main loop       ======================" )
    for (int input = 1; input <= inputs; input++) {
        std::string path = wolf_root + "/" + argv[input];
        frame = cv::imread(path, CV_LOAD_IMAGE_COLOR);

        if( frame.data ) //if imread succeeded
        {

#ifdef CONTRAST
            Scalar alpha = 2.0; // to tune contrast  [1-3]
            int beta = 0;       // to tune lightness [0-100]
            // Do the operation new_image(i,j) = alpha*image(i,j) + beta
            for( int y = 0; y < frame.rows; y++ ){
                for( int x = 0; x < frame.cols; x++ ){
                    for( int c = 0; c < 3; c++ ){
                        frame.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>( alpha*( frame.at<cv::Vec3b>(y,x)[c] ) + beta );
                    }
                }
            }
#endif

            CaptureImagePtr cap = std::make_shared<CaptureImage>(ts, sen_cam, frame);
//            cap->setType(argv[input]); // only for problem->print() to show img filename
            cap->setName(argv[input]);
            WOLF_DEBUG("Processing image...", path);
            sen->process(cap);
#ifdef IMAGE_OUTPUT
    cv::namedWindow( cap->getName(), cv::WINDOW_NORMAL );// Create a window for display.
#endif
        }
        else
            WOLF_WARN("could not load image ", path);

        ts += dt;
    }


#ifdef IMAGE_OUTPUT
    WOLF_INFO( "====================    Draw all detections    ======================" )
    for (auto F : problem->getTrajectoryPtr()->getFrameList())
    {
        if (F->isKey())
        {
            for (auto cap : F->getCaptureList())
            {
                if (cap->getType() == "IMAGE")
                {
                    auto img = std::static_pointer_cast<CaptureImage>(cap);
                    for (FeatureBasePtr f : img->getFeatureList())
                    {
                        FeatureApriltagPtr fa = std::static_pointer_cast<FeatureApriltag>(f);
                        cv::line(img->getImage(), fa->getTagCorners()[0], fa->getTagCorners()[1], cv::Scalar(0, 255, 0));
                        cv::line(img->getImage(), fa->getTagCorners()[1], fa->getTagCorners()[2], cv::Scalar(0, 255, 0));
                        cv::line(img->getImage(), fa->getTagCorners()[2], fa->getTagCorners()[3], cv::Scalar(0, 255, 0));
                        cv::line(img->getImage(), fa->getTagCorners()[3], fa->getTagCorners()[0], cv::Scalar(0, 255, 0));
                    }
                    cv::imshow( img->getName(), img->getImage() );  // display original image.
                    cv::waitKey(0.1);
                }
            }
        }
    }
#endif



    WOLF_INFO( "====================    Provide perturbed prior    ======================" )
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
    {
        Vector7s x;
        if (kf->isKey())
        {
            x.setRandom();
            x.tail(4).normalize();
            kf->setState(x);
        }
    }

    WOLF_INFO( "====================    Solve problem    ======================" )
    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::FULL); // 0: nothing, 1: BriefReport, 2: FullReport
    WOLF_DEBUG(report);
    problem->print(4,1,1,1);



    WOLF_INFO("============= SOLVED PROBLEM : POS | EULER (DEG) ===============")
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
    {
        if (kf->isKey())
            for (auto cap : kf->getCaptureList())
            {
                if (cap->getType() != "POSE")
                {
                    Vector3s T = kf->getPPtr()->getState();
                    Vector4s qv= kf->getOPtr()->getState();
                    Vector3s e = M_TODEG * R2e(q2R(qv));
                    WOLF_DEBUG("KF", kf->id(), " => ", T.transpose(), " | ", e.transpose());
                }
            }
    }
    for (auto lmk : problem->getMapPtr()->getLandmarkList())
    {
        Vector3s T = lmk->getPPtr()->getState();
        Vector4s qv= lmk->getOPtr()->getState();
        Vector3s e = M_TODEG * R2e(q2R(qv));
        WOLF_DEBUG(" L", lmk->id(), " => ", T.transpose(), " | ", e.transpose());
    }


    // ===============================================
    // COVARIANCES ===================================
    // ===============================================
    // Print COVARIANCES of all states
    WOLF_INFO("======== COVARIANCES OF SOLVED PROBLEM : POS | QUAT =======")
    ceres_manager->computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
        if (kf->isKey())
        {
            Eigen::MatrixXs cov = kf->getCovariance();
            WOLF_DEBUG("KF", kf->id(), "_std (sigmas) = ", cov.diagonal().transpose().array().sqrt());
        }
    for (auto lmk : problem->getMapPtr()->getLandmarkList())
    {
        Eigen::MatrixXs cov = lmk->getCovariance();
        WOLF_DEBUG(" L", lmk->id(), "_std (sigmas) = ", cov.diagonal().transpose().array().sqrt());
    }
    std::cout << std::endl;


    // ===============================================
    // SAVE MAP TO YAML ==============================
    // ===============================================
    //
    //    problem->saveMap(wolf_root + "/src/examples/map_apriltag_set3_HC.yaml", "set3");

#ifdef IMAGE_OUTPUT
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

    return 0;

}
