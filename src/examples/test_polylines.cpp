//std includes
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//Ceres includes
#include "glog/logging.h"

//Wolf includes
#include "problem.h"
#include "processor_tracker_landmark_polyline.h"
#include "processor_odom_2D.h"
#include "sensor_laser_2D.h"
#include "sensor_odom_2D.h"
#include "sensor_gps_fix.h"
#include "capture_fix.h"
#include "capture_odom_2D.h"
#include "ceres_wrapper/ceres_manager.h"

// laserscanutils
#include "laser_scan_utils/line_finder_iterative.h"
#include "laser_scan_utils/laser_scan.h"

//C includes for sleep, time and main args
#include "unistd.h"

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << "\n============================================================\n";
    std::cout << "========== 2D Polylines test =============\n";

    // USER INPUT ============================================================================================
//    if (argc != 2 || atoi(argv[1]) < 1 )
//    {
//        std::cout << "Please call me with: [./test_ceres_manager NI PRINT], where:" << std::endl;
//        std::cout << "     - NI is the number of iterations (NI > 0)" << std::endl;
//        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
//        return -1;
//    }


    // INITIALIZATION ============================================================================================

    // Wolf Tree initialization
    Problem problem(FRM_PO_2D);

    // odometry
    std::cout << "Install odometry:" << std::endl;
    IntrinsicsOdom2D odom_intrinsics;
    odom_intrinsics.k_disp_to_disp = 0.1;
    odom_intrinsics.k_rot_to_rot = 0.1;
    SensorBase* odom_sensor_ptr = problem.installSensor("ODOM 2D", "odometer", Eigen::VectorXs::Zero(3), &odom_intrinsics);

    ProcessorParamsOdom2D odom_params;
    odom_params.dist_traveled_th_ = 10;
    odom_params.cov_det_th_ = 1;
    ProcessorOdom2D* odom_processor_ptr_ = (ProcessorOdom2D*)(problem.installProcessor("ODOM 2D", "main odometry", odom_sensor_ptr, &odom_params));
    CaptureMotion2* odom_capture_ptr_ = new CaptureMotion2(TimeStamp(), odom_sensor_ptr, Eigen::Vector2s(1,0), 0.1*Eigen::Matrix2s::Identity(), nullptr);

    // laser
    std::cout << "Install laser:" << std::endl;
    Eigen::VectorXs lidar_extrinsics(3);
    lidar_extrinsics << 3, 2, M_PI/2;
    IntrinsicsLaser2D laser_intrinsics;
    SensorLaser2D* laser_sensor_ptr = (SensorLaser2D*)problem.installSensor("LASER 2D", "laser", lidar_extrinsics, &laser_intrinsics);

    ProcessorParamsPolyline laser_processor_params;
    laser_processor_params.line_finder_params_ = laserscanutils::LineFinderIterativeParams({0.1, 5});
    laser_processor_params.new_features_th = 10;
    laser_processor_params.loop_frames_th = 100;
    ProcessorTrackerLandmarkPolyline* laser_processor = (ProcessorTrackerLandmarkPolyline*)problem.installProcessor("POLYLINE", "laser_processor", laser_sensor_ptr, &laser_processor_params);

    // Set origin
    std::cout << "Set origin:" << std::endl;
    Eigen::VectorXs origin = Eigen::VectorXs::Zero(3);
    origin << 9, 5, M_PI/2;
    Eigen::MatrixXs origin_cov_ = Eigen::MatrixXs::Identity(3,3) * 0.1;
    problem.setOrigin(origin, origin_cov_, TimeStamp(0));

    // Ceres wrapper
    std::cout << "Ceres wrapper:" << std::endl;
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    google::InitGoogleLogging(argv[0]);
    CeresManager ceres_manager(&problem, ceres_options);



    // START TEST ---------------------------

    // First capture
    std::cout << "---------------- 1st capture:" << std::endl;
    CaptureBase* capture = new CaptureBase(TimeStamp(0.1), laser_sensor_ptr);
    // Create a sintetic feature polyline
    Eigen::MatrixXs points(3,2);
    points.col(0) << 4, 2, 1;
    points.col(1) << 5, 3, 1;
    Eigen::MatrixXs points_cov(2,4);
    points_cov.leftCols(2) = Eigen::MatrixXs::Identity(2,2);
    points_cov.rightCols(2) = Eigen::MatrixXs::Identity(2,2);
    laser_processor->polylines_incoming_.push_back(new FeaturePolyline2D(points, points_cov, false, false));
    capture->process();

    std::cout << "Landmarks: " << std::endl;
    auto i = 1;
    for (auto lmk : *problem.getMapPtr()->getLandmarkListPtr())
    {
        std::cout << "\tLandmark " << i++ << ": [" <<((LandmarkPolyline2D*)lmk)->isFirstDefined() << "," << ((LandmarkPolyline2D*)lmk)->isLastDefined() << "]" << std::endl;
        std::cout << "\t\tConstrained by: " << lmk->getHits() << std::endl;
        auto j = 1;
        for (auto point : ((LandmarkPolyline2D*)lmk)->getPointStatePtrDeque())
            std::cout << "\t\tpoint " << j++ << ": " << point->getVector().transpose() << std::endl;
    }

    // motion
    std::cout << "---------------- motion:" << std::endl;
    odom_capture_ptr_->setTimeStamp(0.2);
    odom_capture_ptr_->process();

    // Second capture
    std::cout << "---------------- 2nd capture:" << std::endl;
    points.col(0) << 4.1, 2.9, 1;
    points.col(1) << 6, 5, 1;
    capture = new CaptureBase(TimeStamp(0.2), laser_sensor_ptr);
    laser_processor->polylines_incoming_.push_back(new FeaturePolyline2D(points, points_cov, false, true));
    capture->process();

    // Key frame
    std::cout << "---------------- key frame:" << std::endl;
    FrameBase* key_ptr = problem.createFrame(KEY_FRAME, TimeStamp(0.2));
    laser_processor->keyFrameCallback(key_ptr,0.05);
    odom_processor_ptr_->keyFrameCallback(key_ptr,0.05);

    std::cout << "Landmarks: " << std::endl;
    i = 1;
    for (auto lmk : *problem.getMapPtr()->getLandmarkListPtr())
    {
        std::cout << "\tLandmark " << i++ << ": [" <<((LandmarkPolyline2D*)lmk)->isFirstDefined() << "," << ((LandmarkPolyline2D*)lmk)->isLastDefined() << "]" << std::endl;
        std::cout << "\t\tConstrained by: " << lmk->getHits() << std::endl;
        auto j = 1;
        for (auto point : ((LandmarkPolyline2D*)lmk)->getPointStatePtrDeque())
            std::cout << "\t\tpoint " << j++ << ": " << point->getVector().transpose() << std::endl;
    }

    // Third capture
    std::cout << "---------------- 3rd capture:" << std::endl;
    points.col(0) << 6.05, 5.2, 1;
    points.col(1) << 5, 6, 1;
    capture = new CaptureBase(TimeStamp(0.2), laser_sensor_ptr);
    laser_processor->polylines_incoming_.push_back(new FeaturePolyline2D(points, points_cov, true, true));
    capture->process();

    // Key frame
    std::cout << "---------------- key frame:" << std::endl;
    key_ptr = problem.createFrame(KEY_FRAME, TimeStamp(0.2));
    laser_processor->keyFrameCallback(key_ptr,0.05);
    odom_processor_ptr_->keyFrameCallback(key_ptr,0.05);

    std::cout << "Landmarks: " << std::endl;
    i = 1;
    for (auto lmk : *problem.getMapPtr()->getLandmarkListPtr())
    {
        std::cout << "\tLandmark " << i++ << ": [" <<((LandmarkPolyline2D*)lmk)->isFirstDefined() << "," << ((LandmarkPolyline2D*)lmk)->isLastDefined() << "]" << std::endl;
        std::cout << "\t\tConstrained by: " << lmk->getHits() << std::endl;
        auto j = 1;
        for (auto point : ((LandmarkPolyline2D*)lmk)->getPointStatePtrDeque())
            std::cout << "\t\tpoint " << j++ << ": " << point->getVector().transpose() << std::endl;
    }

//    std::cout << "Compute Transformations:" << std::endl;
//    laser_processor->computeTransformations(TimeStamp(0));
//
//    // Create a sintetic feature polyline
//    std::cout << "Create a polyline feature:" << std::endl;
//    Eigen::MatrixXs points(3,2);
//    points.col(0) << 4, 2, 1;
//    points.col(1) << 5, 3, 1;
//    Eigen::MatrixXs points_cov(2,4);
//    points_cov.leftCols(2) = Eigen::MatrixXs::Identity(2,2);
//    points_cov.rightCols(2) = Eigen::MatrixXs::Identity(2,2);
//    FeaturePolyline2D polyline_ftr(points, points_cov, false, false);
//    std::cout << "points:" << std::endl;
//    std::cout << points.topRows<2>().transpose() << std::endl;
//
//    // Create a landmark polyline
//    std::cout << "Create a polyline landmark from it:" << std::endl;
//    LandmarkPolyline2D* polyline_lmk = (LandmarkPolyline2D*)(laser_processor->createLandmark(&polyline_ftr));
//    std::cout << "points:" << std::endl;
//    auto i = 0;
//    for (auto point : polyline_lmk->getPointStatePtrDeque())
//        std::cout << "point " << i++ << ": " << point->getVector().transpose() << std::endl;
//
//
//    // Create new landmarks
//    std::cout << "Create new landmarks:" << std::endl;
//    laser_processor->last_ptr_ = new CaptureBase(TimeStamp(0), laser_sensor_ptr);
//    laser_processor->polylines_last_.push_back(new FeaturePolyline2D(points, points_cov, false, false));
//    laser_processor->processNew(0);
//    for (auto lmk : *problem.getMapPtr()->getLandmarkListPtr())
//        for (auto point : ((LandmarkPolyline2D*)lmk)->getPointStatePtrDeque())
//            std::cout << "point " << i++ << ": " << point->getVector().transpose() << std::endl;
//    laser_processor->detectNewFeatures(0);
//    LandmarkBaseList new_landmarks_list;
//    laser_processor->createNewLandmarks(new_landmarks_list);
//    i = 0;
//    for (auto lmk : new_landmarks_list)
//        for (auto point : ((LandmarkPolyline2D*)lmk)->getPointStatePtrDeque())
//            std::cout << "point " << i++ << ": " << point->getVector().transpose() << std::endl;
//    problem.getMapPtr()->addLandmarkList(new_landmarks_list);

//    // Expected fetaure
//    std::cout << "Expected feature:" << std::endl;
//    Eigen::MatrixXs expected_points(3,2);
//    Eigen::MatrixXs expected_points_cov(2,4);
//    laser_processor->expectedFeature(polyline_lmk, expected_points, expected_points_cov);
//    std::cout << "points:" << std::endl;
//        std::cout << expected_points.topRows<2>().transpose() << std::endl;
//    std::cout << "points cov:" << std::endl;
//        std::cout << expected_points_cov << std::endl;
//
//    // Matching
//    points.col(0) << 4.1, 1.9, 1;
//    points.col(1) << 6, 4, 1;
//    laser_processor->polylines_incoming_.push_back(new FeaturePolyline2D(points, points_cov, false, false));
//    FeatureBaseList found_features;
//    LandmarkMatchMap found_matches;
//    laser_processor->findLandmarks(*problem.getMapPtr()->getLandmarkListPtr(), found_features, laser_processor->matches_landmark_from_incoming_);
//
//    for (auto feat : found_features)
//        std::cout << "Matched points:" << std::endl << ((FeaturePolyline2D*)feat)->getPoints().transpose() << std::endl;
//
//    laser_processor->advance();


    std::cout << " ========= END ===========" << std::endl << std::endl;

    //exit
    return 0;
}
