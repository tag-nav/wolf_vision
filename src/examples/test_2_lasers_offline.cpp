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
#include "processor_tracker_landmark_corner.h"
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

//faramotics includes
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

void extractVector(std::ifstream& text_file, Eigen::VectorXs& vector, wolf::Scalar& ts)
{
    std::string line;
    std::getline(text_file, line);
    std::stringstream line_stream(line);
    line_stream >> ts;
    for (auto i = 0; i < vector.size(); i++)
        line_stream >> vector(i);
}

void extractScan(std::ifstream& text_file, std::vector<float>& scan, wolf::Scalar& ts)
{
    std::string line;
    std::getline(text_file, line);
    std::stringstream line_stream(line);
    line_stream >> ts;
    for (unsigned int i = 0; i < scan.size(); i++)
        line_stream >> scan[i];
}

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << "\n==================================================================\n";
    std::cout << "========== 2D Robot with offline odometry and 2 LIDARs =============\n";

    // USER INPUT ============================================================================================
    if (argc != 2 || atoi(argv[1]) < 1 )
    {
        std::cout << "Please call me with: [./test_ceres_manager NI PRINT], where:" << std::endl;
        std::cout << "     - NI is the number of iterations (NI > 0)" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    // FILES INPUT ============================================================================================
    std::ifstream laser_1_file, laser_2_file, odom_file, groundtruth_file;
    laser_1_file.open("simulated_laser_1.txt", std::ifstream::in);
    laser_2_file.open("simulated_laser_2.txt", std::ifstream::in);
    odom_file.open("simulated_odom.txt", std::ifstream::in);
    groundtruth_file.open("simulated_groundtruth.txt", std::ifstream::in);

    if (!(laser_1_file.is_open() && laser_2_file.is_open() && odom_file.is_open() && groundtruth_file.is_open()))
    {
        std::cout << "Error opening simulated data files. Remember to run test_faramotics_simulation before this test!" << std::endl;
        return -1;
    }
    else
        std::cout << "Simulated data files opened correctly..." << std::endl;


    unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution

    // INITIALIZATION ============================================================================================
    //init random generators
    Scalar odom_std_factor = 0.5;
    Scalar gps_std = 1;
    std::default_random_engine generator(1);
    std::normal_distribution<Scalar> distribution_odom(0.0, odom_std_factor); //odometry noise
    std::normal_distribution<Scalar> distribution_gps(0.0, gps_std); //GPS noise

    //variables
    std::string line;
    Eigen::VectorXs odom_data = Eigen::VectorXs::Zero(2);
    Eigen::Vector2s gps_fix_reading;
    Eigen::VectorXs ground_truth(n_execution * 3); //all true poses
    Eigen::VectorXs ground_truth_pose(3); //last true pose
    Eigen::VectorXs odom_trajectory(n_execution * 3); //open loop trajectory
    Eigen::VectorXs mean_times = Eigen::VectorXs::Zero(6);
    clock_t t1, t2;
    Scalar timestamp;
    TimeStamp ts(0);

    // Wolf Tree initialization
    Eigen::Vector3s odom_pose = Eigen::Vector3s::Zero();
    Eigen::Vector3s gps_pose = Eigen::Vector3s::Zero();
    Eigen::VectorXs laser_1_params(9), laser_2_params(9);
    Eigen::VectorXs laser_1_pose(4), laser_2_pose(4); //xyz + theta

    // laser 1 extrinsics and intrinsics
    extractVector(laser_1_file, laser_1_params, timestamp);
    extractVector(laser_1_file, laser_1_pose, timestamp);
    std::vector<float> scan1(laser_1_params(8)); // number of ranges in a scan

    // laser 2 extrinsics and intrinsics
    extractVector(laser_2_file, laser_2_params, timestamp);
    extractVector(laser_2_file, laser_2_pose, timestamp);
    std::vector<float> scan2(laser_2_params(8));

    Problem* problem = new Problem(FRM_PO_2D);
    SensorOdom2D* odom_sensor = new SensorOdom2D(new StateBlock(odom_pose.head(2), true), new StateBlock(odom_pose.tail(1), true), odom_std_factor, odom_std_factor);
    SensorGPSFix* gps_sensor = new SensorGPSFix(new StateBlock(gps_pose.head(2), true), new StateBlock(gps_pose.tail(1), true), gps_std);
    SensorLaser2D* laser_1_sensor = new SensorLaser2D(new StateBlock(laser_1_pose.head(2), true), new StateBlock(laser_1_pose.tail(1), true), laserscanutils::LaserScanParams({laser_1_params(0), laser_1_params(1), laser_1_params(2), laser_1_params(3), laser_1_params(4), laser_1_params(5), laser_1_params(6), laser_1_params(7)}));
    SensorLaser2D* laser_2_sensor = new SensorLaser2D(new StateBlock(laser_2_pose.head(2), true), new StateBlock(laser_2_pose.tail(1), true), laserscanutils::LaserScanParams({laser_1_params(0), laser_1_params(1), laser_1_params(2), laser_1_params(3), laser_1_params(4), laser_1_params(5), laser_1_params(6), laser_1_params(7)}));
    ProcessorTrackerLandmarkCorner* laser_1_processor = new ProcessorTrackerLandmarkCorner(laserscanutils::LineFinderIterativeParams({0.1, 5}), 3);
    ProcessorTrackerLandmarkCorner* laser_2_processor = new ProcessorTrackerLandmarkCorner(laserscanutils::LineFinderIterativeParams({0.1, 5}), 3);
    ProcessorOdom2D* odom_processor = new ProcessorOdom2D();
    odom_sensor->addProcessor(odom_processor);
    laser_1_sensor->addProcessor(laser_1_processor);
    laser_2_sensor->addProcessor(laser_2_processor);
    problem->addSensor(odom_sensor);
    problem->addSensor(gps_sensor);
    problem->addSensor(laser_1_sensor);
    problem->addSensor(laser_2_sensor);
    problem->setProcessorMotion(odom_processor);

    std::cout << "Wolf tree setted correctly!" << std::endl;

    CaptureMotion2* odom_capture = new CaptureMotion2(ts, odom_sensor, odom_data, Eigen::Matrix2s::Identity() * odom_std_factor * odom_std_factor);

    // Initial pose
    ground_truth_pose << 2, 8, 0;
    ground_truth.head(3) = ground_truth_pose;
    odom_trajectory.head(3) = ground_truth_pose;

    // Origin Key Frame
    FrameBase* origin_frame = problem->createFrame(KEY_FRAME, ground_truth_pose, ts);

    // Prior covariance
    CaptureFix* initial_covariance = new CaptureFix(ts, gps_sensor, ground_truth_pose, Eigen::Matrix3s::Identity() * 0.1);
    origin_frame->addCapture(initial_covariance);
    initial_covariance->process();

    odom_processor->setOrigin(origin_frame, ts);

    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    google::InitGoogleLogging(argv[0]);

    CeresManager* ceres_manager = new CeresManager(problem);
    std::ofstream log_file, landmark_file;  //output file

    std::cout << "START TRAJECTORY..." << std::endl;
    // START TRAJECTORY ============================================================================================
    for (unsigned int step = 1; step < n_execution; step++)
    {
        //get init time
        t2 = clock();

        // GROUNDTRUTH ---------------------------
        //std::cout << "GROUND TRUTH..." << std::endl;
        t1 = clock();
        extractVector(groundtruth_file, ground_truth_pose, timestamp);
        ground_truth.segment(step * 3, 3) = ground_truth_pose;

        // timestamp
        ts = TimeStamp(timestamp);

        // ODOMETRY DATA -------------------------------------
        //std::cout << "ODOMETRY DATA..." << std::endl;
        extractVector(odom_file, odom_data, timestamp);
        // noisy odometry
        odom_data(0) += distribution_odom(generator) * (odom_data(0) == 0 ? 1e-6 : odom_data(0));
        odom_data(1) += distribution_odom(generator) * (odom_data(1) == 0 ? 1e-6 : odom_data(1));
        // process odometry
        odom_capture->setTimeStamp(TimeStamp(ts));
        odom_capture->setData(odom_data);
        odom_processor->process(odom_capture);
        // odometry integration
        odom_trajectory.segment(step * 3, 3) = problem->getCurrentState();

        // LIDAR DATA ---------------------------
        extractScan(laser_1_file, scan1, timestamp);
        extractScan(laser_2_file, scan2, timestamp);
        if (step % 3 == 0)
        {
            std::cout << "--PROCESS LIDAR 1 DATA..." << laser_1_sensor->id() << std::endl;
            laser_1_processor->process(new CaptureLaser2D(ts, laser_1_sensor, scan1));
            std::cout << "--PROCESS LIDAR 2 DATA..." << laser_2_sensor->id() << std::endl;
            laser_2_processor->process(new CaptureLaser2D(ts, laser_2_sensor, scan2));
        }

        // GPS DATA ---------------------------
        if (step % 5 == 0)
        {
            // compute GPS
            gps_fix_reading  = ground_truth_pose.head<2>();
            gps_fix_reading(0) += distribution_gps(generator);
            gps_fix_reading(1) += distribution_gps(generator);
            // process data
            //(new CaptureGPSFix(ts, &gps_sensor, gps_fix_reading, gps_std * Eigen::MatrixXs::Identity(3,3)));
        }
        mean_times(0) += ((double) clock() - t1) / CLOCKS_PER_SEC;


        // UPDATING CERES ---------------------------
        //std::cout << "UPDATING CERES..." << std::endl;
        t1 = clock();
        // update state units and constraints in ceres
        ceres_manager->update();
        mean_times(2) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // SOLVE OPTIMIZATION ---------------------------
        //std::cout << "SOLVING..." << std::endl;
        t1 = clock();
        ceres::Solver::Summary summary = ceres_manager->solve(ceres_options);
        //std::cout << summary.FullReport() << std::endl;
        mean_times(3) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // COMPUTE COVARIANCES ---------------------------
        //std::cout << "COMPUTING COVARIANCES..." << std::endl;
        t1 = clock();
        //ceres_manager->computeCovariances();
        mean_times(4) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // TIME MANAGEMENT ---------------------------
        double total_t = ((double) clock() - t2) / CLOCKS_PER_SEC;
        mean_times(5) += total_t;
        if (total_t < 0.2)
            usleep(200000 - 1e6 * total_t);

//		std::cout << "\nTree after step..." << std::endl;
    }

    // DISPLAY RESULTS ============================================================================================
    mean_times /= n_execution;
    std::cout << "\nSIMULATION AVERAGE LOOP DURATION [s]" << std::endl;
    std::cout << "  data reading:    " << mean_times(0) << std::endl;
    std::cout << "  wolf managing:      " << mean_times(1) << std::endl;
    std::cout << "  ceres managing:     " << mean_times(2) << std::endl;
    std::cout << "  ceres optimization: " << mean_times(3) << std::endl;
    std::cout << "  ceres covariance:   " << mean_times(4) << std::endl;
    std::cout << "  loop time:          " << mean_times(5) << std::endl;

    //	std::cout << "\nTree before deleting..." << std::endl;

    // Print Final result in a file -------------------------
    // Vehicle poses
    int i = 0;
    Eigen::VectorXs state_poses = Eigen::VectorXs::Zero(n_execution * 3);
    for (auto frame : *(problem->getTrajectoryPtr()->getFrameListPtr()))
    {
        state_poses.segment(i, 3) << frame->getPPtr()->getVector(), frame->getOPtr()->getVector();
        i += 3;
    }

    // Landmarks
    i = 0;
    Eigen::VectorXs landmarks = Eigen::VectorXs::Zero(problem->getMapPtr()->getLandmarkListPtr()->size() * 2);
    for (auto landmark : *(problem->getMapPtr()->getLandmarkListPtr()))
    {
        landmarks.segment(i, 2) = landmark->getPPtr()->getVector();
        i += 2;
    }

    // Print log files
    std::string filepath = getenv("HOME") + std::string("/Desktop/log_file_2.txt");
    log_file.open(filepath, std::ofstream::out); //open log file

    if (log_file.is_open())
    {
        log_file << 0 << std::endl;
        for (unsigned int ii = 0; ii < n_execution; ii++)
            log_file << state_poses.segment(ii * 3, 3).transpose() << "\t" << ground_truth.segment(ii * 3, 3).transpose() << "\t" << (state_poses.segment(ii * 3, 3) - ground_truth.segment(ii * 3, 3)).transpose() << "\t" << odom_trajectory.segment(ii * 3, 3).transpose() << std::endl;
        log_file.close(); //close log file
        std::cout << std::endl << "Result file " << filepath << std::endl;
    }
    else
        std::cout << std::endl << "Failed to write the log file " << filepath << std::endl;

    std::string filepath2 = getenv("HOME") + std::string("/Desktop/landmarks_file_2.txt");
    landmark_file.open(filepath2, std::ofstream::out); //open log file

    if (landmark_file.is_open())
    {
        for (unsigned int ii = 0; ii < landmarks.size(); ii += 2)
            landmark_file << landmarks.segment(ii, 2).transpose() << std::endl;
        landmark_file.close(); //close log file
        std::cout << std::endl << "Landmark file " << filepath << std::endl;
    }
    else
        std::cout << std::endl << "Failed to write the landmark file " << filepath << std::endl;

    std::cout << "Press any key for ending... " << std::endl << std::endl;
    std::getchar();

    problem->destruct();
    std::cout << "wolf deleted" << std::endl;
    delete ceres_manager;
    std::cout << "ceres_manager deleted" << std::endl;

    std::cout << " ========= END ===========" << std::endl << std::endl;

    //exit
    return 0;
}
