/**
 * \file gtest_ceres.cpp
 *
 *  Created on: Jan 18, 2017
 *      \author: Dinesh Atchuthan
 */


#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processor_odom_3D.h"
#include "processor_imu.h"
#include "wolf.h"
#include "problem.h"
#include "ceres_wrapper/ceres_manager.h"
#include "state_quaternion.h"

#include <iostream>

using namespace Eigen;
using namespace std;
using namespace wolf;

TEST(ProcessorOdom3D, static_ceresOptimiszation)
{
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", "odom");
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    TimeStamp t(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    /* We do not need to create features and frames and constraints here. Everything is done in wolf.
    Features and constraint at created automatically when a new Keyframe is generated. Whether a new keyframe should be created or not, this is
    handled by voteForKeyFrame() function for this processorMotion
    */

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/
     ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
     std::cout << summary.BriefReport() << std::endl;

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

TEST(ProcessorIMU, static_ceresOptimiszation)
{
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorBasePtr processor_ptr = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    //setting origin
    Eigen::VectorXs x0(16);
    TimeStamp t(0);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.000,  0,0,.000;
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE IMU CLASSES  **************/
    Eigen::Vector6s data;
    data << 0.15,0.10,9.88, 0.023,0.014,0.06;
    Scalar dt = t.get();
    TimeStamp ts(0);
    while((dt-t.get())<=std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan()+0.001){
    // Time and data variables
    dt += 0.001;
    ts.set(dt);

    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen, data);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(imu_ptr);
    }

    /* We do not need to create features and frames and constraints here. Everything is done in wolf.
    Features and constraint at created automatically when a new Keyframe is generated. Whether a new keyframe should be created or not, this is
    handled by voteForKeyFrame() function for this processorMotion
    */

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/
     ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
     std::cout << summary.BriefReport() << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}