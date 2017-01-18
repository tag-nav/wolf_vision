/**
 * \file gtest_ceres.cpp
 *
 *  Created on: Jan 18, 2017
 *      \author: jsola
 */




#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processor_odom_3D.h"
#include "constraint_odom_3D.h"
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

    VectorXs D(7); D.setRandom(); D.tail<4>().normalize();
    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    TimeStamp t(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(TimeStamp(0), sen, d);
    odom_ptr->setFramePtr(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // assign data to capture
    odom_ptr->setData(d);
    odom_ptr->setTimeStamp(t);
    // process data in capture
    sen->process(odom_ptr);

    //make final a keyframe
    TimeStamp ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
    Eigen::VectorXs state_vec = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState();
    std::cout << "last state : " << state_vec.transpose() << std::endl;
    wolf::FrameBasePtr last_frame = wolf_problem_ptr_->getTrajectoryPtr()->getLastKeyFramePtr();

    //create a feature
    FeatureBasePtr last_feature = std::make_shared<FeatureBase>("ODOM_3D", (Vector7s()<<0,0,0,0,0,0,1).finished(),Eigen::Matrix7s::Identity()); //first KF and last KF at same position
    last_feature->setCapturePtr(odom_ptr);

                                             /************** CREATE ODOM_3D CONSTRAINT  **************/
    //create an ODOM constraint between first and last keyframes
    ConstraintOdom3DPtr constraintOdom_ptr = std::make_shared<ConstraintOdom3D>(last_feature, last_frame);
    last_feature -> addConstraint(constraintOdom_ptr);
    last_frame -> addConstrainedBy(constraintOdom_ptr);

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/                                    
    // COMPUTE COVARIANCES
    std::cout << "computing covariances..." << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    std::cout << "computed!" << std::endl;

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}