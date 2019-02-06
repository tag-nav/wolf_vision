/*
 * gtest_param_prior.cpp
 *
 *  Created on: Feb 6, 2019
 *      Author: jvallve
 */

#include "utils_gtest.h"
#include "../src/logging.h"

#include "../problem.h"
#include "../ceres_wrapper/ceres_manager.h"
#include "../sensor_odom_3D.h"

#include <iostream>

using namespace wolf;

ProblemPtr problem_ptr = Problem::create("PO 3D");
CeresManagerPtr ceres_mgr_ptr = std::make_shared<CeresManager>(problem_ptr);
Eigen::Vector3s initial_extrinsics((Eigen::Vector3s() << 1, 2, 3, 1, 0, 0, 0).finished());
Eigen::Vector3s prior_extrinsics((Eigen::Vector3s() << 10, 20, 30, 0, 0, 0, 1).finished());

SensorOdom3DPtr odom_sensor_ptr_ = std::static_pointer_cast<SensorOdom3D>(problem_ptr->installSensor("ODOM 3D", "odometer", initial_extrinsics, std::make_shared<IntrinsicsOdom3D>()));
SensorOdom3DPtr odom_sensor2_ptr_ = std::static_pointer_cast<SensorOdom3D>(problem_ptr->installSensor("ODOM 3D", "odometer2", initial_extrinsics, std::make_shared<IntrinsicsOdom3D>()));

TEST(ParameterPrior, initial_extrinsics)
{
    ASSERT_TRUE(problem_ptr->check(0));
    ASSERT_TRUE(odom_sensor_ptr_->getPPtr());
    ASSERT_TRUE(odom_sensor_ptr_->getOPtr());
    ASSERT_MATRIX_APPROX(odom_sensor_ptr_->getPPtr()->getState(),initial_extrinsics.head(3),1e-9);
    ASSERT_MATRIX_APPROX(odom_sensor_ptr_->getOPtr()->getState(),initial_extrinsics.tail(4),1e-9);
}

TEST(ParameterPrior, prior_p)
{
    odom_sensor_ptr_->addParameterPrior(odom_sensor_ptr_->getPPtr(), prior_extrinsics,Eigen::Matrix1s::Identity());

    // solve for frm1
    std::string report = ceres_mgr_ptr->solve(SolverManager::ReportVerbosity::BRIEF);

    ASSERT_MATRIX_APPROX(odom_sensor_ptr_->getPPtr()->getState().tail(1),prior_extrinsics.segment(1,1),1e-6);
}

TEST(ParameterPrior, prior_o)
{
    odom_sensor_ptr_->addParameterPrior(odom_sensor_ptr_->getOPtr(), prior_extrinsics.tail(1),Eigen::Matrix1s::Identity());

    // solve for frm1
    std::string report = ceres_mgr_ptr->solve(SolverManager::ReportVerbosity::BRIEF);

    ASSERT_MATRIX_APPROX(odom_sensor_ptr_->getOPtr()->getState(),prior_extrinsics.tail(1),1e-6);
}

TEST(ParameterPrior, prior_p_tail)
{
    odom_sensor2_ptr_->addParameterPrior(odom_sensor2_ptr_->getPPtr(), prior_extrinsics.segment(1,2),Eigen::Matrix1s::Identity(),1,2);

    // solve for frm1
    std::string report = ceres_mgr_ptr->solve(SolverManager::ReportVerbosity::BRIEF);

    ASSERT_MATRIX_APPROX(odom_sensor2_ptr_->getPPtr()->getState().tail(2),prior_extrinsics.segment(1,2),1e-6);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

