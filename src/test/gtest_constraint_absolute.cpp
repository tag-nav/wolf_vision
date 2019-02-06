/**
 * \file gtest_constraint_absolute.cpp
 *
 *  Created on: Dec 9, 2017
 *      \author: datchuth
 */


#include "utils_gtest.h"
#include "constraint_block_absolute.h"
#include "constraint_quaternion_absolute.h"
#include "capture_motion.h"

#include "ceres_wrapper/ceres_manager.h"


using namespace Eigen;
using namespace wolf;
using std::cout;
using std::endl;

Vector10s pose9toPose10(Vector9s _pose9)
{
    return (Vector10s() << _pose9.head<3>() , v2q(_pose9.segment<3>(3)).coeffs(), _pose9.tail<3>()).finished();
}

// Input pose9 and covariance
Vector9s pose(Vector9s::Random());
Vector10s pose10 = pose9toPose10(pose);
Vector9s data_var((Vector9s() << 0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2).finished());
Eigen::Matrix<wolf::Scalar, 9, 9> data_cov = 0.01*Eigen::Matrix<Scalar,9,9>::Identity();//data_var.asDiagonal();

// perturbated priors
Vector10s x0 = pose9toPose10(pose + Vector9s::Random()*0.25);

// Problem and solver
ProblemPtr problem = Problem::create("POV 3D");
CeresManager ceres_mgr(problem);

// Two frames
FrameBasePtr frm0 = problem->emplaceFrame(KEY_FRAME, problem->zeroState(), TimeStamp(0));

// Capture, feature and constraint
CaptureBasePtr cap0 = frm0->addCapture(std::make_shared<CaptureMotion>("IMU ABS", 0, nullptr, pose10, 10, 9, nullptr));

////////////////////////////////////////////////////////
/* In the tests below, we check that ConstraintBlockAbsolute and ConstraintQuaternionAbsolute are working fine
 * These are absolute constraints related to a specific part of the frame's state
 * Both features and constraints are created in the TEST(). Hence, tests will not interfere each others.
 */

TEST(ConstraintBlockAbs, ctr_block_abs_p)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("POSITION", pose10.head<3>(), data_cov.topLeftCorner<3,3>()));
    fea0->addConstraint(std::make_shared<ConstraintBlockAbsolute>(fea0->getFramePtr()->getPPtr()));

    ASSERT_TRUE(problem->check(0));

    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(wolf::SolverManager::ReportVerbosity::BRIEF);

    WOLF_INFO("SOLVER REPORT:\n",brief_report);

    //only orientation is constrained
    ASSERT_MATRIX_APPROX(frm0->getPPtr()->getState(), pose10.head<3>(), 1e-6);
}

TEST(ConstraintBlockAbs, ctr_block_abs_p_tail2)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("POSITION TAIL 2", pose10.segment<2>(1), data_cov.bottomRightCorner<2,2>()));
    fea0->addConstraint(std::make_shared<ConstraintBlockAbsolute>(fea0->getFramePtr()->getPPtr(),1,2));

    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(wolf::SolverManager::ReportVerbosity::BRIEF);

    WOLF_INFO("SOLVER REPORT:\n",brief_report);

    //only orientation is constrained
    ASSERT_MATRIX_APPROX(frm0->getPPtr()->getState().tail<2>(), pose10.tail<2>(), 1e-6);
}

TEST(ConstraintBlockAbs, ctr_block_abs_v)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("VELOCITY", pose10.tail<3>(), data_cov.bottomRightCorner<3,3>()));
    fea0->addConstraint(std::make_shared<ConstraintBlockAbsolute>(fea0->getFramePtr()->getVPtr()));

    ASSERT_TRUE(problem->check(0));
    
    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(wolf::SolverManager::ReportVerbosity::BRIEF);

    WOLF_INFO("SOLVER REPORT:\n",brief_report);

    //only velocity is constrained
    ASSERT_MATRIX_APPROX(frm0->getVPtr()->getState(), pose10.tail<3>(), 1e-6);
}

TEST(ConstraintQuatAbs, ctr_block_abs_o)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("QUATERNION", pose10.segment<4>(3), data_cov.block<3,3>(3,3)));
    fea0->addConstraint(std::make_shared<ConstraintQuaternionAbsolute>(fea0->getFramePtr()->getOPtr()));

    ASSERT_TRUE(problem->check(0));
    
    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(wolf::SolverManager::ReportVerbosity::BRIEF);

    WOLF_INFO("SOLVER REPORT:\n",brief_report);

    //only velocity is constrained
    ASSERT_MATRIX_APPROX(frm0->getOPtr()->getState(), pose10.segment<4>(3), 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

