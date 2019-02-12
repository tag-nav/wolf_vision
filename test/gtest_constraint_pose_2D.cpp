/**
 * \file gtest_constraint_pose_2D.cpp
 *
 *  Created on: Mar 30, 2017
 *      \author: jsola
 */

#include "base/constraint/constraint_pose_2D.h"
#include "utils_gtest.h"

#include "base/capture/capture_motion.h"

#include "base/ceres_wrapper/ceres_manager.h"

using namespace Eigen;
using namespace wolf;
using std::cout;
using std::endl;

// Input data and covariance
Vector3s pose(Vector3s::Random());
Vector3s data_var((Vector3s() << 0.2,0.2,0.2).finished());
Matrix3s data_cov = data_var.asDiagonal();

// perturbated priors
Vector3s x0 = pose + Vector3s::Random()*0.25;

// Problem and solver
ProblemPtr problem = Problem::create("PO 2D");
CeresManager ceres_mgr(problem);

// Two frames
FrameBasePtr frm0 = problem->emplaceFrame(KEY_FRAME, problem->zeroState(), TimeStamp(0));

// Capture, feature and constraint from frm1 to frm0
CaptureBasePtr cap0 = frm0->addCapture(std::make_shared<CaptureMotion>("ODOM 2D", 0, nullptr, pose, 3, 3, nullptr));
FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("ODOM 2D", pose, data_cov));
ConstraintPose2DPtr ctr0 = std::static_pointer_cast<ConstraintPose2D>(fea0->addConstraint(std::make_shared<ConstraintPose2D>(fea0)));

////////////////////////////////////////////////////////

TEST(ConstraintPose2D, check_tree)
{
    ASSERT_TRUE(problem->check(0));
}

//TEST(ConstraintFix, expectation)
//{
//    ASSERT_EIGEN_APPROX(ctr0->expectation() , delta);
//}

TEST(ConstraintPose2D, solve)
{

    // Fix frame 0, perturb frm1
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string report = ceres_mgr.solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport

    ASSERT_MATRIX_APPROX(frm0->getState(), pose, 1e-6);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

