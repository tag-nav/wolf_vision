/**
 * \file gtest_factor_fix_3D.cpp
 *
 *  Created on: Mar 30, 2017
 *      \author: jsola
 */

#include "core/factor/factor_pose_3D.h"
#include "utils_gtest.h"

#include "core/capture/capture_motion.h"

#include "core/ceres_wrapper/ceres_manager.h"

using namespace Eigen;
using namespace wolf;
using std::cout;
using std::endl;

Vector7s pose6toPose7(Vector6s _pose6)
{
    return (Vector7s() << _pose6.head<3>() , v2q(_pose6.tail<3>()).coeffs()).finished();
}

// Input pose6 and covariance
Vector6s pose(Vector6s::Random());
Vector7s pose7 = pose6toPose7(pose);
Vector6s data_var((Vector6s() << 0.2,0.2,0.2,0.2,0.2,0.2).finished());
Matrix6s data_cov = data_var.asDiagonal();

// perturbated priors
Vector7s x0 = pose6toPose7(pose + Vector6s::Random()*0.25);

// Problem and solver
ProblemPtr problem = Problem::create("PO", 3);
CeresManager ceres_mgr(problem);

// Two frames
FrameBasePtr frm0 = problem->emplaceFrame(KEY, problem->zeroState(), TimeStamp(0));

// Capture, feature and factor
CaptureBasePtr cap0 = frm0->addCapture(std::make_shared<CaptureMotion>("ODOM 3D", 0, nullptr, pose7, 7, 6, nullptr));
FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("ODOM 3D", pose7, data_cov));
FactorPose3DPtr ctr0 = std::static_pointer_cast<FactorPose3D>(fea0->addFactor(std::make_shared<FactorPose3D>(fea0)));

////////////////////////////////////////////////////////

TEST(FactorPose3D, check_tree)
{
    ASSERT_TRUE(problem->check(0));
}

//TEST(FactorFix3D, expectation)
//{
//    ASSERT_EIGEN_APPROX(ctr0->expectation() , delta);
//}

TEST(FactorPose3D, solve)
{

    // Fix frame 0, perturb frm1
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(SolverManager::ReportVerbosity::FULL);

    ASSERT_MATRIX_APPROX(frm0->getState(), pose7, 1e-6);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

