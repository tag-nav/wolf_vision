/**
 * \file gtest_constraint_fix_3D.cpp
 *
 *  Created on: Mar 30, 2017
 *      \author: jsola
 */


#include "../constraint_pose_3D.h"
#include "utils_gtest.h"

#include "capture_motion.h"

#include "ceres_wrapper/ceres_manager.h"


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
ProblemPtr problem = Problem::create("PO 3D");
CeresManager ceres_mgr(problem);

// Two frames
FrameBasePtr frm0 = problem->emplaceFrame(KEY_FRAME, problem->zeroState(), TimeStamp(0));

// Capture, feature and constraint
CaptureBasePtr cap0 = frm0->addCapture(std::make_shared<CaptureMotion>(0, nullptr, pose7, 7, 6, nullptr));
FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("ODOM 3D", pose7, data_cov));
ConstraintPose3DPtr ctr0 = std::static_pointer_cast<ConstraintPose3D>(fea0->addConstraint(std::make_shared<ConstraintPose3D>(fea0)));


////////////////////////////////////////////////////////

TEST(ConstraintPose3D, check_tree)
{
    ASSERT_TRUE(problem->check(0));
}

//TEST(ConstraintFix3D, expectation)
//{
//    ASSERT_EIGEN_APPROX(ctr0->expectation() , delta);
//}

TEST(ConstraintPose3D, solve)
{

    // Fix frame 0, perturb frm1
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(1);

    ASSERT_MATRIX_APPROX(frm0->getState(), pose7, 1e-6);

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

