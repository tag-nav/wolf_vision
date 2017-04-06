/**
 * \file gtest_constraint_fix.cpp
 *
 *  Created on: Mar 30, 2017
 *      \author: jsola
 */


#include "utils_gtest.h"

#include "constraint_fix.h"
#include "capture_motion.h"

#include "ceres_wrapper/ceres_manager.h"


using namespace Eigen;
using namespace wolf;
using std::cout;
using std::endl;

// Input data and covariance
Vector3s pose6(Vector3s::Random());
Vector3s data_var((Vector3s() << 0.2,0.2,0.2).finished());
Matrix3s data_cov = data_var.asDiagonal();

// perturbated priors
Vector3s x0 = pose6 + Vector3s::Random()*0.25;

// Problem and solver
ProblemPtr problem = Problem::create(FRM_PO_2D);
CeresManager ceres_mgr(problem);

// Two frames
FrameBasePtr frm0 = problem->emplaceFrame(KEY_FRAME, problem->zeroState(), TimeStamp(0));

// Capture, feature and constraint from frm1 to frm0
CaptureBasePtr cap0 = frm0->addCapture(std::make_shared<CaptureMotion>(0, nullptr, pose6, 7, 6));
FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("ODOM 3D", pose6, data_cov));
ConstraintFixPtr ctr0 = std::static_pointer_cast<ConstraintFix>(fea0->addConstraint(std::make_shared<ConstraintFix>(fea0)));
ConstraintBasePtr dummy = frm0->addConstrainedBy(ctr0);

////////////////////////////////////////////////////////

TEST(ConstraintFix, check_tree)
{
    ASSERT_TRUE(problem->check(0));
}

//TEST(ConstraintFix, expectation)
//{
//    ASSERT_EIGEN_APPROX(ctr0->expectation() , delta);
//}

TEST(ConstraintFix, solve)
{

    // Fix frame 0, perturb frm1
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    ceres::Solver::Summary summary = ceres_mgr.solve();

    ASSERT_EIGEN_APPROX(frm0->getState(), pose6, 1e-6);

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

