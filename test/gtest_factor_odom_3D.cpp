/**
 * \file gtest_factor_odom_3D.cpp
 *
 *  Created on: Mar 30, 2017
 *      \author: jsola
 */

#include "utils_gtest.h"

#include "core/factor/factor_odom_3D.h"
#include "core/capture/capture_motion.h"

#include "core/ceres_wrapper/ceres_manager.h"

using namespace Eigen;
using namespace wolf;
using std::cout;
using std::endl;

Vector7s data2delta(Vector6s _data)
{
    return (Vector7s() << _data.head<3>() , v2q(_data.tail<3>()).coeffs()).finished();
}

// Input odometry data and covariance
Vector6s data(Vector6s::Random());
Vector7s delta = data2delta(data);
Vector6s data_var((Vector6s() << 0.2,0.2,0.2,0.2,0.2,0.2).finished());
Matrix6s data_cov = data_var.asDiagonal();

// perturbated priors
Vector7s x0 = data2delta(Vector6s::Random()*0.25);
Vector7s x1 = data2delta(data + Vector6s::Random()*0.25);

// Problem and solver
ProblemPtr problem_ptr = Problem::create("PO 3D");
CeresManager ceres_mgr(problem_ptr);

// Two frames
FrameBasePtr frm0 = problem_ptr->emplaceFrame(KEY, problem_ptr->zeroState(), TimeStamp(0));
FrameBasePtr frm1 = problem_ptr->emplaceFrame(KEY, delta, TimeStamp(1));

// Capture, feature and factor from frm1 to frm0
CaptureBasePtr cap1 = frm1->addCapture(std::make_shared<CaptureMotion>("ODOM 3D", 1, nullptr, delta, 7, 6, nullptr));
FeatureBasePtr fea1 = cap1->addFeature(std::make_shared<FeatureBase>("ODOM 3D", delta, data_cov));
FactorOdom3DPtr ctr1 = std::static_pointer_cast<FactorOdom3D>(fea1->addFactor(std::make_shared<FactorOdom3D>(fea1, frm0, nullptr))); // create and add
FactorBasePtr dummy = frm0->addConstrainedBy(ctr1);

////////////////////////////////////////////////////////

TEST(FactorOdom3D, check_tree)
{
    ASSERT_TRUE(problem_ptr->check(0));
}

TEST(FactorOdom3D, expectation)
{
    ASSERT_MATRIX_APPROX(ctr1->expectation() , delta, 1e-6);
}

TEST(FactorOdom3D, fix_0_solve)
{

    // Fix frame 0, perturb frm1
    frm0->fix();
    frm1->unfix();
    frm1->setState(x1);

    // solve for frm1
    std::string report = ceres_mgr.solve(SolverManager::ReportVerbosity::BRIEF);

    ASSERT_MATRIX_APPROX(frm1->getState(), delta, 1e-6);

}

TEST(FactorOdom3D, fix_1_solve)
{
    // Fix frame 1, perturb frm0
    frm0->unfix();
    frm1->fix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(SolverManager::ReportVerbosity::BRIEF);

    ASSERT_MATRIX_APPROX(frm0->getState(), problem_ptr->zeroState(), 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
