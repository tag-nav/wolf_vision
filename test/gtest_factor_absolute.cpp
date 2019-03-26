/**
 * \file gtest_factor_absolute.cpp
 *
 *  Created on: Dec 9, 2017
 *      \author: datchuth
 */

#include "utils_gtest.h"
#include "base/factor/factor_block_absolute.h"
#include "base/factor/factor_quaternion_absolute.h"
#include "base/capture/capture_motion.h"

#include "base/ceres_wrapper/ceres_manager.h"

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
Eigen::Matrix<wolf::Scalar, 9, 9> data_cov = 0.2 * Eigen::Matrix<Scalar,9,9>::Identity();

// perturbated priors
Vector10s x0 = pose9toPose10(pose + Vector9s::Random()*0.25);

// Problem and solver
ProblemPtr problem_ptr = Problem::create("POV 3D");
CeresManager ceres_mgr(problem_ptr);

// Two frames
FrameBasePtr frm0 = problem_ptr->emplaceFrame(KEY_FRAME, problem_ptr->zeroState(), TimeStamp(0));

// Capture, feature and factor
CaptureBasePtr cap0 = frm0->addCapture(std::make_shared<CaptureMotion>("IMU ABS", 0, nullptr, pose10, 10, 9, nullptr));

////////////////////////////////////////////////////////
/* In the tests below, we check that FactorBlockAbsolute and FactorQuaternionAbsolute are working fine
 * These are absolute factors related to a specific part of the frame's state
 * Both features and factors are created in the TEST(). Hence, tests will not interfere each others.
 */

TEST(FactorBlockAbs, ctr_block_abs_p)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("POSITION", pose10.head<3>(), data_cov.topLeftCorner<3,3>()));
    fea0->addFactor(std::make_shared<FactorBlockAbsolute>(fea0->getFrame()->getP()));

    ASSERT_TRUE(problem_ptr->check(0));

    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(wolf::SolverManager::ReportVerbosity::BRIEF);

    //only orientation is constrained
    ASSERT_MATRIX_APPROX(frm0->getP()->getState(), pose10.head<3>(), 1e-6);
}

TEST(FactorBlockAbs, ctr_block_abs_p_tail2)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("POSITION TAIL 2", pose10.segment<2>(1), data_cov.bottomRightCorner<2,2>()));
    fea0->addFactor(std::make_shared<FactorBlockAbsolute>(fea0->getFrame()->getP(),1,2));

    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(wolf::SolverManager::ReportVerbosity::BRIEF);

    //only orientation is constrained
    ASSERT_MATRIX_APPROX(frm0->getP()->getState().tail<2>(), pose10.segment<2>(1), 1e-6);
}

TEST(FactorBlockAbs, ctr_block_abs_v)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("VELOCITY", pose10.tail<3>(), data_cov.bottomRightCorner<3,3>()));
    fea0->addFactor(std::make_shared<FactorBlockAbsolute>(fea0->getFrame()->getV()));

    ASSERT_TRUE(problem_ptr->check(0));
    
    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(wolf::SolverManager::ReportVerbosity::BRIEF);

    //only velocity is constrained
    ASSERT_MATRIX_APPROX(frm0->getV()->getState(), pose10.tail<3>(), 1e-6);
}

TEST(FactorQuatAbs, ctr_block_abs_o)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("QUATERNION", pose10.segment<4>(3), data_cov.block<3,3>(3,3)));
    fea0->addFactor(std::make_shared<FactorQuaternionAbsolute>(fea0->getFrame()->getO()));

    ASSERT_TRUE(problem_ptr->check(0));
    
    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(wolf::SolverManager::ReportVerbosity::BRIEF);

    //only velocity is constrained
    ASSERT_MATRIX_APPROX(frm0->getO()->getState(), pose10.segment<4>(3), 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

