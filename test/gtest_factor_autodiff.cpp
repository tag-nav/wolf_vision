/*
 * gtest_factor_autodiff.cpp
 *
 *  Created on: May 24 2017
 *      Author: jvallve
 */

#include "utils_gtest.h"

#include "core/sensor/sensor_odom_2D.h"
#include "core/capture/capture_void.h"
#include "core/feature/feature_odom_2D.h"
#include "core/factor/factor_odom_2D.h"
#include "core/factor/factor_odom_2D_analytic.h"
#include "core/factor/factor_autodiff.h"

using namespace wolf;
using namespace Eigen;

TEST(CaptureAutodiff, ConstructorOdom2d)
{
    FrameBasePtr fr1_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1)));
    FrameBasePtr fr2_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1)));

    // SENSOR
    IntrinsicsOdom2D intrinsics_odo;
    intrinsics_odo.k_disp_to_disp = 0.1;
    intrinsics_odo.k_rot_to_rot = 0.1;
    SensorOdom2DPtr sensor_ptr = std::make_shared<SensorOdom2D>(Vector3s::Zero(), intrinsics_odo);

    // CAPTURE
    // CaptureVoidPtr capture_ptr = std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr);
    // fr2_ptr->addCapture(capture_ptr);
    auto capture_ptr = CaptureBase::emplace<CaptureVoid>(fr2_ptr, TimeStamp(0), sensor_ptr);

    // FEATURE
    // FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(Eigen::Vector3s::Zero(), Eigen::Matrix3s::Identity());
    // capture_ptr->addFeature(feature_ptr);
    auto feature_ptr = FeatureBase::emplace<FeatureOdom2D>(capture_ptr, Eigen::Vector3s::Zero(), Eigen::Matrix3s::Identity());

    // FACTOR
    // FactorOdom2DPtr factor_ptr = std::make_shared<FactorOdom2D>(feature_ptr, fr1_ptr);
    // feature_ptr->addFactor(factor_ptr);
    // fr1_ptr->addConstrainedBy(factor_ptr);
    auto factor_ptr = FactorBase::emplace<FactorOdom2D>(feature_ptr, feature_ptr, fr1_ptr);

    ASSERT_TRUE(factor_ptr->getFeature());
    ASSERT_TRUE(factor_ptr->getFeature()->getCapture());
    ASSERT_TRUE(factor_ptr->getFeature()->getCapture()->getFrame());
    ASSERT_TRUE(factor_ptr->getFeature()->getCapture()->getSensor());
    ASSERT_TRUE(factor_ptr->getFrameOther());
}

TEST(CaptureAutodiff, ResidualOdom2d)
{
    Eigen::Vector3s f1_pose, f2_pose;
    f1_pose << 2,1,M_PI;
    f2_pose << 3,5,3*M_PI/2;

    FrameBasePtr fr1_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(f1_pose.head<2>()), std::make_shared<StateBlock>(f1_pose.tail<1>())));
    FrameBasePtr fr2_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(f2_pose.head<2>()), std::make_shared<StateBlock>(f2_pose.tail<1>())));

    // SENSOR
    IntrinsicsOdom2D intrinsics_odo;
    intrinsics_odo.k_disp_to_disp = 0.1;
    intrinsics_odo.k_rot_to_rot = 0.1;
    SensorOdom2DPtr sensor_ptr = std::make_shared<SensorOdom2D>(Vector3s::Zero(), intrinsics_odo);

    // CAPTURE
    // CaptureVoidPtr capture_ptr = std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr);
    // fr2_ptr->addCapture(capture_ptr);
    auto capture_ptr = CaptureBase::emplace<CaptureVoid>(fr2_ptr, TimeStamp(0), sensor_ptr);


    // FEATURE
    Eigen::Vector3s d;
    d << -1, -4, M_PI/2;
    // FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(d, Eigen::Matrix3s::Identity());
    // capture_ptr->addFeature(feature_ptr);
    auto feature_ptr = FeatureBase::emplace<FeatureOdom2D>(capture_ptr, d, Eigen::Matrix3s::Identity());

    // FACTOR
    // FactorOdom2DPtr factor_ptr = std::make_shared<FactorOdom2D>(feature_ptr, fr1_ptr);
    // feature_ptr->addFactor(factor_ptr);
    // fr1_ptr->addConstrainedBy(factor_ptr);

    auto factor_ptr = FactorBase::emplace<FactorOdom2D>(feature_ptr, feature_ptr, fr1_ptr);

    // EVALUATE

    Eigen::VectorXs fr1_pstate = fr1_ptr->getP()->getState();
    Eigen::VectorXs fr1_ostate = fr1_ptr->getO()->getState();
    Eigen::VectorXs fr2_pstate = fr2_ptr->getP()->getState();
    Eigen::VectorXs fr2_ostate = fr2_ptr->getO()->getState();

    std::vector<Scalar*> states_ptr({fr1_pstate.data(), fr1_ostate.data(), fr2_pstate.data(), fr2_ostate.data()});

    double const* const* parameters = states_ptr.data();
    Eigen::VectorXs residuals(factor_ptr->getSize());
    factor_ptr->evaluate(parameters, residuals.data(), nullptr);

    ASSERT_TRUE(residuals.maxCoeff() < 1e-9);
    ASSERT_TRUE(residuals.minCoeff() > -1e-9);
}

TEST(CaptureAutodiff, JacobianOdom2d)
{
    Eigen::Vector3s f1_pose, f2_pose;
    f1_pose << 2,1,M_PI;
    f2_pose << 3,5,3*M_PI/2;

    FrameBasePtr fr1_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(f1_pose.head<2>()), std::make_shared<StateBlock>(f1_pose.tail<1>())));
    FrameBasePtr fr2_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(f2_pose.head<2>()), std::make_shared<StateBlock>(f2_pose.tail<1>())));

    // SENSOR
    IntrinsicsOdom2D intrinsics_odo;
    intrinsics_odo.k_disp_to_disp = 0.1;
    intrinsics_odo.k_rot_to_rot = 0.1;
    SensorOdom2DPtr sensor_ptr = std::make_shared<SensorOdom2D>(Vector3s::Zero(), intrinsics_odo);

    // CAPTURE
    // CaptureVoidPtr capture_ptr = std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr);
    // fr2_ptr->addCapture(capture_ptr);
    auto capture_ptr = CaptureBase::emplace<CaptureVoid>(fr2_ptr, TimeStamp(0), sensor_ptr);

    // FEATURE
    Eigen::Vector3s d;
    d << -1, -4, M_PI/2;
    // FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(d, Eigen::Matrix3s::Identity());
    // capture_ptr->addFeature(feature_ptr);
    auto feature_ptr = FeatureBase::emplace<FeatureOdom2D>(capture_ptr, d, Eigen::Matrix3s::Identity());

    // FACTOR
    // FactorOdom2DPtr factor_ptr = std::make_shared<FactorOdom2D>(feature_ptr, fr1_ptr);
    // feature_ptr->addFactor(factor_ptr);
    // fr1_ptr->addConstrainedBy(factor_ptr);
    auto factor_ptr = FactorBase::emplace<FactorOdom2D>(feature_ptr, feature_ptr, fr1_ptr);

    // COMPUTE JACOBIANS

    const Eigen::VectorXs fr1_pstate = fr1_ptr->getP()->getState();
    const Eigen::VectorXs fr1_ostate = fr1_ptr->getO()->getState();
    const Eigen::VectorXs fr2_pstate = fr2_ptr->getP()->getState();
    const Eigen::VectorXs fr2_ostate = fr2_ptr->getO()->getState();

    std::vector<const Scalar*> states_ptr({fr1_pstate.data(), fr1_ostate.data(), fr2_pstate.data(), fr2_ostate.data()});

    std::vector<Eigen::MatrixXs> Jauto;
    Eigen::VectorXs residuals(factor_ptr->getSize());
    factor_ptr->evaluate(states_ptr, residuals, Jauto);

    std::cout << Jauto.size() << std::endl;

    // ANALYTIC JACOBIANS
    Eigen::MatrixXs J0(3,2);
    J0 << -cos(f1_pose(2)), -sin(f1_pose(2)),
           sin(f1_pose(2)), -cos(f1_pose(2)),
           0,                0;
    ASSERT_MATRIX_APPROX(J0, Jauto[0], wolf::Constants::EPS);
    //ASSERT_TRUE((J0-Jauto[0]).maxCoeff() < 1e-9 && (J0-Jauto[0]).minCoeff() > -1e-9);

    Eigen::MatrixXs J1(3,1);
    J1 << -(f2_pose(0) - f1_pose(0)) * sin(f1_pose(2)) + (f2_pose(1) - f1_pose(1)) * cos(f1_pose(2)),
          -(f2_pose(0) - f1_pose(0)) * cos(f1_pose(2)) - (f2_pose(1) - f1_pose(1)) * sin(f1_pose(2)),
          -1;
    ASSERT_MATRIX_APPROX(J1, Jauto[1], wolf::Constants::EPS);
    //ASSERT_TRUE((J1-Jauto[1]).maxCoeff() < 1e-9 && (J1-Jauto[1]).minCoeff() > -1e-9);

    Eigen::MatrixXs J2(3,2);
    J2 <<  cos(f1_pose(2)), sin(f1_pose(2)),
           -sin(f1_pose(2)), cos(f1_pose(2)),
           0,               0;
    ASSERT_MATRIX_APPROX(J2, Jauto[2], wolf::Constants::EPS);
    //ASSERT_TRUE((J2-Jauto[2]).maxCoeff() < 1e-9 && (J2-Jauto[2]).minCoeff() > -1e-9);

    Eigen::MatrixXs J3(3,1);
    J3 <<  0, 0, 1;
    ASSERT_MATRIX_APPROX(J3, Jauto[3], wolf::Constants::EPS);
    //ASSERT_TRUE((J3-Jauto[3]).maxCoeff() < 1e-9 && (J3-Jauto[3]).minCoeff() > -1e-9);

//    std::cout << "autodiff J " << 0 << ":\n" << Jauto[0] << std::endl;
//    std::cout << "analytic J " << 0 << ":\n" << J0 << std::endl;
//    std::cout << "autodiff J " << 1 << ":\n" << Jauto[1] << std::endl;
//    std::cout << "analytic J " << 1 << ":\n" << J1 << std::endl;
//    std::cout << "autodiff J " << 2 << ":\n" << Jauto[2] << std::endl;
//    std::cout << "analytic J " << 2 << ":\n" << J2 << std::endl;
//    std::cout << "autodiff J " << 3 << ":\n" << Jauto[3] << std::endl;
//    std::cout << "analytic J " << 3 << ":\n" << J3 << std::endl;
}

TEST(CaptureAutodiff, AutodiffVsAnalytic)
{
    Eigen::Vector3s f1_pose, f2_pose;
    f1_pose << 2,1,M_PI;
    f2_pose << 3,5,3*M_PI/2;

    FrameBasePtr fr1_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(f1_pose.head<2>()), std::make_shared<StateBlock>(f1_pose.tail<1>())));
    FrameBasePtr fr2_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(f2_pose.head<2>()), std::make_shared<StateBlock>(f2_pose.tail<1>())));

    // SENSOR
    IntrinsicsOdom2D intrinsics_odo;
    intrinsics_odo.k_disp_to_disp = 0.1;
    intrinsics_odo.k_rot_to_rot = 0.1;
    SensorOdom2DPtr sensor_ptr = std::make_shared<SensorOdom2D>(Vector3s::Zero(), intrinsics_odo);

    // CAPTURE
    // CaptureVoidPtr capture_ptr = std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr);
    // fr2_ptr->addCapture(capture_ptr);
    auto capture_ptr = CaptureBase::emplace<CaptureVoid>(fr2_ptr, TimeStamp(0), sensor_ptr);

    // FEATURE
    Eigen::Vector3s d;
    d << -1, -4, M_PI/2;
    // FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(d, Eigen::Matrix3s::Identity());
    // capture_ptr->addFeature(feature_ptr);
    auto feature_ptr = FeatureBase::emplace<FeatureOdom2D>(capture_ptr, d, Eigen::Matrix3s::Identity());

    // FACTOR
    // FactorOdom2DPtr fac_autodiff_ptr = std::make_shared<FactorOdom2D>(feature_ptr, fr1_ptr);
    // feature_ptr->addFactor(fac_autodiff_ptr);
    // fr1_ptr->addConstrainedBy(fac_autodiff_ptr);
    auto fac_autodiff_ptr = FactorBase::emplace<FactorOdom2D>(feature_ptr, feature_ptr, fr1_ptr);
    // FactorOdom2DAnalyticPtr fac_analytic_ptr = std::make_shared<FactorOdom2DAnalytic>(feature_ptr, fr1_ptr);
    // feature_ptr->addFactor(fac_analytic_ptr);
    // fr1_ptr->addConstrainedBy(fac_analytic_ptr);
    auto fac_analytic_ptr = FactorBase::emplace<FactorOdom2DAnalytic>(feature_ptr, feature_ptr, fr1_ptr);

    // COMPUTE JACOBIANS

    const Eigen::VectorXs fr1_pstate = fr1_ptr->getP()->getState();
    const Eigen::VectorXs fr1_ostate = fr1_ptr->getO()->getState();
    const Eigen::VectorXs fr2_pstate = fr2_ptr->getP()->getState();
    const Eigen::VectorXs fr2_ostate = fr2_ptr->getO()->getState();

    std::vector<const Scalar*> states_ptr({fr1_pstate.data(), fr1_ostate.data(), fr2_pstate.data(), fr2_ostate.data()});

    std::vector<Eigen::MatrixXs> Jautodiff, Janalytic;
    Eigen::VectorXs residuals(fac_autodiff_ptr->getSize());
    clock_t t = clock();
    fac_autodiff_ptr->evaluate(states_ptr, residuals, Jautodiff);
    std::cout << "autodiff evaluate: " << ((double) clock() - t) / CLOCKS_PER_SEC << "s" << std::endl;
    t = clock();
    //TODO FactorAnalytic::evaluate
//    fac_analytic_ptr->evaluate(states_ptr, residuals, Janalytic);
//    std::cout << "analytic evaluate: " << ((double) clock() - t) / CLOCKS_PER_SEC << "s" << std::endl;
//
//    for (auto i = 0; i < Jautodiff.size(); i++)
//        ASSERT_MATRIX_APPROX(Jautodiff[i], Janalytic[i], wolf::Constants::EPS);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

