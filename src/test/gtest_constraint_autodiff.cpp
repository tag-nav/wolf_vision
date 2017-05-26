/*
 * gtest_constraint_autodiff.cpp
 *
 *  Created on: May 24 2017
 *      Author: jvallve
 */


#include "utils_gtest.h"

#include "sensor_odom_2D.h"
#include "capture_void.h"
#include "feature_odom_2D.h"
#include "constraint_odom_2D.h"
#include "constraint_autodiff.h"

using namespace wolf;
using namespace Eigen;

TEST(CaptureAutodiff, ConstructorOdom2d)
{
    FrameBasePtr fr1_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1)));
    FrameBasePtr fr2_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1)));

    // SENSOR
    SensorOdom2DPtr sensor_ptr = std::make_shared<SensorOdom2D>(std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1), 0.1, 0.1);

    // CAPTURE
    CaptureVoidPtr capture_ptr = std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr);
    fr2_ptr->addCapture(capture_ptr);

    // FEATURE
    FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(Eigen::Vector3s::Zero(), Eigen::Matrix3s::Identity());
    capture_ptr->addFeature(feature_ptr);

    // CONSTRAINT
    ConstraintOdom2DPtr constraint_ptr = std::make_shared<ConstraintOdom2D>(feature_ptr, fr1_ptr);
    feature_ptr->addConstraint(constraint_ptr);
    fr1_ptr->addConstrainedBy(constraint_ptr);

    ASSERT_TRUE(constraint_ptr->getFeaturePtr());
    ASSERT_TRUE(constraint_ptr->getFeaturePtr()->getCapturePtr());
    ASSERT_TRUE(constraint_ptr->getFeaturePtr()->getCapturePtr()->getFramePtr());
    ASSERT_TRUE(constraint_ptr->getFeaturePtr()->getCapturePtr()->getSensorPtr());
    ASSERT_TRUE(constraint_ptr->getFrameOtherPtr());
}

TEST(CaptureAutodiff, ResidualOdom2d)
{
    Eigen::Vector3s f1_pose, f2_pose;
    f1_pose << 2,1,M_PI;
    f2_pose << 3,5,3*M_PI/2;

    FrameBasePtr fr1_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(f1_pose.head<2>()), std::make_shared<StateBlock>(f1_pose.tail<1>())));
    FrameBasePtr fr2_ptr(std::make_shared<FrameBase>(TimeStamp(0), std::make_shared<StateBlock>(f2_pose.head<2>()), std::make_shared<StateBlock>(f2_pose.tail<1>())));

    // SENSOR
    SensorOdom2DPtr sensor_ptr = std::make_shared<SensorOdom2D>(std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1), 0.1, 0.1);

    // CAPTURE
    CaptureVoidPtr capture_ptr = std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr);
    fr2_ptr->addCapture(capture_ptr);

    // FEATURE
    Eigen::Vector3s d;
    d << -1, -4, M_PI/2;
    FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(d, Eigen::Matrix3s::Identity());
    capture_ptr->addFeature(feature_ptr);

    // CONSTRAINT
    ConstraintOdom2DPtr constraint_ptr = std::make_shared<ConstraintOdom2D>(feature_ptr, fr1_ptr);
    feature_ptr->addConstraint(constraint_ptr);
    fr1_ptr->addConstrainedBy(constraint_ptr);

    // EVALUATE
    std::vector<Scalar*> states_ptr({fr1_ptr->getPPtr()->getPtr(), fr1_ptr->getOPtr()->getPtr(), fr2_ptr->getPPtr()->getPtr(), fr2_ptr->getOPtr()->getPtr()});
    double const* const* parameters = states_ptr.data();
    Eigen::VectorXs residuals(constraint_ptr->getSize());
    constraint_ptr->Evaluate(parameters, residuals.data(), nullptr);

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
    SensorOdom2DPtr sensor_ptr = std::make_shared<SensorOdom2D>(std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1), 0.1, 0.1);

    // CAPTURE
    CaptureVoidPtr capture_ptr = std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr);
    fr2_ptr->addCapture(capture_ptr);

    // FEATURE
    Eigen::Vector3s d;
    d << -1, -4, M_PI/2;
    FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(d, Eigen::Matrix3s::Identity());
    capture_ptr->addFeature(feature_ptr);

    // CONSTRAINT
    ConstraintOdom2DPtr constraint_ptr = std::make_shared<ConstraintOdom2D>(feature_ptr, fr1_ptr);
    feature_ptr->addConstraint(constraint_ptr);
    fr1_ptr->addConstrainedBy(constraint_ptr);

    // COMPUTE JACOBIANS
    std::vector<const Scalar*> states_ptr({fr1_ptr->getPPtr()->getPtr(), fr1_ptr->getOPtr()->getPtr(), fr2_ptr->getPPtr()->getPtr(), fr2_ptr->getOPtr()->getPtr()});
    std::vector<Eigen::MatrixXs> Jauto;
    constraint_ptr->computeJacobian(states_ptr, Jauto);

    std::cout << Jauto.size() << std::endl;

    // ANALYTIC JACOBIANS
    Eigen::MatrixXs J0(3,2);
    J0 << -cos(f1_pose(2)), -sin(f1_pose(2)),
           sin(f1_pose(2)), -cos(f1_pose(2)),
           0,                0;
    ASSERT_TRUE((J0-Jauto[0]).maxCoeff() < 1e-9 && (J0-Jauto[0]).minCoeff() > -1e-9);

    Eigen::MatrixXs J1(3,1);
    J1 << -(f2_pose(0) - f1_pose(0)) * sin(f1_pose(2)) + (f2_pose(1) - f1_pose(1)) * cos(f1_pose(2)),
          -(f2_pose(0) - f1_pose(0)) * cos(f1_pose(2)) - (f2_pose(1) - f1_pose(1)) * sin(f1_pose(2)),
          -1;
    ASSERT_TRUE((J1-Jauto[1]).maxCoeff() < 1e-9 && (J1-Jauto[1]).minCoeff() > -1e-9);

    Eigen::MatrixXs J2(3,2);
    J2 <<  cos(f1_pose(2)), sin(f1_pose(2)),
           -sin(f1_pose(2)), cos(f1_pose(2)),
           0,               0;
    ASSERT_TRUE((J2-Jauto[2]).maxCoeff() < 1e-9 && (J2-Jauto[2]).minCoeff() > -1e-9);

    Eigen::MatrixXs J3(3,1);
    J3 <<  0, 0, 1;
    ASSERT_TRUE((J3-Jauto[3]).maxCoeff() < 1e-9 && (J3-Jauto[3]).minCoeff() > -1e-9);

//    std::cout << "autodiff J " << 0 << ":\n" << Jauto[0] << std::endl;
//    std::cout << "analytic J " << 0 << ":\n" << J0 << std::endl;
//    std::cout << "autodiff J " << 1 << ":\n" << Jauto[1] << std::endl;
//    std::cout << "analytic J " << 1 << ":\n" << J1 << std::endl;
//    std::cout << "autodiff J " << 2 << ":\n" << Jauto[2] << std::endl;
//    std::cout << "analytic J " << 2 << ":\n" << J2 << std::endl;
//    std::cout << "autodiff J " << 3 << ":\n" << Jauto[3] << std::endl;
//    std::cout << "analytic J " << 3 << ":\n" << J3 << std::endl;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

