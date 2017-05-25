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
    fr1_ptr->addCapture(capture_ptr);

    // FEATURE
    FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(Eigen::Vector3s::Zero(), Eigen::Matrix3s::Identity());
    capture_ptr->addFeature(feature_ptr);

    // CONSTRAINT
    ConstraintOdom2DPtr constraint_ptr = std::make_shared<ConstraintOdom2D>(feature_ptr, fr2_ptr);
    feature_ptr->addConstraint(constraint_ptr);
    fr2_ptr->addConstrainedBy(constraint_ptr);
}

//TEST(CaptureBase, ConstructorWithSensor)
//{
//    SensorBasePtr S(std::make_shared<SensorBase>("DUMMY", nullptr, nullptr, nullptr, 2));
//    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.5, S)); // timestamp = 1.5
//    ASSERT_EQ(C->getTimeStamp(), 1.5);
//    ASSERT_FALSE(C->getFramePtr());
//    ASSERT_FALSE(C->getProblem());
//    ASSERT_TRUE(C->getSensorPtr());
//    ASSERT_FALSE(C->getSensorPPtr());
//    ASSERT_FALSE(C->getSensorOPtr());
//}
//
//TEST(CaptureBase, addFeature)
//{
//    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.2)); // timestamp = 1.2
//    FeatureBasePtr f = C->addFeature(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
//    ASSERT_FALSE(C->getFeatureList().empty());
//    ASSERT_EQ(C->getFeatureList().front(), f);
//}
//
//TEST(CaptureBase, addFeatureList)
//{
//    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.2)); // timestamp = 1.2
//    FeatureBasePtr f_first = C->addFeature(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
//    ASSERT_EQ(C->getFeatureList().size(), 1);
//
//    // make a list to add
//    std::list<FeatureBasePtr> fl;
//    for (int i = 0; i<3; i++)
//    {
//        fl.push_back(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
//    }
//    FeatureBasePtr f_last = fl.back();
//
//    C->addFeatureList((fl));
//    ASSERT_EQ(C->getFeatureList().size(), 4);
//    ASSERT_EQ(fl.size(), 0); // features have been moved, not copied
//    ASSERT_EQ(C->getFeatureList().front(), f_first);
//    ASSERT_EQ(C->getFeatureList().back(), f_last);
//}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

