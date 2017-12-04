/*
 * gtest_capture_base.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: jsola
 */


#include "utils_gtest.h"

#include "capture_base.h"
#include "state_angle.h"

using namespace wolf;
using namespace Eigen;

TEST(CaptureBase, ConstructorNoSensor)
{
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.2)); // timestamp = 1.2

    ASSERT_EQ(C->getTimeStamp(), 1.2);
    ASSERT_FALSE(C->getFramePtr());
    ASSERT_FALSE(C->getProblem());
    ASSERT_FALSE(C->getSensorPtr());
}

TEST(CaptureBase, ConstructorWithSensor)
{
    SensorBasePtr S(std::make_shared<SensorBase>("DUMMY", nullptr, nullptr, nullptr, 2));
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.5, S)); // timestamp = 1.5
    ASSERT_EQ(C->getTimeStamp(), 1.5);
    ASSERT_FALSE(C->getFramePtr());
    ASSERT_FALSE(C->getProblem());
    ASSERT_TRUE(C->getSensorPtr());
    ASSERT_FALSE(C->getSensorPPtr());
    ASSERT_FALSE(C->getSensorOPtr());
}

TEST(CaptureBase, Static_sensor_params)
{
    StateBlockPtr p(std::make_shared<StateBlock>(2));
    StateBlockPtr o(std::make_shared<StateAngle>() );
    StateBlockPtr i(std::make_shared<StateBlock>(2));
    SensorBasePtr S(std::make_shared<SensorBase>("DUMMY", p, o, i, 2));
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.5, S)); // timestamp = 1.5

    // query sensor blocks
    ASSERT_EQ(S->getPPtr(), p);
    ASSERT_EQ(S->getOPtr(), o);
    ASSERT_EQ(S->getIntrinsicPtr(), i);

    // query capture blocks
    ASSERT_EQ(C->getSensorPPtr(), p);
    ASSERT_EQ(C->getSensorOPtr(), o);
    ASSERT_EQ(C->getSensorIntrinsicPtr(), i);
}

TEST(CaptureBase, Dynamic_sensor_params)
{
    StateBlockPtr p(std::make_shared<StateBlock>(2));
    StateBlockPtr o(std::make_shared<StateAngle>() );
    StateBlockPtr i(std::make_shared<StateBlock>(2));
    SensorBasePtr S(std::make_shared<SensorBase>("DUMMY", nullptr, nullptr, nullptr, 2, true, true)); // last 2 'true' mark dynamic
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.5, S, p, o, i)); // timestamp = 1.5

    // query sensor blocks -- need KFs to find some Capture with the params
    //    ASSERT_EQ(S->getPPtr(), p);
    //    ASSERT_EQ(S->getOPtr(), o);
    //    ASSERT_EQ(S->getIntrinsicPtr(), i);

    // query capture blocks
    ASSERT_EQ(C->getSensorPPtr(), p);
    ASSERT_EQ(C->getSensorOPtr(), o);
    ASSERT_EQ(C->getSensorIntrinsicPtr(), i);
}

TEST(CaptureBase, addFeature)
{
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.2)); // timestamp = 1.2
    FeatureBasePtr f = C->addFeature(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
    ASSERT_FALSE(C->getFeatureList().empty());
    ASSERT_EQ(C->getFeatureList().front(), f);
}

TEST(CaptureBase, addFeatureList)
{
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.2)); // timestamp = 1.2
    FeatureBasePtr f_first = C->addFeature(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
    ASSERT_EQ(C->getFeatureList().size(), 1);

    // make a list to add
    std::list<FeatureBasePtr> fl;
    for (int i = 0; i<3; i++)
    {
        fl.push_back(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
    }
    FeatureBasePtr f_last = fl.back();

    C->addFeatureList((fl));
    ASSERT_EQ(C->getFeatureList().size(), 4);
    ASSERT_EQ(fl.size(), 0); // features have been moved, not copied
    ASSERT_EQ(C->getFeatureList().front(), f_first);
    ASSERT_EQ(C->getFeatureList().back(), f_last);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

