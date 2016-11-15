/*
 * gtest_frame_base.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: jsola
 */



#include "utils_gtest.h"
#include "../logging.h"

#include "../frame_base.h"
#include "../sensor_odom_2D.h"

#include <iostream>


using namespace Eigen;
using namespace std;
using namespace wolf;


TEST(FrameBase, GettersAndSetters)
{
    FrameBasePtr f = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));

    // getters
    ASSERT_EQ(f->id(), 1);
    ASSERT_EQ(f->getTimeStamp(), 1);
    TimeStamp t;
    f->getTimeStamp(t);
    ASSERT_EQ(t, 1);
    ASSERT_EQ(f->getPPtr()->getVector().size(), 2);
    ASSERT_EQ(f->getOPtr()->getVector().size(), 1);
    ASSERT_EQ(f->getVPtr(), nullptr);
    ASSERT_EQ(f->isFixed(), false);
    ASSERT_EQ(f->isKey(), false);
    ASSERT_EQ(f->getStateBlockVec().size(), 3);
    ASSERT_FALSE(f->getTrajectoryPtr());
    ASSERT_FALSE(f->getProblem());
    //    ASSERT_THROW(f->getPreviousFrame(), std::runtime_error);  // protected by assert()
    //    ASSERT_EQ(f->getStatus(), ST_ESTIMATED);                  // protected
    ASSERT_FALSE(f->getCaptureOf(make_shared<SensorOdom2D>(nullptr, nullptr, 1,1)));
    ASSERT_TRUE(f->getCaptureList().empty());
    ASSERT_TRUE(f->getConstrainedByList().empty());
}

TEST(FrameBase, TimeStamp)
{

}

TEST(FrameBase, LinksToTree)
{

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




