/*
 * gtest_trajectory.cpp
 *
 *  Created on: Nov 13, 2016
 *      Author: jsola
 */


#include "utils_gtest.h"
#include "../src/logging.h"

#include "../src/wolf.h"
#include "../src/trajectory_base.h"

#include <iostream>

using namespace wolf;

TEST(TrajectoryBase, ClosestKeyFrame)
{

    ProblemPtr P = Problem::create(FRM_PO_2D);
    TrajectoryBasePtr T = P->getTrajectoryPtr();

    // Trajectory status:
    //
    //  kf1   kf2    f3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr f1 = std::make_shared<FrameBase>(KEY_FRAME,     1, nullptr, nullptr);
    FrameBasePtr f2 = std::make_shared<FrameBase>(KEY_FRAME,     2, nullptr, nullptr);
    FrameBasePtr f3 = std::make_shared<FrameBase>(NON_KEY_FRAME, 3, nullptr, nullptr);
    T->addFrame(f1);
    T->addFrame(f2);
    T->addFrame(f3);

    P->print();

    FrameBasePtr kf; // closest key-frame queried

    kf = T->closestKeyFrameToTimeStamp(-0.8);                // before all keyframes    --> return f0
    ASSERT_EQ(kf->id(), f1->id());                           // same id!

    kf = T->closestKeyFrameToTimeStamp(1.1);                 // between keyframes       --> return f1
    ASSERT_EQ(kf->id(), f1->id());                           // same id!

    kf = T->closestKeyFrameToTimeStamp(1.9);                 // between keyframes       --> return f2
    ASSERT_EQ(kf->id(), f2->id());                           // same id!

    kf = T->closestKeyFrameToTimeStamp(2.6);                 // between keyframe and frame, but closer to frame --> return f2
    ASSERT_EQ(kf->id(), f2->id());                           // same id!

    kf = T->closestKeyFrameToTimeStamp(3.2);                 // after the last frame    --> return f2
    ASSERT_EQ(kf->id(), f2->id());                           // same id!
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

