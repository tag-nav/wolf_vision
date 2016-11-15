/*
 * gtest_trajectory.cpp
 *
 *  Created on: Nov 13, 2016
 *      Author: jsola
 */


#include "utils_gtest.h"
#include "../src/logging.h"

#include "../problem.h"
#include "../trajectory_base.h"
#include "../frame_base.h"

#include <iostream>

using namespace wolf;

/// Set to true if you want debug info
bool debug = false;

TEST(TrajectoryBase, ClosestKeyFrame)
{

    ProblemPtr P = Problem::create(FRM_PO_2D);
    TrajectoryBasePtr T = P->getTrajectoryPtr();

    // Trajectory status:
    //  kf1   kf2    f3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr f1 = std::make_shared<FrameBase>(KEY_FRAME,     1, nullptr, nullptr);
    FrameBasePtr f2 = std::make_shared<FrameBase>(KEY_FRAME,     2, nullptr, nullptr);
    FrameBasePtr f3 = std::make_shared<FrameBase>(NON_KEY_FRAME, 3, nullptr, nullptr);
    T->addFrame(f1);
    T->addFrame(f2);
    T->addFrame(f3);

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

TEST(TrajectoryBase, Add_Remove_Frame)
{
    using std::make_shared;

    ProblemPtr P = Problem::create(FRM_PO_2D);
    TrajectoryBasePtr T = P->getTrajectoryPtr();

    // Trajectory status:
    //  kf1   kf2    f3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr f1 = std::make_shared<FrameBase>(KEY_FRAME,     1, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // 2 non-fixed
    FrameBasePtr f2 = std::make_shared<FrameBase>(KEY_FRAME,     2, make_shared<StateBlock>(2), make_shared<StateBlock>(1, true)); // 1 fixed, 1 not
    FrameBasePtr f3 = std::make_shared<FrameBase>(NON_KEY_FRAME, 3, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame

    // add frames and keyframes
    T->addFrame(f1); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), 1);
    ASSERT_EQ(P->getStateBlockList().            size(), 2);
    ASSERT_EQ(P->getStateBlockNotificationList().size(), 2);

    T->addFrame(f2); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), 2);
    ASSERT_EQ(P->getStateBlockList().            size(), 4);
    ASSERT_EQ(P->getStateBlockNotificationList().size(), 4);

    T->addFrame(f3); // F
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), 3);
    ASSERT_EQ(P->getStateBlockList().            size(), 4);
    ASSERT_EQ(P->getStateBlockNotificationList().size(), 4);

    ASSERT_EQ(T->getLastFramePtr()->id(), f3->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f2->id());

    // remove frames and keyframes
    f2->remove(); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), 2);
    ASSERT_EQ(P->getStateBlockList().            size(), 2);
    ASSERT_EQ(P->getStateBlockNotificationList().size(), 2);

    ASSERT_EQ(T->getLastFramePtr()->id(), f3->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f1->id());

    f3->remove(); // F
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), 1);
    ASSERT_EQ(P->getStateBlockList().            size(), 2);
    ASSERT_EQ(P->getStateBlockNotificationList().size(), 2);

    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f1->id());

    f1->remove(); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), 0);
    ASSERT_EQ(P->getStateBlockList().            size(), 0);
    ASSERT_EQ(P->getStateBlockNotificationList().size(), 0);

}

TEST(TrajectoryBase, KeyFramesAreSorted)
{
    using std::make_shared;

    ProblemPtr P = Problem::create(FRM_PO_2D);
    TrajectoryBasePtr T = P->getTrajectoryPtr();

    // Trajectory status:
    //  kf1   kf2    f3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr f1 = std::make_shared<FrameBase>(KEY_FRAME,     1, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // 2 non-fixed
    FrameBasePtr f2 = std::make_shared<FrameBase>(KEY_FRAME,     2, make_shared<StateBlock>(2), make_shared<StateBlock>(1, true)); // 1 fixed, 1 not
    FrameBasePtr f3 = std::make_shared<FrameBase>(NON_KEY_FRAME, 3, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame

    // add frames and keyframes in random order --> keyframes must be sorted after that
    T->addFrame(f2); // KF2
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFramePtr()   ->id(), f2->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f2->id());

    T->addFrame(f3); // F3
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFramePtr()   ->id(), f3->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f2->id());

    T->addFrame(f1); // KF1
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFramePtr()   ->id(), f3->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f2->id());

    f3->setKey(); // set KF3
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFramePtr()   ->id(), f3->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f3->id());

    FrameBasePtr f4 = std::make_shared<FrameBase>(NON_KEY_FRAME, 1.5, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame
    T->addFrame(f4);
    // Trajectory status:
    //  kf1   kf2   kf3     f4       frames
    //   1     2     3     1.5       time stamps
    // --+-----+-----+------+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFramePtr()   ->id(), f4->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f3->id());

    f4->setKey();
    // Trajectory status:
    //  kf1   kf4   kf2    kf3       frames
    //   1    1.5    2      3        time stamps
    // --+-----+-----+------+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFramePtr()   ->id(), f3->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f3->id());

    f2->setTimeStamp(4);
    // Trajectory status:
    //  kf1   kf4   kf3    kf2       frames
    //   1    1.5    3      4        time stamps
    // --+-----+-----+------+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFramePtr()   ->id(), f2->id());
    ASSERT_EQ(T->getLastKeyFramePtr()->id(), f2->id());

    f4->setTimeStamp(0.5);
    // Trajectory status:
    //  kf4   kf2   kf3    kf2       frames
    //  0.5    1     3      4        time stamps
    // --+-----+-----+------+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getFrameList().front()->id(), f4->id());

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

