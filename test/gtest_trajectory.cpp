/*
 * gtest_trajectory.cpp
 *
 *  Created on: Nov 13, 2016
 *      Author: jsola
 */

#include "utils_gtest.h"
#include "base/utils/logging.h"

#include "base/problem/problem.h"
#include "base/trajectory/trajectory_base.h"
#include "base/frame/frame_base.h"

#include <iostream>

using namespace wolf;

struct DummyNotificationProcessor
{
  DummyNotificationProcessor(ProblemPtr _problem)
    : problem_(_problem)
  {
    //
  }

  void update()
  {
    if (problem_ == nullptr)
    {
      FAIL() << "problem_ is nullptr !";
    }

    auto sb_noti_map = problem_->consumeStateBlockNotificationMap();
    ASSERT_TRUE(problem_->consumeStateBlockNotificationMap().empty()); // consume should empty the notification map

    while (!sb_noti_map.empty())
    {
        switch (sb_noti_map.begin()->second)
        {
          case ADD:
          {
            break;
          }
          case REMOVE:
          {
            break;
          }
          default:
            throw std::runtime_error("SolverManager::update: State Block notification must be ADD or REMOVE.");
        }
        sb_noti_map.erase(sb_noti_map.begin());
    }
    ASSERT_TRUE(sb_noti_map.empty());
  }

  ProblemPtr problem_;
};

/// Set to true if you want debug info
bool debug = false;

TEST(TrajectoryBase, ClosestKeyFrame)
{

    ProblemPtr P = Problem::create("PO 2D");
    TrajectoryBasePtr T = P->getTrajectory();

    // Trajectory status:
    //  KF1   KF2  AuxF3  F4       frames
    //   1     2     3     4       time stamps
    // --+-----+-----+-----+--->   time

    FrameBasePtr F1 = std::make_shared<FrameBase>(KEY,     1, nullptr, nullptr);
    FrameBasePtr F2 = std::make_shared<FrameBase>(KEY,     2, nullptr, nullptr);
    FrameBasePtr F3 = std::make_shared<FrameBase>(AUXILIARY,     3, nullptr, nullptr);
    FrameBasePtr F4 = std::make_shared<FrameBase>(NON_ESTIMATED, 4, nullptr, nullptr);
    T->addFrame(F1);
    T->addFrame(F2);
    T->addFrame(F3);
    T->addFrame(F4);

    FrameBasePtr KF; // closest key-frame queried

    KF = T->closestKeyFrameToTimeStamp(-0.8);                // before all keyframes    --> return F1
    ASSERT_EQ(KF->id(), F1->id());                           // same id!

    KF = T->closestKeyFrameToTimeStamp(1.1);                 // between keyframes       --> return F1
    ASSERT_EQ(KF->id(), F1->id());                           // same id!

    KF = T->closestKeyFrameToTimeStamp(1.9);                 // between keyframes       --> return F2
    ASSERT_EQ(KF->id(), F2->id());                           // same id!

    KF = T->closestKeyFrameToTimeStamp(2.6);                 // between keyframe and auxiliary frame, but closer to auxiliary frame --> return F2
    ASSERT_EQ(KF->id(), F2->id());                           // same id!

    KF = T->closestKeyFrameToTimeStamp(3.2);                 // after the auxiliary frame, between closer to frame --> return F2
    ASSERT_EQ(KF->id(), F2->id());                           // same id!

    KF = T->closestKeyFrameToTimeStamp(4.2);                 // after the last frame --> return F2
    ASSERT_EQ(KF->id(), F2->id());                           // same id!
}

TEST(TrajectoryBase, ClosestKeyOrAuxFrame)
{

    ProblemPtr P = Problem::create("PO 2D");
    TrajectoryBasePtr T = P->getTrajectory();

    // Trajectory status:
    //  KF1   KF2    F3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr F1 = std::make_shared<FrameBase>(KEY,           1, nullptr, nullptr);
    FrameBasePtr F2 = std::make_shared<FrameBase>(AUXILIARY,     2, nullptr, nullptr);
    FrameBasePtr F3 = std::make_shared<FrameBase>(NON_ESTIMATED, 3, nullptr, nullptr);
    T->addFrame(F1);
    T->addFrame(F2);
    T->addFrame(F3);

    FrameBasePtr KF; // closest key-frame queried

    KF = T->closestKeyOrAuxFrameToTimeStamp(-0.8);          // before all keyframes    --> return f0
    ASSERT_EQ(KF->id(), F1->id());                           // same id!

    KF = T->closestKeyOrAuxFrameToTimeStamp(1.1);           // between keyframes       --> return F1
    ASSERT_EQ(KF->id(), F1->id());                           // same id!

    KF = T->closestKeyOrAuxFrameToTimeStamp(1.9);           // between keyframes       --> return F2
    ASSERT_EQ(KF->id(), F2->id());                           // same id!

    KF = T->closestKeyOrAuxFrameToTimeStamp(2.6);           // between keyframe and frame, but closer to frame --> return F2
    ASSERT_EQ(KF->id(), F2->id());                           // same id!

    KF = T->closestKeyOrAuxFrameToTimeStamp(3.2);           // after the last frame    --> return F2
    ASSERT_EQ(KF->id(), F2->id());                           // same id!
}

TEST(TrajectoryBase, Add_Remove_Frame)
{
    using std::make_shared;

    ProblemPtr P = Problem::create("PO 2D");
    TrajectoryBasePtr T = P->getTrajectory();

    DummyNotificationProcessor N(P);

    // Trajectory status:
    //  KF1   KF2    F3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr F1 = std::make_shared<FrameBase>(KEY,     1, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // 2 non-fixed
    FrameBasePtr F2 = std::make_shared<FrameBase>(KEY,     2, make_shared<StateBlock>(2), make_shared<StateBlock>(1, true)); // 1 fixed, 1 not
    FrameBasePtr F3 = std::make_shared<FrameBase>(NON_ESTIMATED, 3, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame

    std::cout << __LINE__ << std::endl;

    // add frames and keyframes
    T->addFrame(F1); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), (unsigned int) 1);
    ASSERT_EQ(P->consumeStateBlockNotificationMap(). size(), (unsigned int) 2);
    std::cout << __LINE__ << std::endl;

    T->addFrame(F2); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), (unsigned int) 2);
    std::cout << __LINE__ << std::endl;

    T->addFrame(F3); // F (not KF so state blocks are not notified)
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), (unsigned int) 3);
    ASSERT_EQ(P->consumeStateBlockNotificationMap(). size(), (unsigned int) 2); // consumeStateBlockNotificationMap empties the notification map, 2 state blocks were notified since last call to consumeStateBlockNotificationMap
    std::cout << __LINE__ << std::endl;

    ASSERT_EQ(T->getLastFrame()->id(), F3->id());
    ASSERT_EQ(T->getLastKeyFrame()->id(), F2->id());
    std::cout << __LINE__ << std::endl;

    N.update();
    ASSERT_TRUE(P->consumeStateBlockNotificationMap().empty()); // consumeStateBlockNotificationMap was called in update() so it should be empty
    std::cout << __LINE__ << std::endl;

    // remove frames and keyframes
    F2->remove(); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), (unsigned int) 2);
    ASSERT_EQ(P->consumeStateBlockNotificationMap(). size(), (unsigned int) 2); // consumeStateBlockNotificationMap empties the notification map, 2 state blocks were notified since last call to consumeStateBlockNotificationMap
    std::cout << __LINE__ << std::endl;

    ASSERT_EQ(T->getLastFrame()->id(), F3->id());
    ASSERT_EQ(T->getLastKeyFrame()->id(), F1->id());
    std::cout << __LINE__ << std::endl;

    F3->remove(); // F
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), (unsigned int) 1);
    std::cout << __LINE__ << std::endl;

    ASSERT_EQ(T->getLastKeyFrame()->id(), F1->id());

    F1->remove(); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().                 size(), (unsigned int) 0);
    std::cout << __LINE__ << std::endl;

    N.update();

    ASSERT_TRUE(P->consumeStateBlockNotificationMap().empty()); // consumeStateBlockNotificationMap was called in update() so it should be empty
    std::cout << __LINE__ << std::endl;
}

TEST(TrajectoryBase, KeyFramesAreSorted)
{
    using std::make_shared;

    ProblemPtr P = Problem::create("PO 2D");
    TrajectoryBasePtr T = P->getTrajectory();

    // Trajectory status:
    //  KF1   KF2    F3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr F1 = std::make_shared<FrameBase>(KEY,     1, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // 2 non-fixed
    FrameBasePtr F2 = std::make_shared<FrameBase>(KEY,     2, make_shared<StateBlock>(2), make_shared<StateBlock>(1, true)); // 1 fixed, 1 not
    FrameBasePtr F3 = std::make_shared<FrameBase>(NON_ESTIMATED, 3, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame

    // add frames and keyframes in random order --> keyframes must be sorted after that
    T->addFrame(F2); // KF2
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F2->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F2->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    T->addFrame(F3); // F3
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F3->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F2->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    T->addFrame(F1); // KF1
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F3->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F2->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    F3->setKey(); // set KF3
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F3->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F3->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F3->id());

    FrameBasePtr F4 = std::make_shared<FrameBase>(NON_ESTIMATED, 1.5, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame
    T->addFrame(F4);
    // Trajectory status:
    //  KF1   KF2   KF3     F4       frames
    //   1     2     3     1.5       time stamps
    // --+-----+-----+------+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F4->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F3->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F3->id());

    F4->setKey();
    // Trajectory status:
    //  KF1   KF4   KF2    KF3       frames
    //   1    1.5    2      3        time stamps
    // --+-----+-----+------+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F3->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F3->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F3->id());

    F2->setTimeStamp(4);
    // Trajectory status:
    //  KF1   KF4   KF3    KF2       frames
    //   1    1.5    3      4        time stamps
    // --+-----+-----+------+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F2->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F2->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    F4->setTimeStamp(0.5);
    // Trajectory status:
    //  KF4   KF2   KF3    KF2       frames
    //  0.5    1     3      4        time stamps
    // --+-----+-----+------+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getFrameList().front()->id(), F4->id());

    FrameBasePtr F5 = std::make_shared<FrameBase>(AUXILIARY, 1.5, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame
    T->addFrame(F5);
    // Trajectory status:
    //  KF4   KF2  AuxF5  KF3   KF2       frames
    //  0.5    1    1.5    3     4        time stamps
    // --+-----+-----+-----+-----+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F2->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F2->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    F5->setTimeStamp(5);
    // Trajectory status:
    //  KF4   KF2   KF3   KF2   AuxF5     frames
    //  0.5    1     3     4     5        time stamps
    // --+-----+-----+-----+-----+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F5->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F5->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    FrameBasePtr F6 = std::make_shared<FrameBase>(NON_ESTIMATED, 1.5, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame
    T->addFrame(F6);
    // Trajectory status:
    //  KF4   KF2   KF3   KF2   AuxF5  F6       frames
    //  0.5    1     3     4     5     6        time stamps
    // --+-----+-----+-----+-----+-----+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F6->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F5->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

