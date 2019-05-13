/*
 * gtest_trajectory.cpp
 *
 *  Created on: Nov 13, 2016
 *      Author: jsola
 */

#include "utils_gtest.h"
#include "core/utils/logging.h"

#include "core/problem/problem.h"
#include "core/solver/solver_manager.h"
#include "core/trajectory/trajectory_base.h"
#include "core/frame/frame_base.h"

#include <iostream>

using namespace wolf;

struct DummySolverManager : public SolverManager
{
  DummySolverManager(const ProblemPtr& _problem)
    : SolverManager(_problem)
  {
    //
  }
  virtual void computeCovariances(const CovarianceBlocksToBeComputed blocks){};
  virtual void computeCovariances(const std::vector<StateBlockPtr>& st_list){};
  virtual bool hasConverged(){return true;};
  virtual SizeStd iterations(){return 0;};
  virtual Scalar initialCost(){return 0;};
  virtual Scalar finalCost(){return 0;};
  virtual std::string solveImpl(const ReportVerbosity report_level){return std::string("");};
  virtual void addFactor(const FactorBasePtr& fac_ptr){};
  virtual void removeFactor(const FactorBasePtr& fac_ptr){};
  virtual void addStateBlock(const StateBlockPtr& state_ptr){};
  virtual void removeStateBlock(const StateBlockPtr& state_ptr){};
  virtual void updateStateBlockStatus(const StateBlockPtr& state_ptr){};
  virtual void updateStateBlockLocalParametrization(const StateBlockPtr& state_ptr){};
};

/// Set to true if you want debug info
bool debug = false;

TEST(TrajectoryBase, ClosestKeyFrame)
{

    ProblemPtr P = Problem::create("PO", 2);
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

    ProblemPtr P = Problem::create("PO", 2);
    TrajectoryBasePtr T = P->getTrajectory();

    // Trajectory status:
    //  KF1   KF2    F3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr F1 = FrameBase::emplace<FrameBase>(T, KEY,     1, nullptr, nullptr);
    FrameBasePtr F2 = FrameBase::emplace<FrameBase>(T, AUXILIARY,     2, nullptr, nullptr);
    FrameBasePtr F3 = FrameBase::emplace<FrameBase>(T, NON_ESTIMATED, 3, nullptr, nullptr);

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

    ProblemPtr P = Problem::create("PO", 2);
    TrajectoryBasePtr T = P->getTrajectory();

    DummySolverManager N(P);

    // Trajectory status:
    //  KF1   KF2    F3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr F1 = std::make_shared<FrameBase>(KEY,     1, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // 2 non-fixed
    FrameBasePtr F2 = std::make_shared<FrameBase>(KEY,     2, make_shared<StateBlock>(2), make_shared<StateBlock>(1, true)); // 1 fixed, 1 not
    FrameBasePtr F3 = std::make_shared<FrameBase>(NON_ESTIMATED, 3, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame

    // FrameBasePtr f1 = FrameBase::emplace<FrameBase>(T, KEY_FRAME,     1, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // 2 non-fixed
    // FrameBasePtr f2 = FrameBase::emplace<FrameBase>(T, KEY_FRAME,     2, make_shared<StateBlock>(2), make_shared<StateBlock>(1, true)); // 1 fixed, 1 not
    // FrameBasePtr f3 = FrameBase::emplace<FrameBase>(T, NON_KEY_FRAME, 3, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame

    std::cout << __LINE__ << std::endl;

    // add frames and keyframes
    F1->link(T);
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().             size(), (SizeStd) 1);
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) 2);
    std::cout << __LINE__ << std::endl;

    F2->link(T);
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().             size(), (SizeStd) 2);
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) 4);
    std::cout << __LINE__ << std::endl;

    F3->link(T);
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().             size(), (SizeStd) 3);
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) 4);
    std::cout << __LINE__ << std::endl;

    ASSERT_EQ(T->getLastFrame()->id(), F3->id());
    ASSERT_EQ(T->getLastKeyFrame()->id(), F2->id());
    std::cout << __LINE__ << std::endl;

    N.update();
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) 0); // consumeStateBlockNotificationMap was called in update() so it should be empty
    std::cout << __LINE__ << std::endl;

    // remove frames and keyframes
    F2->remove(); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().             size(), (SizeStd) 2);
    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) 2);
    std::cout << __LINE__ << std::endl;

    ASSERT_EQ(T->getLastFrame()->id(), F3->id());
    ASSERT_EQ(T->getLastKeyFrame()->id(), F1->id());
    std::cout << __LINE__ << std::endl;

    F3->remove(); // F
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().             size(), (SizeStd) 1);
    std::cout << __LINE__ << std::endl;

    ASSERT_EQ(T->getLastKeyFrame()->id(), F1->id());

    F1->remove(); // KF
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getFrameList().             size(), (SizeStd) 0);
    std::cout << __LINE__ << std::endl;

    N.update();

    ASSERT_EQ(P->getStateBlockNotificationMapSize(), (SizeStd) 0); // consumeStateBlockNotificationMap was called in update() so it should be empty
    std::cout << __LINE__ << std::endl;
}

TEST(TrajectoryBase, KeyFramesAreSorted)
{
    using std::make_shared;

    ProblemPtr P = Problem::create("PO", 2);
    TrajectoryBasePtr T = P->getTrajectory();

    // Trajectory status:
    //  KF1   KF2    F3      frames
    //   1     2     3       time stamps
    // --+-----+-----+--->   time

    FrameBasePtr F1 = std::make_shared<FrameBase>(KEY,     1, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // 2 non-fixed
    FrameBasePtr F2 = std::make_shared<FrameBase>(KEY,     2, make_shared<StateBlock>(2), make_shared<StateBlock>(1, true)); // 1 fixed, 1 not
    FrameBasePtr F3 = std::make_shared<FrameBase>(NON_ESTIMATED, 3, make_shared<StateBlock>(2), make_shared<StateBlock>(1)); // non-key-frame

    // add frames and keyframes in random order --> keyframes must be sorted after that
    F2->link(T);
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F2->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F2->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    F3->link(T);
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F3->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F2->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    F1->link(T);
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F3->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F2->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    F3->setKey(); // set KF3
    if (debug) P->print(2,0,0,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F3->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F3->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F3->id());

    auto F4 = P->emplaceFrame(NON_ESTIMATED, Eigen::Vector3s::Zero(), 1.5);
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

    auto F5 = P->emplaceFrame(AUXILIARY, Eigen::Vector3s::Zero(), 1.5);
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

    auto F6 = P->emplaceFrame(NON_ESTIMATED, Eigen::Vector3s::Zero(), 6);
    // Trajectory status:
    //  KF4   KF2   KF3   KF2   AuxF5  F6       frames
    //  0.5    1     3     4     5     6        time stamps
    // --+-----+-----+-----+-----+-----+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F6->id());
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F5->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

    auto F7 = P->emplaceFrame(NON_ESTIMATED, Eigen::Vector3s::Zero(), 5.5);
    // Trajectory status:
    //  KF4   KF2   KF3   KF2   AuxF5  F7   F6       frames
    //  0.5    1     3     4     5     5.5   6        time stamps
    // --+-----+-----+-----+-----+-----+-----+--->    time
    if (debug) P->print(2,0,1,0);
    ASSERT_EQ(T->getLastFrame()         ->id(), F7->id()); //Only auxiliary and key-frames are sorted
    ASSERT_EQ(T->getLastKeyOrAuxFrame()->id(), F5->id());
    ASSERT_EQ(T->getLastKeyFrame()      ->id(), F2->id());

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

