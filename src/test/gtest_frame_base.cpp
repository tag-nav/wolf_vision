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
#include "../constraint_odom_2D.h"
#include "../capture_motion.h"

#include <iostream>


using namespace Eigen;
using namespace std;
using namespace wolf;


TEST(FrameBase, GettersAndSetters)
{
    FrameBasePtr F = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));

    // getters
    ASSERT_EQ(F->id(), 1);
    ASSERT_EQ(F->getTimeStamp(), 1);
    TimeStamp t;
    F->getTimeStamp(t);
    ASSERT_EQ(t, 1);
    ASSERT_EQ(F->isFixed(), false);
    ASSERT_EQ(F->isKey(), false);
}

TEST(FrameBase, StateBlocks)
{
    FrameBasePtr F = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));

    ASSERT_EQ(F->getStateBlockVec().size(), 3);
    ASSERT_EQ(F->getPPtr()->getVector().size(), 2);
    ASSERT_EQ(F->getOPtr()->getVector().size(), 1);
    ASSERT_EQ(F->getVPtr(), nullptr);
}

TEST(FrameBase, LinksBasic)
{
    FrameBasePtr F = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));

    ASSERT_FALSE(F->getTrajectoryPtr());
    ASSERT_FALSE(F->getProblem());
    //    ASSERT_THROW(f->getPreviousFrame(), std::runtime_error);  // protected by assert()
    //    ASSERT_EQ(f->getStatus(), ST_ESTIMATED);                  // protected
    ASSERT_FALSE(F->getCaptureOf(make_shared<SensorOdom2D>(nullptr, nullptr, 1,1)));
    ASSERT_TRUE(F->getCaptureList().empty());
    ASSERT_TRUE(F->getConstrainedByList().empty());
    ASSERT_EQ(F->getHits() , 0);
}


TEST(FrameBase, LinksToTree)
{
    // Problem with 2 frames and one motion constraint between them
    ProblemPtr P = Problem::create(FRM_PO_2D);
    TrajectoryBasePtr T = P->getTrajectoryPtr();
    SensorOdom2D::Ptr S = make_shared<SensorOdom2D>(make_shared<StateBlock>(2), make_shared<StateBlock>(1), 1,1);
    P->getHardwarePtr()->addSensor(S);
    FrameBasePtr F1 = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));
    T->addFrame(F1);
    FrameBasePtr F2 = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));
    T->addFrame(F2);
    CaptureMotion::Ptr C = make_shared<CaptureMotion>(1, S, Vector3s::Zero());
    F1->addCapture(C);
    FeatureBasePtr f = make_shared<FeatureBase>("f", 1);
    C->addFeature(f);
    ConstraintOdom2D::Ptr c = make_shared<ConstraintOdom2D>(f, F2);
    f->addConstraint(c);

    // c-by link F2 -> c not yet established
    ASSERT_TRUE(F2->getConstrainedByList().empty());

    // establish link F2 -> c
    F2->addConstrainedBy(c);

    // F1 has one capture and no constraints-by
    ASSERT_FALSE(F1->getCaptureList().empty());
    ASSERT_TRUE(F1->getConstrainedByList().empty());
    ASSERT_EQ(F1->getHits() , 0);

    // F2 has no capture and one constraint-by
    ASSERT_TRUE(F2->getCaptureList().empty());
    ASSERT_FALSE(F2->getConstrainedByList().empty());
    ASSERT_EQ(F2->getHits() , 1);

    // fix and unfix
    F1->fix();
    ASSERT_TRUE(F1->isFixed());
    F1->unfix();
    ASSERT_FALSE(F1->isFixed());

    // set key
    F1->setKey();
    ASSERT_TRUE(F1->isKey());

    // Unlink
    F1->unlinkCapture(C);
    ASSERT_TRUE(F1->getCaptureList().empty());
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




