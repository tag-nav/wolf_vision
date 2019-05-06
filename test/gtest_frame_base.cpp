/*
 * gtest_frame_base.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: jsola
 */

#include "utils_gtest.h"
#include "core/utils/logging.h"

#include "core/frame/frame_base.h"
#include "core/sensor/sensor_odom_2D.h"
#include "core/processor/processor_odom_2D.h"
#include "core/factor/factor_odom_2D.h"
#include "core/capture/capture_motion.h"

#include <iostream>

using namespace Eigen;
using namespace std;
using namespace wolf;

TEST(FrameBase, GettersAndSetters)
{
    FrameBasePtr F = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));

    // getters
    ASSERT_EQ(F->id(), (unsigned int) 1);
    ASSERT_EQ(F->getTimeStamp(), 1);
    TimeStamp t;
    F->getTimeStamp(t);
    ASSERT_EQ(t, 1);
    ASSERT_FALSE(F->isFixed());
    ASSERT_EQ(F->isKey(), false);
    ASSERT_EQ(F->isKeyOrAux(), false);
}

TEST(FrameBase, StateBlocks)
{
    FrameBasePtr F = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));

    ASSERT_EQ(F->getStateBlockVec().size(),   (unsigned int) 3);
    ASSERT_EQ(F->getP()->getState().size(),(unsigned int) 2);
    ASSERT_EQ(F->getO()->getState().size(),(unsigned int) 1);
    ASSERT_EQ(F->getV(), nullptr);
}

TEST(FrameBase, LinksBasic)
{
    FrameBasePtr F = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));

    ASSERT_FALSE(F->getTrajectory());
    ASSERT_FALSE(F->getProblem());
    //    ASSERT_THROW(f->getPreviousFrame(), std::runtime_error);  // protected by assert()
    //    ASSERT_EQ(f->getStatus(), ST_ESTIMATED);                  // protected
    SensorOdom2DPtr S = make_shared<SensorOdom2D>(Vector3s::Zero(), IntrinsicsOdom2D());
    ASSERT_FALSE(F->getCaptureOf(S));
    ASSERT_TRUE(F->getCaptureList().empty());
    ASSERT_TRUE(F->getConstrainedByList().empty());
    ASSERT_EQ(F->getHits() , (unsigned int) 0);
}

TEST(FrameBase, LinksToTree)
{
    // Problem with 2 frames and one motion factor between them
    ProblemPtr P = Problem::create("PO 2D");
    TrajectoryBasePtr T = P->getTrajectory();
    IntrinsicsOdom2D intrinsics_odo;
    intrinsics_odo.k_disp_to_disp = 1;
    intrinsics_odo.k_rot_to_rot = 1;
    SensorOdom2DPtr S = make_shared<SensorOdom2D>(Vector3s::Zero(), intrinsics_odo);
    P->getHardware()->addSensor(S);
    FrameBasePtr F1 = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));
    T->addFrame(F1);
    FrameBasePtr F2 = make_shared<FrameBase>(1, make_shared<StateBlock>(2), make_shared<StateBlock>(1));
    T->addFrame(F2);
    CaptureMotionPtr C = make_shared<CaptureMotion>("MOTION", 1, S, Vector3s::Zero(), 3, 3, nullptr);
    F1->addCapture(C);
    /// @todo link sensor & proccessor
    ProcessorBasePtr p = std::make_shared<ProcessorOdom2D>(make_shared<ProcessorParamsOdom2D>());
    FeatureBasePtr f = make_shared<FeatureBase>("f", Vector1s(1), Matrix<Scalar,1,1>::Identity()*.01);
    C->addFeature(f);
    FactorOdom2DPtr c = make_shared<FactorOdom2D>(f, F2, p);
    f->addFactor(c);

    // c-by link F2 -> c not yet established
    ASSERT_TRUE(F2->getConstrainedByList().empty());

    // tree is inconsistent since we are missing the constrained_by link
    ASSERT_FALSE(P->check(0));

    // establish constrained_by link F2 -> c
    F2->addConstrainedBy(c);

    // tree is now consistent
    ASSERT_TRUE(P->check(0));

    // F1 has one capture and no factors-by
    ASSERT_FALSE(F1->getCaptureList().empty());
    ASSERT_TRUE(F1->getConstrainedByList().empty());
    ASSERT_EQ(F1->getHits() , (unsigned int) 0);

    // F2 has no capture and one factor-by
    ASSERT_TRUE(F2->getCaptureList().empty());
    ASSERT_FALSE(F2->getConstrainedByList().empty());
    ASSERT_EQ(F2->getHits() , (unsigned int) 1);

    // fix and unfix
    F1->fix();
    ASSERT_TRUE(F1->isFixed());
    F1->unfix();
    ASSERT_FALSE(F1->isFixed());
    F1->fix();
    ASSERT_TRUE(F1->isFixed());
    F1->getP()->unfix();
    ASSERT_FALSE(F1->isFixed());
    F1->unfix();
    ASSERT_FALSE(F1->isFixed());
    F1->getP()->fix();
    ASSERT_FALSE(F1->isFixed());
    F1->getO()->fix();
    ASSERT_TRUE(F1->isFixed());

    // set key
    F1->setKey();
    ASSERT_TRUE(F1->isKey());
    ASSERT_TRUE(F1->isKeyOrAux());

    // Unlink
    F1->unlinkCapture(C);
    ASSERT_TRUE(F1->getCaptureList().empty());
}

#include "core/state_block/state_quaternion.h"
TEST(FrameBase, GetSetState)
{
    // Create PQV_3D state blocks
    StateBlockPtr sbp = make_shared<StateBlock>(3);
    StateBlockPtr sbv = make_shared<StateBlock>(3);
    StateQuaternionPtr sbq = make_shared<StateQuaternion>();

    // Create frame
    FrameBase F(1, sbp, sbq, sbv);

    // Give values to vectors and vector blocks
    VectorXs x(10), x1(10), x2(10);
    VectorXs p(3), v(3), q(4);
    p << 1,2,3;
    v << 9,8,7;
    q << .5, -.5, .5, -.5;

    x << p, q, v;

    // Set the state, check that state blocks hold the current states
    F.setState(x);
    ASSERT_TRUE((p - F.getP()->getState()).isMuchSmallerThan(1, Constants::EPS_SMALL));
    ASSERT_TRUE((q - F.getO()->getState()).isMuchSmallerThan(1, Constants::EPS_SMALL));
    ASSERT_TRUE((v - F.getV()->getState()).isMuchSmallerThan(1, Constants::EPS_SMALL));

    // Get the state, form 1 by reference
    F.getState(x1);
    ASSERT_TRUE((x1 - x).isMuchSmallerThan(1, Constants::EPS_SMALL));

    // get the state, form 2 by return value
    x2 = F.getState();
    ASSERT_TRUE((x2 - x).isMuchSmallerThan(1, Constants::EPS_SMALL));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

