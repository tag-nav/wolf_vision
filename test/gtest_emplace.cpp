/*
 * gtest_emplace.cpp
 *
 *  Created on: Mar 20, 2019
 *      Author: jcasals
 */

#include "utils_gtest.h"
#include "base/logging.h"

#include "base/problem.h"
#include "base/sensor/sensor_base.h"
#include "base/sensor/sensor_odom_3D.h"
#include "base/processor/processor_odom_3D.h"
#include "base/processor/processor_odom_2D.h"
#include "base/feature/feature_odom_2D.h"
#include "base/processor/processor_tracker_feature_dummy.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;

TEST(Emplace, Landmark)
{
    ProblemPtr P = Problem::create("POV 3D");

    // LandmarkBase::emplace<LandmarkBase>(MapBaseWPtr(P->getMapPtr()),"Dummy", nullptr, nullptr);
    // LandmarkBase::emplace<LandmarkBase>(nullptr,"Dummy", nullptr, nullptr);
    LandmarkBase::emplace<LandmarkBase>(P->getMapPtr(),"Dummy", nullptr, nullptr);
    // LandmarkBasePtr l = std::make_shared<LandmarkBase>("Dummy", nullptr, nullptr);
    // P->addLandmark(l);
    // check double ointers to branches
    ASSERT_EQ(P, P->getMapPtr()->getLandmarkList().front()->getMapPtr()->getProblem());
}

TEST(Emplace, Sensor)
{
    ProblemPtr P = Problem::create("POV 3D");

    SensorBase::emplace<SensorBase>(P->getHardwarePtr(), "Dummy", nullptr, nullptr, nullptr, 2, false);
    ASSERT_EQ(P, P->getHardwarePtr()->getSensorList().front()->getHardwarePtr()->getProblem());
}
TEST(Emplace, Frame)
{
    ProblemPtr P = Problem::create("POV 3D");

    ASSERT_NE(P->getTrajectoryPtr(), nullptr);
    FrameBase::emplace<FrameBase>(P->getTrajectoryPtr(),  KEY_FRAME, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getTrajectoryPtr()->getProblem());
}

TEST(Emplace, Processor)
{
    ProblemPtr P = Problem::create("POV 3D");

    auto sen = SensorBase::emplace<SensorBase>(P->getHardwarePtr(), "Dummy", nullptr, nullptr, nullptr, 2, false);
    auto prc = ProcessorOdom2D::emplace<ProcessorOdom2D>(sen, std::make_shared<ProcessorParamsOdom2D>());
    ASSERT_EQ(P, P->getHardwarePtr()->getSensorList().front()->getProcessorList().front()->getSensorPtr()->getProblem());
    ASSERT_EQ(sen, sen->getProcessorList().front()->getSensorPtr());
    ASSERT_EQ(prc, sen->getProcessorList().front());
}

TEST(Emplace, Capture)
{
    ProblemPtr P = Problem::create("POV 3D");

    ASSERT_NE(P->getTrajectoryPtr(), nullptr);
    auto frm = FrameBase::emplace<FrameBase>(P->getTrajectoryPtr(),  KEY_FRAME, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getTrajectoryPtr()->getProblem());

    auto cpt = CaptureBase::emplace<CaptureBase>(frm, "Dummy", TimeStamp(0), nullptr, nullptr, nullptr, nullptr);
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFramePtr()->getTrajectoryPtr()->getProblem());
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getProblem());
    ASSERT_EQ(frm, frm->getCaptureList().front()->getFramePtr());
}

TEST(Emplace, Feature)
{
    ProblemPtr P = Problem::create("POV 3D");

    ASSERT_NE(P->getTrajectoryPtr(), nullptr);
    auto frm = FrameBase::emplace<FrameBase>(P->getTrajectoryPtr(),  KEY_FRAME, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getTrajectoryPtr()->getProblem());

    auto cpt = CaptureBase::emplace<CaptureBase>(frm, "Dummy", TimeStamp(0), nullptr, nullptr, nullptr, nullptr);
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFramePtr()->getTrajectoryPtr()->getProblem());
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getProblem());
    ASSERT_EQ(frm, frm->getCaptureList().front()->getFramePtr());
    auto cov = Eigen::MatrixXs(2,2);
    cov(0,0) = 1;
    cov(1,1) = 1;
    FeatureBase::emplace<FeatureBase>(cpt, "Dummy", Eigen::VectorXs(5), cov);
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getCapturePtr()->getFramePtr()->getTrajectoryPtr()->getProblem());
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getProblem());
    ASSERT_EQ(cpt, cpt->getFeatureList().front()->getCapturePtr());
}
TEST(Emplace, Factor)
{
    ProblemPtr P = Problem::create("POV 3D");

    ASSERT_NE(P->getTrajectoryPtr(), nullptr);
    auto frm = FrameBase::emplace<FrameBase>(P->getTrajectoryPtr(),  KEY_FRAME, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getTrajectoryPtr()->getProblem());

    auto cpt = CaptureBase::emplace<CaptureBase>(frm, "Dummy", TimeStamp(0), nullptr, nullptr, nullptr, nullptr);
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFramePtr()->getTrajectoryPtr()->getProblem());
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getProblem());
    ASSERT_EQ(frm, frm->getCaptureList().front()->getFramePtr());
    auto cov = Eigen::MatrixXs(2,2);
    cov(0,0) = 1;
    cov(1,1) = 1;
    auto ftr = FeatureBase::emplace<FeatureOdom2D>(cpt, Eigen::VectorXs(5), cov);
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getCapturePtr()->getFramePtr()->getTrajectoryPtr()->getProblem());
    ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getProblem());
    ASSERT_EQ(cpt, cpt->getFeatureList().front()->getCapturePtr());
    auto cnt = ConstraintBase::emplace<FeatureBasePtr,ConstraintOdom2D>(ftr, ftr, frm);
    // ftr->addConstrainedBy(cnt);
    // ftr->addConstraint(cnt);
    ASSERT_NE(nullptr, ftr->getConstraintList().front().get());
    // ASSERT_NE(nullptr, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getConstraintList().front());
    // ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getConstraintList().front()->getFeaturePtr()->getCapturePtr()->getFramePtr()->getTrajectoryPtr()->getProblem());
    // ASSERT_EQ(P, P->getTrajectoryPtr()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getConstraintList().front()->getProblem());
    // ASSERT_EQ(ftr, cnt->getFeaturePtr());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

