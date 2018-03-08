/*
 * gtest_problem.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: jsola
 */

#include "utils_gtest.h"
#include "../src/logging.h"

#include "../problem.h"
#include "../sensor_base.h"
#include "../sensor_odom_3D.h"
#include "../processor_odom_3D.h"
#include "../processor_tracker_feature_dummy.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;

TEST(Problem, create)
{
    ProblemPtr P = Problem::create("POV 3D");

    // check double ointers to branches
    ASSERT_EQ(P, P->getHardwarePtr()->getProblem());
    ASSERT_EQ(P, P->getTrajectoryPtr()->getProblem());
    ASSERT_EQ(P, P->getMapPtr()->getProblem());

    // check frame structure through the state size
    ASSERT_EQ(P->getFrameStructureSize(), 10);
}

TEST(Problem, Sensors)
{
    ProblemPtr P = Problem::create("POV 3D");

    // add a dummy sensor
    SensorBasePtr S = std::make_shared<SensorBase>("Dummy", nullptr, nullptr, nullptr, 2, false);
    P->addSensor(S);

    // check pointers
    ASSERT_EQ(P, S->getProblem());
    ASSERT_EQ(P->getHardwarePtr(), S->getHardwarePtr());

}

TEST(Problem, Processor)
{
    ProblemPtr P = Problem::create("PO 3D");

    // check motion processor is null
    ASSERT_FALSE(P->getProcessorMotionPtr());

    // add a motion sensor and processor
    SensorBasePtr Sm = std::make_shared<SensorOdom3D>(nullptr, nullptr, std::make_shared<IntrinsicsOdom3D>());
    P->addSensor(Sm);

    // add motion processor
    ProcessorMotionPtr Pm = std::make_shared<ProcessorOdom3D>();
    Sm->addProcessor(Pm);

    // check motion processor IS NOT set by addSensor <-- using InstallProcessor it should, see test Installers
    ASSERT_FALSE(P->getProcessorMotionPtr());

    // set processor motion
    P->setProcessorMotion(Pm);
    // re-check motion processor IS set by addSensor
    ASSERT_EQ(P->getProcessorMotionPtr(), Pm);
}

TEST(Problem, Installers)
{
    std::string wolf_root = _WOLF_ROOT_DIR;
    ProblemPtr P = Problem::create("PO 3D");
    Eigen::Vector7s xs;

    SensorBasePtr    S = P->installSensor   ("ODOM 3D", "odometer",        xs,         wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // install processor tracker (dummy installation under an Odometry sensor -- it's OK for this test)
    ProcessorBasePtr pt = std::make_shared<ProcessorTrackerFeatureDummy>(ProcessorTrackerFeatureDummy(0.1, 5, 10));
    S->addProcessor(pt);


    // check motion processor IS NOT set
    ASSERT_FALSE(P->getProcessorMotionPtr());

    // install processor motion
    ProcessorBasePtr pm = P->installProcessor("ODOM 3D", "odom integrator", "odometer", wolf_root + "/src/examples/processor_odom_3D.yaml");

    // check motion processor is set
    ASSERT_TRUE(P->getProcessorMotionPtr());

    // check motion processor is correct
    ASSERT_EQ(P->getProcessorMotionPtr(), pm);
}

TEST(Problem, SetOrigin_PO_2D)
{
    ProblemPtr P = Problem::create("PO 2D");
    TimeStamp       t0(0);
    Eigen::VectorXs x0(3); x0 << 1,2,3;
    Eigen::MatrixXs P0(3,3); P0.setIdentity(); P0 *= 0.1; // P0 is 0.1*Id

    P->setPrior(x0, P0, t0, 1.0);

    // check that no sensor has been added
    ASSERT_EQ(P->getHardwarePtr()->getSensorList().size(), 0);

    // check that the state is correct
    ASSERT_TRUE((x0 - P->getCurrentState()).isMuchSmallerThan(1, Constants::EPS_SMALL));

    // check that we have one frame, one capture, one feature, one constraint
    TrajectoryBasePtr T = P->getTrajectoryPtr();
    ASSERT_EQ(T->getFrameList().size(), 1);
    FrameBasePtr F = P->getLastFramePtr();
    ASSERT_EQ(F->getCaptureList().size(), 1);
    CaptureBasePtr C = F->getCaptureList().front();
    ASSERT_EQ(C->getFeatureList().size(), 1);
    FeatureBasePtr f = C->getFeatureList().front();
    ASSERT_EQ(f->getConstraintList().size(), 1);

    // check that the constraint is absolute (no pointers to other F, f, or L)
    ConstraintBasePtr c = f->getConstraintList().front();
    ASSERT_FALSE(c->getFrameOtherPtr());
    ASSERT_FALSE(c->getFrameOtherPtr());
    ASSERT_FALSE(c->getLandmarkOtherPtr());

    // check that the Feature measurement and covariance are the ones provided
    ASSERT_TRUE((x0 - f->getMeasurement()).isMuchSmallerThan(1, Constants::EPS_SMALL));
    ASSERT_TRUE((P0 - f->getMeasurementCovariance()).isMuchSmallerThan(1, Constants::EPS_SMALL));

    //    P->print(4,1,1,1);
}


TEST(Problem, SetOrigin_PO_3D)
{
    ProblemPtr P = Problem::create("PO 3D");
    TimeStamp       t0(0);
    Eigen::VectorXs x0(7); x0 << 1,2,3,4,5,6,7;
    Eigen::MatrixXs P0(6,6); P0.setIdentity(); P0 *= 0.1; // P0 is 0.1*Id

    P->setPrior(x0, P0, t0, 1.0);

    // check that no sensor has been added
    ASSERT_EQ(P->getHardwarePtr()->getSensorList().size(), 0);

    // check that the state is correct
    ASSERT_TRUE((x0 - P->getCurrentState()).isMuchSmallerThan(1, Constants::EPS_SMALL));

    // check that we have one frame, one capture, one feature, one constraint
    TrajectoryBasePtr T = P->getTrajectoryPtr();
    ASSERT_EQ(T->getFrameList().size(), 1);
    FrameBasePtr F = P->getLastFramePtr();
    ASSERT_EQ(F->getCaptureList().size(), 1);
    CaptureBasePtr C = F->getCaptureList().front();
    ASSERT_EQ(C->getFeatureList().size(), 1);
    FeatureBasePtr f = C->getFeatureList().front();
    ASSERT_EQ(f->getConstraintList().size(), 1);

    // check that the constraint is absolute (no pointers to other F, f, or L)
    ConstraintBasePtr c = f->getConstraintList().front();
    ASSERT_FALSE(c->getFrameOtherPtr());
    ASSERT_FALSE(c->getFrameOtherPtr());
    ASSERT_FALSE(c->getLandmarkOtherPtr());

    // check that the Feature measurement and covariance are the ones provided
    ASSERT_TRUE((x0 - f->getMeasurement()).isMuchSmallerThan(1, Constants::EPS_SMALL));
    ASSERT_TRUE((P0 - f->getMeasurementCovariance()).isMuchSmallerThan(1, Constants::EPS_SMALL));

    //    P->print(4,1,1,1);
}



TEST(Problem, emplaceFrame_factory)
{
    ProblemPtr P = Problem::create("PO 2D");

    FrameBasePtr f0 = P->emplaceFrame("PO 2D",    KEY_FRAME, VectorXs(3),  TimeStamp(0.0));
    FrameBasePtr f1 = P->emplaceFrame("PO 3D",    KEY_FRAME, VectorXs(7),  TimeStamp(1.0));
    FrameBasePtr f2 = P->emplaceFrame("POV 3D",   KEY_FRAME, VectorXs(10), TimeStamp(2.0));

    //    std::cout << "f0: type PO 2D?    "  << f0->getType() << std::endl;
    //    std::cout << "f1: type PO 3D?    "  << f1->getType() << std::endl;
    //    std::cout << "f2: type POV 3D?   "  << f2->getType() << std::endl;

    ASSERT_EQ(f0->getType().compare("PO 2D"), 0);
    ASSERT_EQ(f1->getType().compare("PO 3D"), 0);
    ASSERT_EQ(f2->getType().compare("POV 3D"), 0);

    // check that all frames are effectively in the trajectory
    ASSERT_EQ(P->getTrajectoryPtr()->getFrameList().size(), 3);

    // check that all frames are linked to Problem
    ASSERT_EQ(f0->getProblem(), P);
    ASSERT_EQ(f1->getProblem(), P);
    ASSERT_EQ(f2->getProblem(), P);
}


TEST(Problem, StateBlocks)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    ProblemPtr P = Problem::create("PO 3D");
    Eigen::Vector7s xs;

    // 2 state blocks, fixed
    SensorBasePtr    Sm = P->installSensor   ("ODOM 3D", "odometer",xs, wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ASSERT_EQ(P->getStateBlockList().size(),                2);
    ASSERT_EQ(P->getStateBlockNotificationList().size(),    2);

    // 3 state blocks, fixed
    SensorBasePtr    St = P->installSensor   ("CAMERA", "camera",   xs, wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    ASSERT_EQ(P->getStateBlockList().size(),                2 + 3);
    ASSERT_EQ(P->getStateBlockNotificationList().size(),    2 + 3);

    ProcessorBasePtr pt = std::make_shared<ProcessorTrackerFeatureDummy>(ProcessorTrackerFeatureDummy(0.1, 5, 10));
    St->addProcessor(pt);
    ProcessorBasePtr pm = P->installProcessor("ODOM 3D",            "odom integrator",      "odometer", wolf_root + "/src/examples/processor_odom_3D.yaml");

    // 2 state blocks, estimated
    P->emplaceFrame("PO 3D", KEY_FRAME, xs, 0);
    ASSERT_EQ(P->getStateBlockList().size(),                2 + 3 + 2);
    ASSERT_EQ(P->getStateBlockNotificationList().size(),    2 + 3 + 2);


    //    P->print(4,1,1,1);

    // change some SB properties
    St->unfixExtrinsics();
    ASSERT_EQ(P->getStateBlockList().size(),                2 + 3 + 2);
    ASSERT_EQ(P->getStateBlockNotificationList().size(),    2 + 3 + 2 + 2); // XXX: 2 more notifications on the same SB!

    //    P->print(4,1,1,1);
}

TEST(Problem, Covariances)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    ProblemPtr P = Problem::create("PO 3D");
    Eigen::Vector7s xs;

    SensorBasePtr    Sm = P->installSensor   ("ODOM 3D", "odometer",xs, wolf_root + "/src/examples/sensor_odom_3D.yaml");
    SensorBasePtr    St = P->installSensor   ("CAMERA", "camera",   xs, wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    ProcessorBasePtr pt = std::make_shared<ProcessorTrackerFeatureDummy>(ProcessorTrackerFeatureDummy(0.1, 5, 10));
    St->addProcessor(pt);
    ProcessorBasePtr pm = P->installProcessor("ODOM 3D",            "odom integrator",      "odometer", wolf_root + "/src/examples/processor_odom_3D.yaml");

    // 4 state blocks, estimated
    St->unfixExtrinsics();
    FrameBasePtr F = P->emplaceFrame("PO 3D", KEY_FRAME, xs, 0);

    Eigen::MatrixXs Cov = P->getFrameCovariance(F);

    // FIXME Frame covariance should be 6x6, but it is actually 7x7 (the state of the state blocks, not of the local parametrizations)
    ASSERT_EQ(Cov.cols() , 7);
    ASSERT_EQ(Cov.rows() , 7);

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

