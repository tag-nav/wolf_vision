/*
 * gtest_problem.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: jsola
 */

#include "utils_gtest.h"
#include "../src/logging.h"

#include "../problem.h"
#include "../sensor_odom_3D.h"
#include "../processor_odom_3D.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;

TEST(Problem, create)
{
    ProblemPtr P = Problem::create(FRM_PQVBB_3D);

    // check double ointers to branches
    ASSERT_EQ(P, P->getHardwarePtr()->getProblem());
    ASSERT_EQ(P, P->getTrajectoryPtr()->getProblem());
    ASSERT_EQ(P, P->getMapPtr()->getProblem());

    // check frame structure through the state size
    ASSERT_EQ(P->getFrameStructureSize(), 16);
}

TEST(Problem, Sensors)
{
    ProblemPtr P = Problem::create(FRM_PQVBB_3D);

    // add a dummy sensor
    SensorBasePtr S = std::make_shared<SensorBase>("Dummy", nullptr, nullptr, nullptr, 2, false);
    P->addSensor(S);

    // check pointers
    ASSERT_EQ(P, S->getProblem());
    ASSERT_EQ(P->getHardwarePtr(), S->getHardwarePtr());

}

TEST(Problem, Processor)
{
    ProblemPtr P = Problem::create(FRM_PO_3D);

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
    ProblemPtr P = Problem::create(FRM_PO_3D);
    Eigen::Vector7s xs;

    SensorBasePtr    S = P->installSensor   ("ODOM 3D", "odometer",        xs,         wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // install processor tracker (dummy installation under an Odometry sensor -- it's OK for this test)
    ProcessorBasePtr pt = P->installProcessor("IMAGE LANDMARK", "ORB landmark tracker", "odometer", wolf_root + "/src/examples/processor_image_ORB.yaml");

    // check motion processor IS NOT set
    ASSERT_FALSE(P->getProcessorMotionPtr());

    // install processor motion
    ProcessorBasePtr pm = P->installProcessor("ODOM 3D", "odom integrator", "odometer", wolf_root + "/src/examples/processor_odom_3D.yaml");

    // check motion processor is set
    ASSERT_TRUE(P->getProcessorMotionPtr());

    // check motion processor is correct
    ASSERT_EQ(P->getProcessorMotionPtr(), pm);
}



TEST(Problem, emplaceFrame_factory)
{
    ProblemPtr P = Problem::create(FRM_PO_2D);

    FrameBasePtr f0 = P->emplaceFrame("PO 2D",    KEY_FRAME, VectorXs(3),  TimeStamp(0.0));
    FrameBasePtr f1 = P->emplaceFrame("PO 3D",    KEY_FRAME, VectorXs(7),  TimeStamp(1.0));
    FrameBasePtr f2 = P->emplaceFrame("POV 3D",   KEY_FRAME, VectorXs(10), TimeStamp(2.0));
    FrameBasePtr f3 = P->emplaceFrame("PQVBB 3D", KEY_FRAME, VectorXs(16), TimeStamp(3.0));
    FrameBasePtr f4 = P->emplaceFrame("IMU",      KEY_FRAME, VectorXs(16), TimeStamp(4.0));

    //    std::cout << "f0: type PO 2D?    "  << f0->getType() << std::endl;
    //    std::cout << "f1: type PO 3D?    "  << f1->getType() << std::endl;
    //    std::cout << "f2: type POV 3D?   "  << f2->getType() << std::endl;
    //    std::cout << "f3: type PQVBB 3D? "  << f3->getType() << std::endl;
    //    std::cout << "f4: type IMU?      "  << f4->getType() << std::endl;

    ASSERT_EQ(f0->getType().compare("PO 2D"), 0);
    ASSERT_EQ(f1->getType().compare("PO 3D"), 0);
    ASSERT_EQ(f2->getType().compare("POV 3D"), 0);
    ASSERT_EQ(f3->getType().compare("IMU"), 0);
    ASSERT_EQ(f4->getType().compare("IMU"), 0);

    // check that all frames are effectively in the trajectory
    ASSERT_EQ(P->getTrajectoryPtr()->getFrameList().size(), 5);

    // check that all frames are linked to Problem
    ASSERT_EQ(f0->getProblem(), P);
    ASSERT_EQ(f1->getProblem(), P);
    ASSERT_EQ(f2->getProblem(), P);
    ASSERT_EQ(f3->getProblem(), P);
    ASSERT_EQ(f4->getProblem(), P);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

