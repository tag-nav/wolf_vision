/*
 * gtest_problem.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: jsola
 */

#include "utils_gtest.h"
#include "../src/logging.h"

#include "../problem.h"
#include "../trajectory_base.h"
#include "../frame_base.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;


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

