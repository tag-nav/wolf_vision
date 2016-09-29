/**
 * \file test_sort_keyframes.cpp
 *
 *  Created on: May 24, 2016
 *      \author: jvallve
 */

// Wolf includes
#include "../wolf.h"
#include "../problem.h"
#include "../frame_base.h"
#include "../trajectory_base.h"

// STL includes
#include <list>

// General includes
#include <iostream>

using namespace wolf;

void printFrames(Problem* _problem_ptr)
{
    std::cout << "TRAJECTORY:" << std::endl;
    for (auto frm : *(_problem_ptr->getTrajectoryPtr()->getFrameListPtr()))
        std::cout << "\t " << (frm->isKey() ? "KEY FRAME: " : "FRAME: ") << frm->id() << " - TS: " << frm->getTimeStamp().getSeconds() << "." << frm->getTimeStamp().getNanoSeconds() << std::endl;
}

int main()
{
    Problem problem(FRM_PO_2D);

    problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.1));
    FrameBase* frm2 = problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.2));
    FrameBase* frm3 = problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.3));
    problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.4));
    FrameBase* frm5 = problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.5));
    FrameBase* frm6 = problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.6));

    printFrames(&problem);

    std::cout << std::endl << "Frame " << frm5->id() << " set KEY" << std::endl;
    frm5->setKey();

    printFrames(&problem);

    std::cout << std::endl << "Frame " << frm2->id() << " set KEY" << std::endl;
    frm2->setKey();

    printFrames(&problem);

    std::cout << std::endl << "Frame " << frm3->id() << " new TimeStamp:" << 0.45 << std::endl;
    frm3->setTimeStamp(TimeStamp(0.45));

    printFrames(&problem);

    std::cout << std::endl << "Frame " << frm3->id() << " set KEY" << std::endl;
    frm3->setKey();

    printFrames(&problem);

    std::cout << std::endl << "Frame " << frm6->id() << " set KEY" << std::endl;
    frm6->setKey();

    printFrames(&problem);

    FrameBase* frm7 = problem.createFrame(KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.7));
    std::cout << std::endl << "created Key Frame " << frm7->id() << " TS: " << 0.7 << std::endl;

    printFrames(&problem);

    FrameBase* frm8 = problem.createFrame(KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.35));
    std::cout << std::endl << "created Key Frame " << frm8->id() << " TS: " << 0.35 << std::endl;

    printFrames(&problem);



    return 0;
}
