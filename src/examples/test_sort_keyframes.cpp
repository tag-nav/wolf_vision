/**
 * \file test_sort_keyframes.cpp
 *
 *  Created on: May 24, 2016
 *      \author: jvallve
 */

// Wolf includes
#include "capture_fix.h"
#include "state_block.h"
#include "wolf.h"
#include "ceres_wrapper/ceres_manager.h"

// STL includes
#include <map>
#include <list>
#include <algorithm>
#include <iterator>

// General includes
#include <iostream>
#include <iomanip>      // std::setprecision

int main()
{
    std::cout << std::setprecision(3);

    using namespace wolf;

    Problem problem = new Problem(FRM_PO_2D);

    problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.1));
    problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.2));
    problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.3));
    problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.4));
    problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.5));
    problem.createFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.6));

    return 0;
}
