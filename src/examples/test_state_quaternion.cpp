/*
 * \file test_state_quaternion.cpp
 *
 *  Created on: Mar 7, 2016
 *      \author: jsola
 */


#include "frame_base.h"
#include "state_quaternion.h"
#include "time_stamp.h"

#include <iostream>

int main (void)
{
    using namespace wolf;

    // 3D
    StateBlockPtr pp = new StateBlock(3);
    StateQuaternion* op = new StateQuaternion;
    StateBlockPtr vp = new StateBlock(3);

    TimeStamp t;

    FrameBase pqv(t,pp,op,vp);

    std::cout << "P local param: " << pqv.getPPtr()->getLocalParametrizationPtr() << std::endl;
    std::cout << "Q local param: " << pqv.getOPtr()->getLocalParametrizationPtr() << std::endl;
    std::cout << "V local param: " << pqv.getVPtr()->getLocalParametrizationPtr() << std::endl;

    //    delete pp;
    //    delete op;
    //    delete vp;
    // Note: Deleting the StateBlock pointers will be done at the destruction of FrameBase.

    std::cout << "Done" << std::endl;


    return 1;
}
