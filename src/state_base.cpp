/*
 * state_base.cpp
 *
 *  Created on: Jun 2, 2014
 *      \author: jsola
 */

#include "state_base.h"

using namespace Eigen;

StateBase::StateBase(const unsigned int _size) :
        StateRootBase(_size)//
{
    //
}

StateBase::StateBase(const VectorXs& _x) :
        StateRootBase(_x)//
{
    //
}

StateBase::StateBase(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
        StateRootBase(_st_remote, _idx, _size)//
{
    //
}

StateBase::StateBase(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x) :
        StateRootBase(_st_remote, _idx, _x)//
{
    //
}

StateBase::~StateBase()
{
    //
}


