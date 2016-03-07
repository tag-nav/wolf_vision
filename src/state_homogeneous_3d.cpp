/*
 * \file state_homogeneous_3d.cpp
 *
 *  Created on: Mar 7, 2016
 *      \author: jsola
 */

#include "state_homogeneous_3d.h"
#include "local_parametrization_homogeneous.h"

StateHomogeneous3d::StateHomogeneous3d(const unsigned int _size, bool _fixed) :
        StateBlock(_size, _fixed)
{
    local_param_ptr_ = new LocalParametrizationHomogeneous;
}

StateHomogeneous3d::~StateHomogeneous3d()
{
    delete local_param_ptr_;
}

