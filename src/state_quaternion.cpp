/*
 * \file StateQuaternion.cpp
 *
 *  Created on: Mar 7, 2016
 *      author: jsola
 */

#include "state_quaternion.h"
#include "local_parametrization_quaternion.h"

StateQuaternion::StateQuaternion(const Eigen::VectorXs _state, bool _fixed) :
        StateBlock(_state, _fixed)
{
    assert(_state.size() == 4 && "Quaternion must be size 4.");
    local_param_ptr_ = new LocalParametrizationQuaternion;
}

StateQuaternion::StateQuaternion(bool _fixed) :
        StateBlock(4, _fixed)
{
    local_param_ptr_ = new LocalParametrizationQuaternion;

}

StateQuaternion::~StateQuaternion()
{
    delete local_param_ptr_;
}

