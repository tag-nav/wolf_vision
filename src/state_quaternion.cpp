/*
 * \file StateQuaternion.cpp
 *
 *  Created on: Mar 7, 2016
 *      author: jsola
 */

#include "state_quaternion.h"
#include "local_parametrization_quaternion.h"

StateQuaternion::StateQuaternion(const unsigned int _size, bool _fixed) :
        StateBlock(_size, _fixed)
{
    local_param_ptr_ = new LocalParametrizationQuaternion;

}

StateQuaternion::~StateQuaternion()
{
    delete local_param_ptr_;
}

