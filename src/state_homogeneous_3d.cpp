/*
 * \file state_homogeneous_3d.cpp
 *
 *  Created on: Mar 7, 2016
 *      \author: jsola
 */

#include "state_homogeneous_3d.h"
#include "local_parametrization_homogeneous.h"

namespace wolf {

StateHomogeneous3d::StateHomogeneous3d(const Eigen::VectorXs _state, bool _fixed) :
        StateBlock(_state, _fixed)
{
    assert(_state.size() == 4 && "Homogeneous 3d must be size 4.");
    local_param_ptr_ = new LocalParametrizationHomogeneous;
}

StateHomogeneous3d::StateHomogeneous3d(bool _fixed) :
        StateBlock(4, _fixed)
{
    local_param_ptr_ = new LocalParametrizationHomogeneous;
}

StateHomogeneous3d::~StateHomogeneous3d()
{
    delete local_param_ptr_;
}

} // namespace wolf
