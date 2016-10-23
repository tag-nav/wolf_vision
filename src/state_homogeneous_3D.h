/*
 * \file state_homogeneous_3D.h
 *
 *  Created on: Mar 7, 2016
 *      \author: jsola
 */

#ifndef SRC_STATE_HOMOGENEOUS_3D_H_
#define SRC_STATE_HOMOGENEOUS_3D_H_

#include "state_block.h"
#include "local_parametrization_homogeneous.h"

namespace wolf {

class StateHomogeneous3D : public StateBlock
{
    public:
        StateHomogeneous3D(bool _fixed = false);
        StateHomogeneous3D(const Eigen::VectorXs _state, bool _fixed = false);
        virtual ~StateHomogeneous3D();
};

inline StateHomogeneous3D::StateHomogeneous3D(const Eigen::VectorXs _state, bool _fixed) :
        StateBlock(_state, _fixed)
{
    assert(_state.size() == 4 && "Homogeneous 3D must be size 4.");
    local_param_ptr_ = std::make_shared<LocalParametrizationHomogeneous>();
}

inline StateHomogeneous3D::StateHomogeneous3D(bool _fixed) :
        StateBlock(4, _fixed)
{
    local_param_ptr_ = std::make_shared<LocalParametrizationHomogeneous>();
    state_ << 0, 0, 0, 1; // Set origin
}

inline StateHomogeneous3D::~StateHomogeneous3D()
{
    // The local_param_ptr_ pointer is already deleted by the base class
}

} // namespace wolf

#endif /* SRC_STATE_HOMOGENEOUS_3D_H_ */
