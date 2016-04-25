/*
 * \file state_homogeneous_3d.h
 *
 *  Created on: Mar 7, 2016
 *      \author: jsola
 */

#ifndef SRC_STATE_HOMOGENEOUS_3D_H_
#define SRC_STATE_HOMOGENEOUS_3D_H_

#include "state_block.h"
#include "local_parametrization_homogeneous.h"

namespace wolf {

class StateHomogeneous3d : public StateBlock
{
    public:
        StateHomogeneous3d(bool _fixed = false);
        StateHomogeneous3d(const Eigen::VectorXs _state, bool _fixed = false);
        virtual ~StateHomogeneous3d();
};

inline StateHomogeneous3d::StateHomogeneous3d(const Eigen::VectorXs _state, bool _fixed) :
        StateBlock(_state, _fixed)
{
    assert(_state.size() == 4 && "Homogeneous 3d must be size 4.");
    local_param_ptr_ = new LocalParametrizationHomogeneous;
}

inline StateHomogeneous3d::StateHomogeneous3d(bool _fixed) :
        StateBlock(4, _fixed)
{
    local_param_ptr_ = new LocalParametrizationHomogeneous;
}

inline StateHomogeneous3d::~StateHomogeneous3d()
{
    delete local_param_ptr_;
}

} // namespace wolf

#endif /* SRC_STATE_HOMOGENEOUS_3D_H_ */
