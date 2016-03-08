/*
 * \file state_homogeneous_3d.h
 *
 *  Created on: Mar 7, 2016
 *      \author: jsola
 */

#ifndef SRC_STATE_HOMOGENEOUS_3D_H_
#define SRC_STATE_HOMOGENEOUS_3D_H_

#include "state_block.h"

class StateHomogeneous3d : public StateBlock
{
    public:
        StateHomogeneous3d(bool _fixed = false);
        StateHomogeneous3d(const Eigen::VectorXs _state, bool _fixed = false);
        virtual ~StateHomogeneous3d();
};

#endif /* SRC_STATE_HOMOGENEOUS_3D_H_ */
