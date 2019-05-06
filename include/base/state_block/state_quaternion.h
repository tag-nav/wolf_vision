/*
 * \file StateQuaternion.h
 *
 *  Created on: Mar 7, 2016
 *      author: jsola
 */

#ifndef SRC_STATE_QUATERNION_H_
#define SRC_STATE_QUATERNION_H_

#include "base/state_block/state_block.h"
#include "base/state_block/local_parametrization_quaternion.h"

namespace wolf {

class StateQuaternion : public StateBlock
{
    public:
        StateQuaternion(bool _fixed = false);
        StateQuaternion(const Eigen::VectorXs& _state, bool _fixed = false);
        StateQuaternion(const Eigen::Quaternions& _quaternion, bool _fixed = false);
        virtual ~StateQuaternion();
};

inline StateQuaternion::StateQuaternion(const Eigen::Quaternions& _quaternion, bool _fixed) :
        StateBlock(_quaternion.coeffs(), _fixed, std::make_shared<LocalParametrizationQuaternion<DQ_LOCAL>>())
{
}

inline StateQuaternion::StateQuaternion(const Eigen::VectorXs& _state, bool _fixed) :
        StateBlock(_state, _fixed, std::make_shared<LocalParametrizationQuaternion<DQ_LOCAL>>())
{
    assert(_state.size() == 4 && "The quaternion state vector must be of size 4");
}

inline StateQuaternion::StateQuaternion(bool _fixed) :
        StateBlock(4, _fixed, std::make_shared<LocalParametrizationQuaternion<DQ_LOCAL>>())
{
    state_ = Eigen::Quaternions::Identity().coeffs();
    //
}

inline StateQuaternion::~StateQuaternion()
{
    // The local_param_ptr_ pointer is already removed by the base class
}

} // namespace wolf

#endif /* SRC_STATE_QUATERNION_H_ */
