/**
 * \file state_angle.h
 *
 *  Created on: Apr 4, 2017
 *      \author: jsola
 */

#ifndef STATE_ANGLE_H_
#define STATE_ANGLE_H_

#include "state_block.h"
#include "local_parametrization_angle.h"

namespace wolf
{

class StateAngle : public StateBlock
{
    public:
        StateAngle(Scalar _angle, bool _fixed = false);

        virtual ~StateAngle();
};

inline StateAngle::StateAngle(Scalar _angle, bool _fixed) :
        StateBlock(1, _fixed, std::make_shared<LocalParametrizationAngle>())
{
    state_(0) = _angle;
}

inline StateAngle::~StateAngle()
{
    //
}

} /* namespace wolf */

#endif /* STATE_ANGLE_H_ */
