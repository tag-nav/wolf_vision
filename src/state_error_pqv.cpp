/*
 * state_error_pqv.cpp
 *
 *  Created on: Jun 2, 2014
 *      \author: jsola
 */

#include "state_error_pqv.h"
#include "quaternion_tools.h"

using namespace Eigen;

StateErrorPQV::StateErrorPQV(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::Vector3s& _p_n,
                             Eigen::Quaternions& _q_n, Eigen::Vector3s& _v_n) :
        StateErrorPose(_storage, _idx, SIZE_NOMINAL_, SIZE_ERROR_, _p_n, _q_n), //  constructor
        v_error_(_storage.data() + _idx + 6, 3), // velocity
        v_nominal_(&xn()(7), 3), //,
        v_composed_(&xc()(7), 3) //
{
    v_nominal_ = _v_n;
    compose();
}

StateErrorPQV::StateErrorPQV(Eigen::VectorXs& _storage, unsigned int _idx, Eigen::VectorXs& _xn) :
        StateErrorPose(_storage, _idx, _xn, SIZE_ERROR_), //  constructor
        v_error_(_storage.data() + _idx + 6, 3), // w b
        v_nominal_(&xn()(7), 3), //,
        v_composed_(&xc()(7), 3) //
{
    compose();
}

StateErrorPQV::StateErrorPQV(Eigen::VectorXs& _storage, unsigned int _idx) :
        StateErrorPose(_storage, _idx, SIZE_NOMINAL_, SIZE_ERROR_), //  constructor
        v_error_(_storage.data() + _idx + 6, 3), // w b
        v_nominal_(&xn()(7), 3), //,
        v_composed_(&xc()(7), 3) //
{
    compose();
}

StateErrorPQV::StateErrorPQV(Eigen::Vector3s& _p_n, Eigen::Quaternions& _q_n, Eigen::Vector3s& _v_n) :
        StateErrorPose(SIZE_NOMINAL_, SIZE_ERROR_), //
        v_error_(&state_estimated_map_(6), 3), //
        v_nominal_(&xn()(7), 3), //,
        v_composed_(&xc()(7), 3) //
{
    qn() = _q_n;
    compose();
}

StateErrorPQV::StateErrorPQV(Eigen::VectorXs& _xn) :
        StateErrorPose(_xn, SIZE_ERROR_), //
        v_error_(&state_estimated_map_(6), 3), // w b
        v_nominal_(&xn()(7), 3), //,
        v_composed_(&xc()(7), 3) //
{
    compose();
}

StateErrorPQV::StateErrorPQV() :
        StateErrorPose(SIZE_NOMINAL_, SIZE_ERROR_), //
        v_error_(&state_estimated_map_(6), 3), //,
        v_nominal_(&xn()(7), 3), //,
        v_composed_(&xc()(7), 3) //
{
    v_nominal_ = Eigen::Vector3s::Zero();
    compose();
}

void StateErrorPQV::compose()
{
    // additive compose of position
    pc() = pn() + pe();
    // quaternion product compose of quaternion
    qc() = qn() * Wolf::quaternionFromVector(qe());
    // additive compose of velocity
    v_composed_ = v_nominal_ + v_error_;
}
