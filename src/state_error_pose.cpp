/**
 * \file state_error_pose.cpp
 *
 *  Created on: 13/08/2014
 *     \author: jsola
 */

#include "state_error_pose.h"
#include "quaternion_tools.h"

using namespace Eigen;

StateErrorPose::StateErrorPose(unsigned int _size_nom, unsigned int _size_err) :
        StateError(_size_nom, _size_err),//
        p_error_(&state_estimated_map_(0), 3),//
        q_error_(&state_estimated_map_(3), 3),//
        p_nominal_(&xn()(0), 3),//
        q_nominal_(&xn()(3)),//
        p_composed_(&xc()(0), 3),//
        q_composed_(&xc()(3))//
{
    p_nominal_ = Vector3s::Zero();
    q_nominal_ = Quaternions::Identity();
    compose();
}

StateErrorPose::StateErrorPose(VectorXs& _xn, unsigned int _size_err) :
        StateError(_xn, _size_err),
        p_error_(&stateEstimatedMap()(0), 3), //
        q_error_(&stateEstimatedMap()(3), 3),//
        p_nominal_(&xn()(0), 3),//
        q_nominal_(&xn()(3)),//
        p_composed_(&xc()(0), 3),//
        q_composed_(&xc()(3))//
{
    compose();
}

StateErrorPose::StateErrorPose(unsigned int _size_nom, unsigned int _size_err, Vector3s& _p_n, Quaternions& _q_n) :
        StateError(_size_nom, _size_nom),
        p_error_(&state_estimated_map_(0), 3), //
        q_error_(&state_estimated_map_(3), 3),//
        p_nominal_(&xn()(0), 3),//
        q_nominal_(&xn()(3)),//
        p_composed_(&xc()(0), 3),//
        q_composed_(&xc()(3))//
{
    p_nominal_ = _p_n;
    q_nominal_ = _q_n;
    compose();
}

StateErrorPose::StateErrorPose(VectorXs& _storage, unsigned int _idx, unsigned int _size_nom, unsigned int _size_err) :
        StateError(_storage, _idx, _size_nom, _size_err),    //  constructor
        p_error_(_storage.data() + _idx, 3),                 //
        q_error_(_storage.data() + _idx + 3, 3),//
        p_nominal_(&xn()(0), 3),//
        q_nominal_(&xn()(3)),//
        p_composed_(&xc()(0), 3),//
        q_composed_(&xc()(3))//
{
    compose();
}

StateErrorPose::StateErrorPose(VectorXs& _storage, unsigned int _idx, VectorXs& _xn, unsigned int _size_err) :
        StateError(_storage, _idx, _xn, _size_err),                   //  constructor
        p_error_(_storage.data() + _idx, 3),                 // a b
        q_error_(_storage.data() + _idx + 3, 3),//
        p_nominal_(&xn()(0), 3),//
        q_nominal_(&xn()(3)),//
        p_composed_(&xc()(0), 3),//
        q_composed_(&xc()(3))//
{
    compose();
}

StateErrorPose::StateErrorPose(VectorXs& _storage, unsigned int _idx, unsigned int _size_nom, unsigned int _size_err, Vector3s& _p_n, Quaternions& _q_n) :
        StateError(_storage, _idx, _size_nom, _size_err),    //  constructor
        p_error_(_storage.data() + _idx, 3),                 // position
        q_error_(_storage.data() + _idx + 3, 3),//
        p_nominal_(&xn()(0), 3),//
        q_nominal_(&xn()(3)),//
        p_composed_(&xc()(0), 3),//
        q_composed_(&xc()(3))//
{
    p_nominal_ = _p_n;
    q_nominal_ = _q_n;
    compose();
}

StateErrorPose::~StateErrorPose()
{
}

void StateErrorPose::compose()
{
    // additive compose of position and velocity
    p_composed_ = p_nominal_ + p_error_;
    // quaternion product compose of quaternion
    q_composed_ = q_nominal_ * Wolf::quaternionFromVector(q_error_);
}
