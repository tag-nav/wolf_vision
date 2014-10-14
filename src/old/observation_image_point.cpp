/*
 * observation_image_point.cpp
 *
 *  Created on: Jun 4, 2014
 *      \author: jsola
 */

#include "observation_image_point.h"

ObservationImagePoint::ObservationImagePoint(SensorBase & _sensor) :
        ObservationBase(SIZE_),//
        intrinsic_(_sensor.intrinsic_.data(), 4)//
{
    //
}

ObservationImagePoint::~ObservationImagePoint()
{
    //
}

VectorXs& ObservationImagePoint::computeExpectation(StateRootBase& _state)
{
//    expectation_.mean() = computeExpectation( _state, _sensor_pose, intrinsic_, _point );
    return expectation_.mean();
}

VectorXs& ObservationImagePoint::computeInnovation(StateRootBase& _state)
{
//    innovation_.mean() = measurement_.mean() - computeExpectation(_state, _sensor_pose, intrinsic_, _point);
    return innovation_.mean();
}

scalar_t ObservationImagePoint::computeCost(StateRootBase& _state)
{
    scalar_t cost = computeInnovation(_state).norm();
    return cost;
}

Vector2s ObservationImagePoint::pinHoleProject(StatePose& _pose, Vector4s _k, Vector3s _point)
{
    Vector3s point_c = _pose.q() * (_point - _pose.p());
    Vector2s pix;
    pix(0) = _k(0) + _k(2) * point_c(0) / point_c(2);
    pix(1) = _k(1) + _k(3) * point_c(1) / point_c(2);
    return pix;
}

Vector2s ObservationImagePoint::computeExpectation(StatePose& _state, StatePose& _sensor_pose, Vector4s& _intrinsic,
                                                   Vector3s& _point)
{
    return pinHoleProject(_state, _intrinsic, _point);
}

