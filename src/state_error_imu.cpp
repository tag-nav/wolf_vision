/*
 * state_error_imu.cpp
 *
 *  Created on: Jun 2, 2014
 *      \author: jsola
 */

#include <state_error_imu.h>

using namespace Eigen;

StateErrorIMU::StateErrorIMU() :
        StateError(SIZE_NOMINAL_, SIZE_ERROR_),//
        ab_error_(&state_estimated_map_(0), 3),//
        wb_error_(&state_estimated_map_(0) + 3, 3),//
        ab_nominal_(&state_nominal_(0), 3),//
        wb_nominal_(&state_nominal_(3), 3),//
        ab_composed_(&state_composed_(0), 3),//
        wb_composed_(&state_composed_(3), 3)//
{
    ab_nominal_ = Vector3s::Zero();
    wb_nominal_ = Vector3s::Zero();
    compose();
}

StateErrorIMU::StateErrorIMU(Eigen::VectorXs& _xn) :
        StateError(_xn, SIZE_ERROR_),//
        ab_error_(&state_estimated_map_(0), 3), // a bias
        wb_error_(&state_estimated_map_(0) + 3, 3), // w bias
        ab_nominal_(&state_nominal_(0), 3),//
        wb_nominal_(&state_nominal_(3), 3),//
        ab_composed_(&state_composed_(0), 3),//
        wb_composed_(&state_composed_(3), 3)//
{
    compose();
}

StateErrorIMU::StateErrorIMU(Vector3s& _abias_n, Vector3s& _wbias_n) :
        StateError(SIZE_NOMINAL_, SIZE_ERROR_),//
        ab_error_(&state_estimated_map_(0), 3), // a bias
        wb_error_(&state_estimated_map_(0) + 3, 3), // w bias
        ab_nominal_(&state_nominal_(0), 3),//
        wb_nominal_(&state_nominal_(3), 3),//
        ab_composed_(&state_composed_(0), 3),//
        wb_composed_(&state_composed_(3), 3)//
{
    ab_nominal_ = _abias_n;
    wb_nominal_ = _wbias_n;
    compose();
}

StateErrorIMU::StateErrorIMU(VectorXs& _storage, unsigned int _idx) :
        StateError(_storage, _idx, SIZE_NOMINAL_, SIZE_ERROR_),    //  constructor
        ab_error_(_storage.data() + _idx, 3),                 // a bias
        wb_error_(_storage.data() + _idx + 3, 3),              // w bias
        ab_nominal_(&state_nominal_(0), 3),//
        wb_nominal_(&state_nominal_(3), 3),//
        ab_composed_(&state_composed_(0), 3),//
        wb_composed_(&state_composed_(3), 3)//
{
    state_nominal_ = VectorXs::Zero(state_nominal_.size());
    compose();
}

StateErrorIMU::StateErrorIMU(VectorXs& _storage, unsigned int _idx, VectorXs& _xn) :
        StateError(_storage, _idx, _xn, SIZE_ERROR_),                   //  constructor
        ab_error_(_storage.data() + _idx, 3),                 // a bias
        wb_error_(_storage.data() + _idx + 3, 3),              // w bias
        ab_nominal_(&state_nominal_(0), 3),//
        wb_nominal_(&state_nominal_(3), 3),//
        ab_composed_(&state_composed_(0), 3),//
        wb_composed_(&state_composed_(3), 3)//
{
    compose();
}

StateErrorIMU::StateErrorIMU(VectorXs& _storage, unsigned int _idx, Vector3s& _abias_n, Vector3s& _wbias_n) :
        StateError(_storage, _idx, SIZE_NOMINAL_, SIZE_ERROR_),    //  constructor
        ab_error_(_storage.data() + _idx, 3),                 // position
        wb_error_(_storage.data() + _idx + 3, 3),              // velocity
        ab_nominal_(&state_nominal_(0), 3),//
        wb_nominal_(&state_nominal_(3), 3),//
        ab_composed_(&state_composed_(0), 3),//
        wb_composed_(&state_composed_(3), 3)//
{
    ab_nominal_ = _abias_n;
    wb_nominal_ = _wbias_n;
    compose();
}

StateErrorIMU::~StateErrorIMU()
{
}
