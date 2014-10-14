/*
 * state_imu.cpp
 *
 *  Created on: May 22, 2014
 *      \author: jsola
 */

#include "state_imu.h"

using namespace Eigen;

StateIMU::StateIMU() :
        StateBase(SIZE_), //
        a_bias_(state_estimated_map_.data(), 3), // a bias
        w_bias_(state_estimated_map_.data() + 3, 3) // w bias
{
}

StateIMU::StateIMU(VectorXs& _x) :
        StateBase(_x), //
        a_bias_(state_estimated_map_.data(), 3), // a bias
        w_bias_(state_estimated_map_.data() + 3, 3) // w bias
{
}

StateIMU::StateIMU(Vector3s& _abias, Vector3s& _wbias) :
        StateBase(SIZE_), //
        a_bias_(state_estimated_map_.data(), 3), // a bias
        w_bias_(state_estimated_map_.data() + 3, 3) // w bias
{
    ab(_abias);
    wb(_wbias);
}

StateIMU::StateIMU(VectorXs& _storage, unsigned int _idx) :
        StateBase(_storage, _idx, SIZE_),    //  constructor
        a_bias_(_storage.data() + _idx, 3),                 // a bias
        w_bias_(_storage.data() + _idx + 3, 3)              // w bias
{
    // nothing to do
}

StateIMU::StateIMU(VectorXs& _storage, unsigned int _idx, VectorXs& _x) :
        StateBase(_storage, _idx, _x),                   //  constructor
        a_bias_(_storage.data() + _idx, 3),                 // a bias
        w_bias_(_storage.data() + _idx + 3, 3)              // w bias
{
    // nothing to do
}

StateIMU::StateIMU(VectorXs& _storage, unsigned int _idx, Vector3s& _abias, Vector3s& _wbias) :
        StateBase(_storage, _idx, SIZE_),    //  constructor
        a_bias_(_storage.data() + _idx, 3),                 // position
        w_bias_(_storage.data() + _idx + 3, 3)              // velocity
{
    ab(_abias);
    wb(_wbias);
}

StateIMU::~StateIMU()
{
    // nothing to do
}
