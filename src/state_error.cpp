/*
 * state_error.cpp
 *
 *  Created on: 29/05/2014
 *      \author: jsola
 */

#include "state_error.h"

using namespace Eigen;

StateError::StateError(const unsigned int _size_nominal, const unsigned int _size_error) :
        StateRootBase(_size_error),//
        state_nominal_(_size_nominal),//
        state_composed_(_size_nominal)//
{
    clearError();
}

StateError::StateError(const VectorXs& _x_nominal, const unsigned int _size_error) :
        StateRootBase(_size_error),//
        state_nominal_(_x_nominal),//
        state_composed_(_x_nominal.size())//
{
    clearError();
}

StateError::StateError(const VectorXs& _x_nominal, const VectorXs& _x_error) :
        StateRootBase(_x_error),//
        state_nominal_(_x_nominal),//
        state_composed_(_x_nominal.size())//
{
}

StateError::StateError(VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size_nominal,
                       const unsigned int _size_error) :
        StateRootBase(_st_remote, _idx, _size_error),//
        state_nominal_(_size_nominal),//
        state_composed_(_size_nominal)//
{
    clearError();
}

StateError::StateError(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x_nominal, const unsigned int _size_error) :
        StateRootBase(_st_remote, _idx, _size_error),//
        state_nominal_(_x_nominal),//
        state_composed_(_x_nominal.size())//
{
    clearError();
}

StateError::StateError(VectorXs& _st_remote, const unsigned int _idx, const VectorXs& _x_nominal,
                       const VectorXs& _x_error) :
        StateRootBase(_st_remote, _idx, _x_error),//
        state_nominal_(_x_nominal),//
        state_composed_(_x_nominal.size())//
{
}

StateError::~StateError()
{
    //
}
