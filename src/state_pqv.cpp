/*
 * State_pqv.cpp
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

#include "state_pqv.h"

using namespace Eigen;

StatePQV::StatePQV() :
        StatePose(SIZE_), //
        v_(&state_estimated_map_(7), 3) //
{
}

StatePQV::StatePQV(VectorXs& _x) :
        StatePose(_x), //
        v_(&state_estimated_map_(7), 3) //
{
}

StatePQV::StatePQV(Vector3s& _p, Quaternions& _q, Vector3s& _v) :
        StatePose(SIZE_, _p, _q), //
        v_(&state_estimated_map_(7), 3) //
{
    v(_v);
}

StatePQV::StatePQV(VectorXs& _storage, unsigned int _idx) :
        StatePose(_storage, _idx, SIZE_), //  constructor
        v_(_storage.data() + _idx + 7, 3) // velocity
{
    // nothing to do
}

StatePQV::StatePQV(VectorXs& _storage, unsigned int _idx, VectorXs& _x) :
        StatePose(_storage, _idx, _x), //  constructor
        v_(_storage.data() + _idx + 7, 3) // velocity
{
}

StatePQV::StatePQV(VectorXs& _storage, unsigned int _idx, Vector3s& _p, Quaternions& _q, Vector3s& _v) :
        StatePose(_storage, _idx, SIZE_, _p, _q), //  constructor
        v_(_storage.data() + _idx + 7, 3) // velocity
{
    v(_v);
}

StatePQV::~StatePQV()
{
    // nothing to do
}
