/*
 * State_pose.cpp
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

#include "state_pose.h"

using namespace Eigen;

StatePose::StatePose() :
        StateBase(7), //
        p_(state_estimated_map_.data(), 3), //
        q_(state_estimated_map_.data() + 3)//
{
//    setToOrigin();
}

StatePose::StatePose(unsigned int _size) :
        StateBase(_size), //
        p_(state_estimated_map_.data(), 3), //
        q_(state_estimated_map_.data() + 3)//
{
//    setToOrigin();
}

StatePose::StatePose(const VectorXs & _x) :
        StateBase(_x), //
        p_(state_estimated_map_.data(), 3), //
        q_(state_estimated_map_.data() + 3)//
{
}

StatePose::StatePose(const StatePose& _pose) : 
        StateBase(7),
        p_(state_estimated_map_.data(), 3), //
        q_(state_estimated_map_.data() + 3)//        
{
    p_ = _pose.p();
    q_ = _pose.q();
}

StatePose::StatePose(unsigned int _size, Vector3s& _p, Quaternions& _q) :
        StateBase(_size), //
        p_(state_estimated_map_.data(), 3), //
        q_(state_estimated_map_.data() + 3)//
{
    p(_p);
    q(_q);
}

StatePose::StatePose(VectorXs& _storage, unsigned int _idx, unsigned int _size) :
        StateBase(_storage, _idx, _size), //  constructor
        p_(_storage.data() + _idx, 3), // position
        q_(_storage.data() + _idx + 3) // orientation quaternion
{
    // nothing to do
}

StatePose::StatePose(VectorXs& _storage, unsigned int _idx, VectorXs& _x) :
        StateBase(_storage, _idx, _x), //  constructor
        p_(_storage.data() + _idx, 3), // position
        q_(_storage.data() + _idx + 3) // orientation quaternion
{
}

StatePose::StatePose(VectorXs& _storage, unsigned int _idx, unsigned int _size, Vector3s& _p, Quaternions& _q) :
        StateBase(_storage, _idx, _size), //  constructor
        p_(_storage.data() + _idx, 3), // position
        q_(_storage.data() + _idx + 3) // orientation quaternion
{
    p(_p);
    q(_q);
}

StatePose::~StatePose()
{
    // nothing to do
}
