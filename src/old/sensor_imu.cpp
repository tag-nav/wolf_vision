/*
 * sensor_imu.cpp
 *
 *  Created on: 15/06/2014
 *      \author: jsola
 */

#include "sensor_imu.h"

SensorIMU::SensorIMU(StatePose& _sp) :
        SensorBase(_sp, SIZE_PARAMS_)
{
    intrinsic_ = VectorXs::Constant(6,1);
}

SensorIMU::SensorIMU(StatePose& _sp, VectorXs& _k) :
        SensorBase(_sp, SIZE_PARAMS_) //
{
    assert(_k.size() == SIZE_PARAMS_);
    intrinsic_ = _k;
}

SensorIMU::~SensorIMU()
{
    //
}

