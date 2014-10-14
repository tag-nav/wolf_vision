/*
 * sensor_data_base.cpp
 *
 *  Created on: Jun 10, 2014
 *      \author: jsola
 */

#include "sensor_data_base.h"

SensorDataBase::SensorDataBase(const weak_ptr<SensorBase> & _sensor_ptr, const shared_ptr<RawBase> & _raw_ptr) :
        sensor_(_sensor_ptr), //
        raw_(_raw_ptr)//, //
{
    //
}

SensorDataBase::~SensorDataBase()
{
    //
}

void SensorDataBase::detect()
{
    //
}

bool SensorDataBase::isMatchable(const shared_ptr<SensorDataBase> & _sensor_ptr) const
{
    return true;
}


void SensorDataBase::match(const shared_ptr<SensorDataBase>& _sensor_ptr)
{
    //
}
