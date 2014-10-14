
#include "sensor_data_base.h"

SensorDataBase::SensorDataBase(const NodeLocation _loc, const unsigned int _dim) : 
    NodeConstrainer(_loc, _dim)
{
    //
}

SensorDataBase::~SensorDataBase()
{
    //
}

TimeStamp & SensorDataBase::timeStamp()
{
    return time_stamp_;
}


