
#include "sensor_data_absolute_pose.h"

SensorDataAbsolutePose::SensorDataAbsolutePose(shared_ptr<RawAbsolutePose> & _raw_ptr) :
    SensorDataBase(BOTTOM, 7, _raw_ptr)
{
    //
}

SensorDataAbsolutePose::~SensorDataAbsolutePose()
{
    //
}

void SensorDataAbsolutePose::computeExpectation()
{
    //
}
