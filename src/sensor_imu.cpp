#include "sensor_imu.h"

SensorOdom2D::SensorOdom2D(StateBlock* _p_ptr, StateBlock* _o_ptr) :
        SensorBase(IMU, _p_ptr, _o_ptr, nullptr, 2)
{
}

SensorOdom2D::~SensorOdom2D()
{
    //
}

