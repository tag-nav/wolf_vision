#include "sensor_odom_2D.h"

SensorOdom2D::SensorOdom2D(StatePoint3D* _p_ptr, StateOrientation* _o_ptr, const WolfScalar& _disp_noise_factor, const WolfScalar&  _rot_noise_factor) :
        SensorBase(ODOM_2D, _p_ptr, _o_ptr, 2)
{
    params_ << _disp_noise_factor, _rot_noise_factor;
}

SensorOdom2D::~SensorOdom2D()
{
    //
}

WolfScalar SensorOdom2D::getDisplacementNoiseFactor() const
{
    return params_(0);
}

WolfScalar SensorOdom2D::getRotationNoiseFactor() const
{
    return params_(1);
}
