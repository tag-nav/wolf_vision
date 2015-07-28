#include "sensor_twist_2D.h"

SensorTwist2D::SensorTwist2D(StatePoint3D* _p_ptr, StateOrientation* _o_ptr, const WolfScalar& _lineal_noise, const WolfScalar&  _angular_noise) :
        SensorBase(TWIST_2D, _p_ptr, _o_ptr, 2)
{
    params_ << _lineal_noise, _angular_noise;
}

SensorTwist2D::~SensorTwist2D()
{
    //
}

WolfScalar SensorTwist2D::getLinealNoise() const
{
    return params_(0);
}

WolfScalar SensorTwist2D::getAngularNoise() const
{
    return params_(1);
}
