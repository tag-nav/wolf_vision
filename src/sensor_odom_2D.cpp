#include "sensor_odom_2D.h"

SensorOdom2D::SensorOdom2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const WolfScalar& _disp_noise_factor, const WolfScalar&  _rot_noise_factor) :
        SensorBase(SEN_ODOM_2D, _p_ptr, _o_ptr, nullptr, 2), k_disp_to_disp_(_disp_noise_factor), k_rot_to_rot_(_rot_noise_factor)
{
}

SensorOdom2D::~SensorOdom2D()
{
    //
}

WolfScalar SensorOdom2D::getDispVarToDispNoiseFactor() const
{
    return k_disp_to_disp_;
}

WolfScalar SensorOdom2D::getRotVarToRotNoiseFactor() const
{
    return k_rot_to_rot_;
}
