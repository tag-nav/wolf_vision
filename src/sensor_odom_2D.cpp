#include "sensor_odom_2D.h"

namespace wolf {

SensorOdom2D::SensorOdom2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const Scalar& _disp_noise_factor, const Scalar&  _rot_noise_factor) :
        SensorBase(SEN_ODOM_2D, _p_ptr, _o_ptr, nullptr, 2), k_disp_to_disp_(_disp_noise_factor), k_rot_to_rot_(_rot_noise_factor)
{
    setType("ODOM 2D");
}

SensorOdom2D::~SensorOdom2D()
{
    //
}

Scalar SensorOdom2D::getDispVarToDispNoiseFactor() const
{
    return k_disp_to_disp_;
}

Scalar SensorOdom2D::getRotVarToRotNoiseFactor() const
{
    return k_rot_to_rot_;
}

} // namespace wolf
