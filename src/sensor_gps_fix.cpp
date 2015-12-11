#include "sensor_gps_fix.h"

SensorGPSFix::SensorGPSFix(StateBlock* _p_ptr, StateBlock* _o_ptr, const double& _noise) :
        SensorBase(GPS_FIX, _p_ptr, _o_ptr, nullptr, Eigen::VectorXs::Constant(1,_noise))
{
    //
}

SensorGPSFix::~SensorGPSFix()
{
    //
}

WolfScalar SensorGPSFix::getNoise() const
{
    return noise_std_(0);
}
