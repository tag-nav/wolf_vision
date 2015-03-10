#include "sensor_gps_fix.h"

SensorGPSFix::SensorGPSFix(const Eigen::Vector6s & _sp, const double& _noise) :
        SensorBase(GPS_FIX, _sp, Eigen::VectorXs::Constant(1,_noise))
{
    //
}

SensorGPSFix::~SensorGPSFix()
{
    //
}

WolfScalar SensorGPSFix::getNoise() const
{
    return params_(0);
}
