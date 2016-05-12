#include "feature_gps_fix.h"

namespace wolf {

FeatureGPSFix::FeatureGPSFix(unsigned int _dim_measurement) :
    FeatureBase(FEATURE_GPS_FIX, _dim_measurement)
{
    setType("GPS FIX");
}

FeatureGPSFix::FeatureGPSFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(FEATURE_GPS_FIX, _measurement, _meas_covariance)
{
    setType("GPS FIX");
}

FeatureGPSFix::~FeatureGPSFix()
{
    //
}

} // namespace wolf
