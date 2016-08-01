#include "feature_gps_fix.h"

namespace wolf {

FeatureGPSFix::FeatureGPSFix(unsigned int _dim_measurement) :
    FeatureBase(FEATURE_GPS_FIX, "GPS FIX", _dim_measurement)
{
    //
}

FeatureGPSFix::FeatureGPSFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(FEATURE_GPS_FIX, "GPS FIX", _measurement, _meas_covariance)
{
    //
}

FeatureGPSFix::~FeatureGPSFix()
{
    //
}

} // namespace wolf
