#include "feature_gps_fix.h"

namespace wolf {

FeatureGPSFix::FeatureGPSFix(unsigned int _dim_measurement) :
    FeatureBase("GPS FIX", _dim_measurement)
{
    //
}

FeatureGPSFix::FeatureGPSFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase("GPS FIX", _measurement, _meas_covariance)
{
    //
}

FeatureGPSFix::~FeatureGPSFix()
{
    //
}

} // namespace wolf
