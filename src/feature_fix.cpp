#include "feature_fix.h"


namespace wolf {

FeatureFix::FeatureFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(FEATURE_FIX, _measurement, _meas_covariance)
{
    setType("FIX");
}

FeatureFix::~FeatureFix()
{
    //
}

} // namespace wolf
