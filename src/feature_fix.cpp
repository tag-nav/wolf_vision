#include "feature_fix.h"


namespace wolf {

FeatureFix::FeatureFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(FEAT_FIX, _measurement, _meas_covariance)
{
	//
}

FeatureFix::~FeatureFix()
{
    //
}

} // namespace wolf
