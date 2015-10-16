#include "feature_fix.h"

FeatureFix::FeatureFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(_measurement, _meas_covariance)
{
	//
}

FeatureFix::~FeatureFix()
{
    //
}
