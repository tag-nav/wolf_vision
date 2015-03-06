#include "feature_gps_fix.h"

FeatureGPSFix::FeatureGPSFix(unsigned int _dim_measurement) :
    FeatureBase(_dim_measurement)
{
    //
}

FeatureGPSFix::FeatureGPSFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(_measurement, _meas_covariance)
{
	//
}

FeatureGPSFix::~FeatureGPSFix()
{
    //
}

void FeatureGPSFix::findConstraints()
{
	//ConstraintBase* gps_constraint(new ConstraintGPS2D(this, getCapturePtr()->getFramePtr()->getPPtr()));
	addConstraint(new ConstraintGPS2D(this, getCapturePtr()->getFramePtr()->getPPtr()));
}
