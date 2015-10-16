#include "capture_fix.h"


CaptureFix::CaptureFix(const TimeStamp& _ts, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase(_ts, nullptr, _data, _data_covariance)
{
	//
}

CaptureFix::~CaptureFix()
{
	//
}

void CaptureFix::processCapture()
{
	// EXTRACT AND ADD FEATURES
    addFeature(new FeatureFix(data_,data_covariance_));

    // ADD CONSTRAINT
	getFeatureListPtr()->front()->addConstraint(new ConstraintFix(getFeatureListPtr()->front(), getFramePtr()->getPPtr(), getFramePtr()->getOPtr()));
}

Eigen::VectorXs CaptureFix::computePrior(const TimeStamp& _now) const
{
	return data_;
}
