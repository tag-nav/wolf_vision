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
    //std::cout << "FeatureFix extracted " << std::endl;

    // ADD CONSTRAINT
	getFeatureListPtr()->front()->addConstraintFrom(new ConstraintFix(getFeatureListPtr()->front()));
    //std::cout << "ConstraintFix added " << std::endl;
}

Eigen::VectorXs CaptureFix::computePrior(const TimeStamp& _now) const
{
	return data_;
}
