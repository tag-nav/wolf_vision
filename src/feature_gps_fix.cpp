#include "feature_gps_fix.h"

FeatureGPSFix::FeatureGPSFix(const CaptureBasePtr& _capt_ptr, unsigned int _dim_measurement) :
    FeatureBase(_capt_ptr, _dim_measurement)
{
    //
}

//FeatureGPSFix::FeatureGPSFix(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement) :
//	FeatureBase(_capt_ptr, _measurement)
//{
//	//
//}

FeatureGPSFix::FeatureGPSFix(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	FeatureBase(_capt_ptr, _measurement, _meas_covariance)
{
	//
}

FeatureGPSFix::~FeatureGPSFix()
{
    //
}

void FeatureGPSFix::findCorrespondences()
{
	CorrespondenceBaseShPtr gps_correspondence(new CorrespondenceGPS2D(this, getCapturePtr()->getFramePtr()->getPPtr()));
	addCorrespondence(gps_correspondence);
}
