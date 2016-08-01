#include "capture_gps_fix.h"


namespace wolf {

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase("GPS FIX", _ts, _sensor_ptr),
	data_(_data)
{
    //
}

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase("GPS FIX", _ts, _sensor_ptr),
	data_(_data),
	data_covariance_(_data_covariance)
{
    //
}

CaptureGPSFix::~CaptureGPSFix()
{
	//std::cout << "Destroying GPS fix capture...\n";
}

void CaptureGPSFix::process()
{
	// EXTRACT AND ADD FEATURES
    addFeature(new FeatureGPSFix(data_,data_covariance_));

    // ADD CONSTRAINT
	getFeatureListPtr()->front()->addConstraint(new ConstraintGPS2D(getFeatureListPtr()->front(), getFramePtr()));
}

Eigen::VectorXs CaptureGPSFix::computeFramePose(const TimeStamp& _now) const
{
	return data_;
}

//void CaptureGPSFix::printSelf(unsigned int _ntabs, std::ostream & _ost) const
//{
//	return data_;
//}

} //namespace wolf
