#include "capture_gps_fix.h"


namespace wolf {

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase(_ts, _sensor_ptr),
	data_(_data)
{
    setType("GPS FIX");
}

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase(_ts, _sensor_ptr),
	data_(_data),
	data_covariance_(_data_covariance)
{
    setType("GPS FIX");
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
//    NodeLinked::printSelf(_ntabs, _ost);
//    //printTabs(_ntabs);
//    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
//    //printNTabs(_ntabs);
//    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
//}


} //namespace wolf
