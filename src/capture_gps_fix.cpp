#include "capture_gps_fix.h"

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr) :
    CaptureBase(_ts, _sensor_ptr)
{
    //
}

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase(_ts, _sensor_ptr, _data)
{
	//
}

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase(_ts, _sensor_ptr, _data, _data_covariance)
{
	//
}

CaptureGPSFix::~CaptureGPSFix()
{
	//std::cout << "Destroying GPS fix capture...\n";
}

void CaptureGPSFix::processCapture()
{
	// EXTRACT AND ADD FEATURES
    addFeature(new FeatureGPSFix(data_,data_covariance_));

    // ADD CONSTRAINT
	getFeatureListPtr()->front()->addConstraint(new ConstraintGPS2D(getFeatureListPtr()->front(), getFramePtr()->getPPtr()));
}

Eigen::VectorXs CaptureGPSFix::computePrior(const TimeStamp& _now) const
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



