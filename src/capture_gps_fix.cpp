#include "capture_gps_fix.h"

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr) :
    CaptureBase(_ts, _sensor_ptr)
{
    //
}

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase(_ts, _sensor_ptr, _data)
{
	//
}

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
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
    FeatureBaseShPtr new_feature = FeatureBaseShPtr(new FeatureGPSFix(CaptureBasePtr(this),data_,data_covariance_));
    addFeature(new_feature);
}

Eigen::VectorXs CaptureGPSFix::computePrior() const
{
	return data_;
}

void CaptureGPSFix::findCorrespondences()
{
	CorrespondenceBaseShPtr gps_correspondence(new CorrespondenceGPS2D(getFeatureListPtr()->front().get(), getFramePtr()->getPPtr()));
	getFeatureListPtr()->front()->addCorrespondence(gps_correspondence);
}

//void CaptureGPSFix::printSelf(unsigned int _ntabs, std::ostream & _ost) const
//{
//    NodeLinked::printSelf(_ntabs, _ost);
//    //printTabs(_ntabs);
//    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
//    //printNTabs(_ntabs);
//    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
//}



