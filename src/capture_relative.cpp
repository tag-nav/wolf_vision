#include "capture_relative.h"

CaptureRelative::CaptureRelative(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr) :
    CaptureBase(_ts, _sensor_ptr)
{
    //
}

CaptureRelative::CaptureRelative(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase(_ts, _sensor_ptr, _data)
{
	//
}

CaptureRelative::CaptureRelative(const TimeStamp& _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase(_ts, _sensor_ptr, _data, _data_covariance)
{
	//
}
CaptureRelative::~CaptureRelative()
{
	//
}
