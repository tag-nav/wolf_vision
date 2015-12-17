#include "capture_void.h"

CaptureVoid::CaptureVoid(const TimeStamp& _ts, SensorBase* _sensor_ptr) :
    CaptureBase(_ts, _sensor_ptr)
{
    //
}

CaptureVoid::~CaptureVoid()
{
	//std::cout << "deleting CaptureVoid " << nodeId() << std::endl;
}

Eigen::VectorXs CaptureVoid::computeFramePose(const TimeStamp& _now) const
{
    return Eigen::VectorXs::Zero(3);
}
