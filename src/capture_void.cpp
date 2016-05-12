#include "capture_void.h"

namespace wolf {

CaptureVoid::CaptureVoid(const TimeStamp& _ts, SensorBase* _sensor_ptr) :
    CaptureBase(_ts, _sensor_ptr)
{
    setType("VOID");
}

CaptureVoid::~CaptureVoid()
{
	//std::cout << "deleting CaptureVoid " << nodeId() << std::endl;
}

Eigen::VectorXs CaptureVoid::computeFramePose(const TimeStamp& _now) const
{
    return Eigen::VectorXs::Zero(3);
}

} // namespace wolf
