#include "capture_laser_2D.h"

namespace wolf {

CaptureLaser2D::CaptureLaser2D(const TimeStamp& _ts, SensorBase* _sensor_ptr, const std::vector<float>& _ranges) :
        CaptureBase(_ts, _sensor_ptr), laser_ptr_((SensorLaser2D*)(sensor_ptr_)), scan_(_ranges)
{
    setType("LASER 2D");
}

CaptureLaser2D::~CaptureLaser2D()
{
    //
}

} // namespace wolf
