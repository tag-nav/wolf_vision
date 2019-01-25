#include "capture_laser_2D.h"

namespace wolf {

CaptureLaser2D::CaptureLaser2D(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const std::vector<float>& _ranges) :
        CaptureBase("LASER 2D", _ts, _sensor_ptr), laser_ptr_(std::static_pointer_cast<SensorLaser2D>(getSensorPtr())), scan_(_ranges)
{
    //
}

} // namespace wolf
