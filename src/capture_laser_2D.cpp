#include "capture_laser_2D.h"

CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, const SensorLaser2DPtr & _sensor_ptr, const Eigen::VectorXs& _ranges):
    CaptureBase(_ts, _sensor_ptr, _ranges)
{
    // 
}

CaptureLaser2D::~CaptureLaser2D()
{
    //
}

void CaptureLaser2D::processCapture()
{
    extractCorners();
}

void CaptureLaser2D::extractCorners()
{
    
}
