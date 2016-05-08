#include "capture_laser_2D.h"

namespace wolf {

// unsigned int CaptureLaser2D::segment_window_size = 8;//window size to extract segments
// double CaptureLaser2D::theta_min = 0.4; //minimum theta between consecutive segments to detect corner. PI/8=0.39
// double CaptureLaser2D::theta_max_parallel = 0.1; //maximum theta between consecutive segments to fuse them in a single line.
// double CaptureLaser2D::k_sigmas = 3.;//How many std_dev are tolerated to count that a point is supporting a line
// unsigned int CaptureLaser2D::max_beam_distance = 5;//max number of beams of distance between lines to consider corner or concatenation
// double CaptureLaser2D::max_distance = 0.5;//max distance between line ends to consider corner or concatenation

CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, 
                               SensorBase* _sensor_ptr, // TODO change this pointer type to SensorLaser2D*
                               const std::vector<float>& _ranges) 
                               :
                               CaptureBase(_ts, _sensor_ptr), 
                               ranges_(_ranges)
{
    setType("LASER 2D");
    laser_ptr_ = (SensorLaser2D*) sensor_ptr_;
}


CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, 
                               SensorBase* _sensor_ptr,
                               const std::vector<float>& _ranges, 
                               const std::vector<float>& _intensities) 
                               :
                               CaptureBase(_ts, _sensor_ptr),
                               ranges_(_ranges), 
                               intensities_(_intensities)
{
    setType("LASER 2D");
    laser_ptr_ = (SensorLaser2D*) sensor_ptr_;
}


CaptureLaser2D::~CaptureLaser2D()
{
    //
}

// TODO: What the heck?
Eigen::VectorXs CaptureLaser2D::computeFramePose(const TimeStamp& _now) const
{
    return Eigen::Vector3s(1, 2, 3);
}

} // namespace wolf
