#include "capture_velocity.h"

namespace wolf {

CaptureVelocity::CaptureVelocity(const TimeStamp& _ts,
                                 const SensorBasePtr& _sensor_ptr,
                                 const Eigen::VectorXs& _velocity,
                                 Size _delta_size, Size _delta_cov_size,
                                 FrameBasePtr _origin_frame_ptr) :
  CaptureMotion(_ts, _sensor_ptr, _velocity,
                _delta_size, _delta_cov_size, _origin_frame_ptr)
{
  setType("VELOCITY");
}

CaptureVelocity::CaptureVelocity(const TimeStamp& _ts,
                                 const SensorBasePtr& _sensor_ptr,
                                 const Eigen::VectorXs& _velocity,
                                 const Eigen::MatrixXs& _velocity_cov,
                                 Size _delta_size, Size _delta_cov_size,
                                 FrameBasePtr _origin_frame_ptr,
                                 StateBlockPtr _p_ptr,
                                 StateBlockPtr _o_ptr,
                                 StateBlockPtr _intr_ptr) :
  CaptureMotion(_ts, _sensor_ptr, _velocity, _velocity_cov,
                _delta_size, _delta_cov_size, _origin_frame_ptr,
                _p_ptr, _o_ptr, _intr_ptr)
{
  setType("VELOCITY");
}

const Eigen::VectorXs& CaptureVelocity::getVelocity() const
{
  return getData();
}

const Eigen::MatrixXs& CaptureVelocity::getVelocityCov() const
{
  return getDataCovariance();
}

} // namespace wolf
