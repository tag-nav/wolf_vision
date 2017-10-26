#include "capture_wheel_joint_position.h"
#include "../sensors/sensor_diff_drive.h"
#include "../rotations.h"

namespace wolf {

CaptureWheelJointPosition::CaptureWheelJointPosition(const TimeStamp& _ts,
                                                     const SensorBasePtr& _sensor_ptr,
                                                     const Eigen::VectorXs& _positions,
                                                     FrameBasePtr _origin_frame_ptr) :
  CaptureMotion(_ts, _sensor_ptr, _positions, Eigen::Matrix2s::Zero(), 3, 3,
                _origin_frame_ptr/*, nullptr, nullptr, std::make_shared<StateBlock>(3, false)*/)
{
//  setType("WHEEL JOINT POSITION");

  const IntrinsicsDiffDrive intrinsics =
      *(std::static_pointer_cast<SensorDiffDrive>(getSensorPtr())->getIntrinsics());

  setDataCovariance(computeWheelJointPositionCov(getPositions(),
                                                 intrinsics.left_resolution_,
                                                 intrinsics.right_resolution_,
                                                 intrinsics.left_gain_,
                                                 intrinsics.right_gain_));
}

CaptureWheelJointPosition::CaptureWheelJointPosition(const TimeStamp& _ts,
                                                     const SensorBasePtr& _sensor_ptr,
                                                     const Eigen::VectorXs& _positions,
                                                     const Eigen::MatrixXs& _positions_cov,
                                                     FrameBasePtr _origin_frame_ptr,
                                                     StateBlockPtr _p_ptr,
                                                     StateBlockPtr _o_ptr,
                                                     StateBlockPtr _intr_ptr) :
  CaptureMotion(_ts, _sensor_ptr, _positions, _positions_cov, 3, 3,
                _origin_frame_ptr, _p_ptr, _o_ptr, _intr_ptr)
{
//  setType("WHEEL JOINT POSITION");
}

Eigen::VectorXs CaptureWheelJointPosition::correctDelta(const VectorXs& _delta,
                                                        const VectorXs& _delta_error)
{
  Vector3s delta = _delta + _delta_error;
  delta(2) = pi2pi(delta(2));
  return delta;
}

const Eigen::VectorXs& CaptureWheelJointPosition::getPositions() const
{
  return getData();
}

const Eigen::MatrixXs& CaptureWheelJointPosition::getPositionsCov() const
{
  return getDataCovariance();
}

} // namespace wolf
