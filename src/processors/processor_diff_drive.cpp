#include "processor_diff_drive.h"

#include "../sensors/sensor_diff_drive.h"

#include "../captures/capture_wheel_joint_position.h"
#include "../captures/capture_velocity.h"

#include "../rotations.h"
#include "../constraint_odom_2D.h"
#include "../features/feature_diff_drive.h"

namespace wolf
{

ProcessorDiffDrive::ProcessorDiffDrive(const ProcessorParamsDiffDrive &params) :
  ProcessorMotion("DIFF DRIVE", 3, 3, 3, 2, 0.15, 3),
  unmeasured_perturbation_cov_(Matrix3s::Identity()*
                               params.unmeasured_perturbation_std_*
                               params.unmeasured_perturbation_std_),
  params_(params)
{
  //
}

void ProcessorDiffDrive::computeCurrentDelta(const Eigen::VectorXs& _data,
                                             const Eigen::MatrixXs& _data_cov,
                                             const Eigen::VectorXs& _calib,
                                             const Scalar _dt,
                                             Eigen::VectorXs& _delta,
                                             Eigen::MatrixXs& _delta_cov,
                                             Eigen::MatrixXs& _jacobian_delta_calib)
{
  assert(_data.size()     == data_size_  && "Wrong _data vector size");
  assert(_data_cov.rows() == data_size_  && "Wrong _data_cov size");
  assert(_data_cov.cols() == data_size_  && "Wrong _data_cov size");
  assert(_calib.size()    == calib_size_ && "Wrong _calib vector size");

  /// Retrieve sensor intrinsics
  const IntrinsicsDiffDrive& intrinsics =
      *(std::static_pointer_cast<SensorDiffDrive>(getSensorPtr())->getIntrinsics());

  VelocityComand<Scalar> vel;
  Eigen::MatrixXs J_vel_data;
  Eigen::MatrixXs J_vel_calib;

  switch (intrinsics.model_) {
  case DiffDriveModel::Two_Factor_Model:
    std::tie(vel, J_vel_data, J_vel_calib) =
        wheelPositionIncrementToVelocity<DiffDriveModel::Two_Factor_Model>(
          _data, _data_cov,
          intrinsics.left_radius_, intrinsics.right_radius_, intrinsics.separation_,
          _calib, _dt);
    break;
  case DiffDriveModel::Three_Factor_Model:
    std::tie(vel, J_vel_data, J_vel_calib) =
        wheelPositionIncrementToVelocity<DiffDriveModel::Three_Factor_Model>(
          _data, _data_cov,
          intrinsics.left_radius_, intrinsics.right_radius_, intrinsics.separation_,
          _calib, _dt);
    break;
  case DiffDriveModel::Five_Factor_Model:
    //      std::tie(vel, J_vel_data, J_vel_calib) =
    //          wheelPositionIncrementToVelocity<DiffDriveModel::Two_Factor_Model>(
    //            data, data_cov,
    //            intrinsics.left_radius_, intrinsics.right_radius_, intrinsics.separation_,
    //            _calib, _dt);
    throw std::runtime_error("DiffDriveModel::Five_Factor_Model not implemented !");
    break;
  default:
    throw std::runtime_error("Unknown DiffDrive model.");
    break;
  }

  /// Integrate delta vel to zero vel thus
  /// Convert delta vel to delta 2d pose
  Eigen::MatrixXs J_delta_vel;
  integrate(vel.comand, vel.comand_cov, _delta, _delta_cov, J_delta_vel);

  _delta_cov += unmeasured_perturbation_cov_;

  _jacobian_delta_calib = J_delta_vel * J_vel_calib;
}

bool ProcessorDiffDrive::voteForKeyFrame()
{
  // Distance criterion
  if (getBuffer().get().back().delta_integr_.head<2>().norm() > params_.dist_traveled_th_)
  {
    //WOLF_PROCESSOR_DEBUG("vote for key-frame on distance criterion.");
      return true;
  }
//  else
//  {
//    WOLF_PROCESSOR_DEBUG(getBuffer().get().back().delta_integr_.head<2>().norm(),
//                         " < ", params_.dist_traveled_th_);
//  }

  if (std::abs(getBuffer().get().back().delta_integr_.tail<1>()(0)) > params_.theta_traveled_th_)
  {
    //WOLF_PROCESSOR_DEBUG("vote for key-frame on rotation criterion.");
    return true;
  }
//  else
//  {
//    WOLF_PROCESSOR_DEBUG(getBuffer().get().back().delta_integr_.tail<1>()(0),
//                         " < ", params_.theta_traveled_th_);
//  }

  return false;
}

void ProcessorDiffDrive::deltaPlusDelta(const Eigen::VectorXs& _delta1,
                                        const Eigen::VectorXs& _delta2,
                                        const Scalar /*_Dt2*/,
                                        Eigen::VectorXs& _delta1_plus_delta2)
{
  assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
  assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
  assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");

  /// Simple 2d frame composition

  _delta1_plus_delta2.head<2>() = _delta1.head<2>() + Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();
  _delta1_plus_delta2(2) = pi2pi(_delta1(2) + _delta2(2));
}

void ProcessorDiffDrive::deltaPlusDelta(const Eigen::VectorXs& _delta1,
                                        const Eigen::VectorXs& _delta2,
                                        const Scalar /*_Dt2*/,
                                        Eigen::VectorXs& _delta1_plus_delta2,
                                        MatrixXs& _jacobian1, MatrixXs& _jacobian2)
{
  using std::sin;
  using std::cos;

  assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
  assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
  assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
  assert(_jacobian1.rows() == delta_cov_size_ && "Wrong _jacobian1 size");
  assert(_jacobian1.cols() == delta_cov_size_ && "Wrong _jacobian1 size");
  assert(_jacobian2.rows() == delta_cov_size_ && "Wrong _jacobian2 size");
  assert(_jacobian2.cols() == delta_cov_size_ && "Wrong _jacobian2 size");

  /// Simple 2d frame composition

  _delta1_plus_delta2.head<2>() = _delta1.head<2>() +
      Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();

  _delta1_plus_delta2(2) = pi2pi(_delta1(2) + _delta2(2));

  // Jac wrt delta_integrated
  _jacobian1 = Eigen::MatrixXs::Identity(delta_cov_size_,delta_cov_size_);
  _jacobian1(0,2) = -sin(_delta1(2))*_delta2(0) - cos(_delta1(2))*_delta2(1);
  _jacobian1(1,2) =  cos(_delta1(2))*_delta2(0) - sin(_delta1(2))*_delta2(1);

  // jac wrt delta
  _jacobian2 = Eigen::MatrixXs::Identity(delta_cov_size_, delta_cov_size_);
  _jacobian2.topLeftCorner<2,2>() = Eigen::Rotation2Ds(_delta1(2)).matrix();
}

void ProcessorDiffDrive::statePlusDelta(const Eigen::VectorXs& _x,
                                        const Eigen::VectorXs& _delta,
                                        const Scalar /*_Dt*/,
                                        Eigen::VectorXs& _x_plus_delta)
{
  assert(_x.size() == x_size_ && "Wrong _x vector size");
  assert(_x_plus_delta.size() == x_size_ && "Wrong _x_plus_delta vector size");

  // This is just a frame composition in 2D
  _x_plus_delta.head<2>() = _x.head<2>() + Eigen::Rotation2Ds(_x(2)).matrix() * _delta.head<2>();
  _x_plus_delta(2) = pi2pi(_x(2) + _delta(2));
}

Eigen::VectorXs ProcessorDiffDrive::deltaZero() const
{
  return Eigen::VectorXs::Zero(delta_size_);
}

Motion ProcessorDiffDrive::interpolate(const Motion& _ref,
                                       Motion& _second,
                                       TimeStamp& _ts)

{
  // TODO: Implement actual interpolation
  // Implementation: motion ref keeps the same
  //
//  Motion _interpolated(_ref);
//  _interpolated.ts_                   = _ts;
//  _interpolated.data_                 = Vector3s::Zero();
//  _interpolated.data_cov_             = Matrix3s::Zero();
//  _interpolated.delta_                = deltaZero();
//  _interpolated.delta_cov_            = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
//  _interpolated.delta_integr_         = _ref.delta_integr_;
//  _interpolated.delta_integr_cov_     = _ref.delta_integr_cov_;
//  _interpolated.jacobian_delta_integr_. setIdentity();
//  _interpolated.jacobian_delta_       . setZero();
//  _interpolated.jacobian_calib_       . setZero();
//  return _interpolated;

  return ProcessorMotion::interpolate(_ref, _second, _ts);

}

CaptureMotionPtr ProcessorDiffDrive::createCapture(const TimeStamp& _ts,
                                                   const SensorBasePtr& _sensor,
                                                   const VectorXs& _data,
                                                   const MatrixXs& _data_cov,
                                                   const FrameBasePtr& _frame_origin)
{

  StateBlockPtr i_ptr = _sensor->isIntrinsicDynamic()?
        std::make_shared<StateBlock>(3, false) : nullptr;

  return std::make_shared<CaptureWheelJointPosition>(_ts, _sensor, _data, _data_cov,
                                                     _frame_origin, nullptr, nullptr, i_ptr);
}

ConstraintBasePtr ProcessorDiffDrive::emplaceConstraint(FeatureBasePtr _feature,
                                                        CaptureBasePtr _capture_origin)
{
  ConstraintOdom2DPtr ctr_odom =
      std::make_shared<ConstraintOdom2D>(_feature, _capture_origin->getFramePtr(),
                                         shared_from_this());

  _feature->addConstraint(ctr_odom);
  _capture_origin->getFramePtr()->addConstrainedBy(ctr_odom);

  return ctr_odom;
}

FeatureBasePtr ProcessorDiffDrive::createFeature(CaptureMotionPtr _capture_motion)
{
    FeatureBasePtr key_feature_ptr = std::make_shared<FeatureDiffDrive>(
            _capture_motion->getBuffer().get().back().delta_integr_,
            _capture_motion->getBuffer().get().back().delta_integr_cov_,
            _capture_motion->getBuffer().getCalibrationPreint(),
            _capture_motion->getBuffer().get().back().jacobian_calib_);

    WOLF_INFO(_capture_motion->getBuffer().getCalibrationPreint());
    WOLF_INFO(_capture_motion->getBuffer().get().back().jacobian_calib_);

    return key_feature_ptr;
}

ProcessorBasePtr ProcessorDiffDrive::create(const std::string& _unique_name,
                                            const ProcessorParamsBasePtr _params,
                                            const SensorBasePtr _sensor_ptr)
{
  const auto params = std::static_pointer_cast<ProcessorParamsDiffDrive>(_params);

  if (params == nullptr)
  {
    WOLF_ERROR("ProcessorDiffDrive::create : input arg"
               " _params is NOT of type 'ProcessorParamsDiffDrive' !");
    return nullptr;
  }

  const auto sensor_ptr = std::dynamic_pointer_cast<SensorDiffDrive>(_sensor_ptr);

  if (sensor_ptr == nullptr)
  {
    WOLF_ERROR("ProcessorDiffDrive::create : input arg"
               " '_sensor_ptr' is NOT of type 'SensorDiffDrive' !");
    return nullptr;
  }

  ProcessorBasePtr prc_ptr = std::make_shared<ProcessorDiffDrive>(*params);
  prc_ptr->setName(_unique_name);
  return prc_ptr;
}

} // namespace wolf

// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("DIFF DRIVE", ProcessorDiffDrive)
} // namespace wolf
