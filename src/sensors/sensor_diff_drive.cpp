#include "sensor_diff_drive.h"
#include "../state_block.h"
#include "../capture_motion.h"
#include "../eigen_assert.h"

namespace wolf {

SensorDiffDrive::SensorDiffDrive(const StateBlockPtr& _p_ptr,
                                 const StateBlockPtr& _o_ptr,
                                 const StateBlockPtr& _i_ptr,
                                 const IntrinsicsDiffDrivePtr& _intrinsics) :
  SensorBase("DIFF DRIVE", _p_ptr, _o_ptr, _i_ptr, 2, false, false),
  intrinsics_ptr_{_intrinsics}
{
  //
}

// Define the factory method
SensorBasePtr SensorDiffDrive::create(const std::string& _unique_name,
                                      const Eigen::VectorXs& _extrinsics_po,
                                      const IntrinsicsBasePtr _intrinsics)
{
  Eigen::VectorSizeCheck<3>::check(_extrinsics_po);

  // cast intrinsics into derived type
  IntrinsicsDiffDrivePtr params = std::dynamic_pointer_cast<IntrinsicsDiffDrive>(_intrinsics);

  if (params == nullptr)
  {
    WOLF_ERROR("SensorDiffDrive::create expected IntrinsicsDiffDrive !");
    return nullptr;
  }

  StateBlockPtr pos_ptr = std::make_shared<StateBlock>(_extrinsics_po.head(2), true);
  StateBlockPtr ori_ptr = std::make_shared<StateBlock>(_extrinsics_po.tail(1), true);
  StateBlockPtr int_ptr = std::make_shared<StateBlock>(params->factors_,       true);

  SensorBasePtr odo = std::make_shared<SensorDiffDrive>(pos_ptr, ori_ptr, int_ptr, params);

  odo->setName(_unique_name);

  /// @todo make calibration optional at creation
  //if (calibrate)
    odo->unfixIntrinsics();

  return odo;
}


/// @todo Further work to enforce wheel model

//const IntrinsicsDiffDrive& SensorDiffDrive::getIntrinsics()
//{
////  if (intrinsic_ptr_)
////  {
////    const auto& intrinsics = intrinsic_ptr_->getVector();

////    intrinsics_.left_radius_factor_  = intrinsics(0);
////    intrinsics_.right_radius_factor_ = intrinsics(1);
////    intrinsics_.separation_factor_   = intrinsics(2);
////  }

//  return intrinsics_;
//}

//void SensorDiffDrive::initIntrisicsPtr()
//{
//  assert(intrinsic_ptr_ == nullptr &&
//         "SensorDiffDrive::initIntrisicsPtr should only be called once at construction.");

//  Eigen::Vector3s state;
//  state << intrinsics_.left_radius_factor_,
//           intrinsics_.right_radius_factor_,
//           intrinsics_.separation_factor_;

//  assert(state(0) > 0 && "The left_radius_factor should be rather close to 1.");
//  assert(state(1) > 0 && "The right_radius_factor should be rather close to 1.");
//  assert(state(2) > 0 && "The separation_factor should be rather close to 1.");

//  intrinsic_ptr_ = new StateBlock(state);
//}

//void SensorDiffDrive::computeDataCov(const Eigen::Vector2s &_data, Eigen::Matrix2s &_data_cov)
//{
//  const double dp_std_l = intrinsics_.left_gain_  * _data(0);
//  const double dp_std_r = intrinsics_.right_gain_ * _data(1);

//  const double dp_var_l = dp_std_l * dp_std_l;
//  const double dp_var_r = dp_std_r * dp_std_r;

//  /// Wheel resolution covariance, which is like a DC offset equal to half of
//  /// the resolution, which is the theoretical average error:
//  const double dp_std_avg = Scalar(0.5) * intrinsics_.left_resolution_;
//  const double dp_var_avg = dp_std_avg * dp_std_avg;

//  /// Set covariance matrix (diagonal):
//  _data_cov.diagonal() << dp_var_l + dp_var_avg,
//                          dp_var_r + dp_var_avg;
//}

// This overload is probably not the best solution as it forces
// me to call addCapture from a SensorDiffDrivePtr whereas
// problem->installSensor() return a SensorBasePtr.
//bool SensorDiffDrive::addCapture(CaptureBasePtr _capture_ptr)
//{
//  std::shared_ptr<CaptureMotion> capture_ptr = std::static_pointer_cast<CaptureMotion>(_capture_ptr);

//  if (intrinsics_.data_is_position_)
//  {
//    Eigen::Vector2s data = capture_ptr->getData();

//    // dt is set to one as we are dealing with wheel position
//    data = pose_inc_(data, intrinsics_.left_radius_, intrinsics_.right_radius_,
//                     intrinsics_.separation_, 1);

//    capture_ptr->setData(data);

//    Eigen::Matrix2s data_cov;
//    data_cov << 0.00001, 0, 0, 0.00001; // Todo

//    computeDataCov(data, data_cov);

//    capture_ptr->setDataCovariance(data_cov);
//  }

//  /// @todo tofix
//  return false;//SensorBase::addCapture(_capture_ptr);
//}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("DIFF DRIVE", SensorDiffDrive)
} // namespace wolf
