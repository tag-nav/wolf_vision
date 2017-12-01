/**
 * \file constraint_diff_drive.h
 *
 *  Created on: Oct 27, 2017
 *  \author: Jeremie Deray
 */

#ifndef WOLF_CONSTRAINT_DIFF_DRIVE_H_
#define WOLF_CONSTRAINT_DIFF_DRIVE_H_

//Wolf includes
#include "../constraint_autodiff.h"
#include "../frame_base.h"
#include "../features/feature_diff_drive.h"
#include "../captures/capture_wheel_joint_position.h"

namespace
{

constexpr std::size_t RESIDUAL_SIZE                = 3;
constexpr std::size_t POSITION_STATE_BLOCK_SIZE    = 2;
constexpr std::size_t ORIENTATION_STATE_BLOCK_SIZE = 1;

/// @todo This should be dynamic (2/3/5)
constexpr std::size_t INTRINSICS_STATE_BLOCK_SIZE  = 3;

}

namespace wolf {

class ConstraintDiffDrive :
    public ConstraintAutodiff< ConstraintDiffDrive,
      RESIDUAL_SIZE, POSITION_STATE_BLOCK_SIZE, ORIENTATION_STATE_BLOCK_SIZE,
      INTRINSICS_STATE_BLOCK_SIZE >
{
  using Base = ConstraintAutodiff<ConstraintDiffDrive,
  RESIDUAL_SIZE, POSITION_STATE_BLOCK_SIZE, ORIENTATION_STATE_BLOCK_SIZE,
  INTRINSICS_STATE_BLOCK_SIZE>;

public:

  ConstraintDiffDrive(const FeatureDiffDrivePtr& _ftr_ptr,
                      const CaptureWheelJointPositionPtr& _capture_origin_ptr,
                      const ProcessorBasePtr& _processor_ptr = nullptr,
                      const bool _apply_loss_function = false,
                      const ConstraintStatus _status = CTR_ACTIVE) :
    Base(CTR_DIFF_DRIVE, _capture_origin_ptr->getFramePtr(), _capture_origin_ptr,
         nullptr, nullptr, _processor_ptr,
         _apply_loss_function, _status,
         _frame_ptr->getPPtr(), _frame_ptr->getOPtr(),
         _capture_origin_ptr->getFramePtr()->getPPtr(),
         _capture_origin_ptr->getFramePtr()->getOPtr(),
         _capture_origin_ptr->getFramePtr()->getVPtr(),
         _capture_origin_ptr->getSensorIntrinsicPtr(),
         _ftr_ptr->getFramePtr()->getPPtr(),
         _ftr_ptr->getFramePtr()->getOPtr(),
         _ftr_ptr->getFramePtr()->getVPtr(),
         _ftr_ptr->getCapturePtr()->getSensorIntrinsicPtr()),
    J_delta_calib_(_ftr_ptr->getJacobianFactor())
  {
    setType("DIFF DRIVE");
  }

  /**
   * \brief Default destructor (not recommended)
   *
   * Default destructor.
   *
   **/
  virtual ~ConstraintDiffDrive() = default;

  template<typename T>
  bool operator ()(const T* const _p1, const T* const _o1, const T* const _c1,
                   const T* const _p2, const T* const _o2, const T* const _c2,
                   T* _residuals) const;

  /**
   * \brief Returns the jacobians computation method
   **/
  virtual JacobianMethod getJacobianMethod() const
  {
    return JAC_AUTO;
  }

protected:

  Eigen::MatrixXs J_delta_calib_;
};

template<typename T>
inline bool
ConstraintDiffDrive::operator ()(const T* const _p1, const T* const _o1, const T* const _c1,
                                 const T* const _p2, const T* const _o2, const T* const _c2,
                                 T* _residuals) const
{
  // MAPS
  Eigen::Map<Eigen::Matrix<T, RESIDUAL_SIZE, 1> > residuals_map(_residuals);

  Eigen::Map<const Eigen::Matrix<T, POSITION_STATE_BLOCK_SIZE, 1> > p1_map(_p1);
  Eigen::Map<const Eigen::Matrix<T, POSITION_STATE_BLOCK_SIZE, 1> > p2_map(_p2);

  Eigen::Map<const Eigen::Matrix<T, INTRINSICS_STATE_BLOCK_SIZE, 1> > c1_map(_c1);
  Eigen::Map<const Eigen::Matrix<T, INTRINSICS_STATE_BLOCK_SIZE, 1> > c2_map(_c2);

  // Compute corrected delta

  /// Is this my delta_preint ?
  const Eigen::Matrix<T, 3, 1> measurement = getMeasurement().cast<T>();

  Eigen::Matrix<T, 3, 1> delta_corrected = measurement + J_delta_calib_.cast<T>() * (c2_map - c1_map);

  // First 2d pose residual

  Eigen::Matrix<T, 3, 1> delta_predicted;

  // position
  delta_predicted.head(2) = Eigen::Rotation2D<T>(-_o1[0]) * (p2_map - p1_map);

  // orientation
  delta_predicted(2) = _o2[0] - _o1[0];

  // residual
  residuals_map = delta_corrected - delta_predicted;

  // angle remapping
  while (residuals_map(2) > T(M_PI))
    residuals_map(2) = residuals_map(2) - T(2. * M_PI);
  while (residuals_map(2) <= T(-M_PI))
    residuals_map(2) = residuals_map(2) + T(2. * M_PI);

  // weighted residual
  residuals_map = getMeasurementSquareRootInformationUpper().cast<T>() * residuals_map;

  return true;
}

} // namespace wolf

#endif /* WOLF_CONSTRAINT_DIFF_DRIVE_H_ */
