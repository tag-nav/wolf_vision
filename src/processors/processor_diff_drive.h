/**
 * \file processor_diff_drive.h
 *
 *  Created on: Oct 15, 2016
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_PROCESSOR_DIFF_DRIVE_H_
#define _WOLF_PROCESSOR_DIFF_DRIVE_H_

#include "../processor_motion.h"
#include "../sensors/diff_drive_tools.h"

namespace wolf {

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsDiffDrive);

struct ProcessorParamsDiffDrive : public ProcessorParamsBase
{
  ProcessorParamsDiffDrive(const Scalar _time_tolerance,
                           const Scalar _dist_travel_th,
                           const Scalar _theta_traveled_th,
                           const Scalar _cov_det_th,
                           const Scalar _unmeasured_perturbation_std = 0.0001) :
    dist_traveled_th_(_dist_travel_th),
    theta_traveled_th_(_theta_traveled_th),
    cov_det_th_(_cov_det_th),
    unmeasured_perturbation_std_(_unmeasured_perturbation_std)
  {
      time_tolerance = _time_tolerance;
  }

  Scalar dist_traveled_th_;
  Scalar theta_traveled_th_;
  Scalar cov_det_th_;
  Scalar unmeasured_perturbation_std_ = 0.0001;
};

/**
 * @brief The ProcessorDiffDrive class.
 *
 * Velocity motion model.
 *
 * Integrate odometry from joint position.
 *
 * velocity : data is [d_vx, d_w]
 * position : data is [left_position_increment, right_position_increment]
 *
 * delta is [dx, dy, dtheta]
 */

WOLF_PTR_TYPEDEFS(ProcessorDiffDrive);

class ProcessorDiffDrive : public ProcessorMotion
{
public:

  ProcessorDiffDrive(const ProcessorParamsDiffDrive& params);

  virtual ~ProcessorDiffDrive() = default;
  virtual void configure(SensorBasePtr _sensor) override { }

  virtual bool voteForKeyFrame() override;

protected:

  /// @brief Covariance to be added to the unmeasured perturbation.
  Matrix3s unmeasured_perturbation_cov_;

  /// @brief Intrinsic params
  ProcessorParamsDiffDrive params_;

  virtual void computeCurrentDelta(const Eigen::VectorXs& _data,
                                   const Eigen::MatrixXs& _data_cov,
                                   const Eigen::VectorXs& _calib,
                                   const Scalar _dt,
                                   Eigen::VectorXs& _delta,
                                   Eigen::MatrixXs& _delta_cov,
                                   Eigen::MatrixXs& _jacobian_delta_calib) override;

  virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                              const Eigen::VectorXs& _delta2,
                              const Scalar _Dt2,
                              Eigen::VectorXs& _delta1_plus_delta2) override;

  virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                              const Eigen::VectorXs& _delta2,
                              const Scalar _Dt2,
                              Eigen::VectorXs& _delta1_plus_delta2,
                              Eigen::MatrixXs& _jacobian1,
                              Eigen::MatrixXs& _jacobian2) override;

  virtual void statePlusDelta(const Eigen::VectorXs& _x,
                          const Eigen::VectorXs& _delta,
                          const Scalar _Dt,
                          Eigen::VectorXs& _x_plus_delta) override;

  virtual Eigen::VectorXs deltaZero() const override;

  virtual Motion interpolate(const Motion& _ref,
                             Motion& _second,
                             TimeStamp& _ts) override;

  virtual CaptureMotionPtr createCapture(const TimeStamp& _ts,
                                         const SensorBasePtr& _sensor,
                                         const VectorXs& _data,
                                         const MatrixXs& _data_cov,
                                         const FrameBasePtr& _frame_origin) override;

  virtual ConstraintBasePtr emplaceConstraint(FeatureBasePtr _feature,
                                              CaptureBasePtr _capture_origin) override;

  virtual FeatureBasePtr createFeature(CaptureMotionPtr _capture_motion) override;

public:

  /// @brief Factory method
  static ProcessorBasePtr create(const std::string& _unique_name,
                                 const ProcessorParamsBasePtr _params,
                                 const SensorBasePtr _sensor_ptr);
};

} // namespace wolf

#endif /* _WOLF_PROCESSOR_DIFF_DRIVE_H_ */

