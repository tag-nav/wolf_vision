/**
 * \file processor_diff_drive.h
 *
 *  Created on: Oct 15, 2016
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_PROCESSOR_DIFF_DRIVE_H_
#define _WOLF_PROCESSOR_DIFF_DRIVE_H_

#include "base/processor/processor_motion.h"
#include "base/diff_drive_tools.h"
#include "base/params_server.hpp"

namespace wolf {

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsDiffDrive);

struct ProcessorParamsDiffDrive : public ProcessorParamsMotion
{
  Scalar unmeasured_perturbation_std = 0.0001;
  ProcessorParamsDiffDrive();
  ProcessorParamsDiffDrive(std::string _unique_name, const paramsServer& _server):
    ProcessorParamsMotion(_unique_name, _server)
  {
    unmeasured_perturbation_std = _server.getParam<Scalar>(_unique_name + "/unmeasured_perturbation_std", "0.0001");
  }
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

  ProcessorDiffDrive(ProcessorParamsDiffDrivePtr _params);

  virtual ~ProcessorDiffDrive() = default;
  virtual void configure(SensorBasePtr _sensor) override { }

  virtual bool voteForKeyFrame() override;

protected:

  /// @brief Intrinsic params
  ProcessorParamsDiffDrivePtr params_motion_diff_drive_;
  MatrixXs unmeasured_perturbation_cov_;

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

