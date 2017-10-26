/**
 * \file capture_velocity.h
 *
 *  Created on: Oct 20, 2016
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_CAPTURE_VELOCITY_H_
#define _WOLF_CAPTURE_VELOCITY_H_

//wolf includes
#include "capture_motion.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(CaptureVelocity);

/**
 * @brief The CaptureVelocity class
 *
 * Represents a velocity.
 */
class CaptureVelocity : public CaptureMotion
{
protected:

  using NodeBase::node_type_;

public:

  /**
   * \brief Constructor
   **/
  CaptureVelocity(const TimeStamp& _ts,
                  const SensorBasePtr& _sensor_ptr,
                  const Eigen::VectorXs& _velocity,
                  Size _delta_size, Size _delta_cov_size,
                  FrameBasePtr _origin_frame_ptr);

  CaptureVelocity(const TimeStamp& _ts,
                  const SensorBasePtr& _sensor_ptr,
                  const Eigen::VectorXs& _velocity,
                  const Eigen::MatrixXs& _velocity_cov,
                  Size _delta_size, Size _delta_cov_size,
                  FrameBasePtr _origin_frame_ptr,
                  StateBlockPtr _p_ptr = nullptr,
                  StateBlockPtr _o_ptr = nullptr,
                  StateBlockPtr _intr_ptr = nullptr);

  virtual ~CaptureVelocity() = default;

  const Eigen::VectorXs& getVelocity() const;

  const Eigen::MatrixXs& getVelocityCov() const;
};

} // namespace wolf

#endif /* _WOLF_CAPTURE_VELOCITY_H_ */
