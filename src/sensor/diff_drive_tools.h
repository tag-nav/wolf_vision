/**
 * \file diff_drive_tools.h
 *
 *  Created on: Oct 20, 2016
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_PROCESSOR_DIFF_DRIVE_TOOLS_H_
#define _WOLF_PROCESSOR_DIFF_DRIVE_TOOLS_H_

#include "../eigen_assert.h"

namespace wolf {

enum class DiffDriveModel : std::size_t
{
  Two_Factor_Model   = 0,
  Three_Factor_Model = 1,
  Five_Factor_Model  = 2
};

constexpr double ANGULAR_VELOCITY_MIN(1e-8);

/**
 * @brief computeDiffDriveComand. Compute wheels velocity comands given
 * linear and angular velocity.
 *
 * @param comand. Linear and angular velocity comands.
 * @param wheel_comand. Wheels velocity comands.
 *
 * @param left_radius. Left wheel radius.
 * @param right_radius. Right wheel radius.
 * @param separation. Wheels separation.
 */
//template <typename T0, typename T1>
//void computeDiffDriveComand(const Eigen::MatrixBase<T0> &comand,
//                            Eigen::MatrixBase<T1> &wheel_comand,
//                            const typename T1::Scalar left_radius,
//                            const typename T1::Scalar right_radius,
//                            const typename T1::Scalar separation)
//{
//  Eigen::VectorSizeCheck<2>::check(comand);

//  using T = typename T1::Scalar;

//  const T linear  = comand(0);
//  const T angular = comand(1);

//  wheel_comand = Eigen::Matrix<T, 2, 1>((linear - angular * separation * T(0.5)) / left_radius,
//                                        (linear + angular * separation * T(0.5)) / right_radius);
//}

/**
 * @brief computeDiffDriveComand. Compute wheels velocity comands given
 * linear and angular velocity.
 *
 * @param comand. Linear and angular velocity comands.
 * @param wheel_comand. Wheels velocity comands.
 *
 * @param wheel_radius. Wheel radius.
 * @param separation. Wheels separation.
 */
//template <typename T0, typename T1>
//void computeDiffDriveComand(const Eigen::MatrixBase<T0> &comand,
//                            Eigen::MatrixBase<T1> &wheel_comand,
//                            const typename T1::Scalar wheel_radius,
//                            const typename T1::Scalar separation)
//{
//  computeDiffDriveComand(comand, wheel_comand,
//                         wheel_radius, wheel_radius, separation);
//}

/**
 * @brief VelocityComand. Holds a velocity comand vector
 * together with its covariance.
 *
 * @note
 * 2d : [linear_x, angular]
 *
 */
template <typename Scalar = double>
struct VelocityComand
{
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>              comand;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> comand_cov;
};

namespace detail {

template <DiffDriveModel>
struct wheelPositionIncrementToVelocityHelper
{
  template <typename T0, typename T1, typename T2>
  static std::tuple<VelocityComand<typename T0::Scalar>,
                    Eigen::Matrix<typename T0::Scalar, 2, 2>,
                    Eigen::Matrix<typename T0::Scalar, 2, 3>>
  wheelPositionIncrementToVelocity(const Eigen::MatrixBase<T0>& position_increment,
                                   const Eigen::MatrixBase<T1>& position_increment_cov,
                                   const typename T0::Scalar    left_radius,
                                   const typename T0::Scalar    right_radius,
                                   const typename T0::Scalar    separation,
                                   const Eigen::MatrixBase<T2>& factors,
                                   const typename T0::Scalar    dt);
};

} /* namespace detail */

/**
 * @brief Convert from wheels joint positions to per-wheel velocity comands.
 * @param[in] position_increment. A vector containing the wheels position increments.
 * @param[in] position_increment_cov. The covariance associated to the vector position_increment.
 * @param[in] left_radius. The left wheel radius.
 * @param[in] right_radius. The right wheel radius.
 * @param[in] separation. The distance separing both wheels.
 * @param[in] factors. The diff-drive correction factors (calibration).
 * @param[in] dt. UNUSED.
 *
 * @return std::tuple. First element is the computed VelocityComand,
 * second element is the Jacobian matrix J_vel_data,
 * third element is the Jacobian matrix J_vel_factor.
 */
template <DiffDriveModel M, typename T0, typename T1, typename T2>
std::tuple<VelocityComand<typename T0::Scalar>,
           Eigen::Matrix<typename T0::Scalar, 2, 2>,
           Eigen::Matrix<typename T0::Scalar, 2, 3>>
wheelPositionIncrementToVelocity(const Eigen::MatrixBase<T0>& position_increment,
                                 const Eigen::MatrixBase<T1>& position_increment_cov,
                                 const typename T0::Scalar    left_radius,
                                 const typename T0::Scalar    right_radius,
                                 const typename T0::Scalar    separation,
                                 const Eigen::MatrixBase<T2>& factors,
                                 const typename T0::Scalar    dt)
{
  return detail::wheelPositionIncrementToVelocityHelper<M>::wheelPositionIncrementToVelocity(
        position_increment, position_increment_cov,
        left_radius, right_radius, separation,
        factors, dt);
}

/**
 * @brief WheelPositionAbsoluteToIncrement.
 * Simple class to convert from absolute position to increment position.
 *
 * @todo handle num wheels 4-6-etc
 */
template <typename T>
class WheelPositionAbsoluteToIncrement
{
public:

  WheelPositionAbsoluteToIncrement()  = default;
  ~WheelPositionAbsoluteToIncrement() = default;

  template <typename T0>
  WheelPositionAbsoluteToIncrement(const Eigen::MatrixBase<T0>& positions)
  {
    init(positions);
  }

  inline bool init() const noexcept { return init_; }

  template <typename T0>
  void update(const Eigen::MatrixBase<T0>& positions)
  {
    Eigen::VectorSizeCheck<2>::check(positions);

    positions_ = positions.template cast<T>();
  }

  template <typename T0>
  void init(const Eigen::MatrixBase<T0>& positions)
  {
    update(positions);

    init_ = true;
  }

  template <typename T0, typename T1>
  void toIncrement(const Eigen::MatrixBase<T0>& positions,
                   Eigen::MatrixBase<T1>& positions_increment)
  {
    Eigen::VectorSizeCheck<2>::check(positions);

    if (!init_) init(positions);

    positions_increment =
        (positions - positions_.template cast<typename T0::Scalar>()).
          template cast<typename T1::Scalar>();
  }

  template <typename T0, typename T1>
  void operator() (const Eigen::MatrixBase<T0>& positions,
                   Eigen::MatrixBase<T1>& positions_increment)
  {
    toIncrement(positions, positions_increment);
    update(positions);
  }

protected:

  bool init_ = false;

  Eigen::Matrix<T, Eigen::Dynamic, 1> positions_;
};

/**
 * @brief integrateRungeKutta. Runge-Kutta 2nd order integration.
 *
 * @param[in] data. The input linear and angular velocities.
 * @param[in] data_cov. The covariance matrix associated to data.
 * @param[out] delta. The computed delta (2d).
 * @param[out] delta_cov. The covariance matrix associated to delta.
 * @param[out] jacobian. The Jacobian matrix J_delta_vel
 *
 * @note
 *
 * Linear  velocity [m]   (linear  displacement, i.e.   m/s * dt)
 * Angular velocity [rad] (angular displacement, i.e. rad/s * dt)
 *
 * MATHS of delta
 * dx  = dv * cos( dw * 0.5 )
 * dy  = dv * sin( dw * 0.5 )
 * dth = dw
 */
template <typename T0, typename T1, typename T2, typename T3, typename T4>
void integrateRungeKutta(const Eigen::MatrixBase<T0> &data,
                         const Eigen::MatrixBase<T1> &data_cov,
                         Eigen::MatrixBase<T2>       &delta,
                         Eigen::MatrixBase<T3>       &delta_cov,
                         Eigen::MatrixBase<T4>       &jacobian)
{
  using std::sin;
  using std::cos;

  /// @note data is size 3 -> linear vel on x + angular vel on yaw
  Eigen::VectorSizeCheck<2>::check(data);
  /// @todo check dim
  Eigen::MatrixSizeCheck<2,2>::check(data_cov);

  using T = typename T2::Scalar;

  const T v = data(0);
  const T w = data(1);

  const T w_05 = w * T(.5);

  const T cos_w_05 = cos(w_05);
  const T sin_w_05 = sin(w_05);

  delta = Eigen::Matrix<T, 3, 1>(cos_w_05 * v,
                                 sin_w_05 * v,
                                      w       );
  // Fill delta covariance
  Eigen::Matrix<typename T3::Scalar,
      Eigen::MatrixBase<T2>::RowsAtCompileTime,
      Eigen::MatrixBase<T0>::RowsAtCompileTime> J;

  J.setZero(delta.rows(), data.rows());

  // Compute jacobian
  jacobian =
      Eigen::Matrix<typename T4::Scalar,
        Eigen::MatrixBase<T2>::RowsAtCompileTime,
        Eigen::MatrixBase<T0>::RowsAtCompileTime>::Zero(delta.rows(), data.rows());

  Eigen::MatrixSizeCheck<3,2>::check(jacobian);

  jacobian(0,0) =  cos_w_05;
  jacobian(1,0) =  sin_w_05;
  jacobian(2,0) =  typename T3::Scalar(0);

  jacobian(0,1) = -v * sin_w_05 * typename T3::Scalar(.5);
  jacobian(1,1) =  v * cos_w_05 * typename T3::Scalar(.5);
  jacobian(2,1) =  typename T3::Scalar(1);

  // Fill delta covariance
  delta_cov = J * data_cov * J.transpose();
}

/**
 * @brief integrateExact. Exact integration.
 *
 * @param[in] data. The input linear and angular velocities.
 * @param[in] data_cov. The covariance matrix associated to data.
 * @param[out] delta. The computed delta (2d).
 * @param[out] delta_cov. The covariance matrix associated to delta.
 * @param[out] jacobian. The Jacobian matrix J_delta_vel
 *
 * @note
 *
 * Linear  velocity [m]   (linear  displacement, i.e.   m/s * dt)
 * Angular velocity [rad] (angular displacement, i.e. rad/s * dt)
 *
 * MATHS of delta
 * dx  =  dv / dw * (sin(dw) - sin(0))
 * dy  = -dv / dw * (cos(dw) - cos(0))
 * dth =  dw
 */
template <typename T0, typename T1, typename T2, typename T3, typename T4>
void integrateExact(const Eigen::MatrixBase<T0> &data,
                    const Eigen::MatrixBase<T1> &data_cov,
                    Eigen::MatrixBase<T2>       &delta,
                    Eigen::MatrixBase<T3>       &delta_cov,
                    Eigen::MatrixBase<T4>       &jacobian)
{
  using std::sin;
  using std::cos;

  /// @note data is size 3 -> linear vel on x & y + angular vel on yaw
  Eigen::VectorSizeCheck<2>::check(data);
  /// @todo check dim
  Eigen::MatrixSizeCheck<2,2>::check(data_cov);

  using T = typename T2::Scalar;

  // Compute delta

  const T v = data(0);
  const T w = data(1);

  const T inv_w  = T(1) / w;

  const T r =  v * inv_w;

  const T theta_1 = w;

  const T sin_theta_0 = 0;
  const T cos_theta_0 = 1;

  const T sin_theta_1 = sin(theta_1);
  const T cos_theta_1 = cos(theta_1);

  const T sin_diff = sin_theta_1 - sin_theta_0;
  const T cos_diff = cos_theta_1 - cos_theta_0;

  delta = Eigen::Matrix<T, 3, 1>( r * sin_diff,
                                 -r * cos_diff,
                                      w       );

  const T inv_w2 = inv_w * inv_w;

  // Compute jacobian
  jacobian =
      Eigen::Matrix<typename T4::Scalar,
        Eigen::MatrixBase<T2>::RowsAtCompileTime,
        Eigen::MatrixBase<T0>::RowsAtCompileTime>::Zero(delta.rows(), data.rows());

  Eigen::MatrixSizeCheck<3,2>::check(jacobian);

  jacobian(0,0) =  sin_diff * inv_w;
  jacobian(1,0) = -cos_diff * inv_w;
  jacobian(2,0) = T(0);

  jacobian(0,1) =  (v * cos_theta_1 * inv_w) - (v * sin_diff * inv_w2);
  jacobian(1,1) =  (v * sin_theta_1 * inv_w) + (v * cos_diff * inv_w2);
  jacobian(2,1) =  T(1);

  // Fill delta covariance
  delta_cov = jacobian * data_cov * jacobian.transpose();
}


/**
 * @brief integrate. Helper function to call either
 * `integrateRung` or `integrateExact` depending on the
 * angular velocity value.
 *
 * @see integrateRungeKutta
 * @see integrateExact
 * @see ANGULAR_VELOCITY_MIN
 */
template <typename T0, typename T1, typename T2, typename T3, typename T4>
void integrate(const Eigen::MatrixBase<T0>& data,
               const Eigen::MatrixBase<T1>& data_cov,
               Eigen::MatrixBase<T2>& delta,
               Eigen::MatrixBase<T3>& delta_cov,
               Eigen::MatrixBase<T4>& jacobian)
{
  (data(1) < ANGULAR_VELOCITY_MIN) ?
        integrateRungeKutta(data, data_cov, delta, delta_cov, jacobian) :
        integrateExact(data, data_cov, delta, delta_cov, jacobian);
}

/**
 * @brief computeWheelJointPositionCov. Compute a covariance matrix to associate
 * to wheel position increment.
 *
 * Set covariance matrix (diagonal):
 * second term is the wheel resolution covariance,
 * similar to a DC offset equal to half of the resolution,
 * which is the theoretical average error.
 *
 * Introduction to Autonomous Mobile Robots, 1st Edition, 2004
 * Roland Siegwart, Illah R. Nourbakhsh
 * Section: 5.2.4 'An error model for odometric position estimation' (pp. 186-191)
 */
template <typename T>
Eigen::Matrix<typename T::Scalar, 2, 2> computeWheelJointPositionCov(
    const Eigen::MatrixBase<T>& data,
    const typename T::Scalar left_wheel_resolution,
    const typename T::Scalar right_wheel_resolution,
    const typename T::Scalar left_wheel_cov_gain,
    const typename T::Scalar right_wheel_cov_gain)
{
  using std::abs;

  Eigen::VectorSizeCheck<2>::check(data);

  using S = typename T::Scalar;

  const S dp_var_l = left_wheel_cov_gain  * abs(data(0));
  const S dp_var_r = right_wheel_cov_gain * abs(data(1));

  Eigen::Matrix<S, 2, 2> data_cov;
  data_cov << dp_var_l + (left_wheel_resolution  * S(0.5)), 0,
              0, dp_var_r + (right_wheel_resolution * S(0.5));

  return data_cov;
}

} // namespace wolf

#include "diff_drive_tools.hpp"

#endif /* _WOLF_PROCESSOR_DIFF_DRIVE_TOOLS_H_ */
