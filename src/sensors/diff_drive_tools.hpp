/**
 * \file diff_drive_tools.h
 *
 *  Created on: Oct 20, 2016
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_PROCESSOR_DIFF_DRIVE_TOOLS_HPP_
#define _WOLF_PROCESSOR_DIFF_DRIVE_TOOLS_HPP_

// un-comment for IDE highlight
//#include "diff_drive_tools.h"
#include <tuple>

namespace wolf {
namespace detail {

template <>
template <typename T0, typename T1, typename T2>
std::tuple<VelocityComand<typename T0::Scalar>, Eigen::Matrix<typename T0::Scalar, 2, 2>, Eigen::Matrix<typename T0::Scalar, 2, 3>>
wheelPositionIncrementToVelocityHelper<DiffDriveModel::Two_Factor_Model>::wheelPositionIncrementToVelocity(
    const Eigen::MatrixBase<T0>& position_increment,
    const Eigen::MatrixBase<T1>& position_increment_cov,
    const typename T0::Scalar left_radius,
    const typename T0::Scalar right_radius,
    const typename T0::Scalar separation,
    const Eigen::MatrixBase<T2>& factors,
    const typename T0::Scalar /*dt*/)
{ 
  Eigen::VectorSizeCheck<2>::check(position_increment);
  Eigen::MatrixSizeCheck<2,2>::check(position_increment_cov);

  Eigen::VectorSizeCheck<2>::check(factors);

  using T = typename T0::Scalar;

  const T left_wheel_vel  = (left_radius  * factors(0)) * position_increment(0);
  const T right_wheel_vel = (right_radius * factors(0)) * position_increment(1);

  const T o_5(0.5);
  const T s_f = separation * factors(1);

  const Eigen::Matrix<T, 2, 1> comand =
      Eigen::Matrix<T, 2, 1>((right_wheel_vel + left_wheel_vel) * o_5,
                             (right_wheel_vel - left_wheel_vel) / s_f) /* * dt*/;

  const T p_r = position_increment(1) * right_radius;
  const T p_l = position_increment(0) * left_radius;

  Eigen::Matrix<T, 2, 2> J_vel_posinc;

  J_vel_posinc << left_radius * factors(0) * o_5 ,  right_radius * factors(1) * o_5,
                   left_radius * factors(0) / s_f, -right_radius * factors(1) / s_f;

  Eigen::Matrix<T, 2, 3> J_vel_factor;

  J_vel_factor << (p_l + p_r) * o_5,   0,
                  (p_l - p_r) / s_f,  ((p_r - p_l) * factors(0)) / (s_f * factors(1));

  const Eigen::Matrix<T, 2, 2> comand_cov = J_vel_posinc * position_increment_cov * J_vel_posinc.transpose();

  return std::make_tuple(VelocityComand<T>{comand, comand_cov}, J_vel_posinc, J_vel_factor);
}

template <>
template <typename T0, typename T1, typename T2>
std::tuple<VelocityComand<typename T0::Scalar>, Eigen::Matrix<typename T0::Scalar, 2, 2>, Eigen::Matrix<typename T0::Scalar, 2, 3>>
wheelPositionIncrementToVelocityHelper<DiffDriveModel::Three_Factor_Model>::wheelPositionIncrementToVelocity(
    const Eigen::MatrixBase<T0>& position_increment,
    const Eigen::MatrixBase<T1>& position_increment_cov,
    const typename T0::Scalar left_radius,
    const typename T0::Scalar right_radius,
    const typename T0::Scalar separation,
    const Eigen::MatrixBase<T2>& factors,
    const typename T0::Scalar /*dt*/)
{
  Eigen::VectorSizeCheck<2>::check(position_increment);
  Eigen::MatrixSizeCheck<2,2>::check(position_increment_cov);

  Eigen::VectorSizeCheck<3>::check(factors);

  using T = typename T0::Scalar;

  const T left_wheel_vel  = (left_radius  * factors(0)) * position_increment(0);
  const T right_wheel_vel = (right_radius * factors(1)) * position_increment(1);

  const T o_5(0.5);
  const T s_f = separation * factors(2);

  const Eigen::Matrix<T, 2, 1> comand =
      Eigen::Matrix<T, 2, 1>((right_wheel_vel + left_wheel_vel) * o_5,
                             (right_wheel_vel - left_wheel_vel) / s_f) /* * dt*/;

  const T p_l = position_increment(0) * left_radius;
  const T p_r = position_increment(1) * right_radius;

  Eigen::Matrix<T, 2, 2> J_vel_posinc;

  J_vel_posinc << left_radius * factors(0) * o_5 , right_radius * factors(1) * o_5,
                   left_radius * factors(0) / s_f, -right_radius * factors(1) / s_f;

  Eigen::Matrix<T, 2, 3> J_vel_factor;

  J_vel_factor << p_l * o_5,   p_r * o_5,     0,
                  p_l / s_f,  -p_r / s_f,  (p_r * factors(1) - p_l * factors(0)) / (s_f * factors(2));

  const Eigen::Matrix<T, 2, 2> comand_cov = J_vel_posinc * position_increment_cov * J_vel_posinc.transpose();

  return std::make_tuple(VelocityComand<T>{comand, comand_cov}, J_vel_posinc, J_vel_factor);
}

} /* namespace detail */
} /* namespace wolf */

#endif /* _WOLF_PROCESSOR_DIFF_DRIVE_TOOLS_HPP_ */
