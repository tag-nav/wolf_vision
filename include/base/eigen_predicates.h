/**
 * \file eigen_predicates.h
 * \brief Some utils for comparing Eigen types
 * \author Jeremie Deray
 *  Created on: 24/10/2017
 */

#ifndef _WOLF_EIGEN_PREDICATES_H_
#define _WOLF_EIGEN_PREDICATES_H_

#include "rotations.h"

namespace wolf {

/// @brief check that each Matrix/Vector elem is approx equal to value +- precision
///
/// @note we overload this rather than using Eigen::Matrix::isConstant() since it is
/// defined as :
///   abs(x - y) <= (min)(abs(x), abs(y)) * prec
/// which does not play nice with y = 0
auto is_constant = [](const Eigen::MatrixXs& lhs, const Scalar val, const Scalar precision)
                      {
                        for (Eigen::MatrixXs::Index j = 0; j < lhs.cols(); ++j)
                          for (Eigen::MatrixXs::Index i = 0; i < lhs.rows(); ++i)
                            if (std::abs(lhs.coeff(i, j) - val) > precision)
                              return false;
                        return true;
                      };

/// @brief check that each element of the Matrix/Vector difference
/// is approx 0 +- precision
auto pred_zero = [](const Eigen::MatrixXs& lhs, const Scalar precision)
                   { return is_constant(lhs, 0, precision); };

/// @brief check that each element of the Matrix/Vector difference
/// is approx 0 +- precision
auto pred_diff_zero = [](const Eigen::MatrixXs& lhs, const Eigen::MatrixXs& rhs, const Scalar precision)
                        { return pred_zero(lhs - rhs, precision); };

/// @brief check that the Matrix/Vector difference norm is approx 0 +- precision
auto pred_diff_norm_zero = [](const Eigen::MatrixXs& lhs, const Eigen::MatrixXs& rhs, const Scalar precision)
                             { return std::abs((lhs - rhs).norm()) <= std::abs(precision); };

/// @brief check that the lhs Matrix/Vector is elem-wise approx the rhs +-precision
auto pred_is_approx = [](const Eigen::MatrixXs& lhs, const Eigen::MatrixXs& rhs, const Scalar precision)
                        { return lhs.isApprox(rhs, precision); };

/// @brief check that angle θ of rotation required to get from lhs quaternion to rhs
/// is <= precision
auto pred_quat_is_approx = [](const Eigen::Quaternions& lhs, const Eigen::Quaternions& rhs, const Scalar precision)
                             { return pred_zero( log_q(rhs * lhs.inverse()), precision ); };

/// @brief check that angle θ of rotation required to get from lhs quaternion to identity
/// is <= precision
auto pred_quat_identity = [](const Eigen::Quaternions& lhs, const Scalar precision)
                            { return pred_quat_is_approx(lhs, Eigen::Quaternions::Identity(), precision); };

/// @brief check that rotation angle to get from lhs angle to rhs is <= precision
auto pred_angle_is_approx = [](const Scalar lhs, const Scalar rhs, const Scalar precision)
                              { return std::abs(pi2pi(lhs - rhs)) <= std::abs(precision); };

/// @brief check that rotation angle to get from lhs angle to 0 is <= precision
auto pred_angle_zero = [](const Scalar lhs, const Scalar precision)
                          { return pred_angle_is_approx(lhs, 0, precision); };

/// @brief check that the lhs pose is approx rhs +- precision
///
/// @note
/// Comparison is :
/// d = inv(lhs) * rhs
/// d.translation().elem_wise() ~ 0 (+- precision)
///
/// if pose 3d :
/// d.rotation_as_quaternion() ~ quaternion.getIdentity (+- precision)
///
/// else if pose 2d:
/// d.rotation_angle() ~ 0 (+- precision)
///
/// else throw std::runtime
///
/// @see pred_zero for translation comparison
/// @see pred_quat_identity for 3d rotation comparison
/// @see pred_angle_zero for 2d rotation comparison
//auto pred_pose_is_approx = [](const Eigen::MatrixXs lhs, const Eigen::MatrixXs rhs, const Scalar precision)
//                              {
//                                const Eigen::MatrixXs d = lhs.inverse() * rhs;
//                                const bool tok = pred_zero(d.rightCols(1), precision);

//                                const bool qok = (lhs.rows() == 3)?
//                                      pred_quat_identity(Eigen::Quaternions(d.block(0,0,3,1)),
//                                                         precision)
//                                      : (lhs.rows() == 2)? pred_angle_zero(getYaw(d), precision)
//                                                         : throw std::runtime_error("Canno't compare pose in Dim > 3 !");

//                                return tok && qok;
//                              };

} // namespace wolf


#endif /* _WOLF_EIGEN_PREDICATES_H_ */
