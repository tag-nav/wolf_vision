
#include "local_parametrization_quaternion.h"
#include "rotations.h"

#include <iostream>
namespace wolf {

template <>
bool LocalParametrizationQuaternion<wolf::DQ_LOCAL>::plus(const Eigen::Map<const Eigen::VectorXs>& _q,
                                                          const Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                                                          Eigen::Map<Eigen::VectorXs>& _q_plus_delta_theta) const
{

    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_delta_theta.size() == local_size_ && "Wrong size of input delta_theta.");
    assert(_q_plus_delta_theta.size() == global_size_ && "Wrong size of output quaternion.");

    assert(fabs(1.0 - _q.norm()) < Constants::EPS && "Quaternion not normalized.");

    using namespace Eigen;

    double angle = _delta_theta.norm();
    Quaternions dq;
    if (angle > Constants::EPS_SMALL)
    {
        // compute rotation axis -- this guarantees unity norm
        Vector3s axis = _delta_theta / angle;

        // express delta_theta as a quaternion using the angle-axis helper
        dq = AngleAxis<Scalar>(angle, axis);

    }
    else // Consider small angle approx
    {
        dq.w() = 1;
        dq.vec() = _delta_theta/2;
        dq.normalize();
    }

    // result as a quaternion
    // the delta is in local reference: q * dq
    _q_plus_delta_theta = (Map<const Quaternions>(_q.data()) * dq).coeffs();

    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_GLOBAL>::plus(const Eigen::Map<const Eigen::VectorXs>& _q,
                                                           const Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                                                           Eigen::Map<Eigen::VectorXs>& _q_plus_delta_theta) const
{

    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_delta_theta.size() == local_size_ && "Wrong size of input delta_theta.");
    assert(_q_plus_delta_theta.size() == global_size_ && "Wrong size of output quaternion.");

    assert(fabs(1.0 - _q.norm()) < Constants::EPS && "Quaternion not normalized.");

    using namespace Eigen;

    double angle = _delta_theta.norm();
    Quaternions dq;
    if (angle > Constants::EPS_SMALL)
    {
        // compute rotation axis -- this guarantees unity norm
        Vector3s axis = _delta_theta / angle;

        // express delta_theta as a quaternion using the angle-axis helper
        dq = AngleAxis<Scalar>(angle, axis);

    }
    else // Consider small angle approx
    {
        dq.w() = 1;
        dq.vec() = _delta_theta/2;
        dq.normalize();
    }

    // result as a quaternion
    // the delta is in global reference: dq * q
    _q_plus_delta_theta = (dq * Map<const Quaternions>(_q.data())).coeffs();

    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_LOCAL>::computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _q,
                                                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    Eigen::Vector4s qq = _q/2;
    _jacobian <<  qq(3), -qq(2),  qq(1),
                  qq(2),  qq(3), -qq(0),
                 -qq(1),  qq(0),  qq(3),
                 -qq(0), -qq(1), -qq(2) ;

    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_GLOBAL>::computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _q,
                                                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    Eigen::Vector4s qq = _q/2;
    _jacobian <<  qq(3),  qq(2), -qq(1),
                 -qq(2),  qq(3),  qq(0),
                  qq(1), -qq(0),  qq(3),
                 -qq(0), -qq(1), -qq(2);

    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_LOCAL>::minus(const Eigen::Map<const Eigen::VectorXs>& _q1,
                   const Eigen::Map<const Eigen::VectorXs>& _q2,
                   Eigen::Map<Eigen::VectorXs>& _q2_minus_q1)
{
    assert(_q1.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_q2.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_q2_minus_q1.size() == local_size_ && "Wrong size of output quaternion difference.");

    using Eigen::Quaternions;
    _q2_minus_q1 = log_q(Quaternions(_q1.data()).conjugate() * Quaternions(_q2.data()));

    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_GLOBAL>::minus(const Eigen::Map<const Eigen::VectorXs>& _q1,
                   const Eigen::Map<const Eigen::VectorXs>& _q2,
                   Eigen::Map<Eigen::VectorXs>& _q2_minus_q1)
{
    assert(_q1.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_q2.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_q2_minus_q1.size() == local_size_ && "Wrong size of output quaternion difference.");

    using Eigen::Quaternions;
    _q2_minus_q1 = log_q(Quaternions(_q2.data()) * Quaternions(_q1.data()).conjugate());

    return true;
}

} // namespace wolf
