#include "local_parametrization_quaternion.h"

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
    if (angle > Constants::EPS)
    {
        // compute rotation axis -- this guarantees unity norm
        Vector3s axis = _delta_theta / angle;

        // express delta_theta as a quaternion using the angle-axis helper
        Quaternions dq(AngleAxis<Scalar>(angle, axis));

        // result as a quaternion
        // the delta is in local reference: q * dq
        _q_plus_delta_theta = (Map<const Quaternions>(_q.data()) * dq).coeffs();

    }
    else // Consider null rotation
    {
        _q_plus_delta_theta = _q;
    }
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
    if (angle > Constants::EPS)
    {
        // compute rotation axis -- this guarantees unity norm
        Vector3s axis = _delta_theta / angle;

        // express delta_theta as a quaternion using the angle-axis helper
        Quaternions dq(AngleAxis<Scalar>(angle, axis));

        // result as a quaternion
        // the delta is in global reference: dq * q
        _q_plus_delta_theta = (dq * Map<const Quaternions>(_q.data())).coeffs();
    }
    else // Consider null rotation
    {
        _q_plus_delta_theta = _q;
    }
    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_LOCAL>::computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _q,
                                                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    using namespace Eigen;
    _jacobian << -_q(0), -_q(1), -_q(2),
                  _q(3), -_q(2),  _q(1),
                  _q(2),  _q(3), -_q(0),
                 -_q(1),  _q(0),  _q(3);
    _jacobian /= 2;

    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_GLOBAL>::computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _q,
                                                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    using namespace Eigen;
    _jacobian << -_q(0), -_q(1), -_q(2),
                  _q(3),  _q(2), -_q(1),
                 -_q(2),  _q(3),  _q(0),
                  _q(1), -_q(0),  _q(3);
    _jacobian /= 2;

    return true;
}

} // namespace wolf
