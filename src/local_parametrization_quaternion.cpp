#include "local_parametrization_quaternion.h"

LocalParametrizationQuaternion::LocalParametrizationQuaternion(bool _delta_theta_in_global_frame) :
        LocalParametrizationBase(4, 3),
        global_delta_(_delta_theta_in_global_frame)
{
}

LocalParametrizationQuaternion::~LocalParametrizationQuaternion()
{
}

bool LocalParametrizationQuaternion::plus(const Eigen::Map<Eigen::VectorXs>& _q,
                                          const Eigen::Map<Eigen::VectorXs>& _delta_theta,
                                          Eigen::Map<Eigen::VectorXs>& _q_plus_delta_theta) const
{

    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_delta_theta.size() == local_size_ && "Wrong size of input delta_theta.");
    assert(_q_plus_delta_theta.size() == global_size_ && "Wrong size of output quaternion.");

    assert(abs(1.0 - _q.norm()) < WolfConstants::EPS && "Quaternion not normalized.");

    using namespace Eigen;

    double angle = _delta_theta.norm();
    if (angle > WolfConstants::EPS)
    {
        // compute rotation axis -- this guarantees unity norm
        Vector3s axis = _delta_theta / angle;

        // express delta_theta as a quaternion using the angle-axis helper
        Quaternions dq(AngleAxis<WolfScalar>(angle, axis));

        // result as a quaternion
        if (global_delta_)
            // the delta is in global reference: dq * q
            _q_plus_delta_theta = (dq * Map<const Quaternions>(&_q(0))).coeffs();
        else
            // the delta is in local reference: q * dq
            _q_plus_delta_theta = (Map<const Quaternions>(&_q(0)) * dq).coeffs();

    }
    else // Consider null rotation
    {
        _q_plus_delta_theta = _q;
    }
    return true;
}

bool LocalParametrizationQuaternion::computeJacobian(const Eigen::Map<Eigen::VectorXs>& _q,
                                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    using namespace Eigen;
    if (global_delta_) // See comments in method plus()
    {
        _jacobian << -_q(0), -_q(1), -_q(2),
                      _q(3),  _q(2), -_q(1),
                     -_q(2),  _q(3),  _q(0),
                      _q(1), -_q(0),  _q(3);
        _jacobian /= 2;
    }
    else
    {
        _jacobian << -_q(0), -_q(1), -_q(2),
                      _q(3), -_q(2),  _q(1),
                      _q(2),  _q(3), -_q(0),
                     -_q(1),  _q(0),  _q(3);
        _jacobian /= 2;
    }
    return true;
}
