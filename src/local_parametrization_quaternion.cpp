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

    using namespace Eigen;
    double angle = _delta_theta.norm();
    if (angle > 1e-8) // TODO: put a EPSILON in wolf.h
    {
        // compute rotation axis -- this guarantees unity norm
        Vector3s axis = _delta_theta / angle;
        // express delta_theta as a quaternion
        Quaternions dq(AngleAxis<WolfScalar>(angle, axis));
        // result as a quaternion
        Quaternions qout;
        if (global_delta_)
            // the delta is in global reference
            qout = dq * Map<const Quaternions>(&_q(0));
        else
            // the delta is in local reference
            qout = Map<const Quaternions>(&_q(0)) * dq;
        // result as a vector map
        _q_plus_delta_theta = qout.coeffs();
    }
    else
    {
        _q_plus_delta_theta = _q;
    }
    return true;
}

bool LocalParametrizationQuaternion::computeJacobian(const Eigen::Map<Eigen::VectorXs>& _q,
                                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert( _jacobian.rows() == 4 && _jacobian.cols() == 3 &&"wrong size of Jacobian matrix.");
    using namespace Eigen;
    if (global_delta_) // See comments in method plus()
    {
        _jacobian << -_q(0), -_q(1), -_q(2),
                      _q(3),  _q(2), -_q(1),
                     -_q(2),  _q(3),  _q(0),
                      _q(1), -_q(0),  _q(3);
    }
    else
    {
        _jacobian << -_q(0), -_q(1), -_q(2),
                      _q(3), -_q(2),  _q(1),
                      _q(2),  _q(3), -_q(0),
                     -_q(1),  _q(0),  _q(3);
    }
    return true;
}
