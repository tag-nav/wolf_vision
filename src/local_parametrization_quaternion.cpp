
#include "local_parametrization_quaternion.h"
#include "rotations.h"

#include <iostream>
namespace wolf {

////////// LOCAL PERTURBATION //////////////

template <>
bool LocalParametrizationQuaternion<wolf::DQ_LOCAL>::plus(Eigen::Map<const Eigen::VectorXs>& _q,
                                                          Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                                                          Eigen::Map<Eigen::VectorXs>& _q_plus_delta_theta) const
{

    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_delta_theta.size() == local_size_ && "Wrong size of input delta_theta.");
    assert(_q_plus_delta_theta.size() == global_size_ && "Wrong size of output quaternion.");

    assert(fabs(1.0 - _q.norm()) < Constants::EPS && "Quaternion not normalized.");

    using namespace Eigen;

    _q_plus_delta_theta = ( Quaternions(_q.data()) * exp_q(_delta_theta) ).coeffs();

    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_LOCAL>::computeJacobian(Eigen::Map<const Eigen::VectorXs>& _q,
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
bool LocalParametrizationQuaternion<wolf::DQ_LOCAL>::minus(Eigen::Map<const Eigen::VectorXs>& _q1,
                                                           Eigen::Map<const Eigen::VectorXs>& _q2,
                                                           Eigen::Map<Eigen::VectorXs>& _q2_minus_q1)
{
    assert(_q1.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_q2.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_q2_minus_q1.size() == local_size_ && "Wrong size of output quaternion difference.");

    using Eigen::Quaternions;
    _q2_minus_q1 = log_q(Quaternions(_q1.data()).conjugate() * Quaternions(_q2.data()));

    return true;
}

//template <>
//bool LocalParametrizationQuaternion<wolf::DQ_LOCAL>::minus(const Eigen::VectorXs& _q1,
//                                                           const Eigen::VectorXs& _q2,
//                                                           Eigen::VectorXs& _q2_minus_q1)
//{
//    assert(_q1.size() == global_size_ && "Wrong size of input quaternion.");
//    assert(_q2.size() == global_size_ && "Wrong size of input quaternion.");
//    assert(_q2_minus_q1.size() == local_size_ && "Wrong size of output quaternion difference.");
//
//    using Eigen::Quaternions;
//    _q2_minus_q1 = log_q(Quaternions(_q1.data()).conjugate() * Quaternions(_q2.data()));
//
//    return true;
//}

////////// GLOBAL PERTURBATION //////////////

template <>
bool LocalParametrizationQuaternion<wolf::DQ_GLOBAL>::plus(Eigen::Map<const Eigen::VectorXs>& _q,
                                                           Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                                                           Eigen::Map<Eigen::VectorXs>& _q_plus_delta_theta) const
{

    assert(_q.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_delta_theta.size() == local_size_ && "Wrong size of input delta_theta.");
    assert(_q_plus_delta_theta.size() == global_size_ && "Wrong size of output quaternion.");

    assert(fabs(1.0 - _q.norm()) < Constants::EPS && "Quaternion not normalized.");

    using namespace Eigen;

    _q_plus_delta_theta = ( exp_q(_delta_theta) * Quaternions(_q.data()) ).coeffs();

    return true;
}

template <>
bool LocalParametrizationQuaternion<wolf::DQ_GLOBAL>::computeJacobian(Eigen::Map<const Eigen::VectorXs>& _q,
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
bool LocalParametrizationQuaternion<wolf::DQ_GLOBAL>::minus(Eigen::Map<const Eigen::VectorXs>& _q1,
                                                            Eigen::Map<const Eigen::VectorXs>& _q2,
                                                            Eigen::Map<Eigen::VectorXs>& _q2_minus_q1)
{
    assert(_q1.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_q2.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_q2_minus_q1.size() == local_size_ && "Wrong size of output quaternion difference.");

    using Eigen::Quaternions;
    _q2_minus_q1 = log_q(Quaternions(_q2.data()) * Quaternions(_q1.data()).conjugate());

    return true;
}

//template <>
//bool LocalParametrizationQuaternion<wolf::DQ_GLOBAL>::minus(const Eigen::VectorXs& _q1,
//                                                            const Eigen::VectorXs& _q2,
//                                                            Eigen::VectorXs& _q2_minus_q1)
//{
//    assert(_q1.size() == global_size_ && "Wrong size of input quaternion.");
//    assert(_q2.size() == global_size_ && "Wrong size of input quaternion.");
//    assert(_q2_minus_q1.size() == local_size_ && "Wrong size of output quaternion difference.");
//
//    using Eigen::Quaternions;
//    _q2_minus_q1 = log_q(Quaternions(_q2.data()) * Quaternions(_q1.data()).conjugate());
//
//    return true;
//}



} // namespace wolf
