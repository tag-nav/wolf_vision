/*
 * \file local_parametrization_homogeneous.cpp
 *
 *  Created on: 24/02/2016
 *      Author: jsola
 */

#include "local_parametrization_homogeneous.h"
#include "iostream"

namespace wolf {

LocalParametrizationHomogeneous::LocalParametrizationHomogeneous() :
        LocalParametrizationBase(4, 3)
{
    //
}

LocalParametrizationHomogeneous::~LocalParametrizationHomogeneous()
{
    //
}

bool LocalParametrizationHomogeneous::plus(const Eigen::Map<const Eigen::VectorXs>& _h,
                                           const Eigen::Map<const Eigen::VectorXs>& _delta,
                                           Eigen::Map<Eigen::VectorXs>& _h_plus_delta) const
{
    assert(_h.size() == global_size_ && "Wrong size of input homogeneous point.");
    assert(_delta.size() == local_size_ && "Wrong size of input delta.");
    assert(_h_plus_delta.size() == global_size_ && "Wrong size of output homogeneous point.");

    using namespace Eigen;

    double norm_delta = _delta.norm();
    if (norm_delta > Constants::EPS)
    {
        // compute rotation axis -- this guarantees unity norm
        Vector3s axis = _delta / norm_delta;

        // express delta as a quaternion -- this is exp(delta)
        Quaternions dq(AngleAxis<Scalar>(norm_delta, axis));

        // result as a homogeneous point -- we use the quaternion product for keeping in the 4-sphere
        _h_plus_delta = (dq * Map<const Quaternions>(_h.data())).coeffs();
    }
    else
    {
        _h_plus_delta = _h;
    }
    return true;
}

bool LocalParametrizationHomogeneous::computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _h,
                                                      Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_h.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    _jacobian << -_h(0), -_h(1), -_h(2),
                  _h(3),  _h(2), -_h(1),
                 -_h(2),  _h(3),  _h(0),
                  _h(1), -_h(0),  _h(3);
    _jacobian /= 2;
    return true;
}

} // namespace wolf
