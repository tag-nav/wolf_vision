/*
 * \file local_parametrization_homogeneous.cpp
 *
 *  Created on: 24/02/2016
 *      Author: jsola
 */

#include "local_parametrization_homogeneous.h"
#include "iostream"
#include "rotations.h" // we use quaternion algebra here

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

    _h_plus_delta = ( exp_q(_delta) * Quaternions(_h.data()) ).coeffs();

    return true;
}

bool LocalParametrizationHomogeneous::computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _h,
                                                      Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_h.size() == global_size_ && "Wrong size of input quaternion.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    Eigen::Vector4s hh = _h/2;
    _jacobian <<  hh(3),  hh(2), -hh(1),
                 -hh(2),  hh(3),  hh(0),
                  hh(1), -hh(0),  hh(3),
                 -hh(0), -hh(1), -hh(2) ;
    return true;
}

bool LocalParametrizationHomogeneous::minus(const Eigen::Map<const Eigen::VectorXs>& _h1,
                                            const Eigen::Map<const Eigen::VectorXs>& _h2,
                                            Eigen::Map<Eigen::VectorXs>& _h2_minus_h1)
{
    using Eigen::Quaternions;
    _h2_minus_h1 = log_q(Quaternions(_h2.data()) * Quaternions(_h1.data()).conjugate());
    return true;
}

} // namespace wolf

