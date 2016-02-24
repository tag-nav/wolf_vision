/*
 * LocalParametrizationHomogeneous.cpp
 *
 *  Created on: 24/02/2016
 *      Author: jsola
 */

#include "LocalParametrizationHomogeneous.h"

LocalParametrizationHomogeneous::LocalParametrizationHomogeneous()
{
    // TODO Auto-generated constructor stub

}

LocalParametrizationHomogeneous::~LocalParametrizationHomogeneous()
{
    // TODO Auto-generated destructor stub
}

bool LocalParametrizationHomogeneous::plus(const Eigen::Map<Eigen::VectorXs>& _h,
                                           const Eigen::Map<Eigen::VectorXs>& _delta,
                                           Eigen::Map<Eigen::VectorXs>& _h_plus_delta) const
{
    assert(_h.size() == global_size_ && "Wrong size of input homogeneous point.");
    assert(_delta.size() == local_size_ && "Wrong size of input delta.");
    assert(_h_plus_delta.size() == global_size_ && "Wrong size of output homogeneous point.");

    using namespace Eigen;

    double norm_delta = _delta.norm();
    if (norm_delta > WolfConstants::EPS)
    {
        // compute rotation axis -- this guarantees unity norm
        Vector3s axis = _delta / norm_delta;

        // express delta as a quaternion
        Quaternions dq(AngleAxis<WolfScalar>(norm_delta, axis));

        // result as a homogeneous point -- we use the quaternion product for keeping in the 4-sphere
        _h_plus_delta = (dq * Map<const Quaternions>(&_h(0))).coeffs();
    }
    else
    {
        _h_plus_delta = _h;
    }
    return true;
}

bool LocalParametrizationHomogeneous::computeJacobian(const Eigen::Map<Eigen::VectorXs>& _h,
                                                      Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
}
