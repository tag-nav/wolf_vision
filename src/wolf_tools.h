/**
 * \file wolf_tools.h
 *
 *  Created on: 14/08/2014
 *     \author: jsola
 */

#ifndef WOLF_TOOLS_H_
#define WOLF_TOOLS_H_

#include "wolf.h"

//TODO see where we collect all these functions
/**
 * \brief Namespace for diverse helper functions within Wolf.
 *
 * Namespace for diverse helper functions within Wolf.
 *
 * All these functions should be somewhere appropriate.
 * Then, a good namespace for the project would be Wolf, but for everything in Wolf and not just helper functions.
 */
namespace Wolf
{

/**
 * \brief rho-theta line specification from homogeneous specification
 * \param _hmglin a homogeneous line
 * \return the rho-theta line (rho, theta)
 */
template<class Vin>
Eigen::Vector2s lineRhothetaFromHmg(const Vin & _hmglin)
{
    assert(_hmglin.size() == 3);

    WolfScalar r = _hmglin.segment(0, 2).norm();
    WolfScalar ct = _hmglin(0) / r;
    WolfScalar st = _hmglin(1) / r;
    WolfScalar rho = -_hmglin(2) / r;
    WolfScalar theta = atan2(st, ct);
    Eigen::Vector2s rhotheta(2);
    rhotheta(0) = rho;
    rhotheta(1) = theta;
    return rhotheta;
}

/**
 * \brief Build 3x3 skey symmetric matrix
 * \param v a 3-vector
 * \param V_x the skew-symmetric matrix. This is the return value.
 *
 * The skew-symmetric matrix is defined by
 *
 *     [v]_x = [ [   0 , -v(2),  v(1) ] [ v(2),    0 , -v(0) ] [-v(1),  v(0),    0  ] ]
 *
 */
template<class Vin, class Mout>
inline void skew(const Vin & v, Mout & V_x)
{
    assert(v.size() == 3);
    assert(V_x.rows() == 3 && V_x.cols() == 3);

    // helper: V_x =
    //     [ 0  -v2  v1 ]
    //     [ v2  0  -v0 ]
    //     [-v1  v0  0  ]

    V_x(0,0) = 0;
    V_x(0,1) = -v(2);
    V_x(0,2) = v(1);
    V_x(1,0) = v(2);
    V_x(1,1) = 0;
    V_x(1,2) = -v(0);
    V_x(2,0) = -v(1);
    V_x(2,1) = v(0);
    V_x(2,2) = 0;
//    V_x << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

/**
 * \brief Build 3x3 skey symmetric matrix
 * \param v a 3-vector
 * \return the skew-symmetric matrix
 *
 * The skew-symmetric matrix is defined by
 *
 *     [v]_x = [ [   0 , -v(2),  v(1) ] [ v(2),    0 , -v(0) ] [-v(1),  v(0),    0  ] ]
 *
 */
template<class Vin>
inline Eigen::Matrix3s skew(const Vin & v)
{
    assert(v.size() == 3);
    Eigen::Matrix3s V_x;
    skew(v, V_x);
    return V_x;
}

/**
 * \brief Signed distance from homogeneous 2d point to homogeneous 2d line.
 *
 * Signed distance from homogeneous 2d point to homogeneous 2d line. This is computed as
 * \f[ d = pix^\top * line / \sqrt{line_0^2 + line_1^2}\f]
 *
 * Note: Although we'll want to minimize absolute distance, the sign is necessary
 * to correctly generate signed Jacobians and guide the optimizer towards the correct direction.
 */
template<class P, class L>
inline WolfScalar distPointToLine(const P & _point, const L & _line)
{
    assert(_point.size() == 2);
    assert(_line.size() == 3);
    // Compute distance, see SolaIJCV-11: d = u' * l / sqrt(l_1^2+l_2^2)
    // this line below is equivalent to line.dot(point_homogeneous) / line.segment(0, 2).norm();
    return (_line(0) * _point(0) + _line(1) * _point(1) + _line(2)) / _line.segment(0, 2).norm();
}

/**
 * \brief compose frames
 * \param _base_pose base frame to compose. Acts as global wrt \a _local_pose.
 * \param _local_pose local frame to compose with. Acts as local wrt \a _base_pose.
 * \return a composed frame, \a _base_pose * \a _local_pose
 *
 * Composes a local frame on top of a base frame.
 */
template<class BaseType, class LocalType, class ResultType>
inline StatePose composeFrames(const BaseType& _base_pose, const LocalType& _local_pose)
{
      return  StatePose(7, _base_pose.q() * _local_pose.p() + _base_pose.p(), _base_pose.q() * _local_pose.q() );
}

/**
 * \brief compose frames
 * \param _base_pose base frame to compose. Acts as global wrt \a _local_pose.
 * \param _local_pose local frame to compose with. Acts as local wrt \a _base_pose.
 * \param _result_pose the composed frame, \a _base_pose * \a _local_pose. This is the return value.
 *
 * Composes a local frame on top of a base frame.
 */
template<class BaseType, class LocalType, class ResultType>
inline void composeFrames(const BaseType& _base_pose, const LocalType& _local_pose, ResultType& _result_pose)
{
    _result_pose.p() = _base_pose.q() * _local_pose.p() + _base_pose.p() ;
    _result_pose.q() = _base_pose.q() * _local_pose.q() ;
}

}

#endif /* WOLF_TOOLS_H_ */
