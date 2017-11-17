/**
 * \file rotations.h
 *
 *  Created on: Sep 6, 2016
 *      \author: jsola
 */

#ifndef ROTATIONS_H_
#define ROTATIONS_H_

#include "wolf.h"

#include "iostream"

namespace wolf
{

//////////////////////////////////////////////////////////////

/** \brief Return angle between -pi and pi
 *
 * @param angle
 * @return formatted angle
 */
template<typename T>
inline T pi2pi(const T angle)
{
    using std::fmod;

    return (angle > (T)0 ?
            fmod(angle + (T)M_PI, (T)(2*M_PI)) - (T)M_PI :
            fmod(angle - (T)M_PI, (T)(2*M_PI)) + (T)M_PI);
}

/** \brief Conversion to radians
 *
 * @param deg angle in degrees
 * @return angle in radians
 */
template<typename T>
inline T toRad(const T deg)
{
    return (T)M_TORAD * deg;
}

/** \brief Conversion to degrees
 *
 * @param rad angle in radians
 * @return angle in degrees
 */
template<typename T>
inline T toDeg(const T rad)
{
    return (T)M_TODEG * rad;
}

//////////////////////////////////////////////////////////////////
// Operators skew and vee

/** \brief Skew symmetric matrix
 *
 * @param _v a 3D vector
 * @return the skew-symmetric matrix V so that V*u = _v.cross(u), for u in R^3
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived>& _v)
{
    MatrixSizeCheck<3,1>::check(_v);

    typedef typename Derived::Scalar T;

    Eigen::Matrix<T, 3, 3> sk;

    sk << (T)0.0 , -_v(2), +_v(1),
           +_v(2), (T)0.0, -_v(0),
           -_v(1), +_v(0), (T)0.0;

    return sk;
}

/** \brief Inverse of skew symmetric matrix
 *
 * @param _m A skew-symmetric matrix
 * @return a 3-vector v such that skew(v) = _m
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> vee(const Eigen::MatrixBase<Derived>& _m)
{
    MatrixSizeCheck<3,3>::check(_m);

    typedef typename Derived::Scalar T;

    return (Eigen::Matrix<T, 3, 1>() << _m(2, 1), _m(0, 2), _m(1, 0)).finished();
}

///////////////////////////////////////////////////////////////
// Rotation conversions - exp and log maps

/** \brief Quaternion exponential map
 *
 * @param _v a rotation vector with _v.norm() the rotated angle and _v.normalized() the rotation axis.
 * @return the right-handed unit quaternion performing the rotation encoded by _v
 */
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> exp_q(const Eigen::MatrixBase<Derived>& _v)
{
    using std::sqrt;
    using std::cos;
    using std::sin;

    MatrixSizeCheck<3,1>::check(_v);

    typedef typename Derived::Scalar T;

    T angle_squared = _v.squaredNorm();
    T angle         = sqrt(angle_squared);
    T angle_half    = angle / (T)2.0;

    Eigen::Quaternion<T> q;
    if (angle > (T)(wolf::Constants::EPS))
    {
        q.w()   = cos(angle_half);
        q.vec() = sin(angle_half) * _v.normalized();// / angle;
    }
    else
    {
        q.w()   = (T)1.0 - angle_squared/(T)2; // Taylor expansion of cos(x) = 1 - x^2/2!;
        q.vec() = _v * ((T)2.0 - angle_squared / (T)48.0); // Taylor series of sinc(x) ~ 1 - x^2/3!, and have q.vec = v/2 * sinc(angle_half)
    }
    return q.normalized();
}

/** \brief Quaternion logarithmic map
 *
 * @param _q a unit right-handed quaternion
 * @return a rotation vector v such that _q = exp_q(v)
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> log_q(const Eigen::QuaternionBase<Derived>& _q)
{
    using std::sqrt;

    typedef typename Derived::Scalar T;

    Eigen::Matrix<T, 3, 1> vec = _q.vec();
    T vecnorm_squared = vec.squaredNorm();
    T vecnorm = sqrt(vecnorm_squared); // vec.norm();
    if (vecnorm > (T)wolf::Constants::EPS_SMALL)
    { // regular angle-axis conversion
        T angle = (T)2.0 * atan2(vecnorm, _q.w());
        return vec * angle / vecnorm;
    }
    else
    { // small-angle approximation using truncated Taylor series
        T r2 = vecnorm_squared / (_q.w() *_q.w());
        return vec * ( (T)2.0 -  r2 / (T)1.5 ) / _q.w(); // log = 2 * vec * ( 1 - norm(vec)^2 / 3*w^2 ) / w.
    }
}

/** \brief Rotation matrix exponential map
 *
 * @param _v a rotation vector
 * @return the rotation matrix performing the rotation encoded by _v
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> exp_R(const Eigen::MatrixBase<Derived>& _v)
{
    using std::sqrt;

    MatrixSizeCheck<3, 1>::check(_v);

    typedef typename Derived::Scalar T;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> R;

    T angle_squared = _v.squaredNorm();
    T angle = sqrt(angle_squared);

    if (angle > wolf::Constants::EPS)
        R = Eigen::AngleAxis<T>(angle, _v.normalized()).toRotationMatrix();
    else
        R = Eigen::Matrix<T, 3, 3>::Identity() + skew(_v);

    return R;
}

/** \brief Rotation matrix logarithmic map
 *
 * @param _R a 3D rotation matrix
 * @return the rotation vector v such that _R = exp_R(v)
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> log_R(const Eigen::MatrixBase<Derived>& _R)
{
    MatrixSizeCheck<3, 3>::check(_R);

    typedef typename Derived::Scalar T;

    Eigen::AngleAxis<T> aa = Eigen::AngleAxis<T>(_R);
    return aa.axis() * aa.angle();
}

/** \brief Rotation vector to quaternion conversion
 *
 * @param _v a rotation vector
 * @return the equivalent right-handed unit quaternion
 */
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> v2q(const Eigen::MatrixBase<Derived>& _v)
{
    return exp_q(_v);
}

/** \brief Quaternion to rotation vector conversion
 *
 * @param _q a right-handed unit quaternion
 * @return the equivalent rotation vector
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> q2v(const Eigen::QuaternionBase<Derived>& _q)
{
    return log_q(_q);
}

/** \brief Rotation vector to rotation matrix conversion
 *
 * @param _v a rotation vector
 * @return the equivalent rotation matrix
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> v2R(const Eigen::MatrixBase<Derived>& _v)
{
    return exp_R(_v);
}

/** \brief Rotation matrix to rotation vector conversion
 *
 * @param _R a rotation matrix
 * @return the equivalent rotation vector
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> R2v(const Eigen::MatrixBase<Derived>& _R)
{
    return log_R(_R);
}

/** \brief quaternion to rotation matrix conversion
 *
 * @param _q a right-handed unit quaternion
 * @return the equivalent rotation matrix
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> q2R(const Eigen::QuaternionBase<Derived>& _q)
{
    return _q.matrix();
}

/** \brief quaternion to rotation matrix conversion
 *
 * @param _q a right-handed unit quaternion
 * @return the equivalent rotation matrix
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> q2R(const Eigen::MatrixBase<Derived>& _q)
{
    MatrixSizeCheck<4,1>::check(_q);
    Eigen::Quaternion<typename Derived::Scalar> q(_q(3),_q(0),_q(1),_q(2));
    return q2R( q );
}

/** \brief rotation matrix to quaternion conversion
 *
 * @param _R a rotation matrix
 * @return the equivalent right-handed unit quaternion
 */
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> R2q(const Eigen::MatrixBase<Derived>& _R)
{
    MatrixSizeCheck<3,3>::check(_R);

    return Eigen::Quaternion<typename Derived::Scalar>(_R);
}

/////////////////////////////////////////////////////////////////
// Jacobians of SO(3)

/** \brief Compute Jr (Right Jacobian)
 * Right Jacobian for exp map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *
 *      expmap( theta + d_theta ) \approx expmap(theta) * expmap(Jr * d_theta)
 *
 *  where Jr = jac_SO3_right(theta);
 *
 *  This maps a perturbation in the tangent space (d_theta) to a perturbation on the manifold (expmap(Jr * d_theta))
 *  so that:
 *
 *      exp(theta+d_theta) = exp(theta)*exp(Jr(theta)*d_theta)
 */

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> jac_SO3_right(const Eigen::MatrixBase<Derived>& _theta)
{
    using std::sqrt;
    using std::cos;
    using std::sin;

    MatrixSizeCheck<3, 1>::check(_theta);

    typedef typename Derived::Scalar T;

    T theta2 = _theta.squaredNorm();
    Eigen::Matrix<T, 3, 3> W(skew(_theta));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() - (T)0.5 * W; // Small angle approximation
    T theta = sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> M1, M2;
    M1.noalias() = ((T)1 - cos(theta)) / theta2 * W;
    M2.noalias() = (theta - sin(theta)) / (theta2 * theta) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() - M1 + M2;
}

/** \brief Compute Jrinv (inverse of Right Jacobian which corresponds to the jacobian of log)
 *  Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *
 *      logmap( R * expmap(d_theta) ) \approx logmap( R ) + Jrinv * d_theta
 *      logmap( q * expmap(d_theta) ) \approx logmap( q ) + Jrinv * d_theta
 *
 *  where Jrinv = jac_SO3_right_inv(theta);
 *
 *  This maps a perturbation on the manifold (expmap(theta)) to a perturbation in the tangent space (Jrinv * theta) so that
 *
 *      log( exp(theta) * exp(d_theta) ) = theta + Jrinv(theta) * d_theta
 *
 *  or, having R = exp(theta),
 *
 *      log( R * exp(d_theta) ) = log(R) + Jrinv(theta) * d_theta
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> jac_SO3_right_inv(const Eigen::MatrixBase<Derived>& _theta)
{
    using std::sqrt;
    using std::cos;
    using std::sin;

    MatrixSizeCheck<3, 1>::check(_theta);

    typedef typename Derived::Scalar T;

    T theta2 = _theta.squaredNorm();
    Eigen::Matrix<T, 3, 3> W(skew(_theta));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W; // Small angle approximation
    T theta = sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> M;
    M.noalias() = ((T)1.0 / theta2 - ((T)1.0 + cos(theta)) / ((T)2.0 * theta * sin(theta))) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W + M; //is this really more optimized?
}

/** \brief Compute Jl (Left Jacobian)
 * Left Jacobian for exp map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *
 *      expmap( theta + d_theta ) \approx expmap(Jl * d_theta) * expmap(theta)
 *
 *  where Jl = jac_SO3_left(theta);
 *
 *  This maps a perturbation in the tangent space (d_theta) to a perturbation on the manifold (expmap(Jl * d_theta))
 *  so that:
 *
 *      exp(theta+d_theta) = exp(Jr(theta)*d_theta)*exp(theta)
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> jac_SO3_left(const Eigen::MatrixBase<Derived>& _theta)
{
    using std::sqrt;
    using std::cos;
    using std::sin;

    MatrixSizeCheck<3, 1>::check(_theta);

    typedef typename Derived::Scalar T;

    T theta2 = _theta.squaredNorm();
    Eigen::Matrix<T, 3, 3> W(skew(_theta));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() - (T)0.5 * W; // Small angle approximation
    T theta = sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> M1, M2;
    M1.noalias() = ((T)1 - cos(theta)) / theta2 * W;
    M2.noalias() = (theta - sin(theta)) / (theta2 * theta) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() + M1 + M2;
}

/** \brief Compute Jl_inv (inverse of Left Jacobian which corresponds to the jacobian of log)
 *  Left Jacobian for Log map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *
 *      logmap( expmap(d_theta) * R ) \approx logmap( R ) + Jlinv * d_theta
 *
 *  where Jlinv = jac_SO3_left_inv(theta);
 *
 *  This maps a perturbation on the manifold (expmap(theta)) to a perturbation in the tangent space (Jlinv * theta) so that
 *
 *      log( exp(d_theta) * exp(theta) ) = theta + Jlinv(theta) * d_theta
 *
 *  or, having R = exp(theta),
 *
 *      log( exp(d_theta) * R ) = log(R) + Jlinv(theta) * d_theta
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> jac_SO3_left_inv(const Eigen::MatrixBase<Derived>& _theta)
{
    using std::sqrt;
    using std::cos;
    using std::sin;

    MatrixSizeCheck<3, 1>::check(_theta);

    typedef typename Derived::Scalar T;

    T theta2 = _theta.squaredNorm();
    Eigen::Matrix<T, 3, 3> W(skew(_theta));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W; // Small angle approximation
    T theta = sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> M;
    M.noalias() = ((T)1.0 / theta2 - ((T)1.0 + cos(theta)) / ((T)2.0 * theta * sin(theta))) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() - (T)0.5 * W + M; //is this really more optimized?
}

template<typename D1, typename D2, typename D3, typename D4, typename D5>
inline void compose(const Eigen::QuaternionBase<D1>& _q1,
                    const Eigen::QuaternionBase<D2>& _q2,
                    Eigen::QuaternionBase<D3>& _q_comp,
                    Eigen::MatrixBase<D4>& _J_comp_q1,
                    Eigen::MatrixBase<D5>& _J_comp_q2)
{
    MatrixSizeCheck<3, 3>::check(_J_comp_q1);
    MatrixSizeCheck<3, 3>::check(_J_comp_q2);

    _q_comp = _q1 * _q2;

    _J_comp_q1 = q2R(_q2.conjugate()); //  R2.tr
    _J_comp_q2 . setIdentity();
}

template<typename D1, typename D2, typename D3, typename D4, typename D5>
inline void between(const Eigen::QuaternionBase<D1>& _q1,
                    const Eigen::QuaternionBase<D2>& _q2,
                    Eigen::QuaternionBase<D3>& _q_between,
                    Eigen::MatrixBase<D4>& _J_between_q1,
                    Eigen::MatrixBase<D5>& _J_between_q2)
{
    MatrixSizeCheck<3, 3>::check(_J_between_q1);
    MatrixSizeCheck<3, 3>::check(_J_between_q2);

    _q_between = _q1.conjugate() * _q2;

    _J_between_q1 = -q2R(_q2.conjugate()*_q1); // - R2.tr * R1
    _J_between_q2 . setIdentity();
}

template<typename D1, typename D2>
inline Eigen::Quaternion<typename D1::Scalar> plus(const Eigen::QuaternionBase<D1>& q, const Eigen::MatrixBase<D2>& v)
{
    MatrixSizeCheck<3,1>::check(v);
    return q * exp_q(v);
}

template<typename D1, typename D2>
inline  Eigen::Matrix<typename D2::Scalar, 3, 1> minus(const Eigen::QuaternionBase<D1>& q1, const Eigen::QuaternionBase<D2>& q2)
{
    return log_q(q1.conjugate() * q2);
}

template<typename D1, typename D2>
inline  Eigen::Matrix<typename D2::Scalar, 3, 1> diff(const Eigen::QuaternionBase<D1>& q1, const Eigen::QuaternionBase<D2>& q2)
{
    return minus(q1, q2);
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> matrixRollPitchYaw(const T roll,
                                                 const T pitch,
                                                 const T yaw)
{
    const Eigen::AngleAxis<T> ax = Eigen::AngleAxis<T>(roll,  Eigen::Matrix<T, 3, 1>::UnitX());
    const Eigen::AngleAxis<T> ay = Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY());
    const Eigen::AngleAxis<T> az = Eigen::AngleAxis<T>(yaw,   Eigen::Matrix<T, 3, 1>::UnitZ());

    return (az * ay * ax).toRotationMatrix().matrix();
}

template <typename Derived>
inline typename Eigen::MatrixBase<Derived>::Scalar
getYaw(const Eigen::MatrixBase<Derived>& R)
{
    MatrixSizeCheck<3, 3>::check(R);

    using std::atan2;
    return atan2( R(1, 0), R(0, 0) );
}

} // namespace wolf

#endif /* ROTATIONS_H_ */
