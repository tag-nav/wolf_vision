/**
 * \file rotations.h
 *
 *  Created on: Sep 6, 2016
 *      \author: jsola
 */

#ifndef ROTATIONS_H_
#define ROTATIONS_H_

#include "wolf.h"

namespace wolf
{

//////////////////////////////////////////////////////////////
template<typename T>
inline T pi2pi(T angle)
{
    return (angle > (T)0 ?
            fmod(angle + (T)M_PI, (T)(2*M_PI)) - (T)M_PI :
            fmod(angle - (T)M_PI, (T)(2*M_PI)) + (T)M_PI);
}

template<typename T>
inline T toRad(const T& deg)
{
    return (T)M_TORAD * deg;
}

template<typename T>
inline T toDeg(const T& rad)
{
    return (T)M_TODEG * rad;
}

//////////////////////////////////////////////////////////////////
// Operators

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived>& _v)
{

    MatrixSizeCheck<3, 1>::check(_v);
    typedef typename Derived::Scalar T;

    return (Eigen::Matrix<T, 3, 3>() << 0.0, -_v(2), +_v(1), +_v(2), 0.0, -_v(0), -_v(1), +_v(0), 0.0).finished();
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> vee(const Eigen::MatrixBase<Derived>& _m)
{

    MatrixSizeCheck<3, 3>::check(_m);
    typedef typename Derived::Scalar T;

    return (Eigen::Matrix<T, 3, 1>() << _m(2, 1), _m(0, 2), _m(1, 0)).finished();
}

///////////////////////////////////////////////////////////////
// Rotation conversions - exp and log maps

template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> exp_q(const Eigen::MatrixBase<Derived>& _v)
{

    MatrixSizeCheck<3, 1>::check(_v);
    typedef typename Derived::Scalar T;

    Eigen::Quaternion<T> q;
    T angle = _v.norm();
    T angle_half = angle / (T)2.0;
    if (angle > wolf::Constants::EPS)
    {
        q.w() = cos(angle_half);
        q.vec() = _v / angle * sin(angle_half);
        return q;
    }
    else
    {
        q.w() = cos(angle_half);
        q.vec() = _v * ((T)0.5 - angle_half * angle_half / (T)12.0); // see the Taylor series of sinc(x) ~ 1 - x^2/3!, and have q.vec = v/2 * sinc(angle_half)
        return q;
    }
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> log_q(const Eigen::QuaternionBase<Derived>& _q)
{
    typedef typename Derived::Scalar T;
    Eigen::Matrix<T, 3, 1> vec = _q.vec();
    T vecnorm = vec.norm();
    if (vecnorm > wolf::Constants::EPS_SMALL)
    { // regular angle-axis conversion
        T angle = (T)2.0 * atan2(vecnorm, _q.w());
        return vec * angle / vecnorm;
    }
    else
    { // small-angle approximation using truncated Taylor series
        T r2 = vec.squaredNorm() / (_q.w() *_q.w());
        return vec * ( (T)2.0 -  r2 / (T)1.5 ) / _q.w(); // log = 2 * vec * ( 1 - norm(vec)^2 / 3*w^2 ) / w.
    }
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> exp_R(const Eigen::MatrixBase<Derived>& _v)
{

    MatrixSizeCheck<3, 1>::check(_v);
    typedef typename Derived::Scalar T;

    T angle = _v.norm();
    if (angle < wolf::Constants::EPS)
        return Eigen::Matrix<T, 3, 3>::Identity() + skew(_v);
    else
        return Eigen::AngleAxis<T>(angle, _v / angle).matrix();
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> log_R(const Eigen::MatrixBase<Derived>& _R)
{

    MatrixSizeCheck<3, 3>::check(_R);
    typedef typename Derived::Scalar T;

    Eigen::AngleAxis<T> aa = Eigen::AngleAxis<T>(_R);
    return aa.axis() * aa.angle();
}

template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> v2q(const Eigen::MatrixBase<Derived>& _v)
{
    return exp_q(_v);
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> q2v(const Eigen::QuaternionBase<Derived>& _q)
{
    return log_q(_q);
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> v2R(const Eigen::MatrixBase<Derived>& _v)
{
    return exp_R(_v);
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> R2v(const Eigen::MatrixBase<Derived>& _R)
{
    return log_R(_R);
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> q2R(const Eigen::QuaternionBase<Derived>& _q)
{
    return _q.matrix();
}

template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> R2q(const Eigen::MatrixBase<Derived>& _R)
{
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

    MatrixSizeCheck<3, 1>::check(_theta);
    typedef typename Derived::Scalar T;

    T theta2 = _theta.dot(_theta);
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

    MatrixSizeCheck<3, 1>::check(_theta);
    typedef typename Derived::Scalar T;

    T theta2 = _theta.dot(_theta);
    Eigen::Matrix<T, 3, 3> W(skew(_theta));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W; // Small angle approximation
    T theta = std::sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> M;
    M.noalias() = ((T)1 / theta2 - (1 + cos(theta)) / ((T)2 * theta * sin(theta))) * (W * W);
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

    MatrixSizeCheck<3, 1>::check(_theta);
    typedef typename Derived::Scalar T;

    T theta2 = _theta.dot(_theta);
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

    MatrixSizeCheck<3, 1>::check(_theta);
    typedef typename Derived::Scalar T;

    T theta2 = _theta.dot(_theta);
    Eigen::Matrix<T, 3, 3> W(skew(_theta));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W; // Small angle approximation
    T theta = std::sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> M;
    M.noalias() = ((T)1 / theta2 - (1 + cos(theta)) / ((T)2 * theta * sin(theta))) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() - (T)0.5 * W + M; //is this really more optimized?
}

} // namespace wolf

#endif /* ROTATIONS_H_ */
