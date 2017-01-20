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
// Simple angular functions
template<typename T>
inline T pi2pi(const T& angle)
{
    return (angle > (T)0 ? fmod(angle + (T)M_PI, (T)(2 * M_PI)) - (T)M_PI :
            fmod(angle - (T)M_PI, (T)(2 * M_PI)) + (T)M_PI);
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
// Rotation conversions

template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> q2v_original(const Eigen::MatrixBase<Derived>& _v)
{

    MatrixSizeCheck<3, 1>::check(_v);
    typedef typename Derived::Scalar T;

    Eigen::Quaternion<T> q;
    const T& a0 = _v[0];
    const T& a1 = _v[1];
    const T& a2 = _v[2];
    const T angle0 = a0 * a0 + a1 * a1 + a2 * a2;
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
inline Eigen::Quaternion<typename Derived::Scalar> v2q(const Eigen::MatrixBase<Derived>& _v)
{
    MatrixSizeCheck<3, 1>::check(_v);
    typedef typename Derived::Scalar T;

    Eigen::Quaternion<T> q;
    const T& a0 = _v[0];
    const T& a1 = _v[1];
    const T& a2 = _v[2];
    const T angle_square = a0 * a0 + a1 * a1 + a2 * a2;

    //We need the angle : means we have to take the square root of angle_square, 
    // which is defined for all angle_square beonging to R+ (except 0)
    if (angle_square > (T)0.0 ){
        //sqrt is defined here
        const T angle = sqrt(angle_square);
        const T angle_half = angle / (T)2.0;
        
        q.w() = cos(angle_half);
        q.vec() = _v / angle * sin(angle_half);
        return q;
    }
    else
    {
        //sqrt not defined at 0 and will produce NaNs, thuswe use an approximation with taylor series truncated at one term
        q.w() = (T)1.0;
        q.vec() = _v *(T)0.5; // see the Taylor series of sinc(x) ~ 1 - x^2/3!, and have q.vec = v/2 * sinc(angle_half)
                                                                    //                                 for angle_half == 0 then ,     = v/2
        return q;
    }
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> q2v_original(const Eigen::QuaternionBase<Derived>& _q)
{
    typedef typename Derived::Scalar T;
    Eigen::Matrix<T, 3, 1> vec = _q.vec();
    T vecnorm = vec.norm();
    if (vecnorm > wolf::Constants::EPS_SMALL)
    { // regular angle-axis conversion
        T angle = atan2(vecnorm, _q.w());
        return vec * angle / vecnorm;
    }
    else
    { // small-angle approximation using truncated Taylor series
        T r2 = vec.squaredNorm() / (_q.w() *_q.w());
        return vec * ( (T)2.0 -  r2 / (T)1.5 ) / _q.w(); // log = 2 * vec * ( 1 - norm(vec)^2 / 3*w^2 ) / w.
    }
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> q2v(const Eigen::QuaternionBase<Derived>& _q)
{
    typedef typename Derived::Scalar T;
    Eigen::Matrix<T, 3, 1> vec = _q.vec();
    const T sin_angle_square = vec(0) * vec(0) + vec(1) * vec(1) + vec(2) * vec(2);

    //everything shouold be OK for non-zero rotations
    if (sin_angle_square > (T)0.0)
    {
        const T sin_angle = sqrt(sin_angle_square);
        const T& cos_angle = _q.w();

        /* If (cos_theta < 0) then theta >= pi/2 , means : angle for angle_axis vector >= pi (== 2*theta) 
                    |-> results in correct rotation but not a normalized angle_axis vector 
    
        In that case we observe that 2 * theta ~ 2 * theta - 2 * pi,
        which is equivalent saying
    
            theta - pi = atan(sin(theta - pi), cos(theta - pi))
                        = atan(-sin(theta), -cos(theta))
        */
        const T two_angle = T(2.0) * ((cos_angle < 0.0) ? atan2(-sin_angle, -cos_angle) : atan2(sin_angle, cos_angle));
        const T k = two_angle / sin_angle;
        return vec * k;
    }
    else
    { // small-angle approximation using truncated Taylor series
        //zero rotation --> sqrt will result in NaN
        return vec * (T)2.0; // log = 2 * vec * ( 1 - norm(vec)^2 / 3*w^2 ) / w.
    }
}

inline Eigen::VectorXs q2v(const Eigen::Quaternions& _q)
{
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_q);
    return aa.axis() * aa.angle();
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> v2R(const Eigen::MatrixBase<Derived>& _v)
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
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> R2v(const Eigen::MatrixBase<Derived>& _R)
{

    MatrixSizeCheck<3, 3>::check(_R);
    typedef typename Derived::Scalar T;

    Eigen::AngleAxis<T> aa = Eigen::AngleAxis<T>(_R);
    return aa.axis() * aa.angle();
}

/////////////////////////////////////////////////////////////////
// Jacobians of SO(3)

/** \brief Compute Jr (Right Jacobian)
 * Right Jacobian for exp map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *      expmap( omega + d_omega ) \approx expmap(omega) * expmap(Jr * d_omega)
 *  where Jr = expMapDerivative(omega);
 *  This maps a perturbation in the tangent space (d_omega) to a perturbation on the manifold (expmap(Jr * d_omega))
 *  so that:
 *
 *      exp(omega+d_omega) = exp(omega)*exp(Jr(omega)*d_omega)
 */

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> jac_SO3_right(const Eigen::MatrixBase<Derived>& _omega)
{

    MatrixSizeCheck<3, 1>::check(_omega);
    typedef typename Derived::Scalar T;

    T theta2 = _omega.dot(_omega);
    Eigen::Matrix<T, 3, 3> W(skew(_omega));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() - (T)0.5 * W; // Small angle approximation
    T theta = sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> m1, m2;
    m1.noalias() = ((T)1 - cos(theta)) / theta2 * W;
    m2.noalias() = (theta - sin(theta)) / (theta2 * theta) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() - m1 + m2;
}

/** \brief Compute Jrinv (inverse of Right Jacobian which corresponds to the jacobian of log)
 *  Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *      logmap( Rhat * expmap(d_omega) ) \approx logmap( Rhat ) + Jrinv * d_omega
 *  where Jrinv = logMapDerivative(omega);
 *
 *  This maps a perturbation on the manifold (expmap(omega)) to a perturbation in the tangent space (Jrinv * omega) so that
 *
 *      log( exp(omega) * exp(d_omega) ) = omega + Jrinv(omega) * d_omega
 *
 *  or, having R = exp(omega),
 *
 *      log( R * exp(d_omega) ) = log(R) + Jrinv(omega) * d_omega
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> jac_SO3_right_inv(const Eigen::MatrixBase<Derived>& _omega)
{

    MatrixSizeCheck<3, 1>::check(_omega);
    typedef typename Derived::Scalar T;

    T theta2 = _omega.dot(_omega);
    Eigen::Matrix<T, 3, 3> W(skew(_omega));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W; // Small angle approximation
    T theta = std::sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> m2;
    m2.noalias() = ((T)1 / theta2 - (1 + cos(theta)) / ((T)2 * theta * sin(theta))) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W + m2; //is this really more optimized?
}

/** \brief Compute Jl (Left Jacobian)
 * Left Jacobian for exp map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *      expmap( omega + d_omega ) \approx expmap(Jl * d_omega) * expmap(omega)
 *  where Jl = jac_SO3_left(omega);
 *  This maps a perturbation in the tangent space (d_omega) to a perturbation on the manifold (expmap(Jl * d_omega))
 *  so that:
 *
 *      exp(omega+d_omega) = exp(Jr(omega)*d_omega)*exp(omega)
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> jac_SO3_left(const Eigen::MatrixBase<Derived>& _omega)
{

    MatrixSizeCheck<3, 1>::check(_omega);
    typedef typename Derived::Scalar T;

    T theta2 = _omega.dot(_omega);
    Eigen::Matrix<T, 3, 3> W(skew(_omega));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() - (T)0.5 * W; // Small angle approximation
    T theta = sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> m1, m2;
    m1.noalias() = ((T)1 - cos(theta)) / theta2 * W;
    m2.noalias() = (theta - sin(theta)) / (theta2 * theta) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() + m1 + m2;
}

/** \brief Compute Jl_inv (inverse of Left Jacobian which corresponds to the jacobian of log)
 *  Left Jacobian for Log map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *      logmap( expmap(d_omega) * Rhat ) \approx logmap( Rhat ) + Jlinv * d_omega
 *  where Jlinv = jac_SO3_left_inv(omega);
 *
 *  This maps a perturbation on the manifold (expmap(omega)) to a perturbation in the tangent space (Jlinv * omega) so that
 *
 *      log( exp(d_omega) * exp(omega) ) = omega + Jlinv(omega) * d_omega
 *
 *  or, having R = exp(omega),
 *
 *      log( exp(d_omega) * R ) = log(R) + Jlinv(omega) * d_omega
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> jac_SO3_left_inv(const Eigen::MatrixBase<Derived>& _omega)
{

    MatrixSizeCheck<3, 1>::check(_omega);
    typedef typename Derived::Scalar T;

    T theta2 = _omega.dot(_omega);
    Eigen::Matrix<T, 3, 3> W(skew(_omega));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W; // Small angle approximation
    T theta = std::sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> m2;
    m2.noalias() = ((T)1 / theta2 - (1 + cos(theta)) / ((T)2 * theta * sin(theta))) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() - (T)0.5 * W + m2; //is this really more optimized?
}

} // namespace wolf

#endif /* ROTATIONS_H_ */
