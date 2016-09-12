/**
 * \file rotations.h
 *
 *  Created on: Sep 6, 2016
 *      \author: jsola
 */

#ifndef ROTATIONS_H_
#define ROTATIONS_H_

#include "wolf.h"

namespace wolf{

template<typename T>
inline T pi2pi(const T& angle)
{
    return (angle > (T)0 ? fmod(angle + (T)M_PI, (T)(2 * M_PI)) - (T)M_PI : fmod(angle - (T)M_PI, (T)(2 * M_PI)) + (T)M_PI);
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& _v) {
    return (Eigen::Matrix<T, 3, 3>() <<
         0.0  , -_v(2), +_v(1),
        +_v(2),  0.0  , -_v(0),
        -_v(1), +_v(0),  0.0  ).finished();
}

template<typename T>
inline Eigen::Matrix<T, 3, 1> vee(const Eigen::Matrix<T, 3, 3>& _m)
{
    return (Eigen::Matrix<T, 3, 1>() << _m(2,1), _m(0,2), _m(1,0)).finished();
}



template<typename T, int Rows>
inline Eigen::Quaternion<T> v2q(Eigen::Matrix<T, Rows, 1>& _v){

    Eigen::Quaternion<T> q;
    T angle = _v.norm();
    T angle_half = angle/2.0;
    if (angle > wolf::Constants::EPS)
    {
        q.w() = cos(angle_half);
        q.vec() = _v / angle * sin(angle_half);
        return q;
    }
    else
    {
        q.w() = cos(angle_half);
        q.vec() = _v * ( (T)0.5 - angle_half*angle_half/(T)12.0 ); // see the Taylor series of sinc(x) ~ 1 - x^2/3!, and have q.vec = v/2 * sinc(angle_half)
        return q;
    }
}
//template<typename Derived>
//inline Eigen::Quaternion<typename Derived::Scalar> v2q(Eigen::MatrixBase<Derived> _v){
//
//    Eigen::Quaternion<typename Derived::Scalar> q;
//    typename Derived::Scalar angle = _v.norm();
//    typename Derived::Scalar angle_half = angle/2.0;
//    if (angle > wolf::Constants::EPS)
//    {
//        q.w() = cos(angle_half);
//        q.vec() = _v / angle * sin(angle_half);
//        return q;
//    }
//    else
//    {
//        q.w() = cos(angle_half);
//        q.vec() = _v * ( (typename Derived::Scalar)0.5 - angle_half*angle_half/(typename Derived::Scalar)12.0 ); // see the Taylor series of sinc(x) ~ 1 - x^2/3!, and have q.vec = v/2 * sinc(angle_half)
//        return q;
//    }
//}


inline Eigen::Quaternions v2q(const Eigen::Vector3s& _v){
    wolf::Scalar angle = _v.norm();
    if (angle > wolf::Constants::EPS)
        return Eigen::Quaternions(Eigen::AngleAxiss(angle, _v/angle));
    else
    {
        Eigen::Quaternions q;
        q.w() = cos(angle/2.0);
        q.vec() = _v * ( 0.5 - angle*angle/48.0 ); // see the Taylor series of sinc(x) ~ 1 - x^2/3!, and have q.vec = v/2 * sinc(angle_half)
        return q;
    }
}

inline void q2v(const Eigen::Quaternions& _q, Eigen::Vector3s& _v){
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_q);
    _v = aa.axis() * aa.angle();
}

template<typename T>
inline Eigen::Matrix<T, 3, 1> q2v(const Eigen::Quaternion<T>& _q){
    Eigen::Matrix<T,3,1> vec = _q.vec();
    T vecnorm = vec.norm();
    if (vecnorm > wolf::Constants::EPS)
    { // regular angle-axis conversion
        T angle = atan2(vecnorm,_q.w());
        return vec * angle / vecnorm;
    }
    else
    { // small-angle approximation using truncated Taylor series
        T r = vecnorm / _q.w();
        return vec * ( (T)1.0 - r*r ) / _q.w();
    }
}


inline Eigen::VectorXs q2v(const Eigen::Quaternions& _q){
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_q);
    return aa.axis() * aa.angle();
}

template<typename T, int Rows>
inline Eigen::Matrix<T, 3, 3> v2R(const Eigen::Matrix<T, Rows, 1>& _v){
    T angle = _v.norm();
    if (angle < wolf::Constants::EPS)
        return Eigen::Matrix<T, 3, 3>::Identity() + skew(_v);
    else
        return Eigen::AngleAxis<T>(angle, _v/angle).matrix();
}

inline Eigen::Matrix3s v2R(const Eigen::Vector3s& _v){
    wolf::Scalar angle = _v.norm();
    if (angle < wolf::Constants::EPS)
        return Eigen::Matrix3s::Identity() + skew(_v);
    else
        return Eigen::AngleAxiss(angle, _v/angle).matrix();
}

template<typename T, int Rows, int Cols>
inline Eigen::Matrix<T, 3, 1> R2v(const Eigen::Matrix<T, Rows, Cols>& _R){
    Eigen::AngleAxis<T> aa = Eigen::AngleAxis<T>(_R);
    return aa.axis() * aa.angle();
}

inline Eigen::Vector3s R2v(const Eigen::Matrix3s& _R){
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_R);
    return aa.axis() * aa.angle();
}


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
template<typename T, int Rows>
inline Eigen::Matrix<T, 3, 3> expMapDerivative(const Eigen::Matrix<T, Rows, 1>& _omega)
{

    T theta2 = _omega.dot(_omega);
    Eigen::Matrix<T, 3, 3> W(skew<T>(_omega));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() - (T)0.5 * W; // Small angle approximation
    T theta = sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> m1, m2;
    m1.noalias() = ((T)1 - cos(theta)) / theta2 * W;
    m2.noalias() = (theta - sin(theta)) / (theta2 * theta) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() + m1 + m2;
}


/** \brief Compute Jrinv (inverse of Right Jacobian which corresponds to the jacobian of log)
 *  Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
 *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *      logmap( Rhat * expmap( omega ) ) \approx logmap( Rhat ) + Jrinv *  omega   (1) original write with omega
 *      logmap( Rhat * expmap(d_omega) ) \approx logmap( Rhat ) + Jrinv * d_omega  (1) adapted write with d_omega (JS)
 *  where Jrinv = logMapDerivative(omega);
 *
 *  This maps a perturbation on the manifold (expmap(omega)) to a perturbation in the tangent space (Jrinv * omega) so that
 *
 *      log(exp(omega)*exp(d_omega)) = omega + Jrinv(omega)*d_omega
 *
 *  or, having R = exp(omega),
 *
 *      log(R*exp(d_omega)) = log(R) + Jrinv(omega)*d_omega ??? FIXME: this does not fit with the comment above (1)
 *                                                                     where it states Jrinv(d_omega) and not Jrinv(omega)
 *                                                                     (in the original form, omega is the argument of the function)
 *
 *  Who's correct? Let's see:
 *
 *  fix: Taking the log on both sides, and assuming Rhat = exp(omega), we observe
 *
 *      exp(omega)*exp(d_omega) = exp(omega+Jrinv(onega)*d_omega)
 *
 *  If we define dw as
 *
 *      dw = Jrinv(omega)*d_omega <==> d_omega=Jr*dw, where Jrinv = Jr^-1, logically.
 *
 *  then substituting above we get:
 *
 *      exp(omega+dw) = exp(omega)*exp(Jr*dw)
 *
 *  exactly as in expMapDerivative().
 *
 *  This makes sense: in fact Chirikjian describes Jrinv(omega) and not Jrinv(d_omega).
 *  And also, as d_omega is small, we'd have Jrinv(d_omega) \approx Identity all the time,
 *  whereas Jrinv(omega) is in the general case far from Identity.
 */
template<typename T, int Rows>
inline Eigen::Matrix<T, 3, 3> logMapDerivative(const Eigen::Matrix<T, Rows, 1>& _omega)
{

    T theta2 = _omega.dot(_omega);
    Eigen::Matrix<T, 3, 3> W(skew(_omega));
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W; // Small angle approximation
    T theta = std::sqrt(theta2);  // rotation angle
    Eigen::Matrix<T, 3, 3> m1;
    m1.noalias() = ((T)1 / theta2 - (1 + cos(theta)) / ((T)2 * theta * sin(theta))) * (W * W);
    return Eigen::Matrix<T, 3, 3>::Identity() + (T)0.5 * W + m1; //is this really more optimized?
}


} // namespace wolf


#endif /* ROTATIONS_H_ */
