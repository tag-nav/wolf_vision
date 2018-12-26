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

/** \brief Return angle between -pi and pi
 *
 * @param angle
 * @return formatted angle
 */
template<typename T>
inline T
pi2pi(const T angle)
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
inline T
toRad(const T deg)
{
    return (T)M_TORAD * deg;
}

/** \brief Conversion to degrees
 *
 * @param rad angle in radians
 * @return angle in degrees
 */
template<typename T>
inline T
toDeg(const T rad)
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
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
skew(const Eigen::MatrixBase<Derived>& _v)
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
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
vee(const Eigen::MatrixBase<Derived>& _m)
{
    MatrixSizeCheck<3,3>::check(_m);

    typedef typename Derived::Scalar T;

    return (Eigen::Matrix<T, 3, 1>() << _m(2, 1), _m(0, 2), _m(1, 0)).finished();
}


////////////////////////////////////////////////////
// Rotation conversions q, R, Euler, back and forth
//
//  Euler angles are denoted 'e' and are in the ZYX convention

/** \brief quaternion to rotation matrix conversion
 *
 * @param _q a right-handed unit quaternion
 * @return the equivalent rotation matrix
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
q2R(const Eigen::QuaternionBase<Derived>& _q)
{
    return _q.matrix();
}

/** \brief quaternion to rotation matrix conversion
 *
 * @param _q a right-handed unit quaternion
 * @return the equivalent rotation matrix
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
q2R(const Eigen::MatrixBase<Derived>& _q)
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
inline Eigen::Quaternion<typename Derived::Scalar>
R2q(const Eigen::MatrixBase<Derived>& _R)
{
    MatrixSizeCheck<3,3>::check(_R);

    return Eigen::Quaternion<typename Derived::Scalar>(_R);
}

/** \brief Euler angles to quaternion
 * \param _euler = (roll pitch yaw) in ZYX convention
 * \return equivalent quaternion
 */
template<typename D>
inline Eigen::Quaternion<typename D::Scalar>
e2q(const Eigen::MatrixBase<D>& _euler)
{
    MatrixSizeCheck<3,1>::check(_euler);

    typedef typename D::Scalar T;

    const Eigen::AngleAxis<T> ax = Eigen::AngleAxis<T>(_euler(0), Eigen::Matrix<T, 3, 1>::UnitX());
    const Eigen::AngleAxis<T> ay = Eigen::AngleAxis<T>(_euler(1), Eigen::Matrix<T, 3, 1>::UnitY());
    const Eigen::AngleAxis<T> az = Eigen::AngleAxis<T>(_euler(2), Eigen::Matrix<T, 3, 1>::UnitZ());

    return Eigen::Quaternion<T>(az * ay * ax);
}


/** \brief Euler angles to rotation matrix
 * \param _e = (roll pitch yaw) in ZYX convention
 * \return equivalent rotation matrix
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
e2R(const Eigen::MatrixBase<Derived>& _e)
{
    MatrixSizeCheck<3, 1>::check(_e);

    return e2q(_e).matrix();
}


template <typename Derived>
inline typename Eigen::MatrixBase<Derived>::Scalar
getYaw(const Eigen::MatrixBase<Derived>& R)
{
    MatrixSizeCheck<3, 3>::check(R);

    using std::atan2;
    return atan2( R(1, 0), R(0, 0) );
}

template<typename Derived>
inline typename Eigen::Matrix<typename Derived::Scalar, 3, 1>
R2e(const Eigen::MatrixBase<Derived>& _R)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 1> e;

    e(0) = atan2( _R(2,1), _R(2,2));
    e(1) = atan2(-_R(2,0), sqrt(_R(0,0)*_R(0,0) + _R(1,0)*_R(1,0)));
    e(2) = atan2( _R(1,0), _R(0,0));

    return e;
}


template<typename Derived>
inline typename Eigen::Matrix<typename Derived::Scalar, 3, 1>
q2e(const Eigen::MatrixBase<Derived>& _q)
{
    typedef typename Derived::Scalar T;
    Eigen::Matrix<T, 3, 1> e;

    Scalar w  = _q(3);
    Scalar x  = _q(0);
    Scalar y  = _q(1);
    Scalar z  = _q(2);

    Scalar xx = x*x;
    Scalar xy = x*y;
    Scalar xz = x*z;
    Scalar yy = y*y;
    Scalar yz = y*z;
    Scalar zz = z*z;
    Scalar wx = w*x;
    Scalar wy = w*y;
    Scalar wz = w*z;

    Scalar r00 = T(1) - T(2) * ( yy + zz );
    Scalar r10 =        T(2) * ( xy + wz );
    Scalar r20 =        T(2) * ( xz - wy );
    Scalar r21 =        T(2) * ( yz + wx );
    Scalar r22 = T(1) - T(2) * ( xx + yy );

    e(0) = atan2( r21, r22);
    e(1) = atan2(-r20, sqrt(r00*r00 + r10*r10));
    e(2) = atan2( r10, r00);

    return e;
}


template<typename Derived>
inline typename Eigen::Matrix<typename Derived::Scalar, 3, 1>
q2e(const Eigen::QuaternionBase<Derived>& _q)
{
    return q2e(_q.coeffs());
}




///////////////////////////////////////////////////////////////
// Rotation conversions - exp and log maps

/** \brief Quaternion exponential map
 *
 * @param _v a rotation vector with _v.norm() the rotated angle and _v.normalized() the rotation axis.
 * @return the right-handed unit quaternion performing the rotation encoded by _v
 */
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar>
exp_q(const Eigen::MatrixBase<Derived>& _v)
{
    using std::sqrt;
    using std::cos;
    using std::sin;

    MatrixSizeCheck<3,1>::check(_v);

    typedef typename Derived::Scalar T;

    T angle_squared = _v.squaredNorm();
    T angle         = sqrt(angle_squared); // Allow ceres::Jet to use its own sqrt() version.

    if (angle > (T)(wolf::Constants::EPS_SMALL))
    {
        return Eigen::Quaternion<T> ( Eigen::AngleAxis<T>(angle, _v.normalized()) );
    }
    else
    {
        return Eigen::Quaternion<T> ( (T)1.0 , _v(0,0)/(T)2 , _v(1,0)/(T)2 , _v(2,0)/(T)2 );
    }
}

/** \brief Quaternion logarithmic map
 *
 * @param _q a unit right-handed quaternion
 * @return a rotation vector v such that _q = exp_q(v)
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
log_q(const Eigen::QuaternionBase<Derived>& _q)
{

    // Will try this implementation once Eigen accepts it!
    // see https://forum.kde.org/viewtopic.php?f=74&t=143269&p=385299#p385265
    //    typedef typename Derived::Scalar T;
    //    Eigen::AngleAxis<T> aa(_q);
    //    return aa.angle() * aa.axis();


    // In the meanwhile, we have a custom implementation as follows

    typedef typename Derived::Scalar T;

    Eigen::Matrix<T, 3, 1> vec = _q.vec();
    const T sin_angle_squared = vec.squaredNorm();
    if (sin_angle_squared > (T)wolf::Constants::EPS_SMALL)
    {
        const T  sin_angle = sqrt(sin_angle_squared); // Allow ceres::Jet to use its own sqrt() version.
        const T& cos_angle = _q.w();

        /* If (cos_angle < 0) then angle >= pi/2 , means : angle for angle_axis vector >= pi (== 2*angle)
                    |-> results in correct rotation but not a normalized angle_axis vector

        In that case we observe that 2 * angle ~ 2 * angle - 2 * pi,
        which is equivalent saying

            angle - pi = atan(sin(angle - pi), cos(angle - pi))
                       = atan(-sin(angle), -cos(angle))
        */
        const T two_angle = T(2.0) * ((cos_angle < T(0.0)) ? atan2(-sin_angle, -cos_angle) : atan2(sin_angle, cos_angle));
        const T k = two_angle / sin_angle;
        return vec * k;
    }
    else
    {
        // small-angle approximation
        return vec * (T)2.0;
    }
}

/** \brief Rotation matrix exponential map
 *
 * @param _v a rotation vector
 * @return the rotation matrix performing the rotation encoded by _v
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
exp_R(const Eigen::MatrixBase<Derived>& _v)
{
    using std::sqrt;

    MatrixSizeCheck<3, 1>::check(_v);

    typedef typename Derived::Scalar T;

    T angle_squared = _v.squaredNorm();
    T angle = sqrt(angle_squared); // Allow ceres::Jet to use its own sqrt() version.

    if (angle > wolf::Constants::EPS_SMALL)
        return Eigen::AngleAxis<T>(angle, _v.normalized()).toRotationMatrix();
    else
        return Eigen::Matrix<T, 3, 3>::Identity() + skew(_v);
}

/** \brief Rotation matrix logarithmic map
 *
 * @param _R a 3D rotation matrix
 * @return the rotation vector v such that _R = exp_R(v)
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
log_R(const Eigen::MatrixBase<Derived>& _R)
{
    return log_q(R2q(_R));
}

/** \brief Rotation vector to quaternion conversion
 *
 * @param _v a rotation vector
 * @return the equivalent right-handed unit quaternion
 */
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar>
v2q(const Eigen::MatrixBase<Derived>& _v)
{
    return exp_q(_v);
}

/** \brief Quaternion to rotation vector conversion
 *
 * @param _q a right-handed unit quaternion
 * @return the equivalent rotation vector
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
q2v(const Eigen::QuaternionBase<Derived>& _q)
{
    return log_q(_q);
}

/** \brief Rotation vector to rotation matrix conversion
 *
 * @param _v a rotation vector
 * @return the equivalent rotation matrix
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
v2R(const Eigen::MatrixBase<Derived>& _v)
{
    return exp_R(_v);
}

/** \brief Rotation matrix to rotation vector conversion
 *
 * @param _R a rotation matrix
 * @return the equivalent rotation vector
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
R2v(const Eigen::MatrixBase<Derived>& _R)
{
    return log_R(_R);
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
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
jac_SO3_right(const Eigen::MatrixBase<Derived>& _theta)
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
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
jac_SO3_right_inv(const Eigen::MatrixBase<Derived>& _theta)
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
 *      exp(theta+d_theta) = exp(Jl(theta)*d_theta)*exp(theta)
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
jac_SO3_left(const Eigen::MatrixBase<Derived>& _theta)
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
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
jac_SO3_left_inv(const Eigen::MatrixBase<Derived>& _theta)
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
inline void
compose(const Eigen::QuaternionBase<D1>& _q1,
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
inline void
between(const Eigen::QuaternionBase<D1>& _q1,
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
inline Eigen::Quaternion<typename D1::Scalar>
plus_right(const Eigen::QuaternionBase<D1>& q, const Eigen::MatrixBase<D2>& v)
{
    MatrixSizeCheck<3,1>::check(v);
    return q * exp_q(v);
}

template<typename D1, typename D2>
inline  Eigen::Matrix<typename D2::Scalar, 3, 1>
minus_right(const Eigen::QuaternionBase<D1>& q1, const Eigen::QuaternionBase<D2>& q2)
{
    return log_q(q1.conjugate() * q2);
}

template<typename D1, typename D2>
inline Eigen::Quaternion<typename D1::Scalar>
plus_left(const Eigen::MatrixBase<D2>& v, const Eigen::QuaternionBase<D1>& q)
{
    MatrixSizeCheck<3,1>::check(v);
    return exp_q(v) * q;
}

template<typename D1, typename D2>
inline  Eigen::Matrix<typename D2::Scalar, 3, 1>
minus_left(const Eigen::QuaternionBase<D1>& q1, const Eigen::QuaternionBase<D2>& q2)
{
    return log_q(q2 * q1.conjugate());
}

template<typename D1, typename D2>
inline Eigen::Quaternion<typename D1::Scalar>
plus(const Eigen::QuaternionBase<D1>& q, const Eigen::MatrixBase<D2>& v)
{
    return plus_right(q, v);
}

template<typename D1, typename D2>
inline  Eigen::Matrix<typename D2::Scalar, 3, 1>
minus(const Eigen::QuaternionBase<D1>& q1, const Eigen::QuaternionBase<D2>& q2)
{
    return minus_right(q1, q2);
}

template<typename D1, typename D2>
inline  Eigen::Matrix<typename D2::Scalar, 3, 1>
diff(const Eigen::QuaternionBase<D1>& q1, const Eigen::QuaternionBase<D2>& q2)
{
    return minus(q1, q2);
}


} // namespace wolf

#endif /* ROTATIONS_H_ */
