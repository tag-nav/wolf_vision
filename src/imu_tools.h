/*
 * imu_tools.h
 *
 *  Created on: Jul 29, 2017
 *      Author: jsola
 */

#ifndef IMU_TOOLS_H_
#define IMU_TOOLS_H_


#include "wolf.h"
#include "rotations.h"

namespace wolf 
{
namespace imu {
using namespace Eigen;

template<typename D1, typename D2, typename D3>
inline void identity(MatrixBase<D1>& p, QuaternionBase<D2>& q, MatrixBase<D3>& v)
{
    p = MatrixBase<D1>::Zero(3,1);
    q = QuaternionBase<D2>::Identity();
    v = MatrixBase<D3>::Zero(3,1);
}

template<typename D1, typename D2, typename D3>
inline void identity(MatrixBase<D1>& p, MatrixBase<D2>& q, MatrixBase<D3>& v)
{
    typedef typename D1::Scalar T1;
    typedef typename D2::Scalar T2;
    typedef typename D3::Scalar T3;
    p << T1(0), T1(0), T1(0);
    q << T2(0), T2(0), T2(0), T2(1);
    v << T3(0), T3(0), T3(0);
}

template<typename T = wolf::Scalar>
inline Matrix<T, 10, 1> identity()
{
    Matrix<T, 10, 1> ret;
    ret<< T(0), T(0), T(0),
          T(0), T(0), T(0), T(1),
          T(0), T(0), T(0);
    return ret;
}

template<typename D1, typename D2, typename D3, typename D4, typename D5, typename D6, class T>
inline void inverse(const MatrixBase<D1>& dp, const QuaternionBase<D2>& dq, const MatrixBase<D3>& dv,
                    const T dt,
                    MatrixBase<D4>& idp, QuaternionBase<D5>& idq, MatrixBase<D6>& idv )
{
    MatrixSizeCheck<3, 1>::check(dp);
    MatrixSizeCheck<3, 1>::check(dv);
    MatrixSizeCheck<3, 1>::check(idp);
    MatrixSizeCheck<3, 1>::check(idv);

    idp = - ( dq.conjugate() * (dp - dv * typename D3::Scalar(dt) ) );
    idv = - ( dq.conjugate() * dv );
    idq =     dq.conjugate();
}

template<typename D1, typename D2, class T>
inline void inverse(const MatrixBase<D1>& d,
                    T dt,
                    MatrixBase<D2>& id)
{
    MatrixSizeCheck<10, 1>::check(d);
    MatrixSizeCheck<10, 1>::check(id);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp   ( & d( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     dq   ( & d( 3 ) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv   ( & d( 7 ) );
    Map<Matrix<typename D2::Scalar, 3, 1> >         idp  ( & id( 0 ) );
    Map<Quaternion<typename D2::Scalar> >           idq  ( & id( 3 ) );
    Map<Matrix<typename D2::Scalar, 3, 1> >         idv  ( & id( 7 ) );

    inverse(dp, dq, dv, dt, idp, idq, idv);
}

template<typename D, class T>
inline Matrix<typename D::Scalar, 10, 1> inverse(const MatrixBase<D>& d,
                                                 T dt)
{
    Matrix<typename D::Scalar, 10, 1> id;
    inverse(d, dt, id);
    return id;
}

template<typename D1, typename D2, typename D3, typename D4, typename D5, typename D6, typename D7, typename D8, typename D9, class T>
inline void compose(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1, const MatrixBase<D3>& dv1,
                    const MatrixBase<D4>& dp2, const QuaternionBase<D5>& dq2, const MatrixBase<D6>& dv2,
                    const T dt,
                    MatrixBase<D7>& sum_p, QuaternionBase<D8>& sum_q, MatrixBase<D9>& sum_v )
{
        MatrixSizeCheck<3, 1>::check(dp1);
        MatrixSizeCheck<3, 1>::check(dv1);
        MatrixSizeCheck<3, 1>::check(dp2);
        MatrixSizeCheck<3, 1>::check(dv2);
        MatrixSizeCheck<3, 1>::check(sum_p);
        MatrixSizeCheck<3, 1>::check(sum_v);

        sum_p = dp1 + dv1*dt + dq1*dp2;
        sum_v = dv1 +          dq1*dv2;
        sum_q =                dq1*dq2; // dq here to avoid possible aliasing between d1 and sum
}

template<typename D1, typename D2, typename D3, class T>
inline void compose(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    T dt,
                    MatrixBase<D3>& sum)
{
    MatrixSizeCheck<10, 1>::check(d1);
    MatrixSizeCheck<10, 1>::check(d2);
    MatrixSizeCheck<10, 1>::check(sum);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1( 3 ) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( & d1( 7 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2( 0 ) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( & d2( 3 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( & d2( 7 ) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_p  ( & sum( 0 ) );
    Map<Quaternion<typename D3::Scalar> >           sum_q  ( & sum( 3 ) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_v  ( & sum( 7 ) );

    compose(dp1, dq1, dv1, dp2, dq2, dv2, dt, sum_p, sum_q, sum_v);
}

template<typename D1, typename D2, class T>
inline Matrix<typename D1::Scalar, 10, 1> compose(const MatrixBase<D1>& d1,
                                                  const MatrixBase<D2>& d2,
                                                  T dt)
{
    Matrix<typename D1::Scalar, 10, 1>  ret;
    compose(d1, d2, dt, ret);
    return ret;
}

template<typename D1, typename D2, typename D3, typename D4, typename D5, class T>
inline void compose(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    T dt,
                    MatrixBase<D3>& sum,
                    MatrixBase<D4>& J_sum_d1,
                    MatrixBase<D5>& J_sum_d2)
{
    MatrixSizeCheck<10, 1>::check(d1);
    MatrixSizeCheck<10, 1>::check(d2);
    MatrixSizeCheck<10, 1>::check(sum);
    MatrixSizeCheck< 9, 9>::check(J_sum_d1);
    MatrixSizeCheck< 9, 9>::check(J_sum_d2);

    // Some useful temporaries
    Matrix<typename D1::Scalar, 3, 3> dR1 = q2R(d1.segment(3,4)); //dq1.matrix(); // First  Delta, DR
    Matrix<typename D2::Scalar, 3, 3> dR2 = q2R(d2.segment(3,4)); //dq2.matrix(); // Second delta, dR

    // Jac wrt first delta
    J_sum_d1.setIdentity();                                     // dDp'/dDp = dDv'/dDv = I
    J_sum_d1.block(0,3,3,3).noalias() = - dR1 * skew(d2.head(3)) ;     // dDp'/dDo
    J_sum_d1.block(0,6,3,3) = Matrix3s::Identity() * dt;        // dDp'/dDv = I*dt
    J_sum_d1.block(3,3,3,3) = dR2.transpose();                  // dDo'/dDo
    J_sum_d1.block(6,3,3,3).noalias() = - dR1 * skew(d2.tail(3)) ;     // dDv'/dDo

    // Jac wrt second delta
    J_sum_d2.setIdentity();                                     //
    J_sum_d2.block(0,0,3,3) = dR1;                              // dDp'/ddp
    J_sum_d2.block(6,6,3,3) = dR1;                              // dDv'/ddv
    // J_sum_d2.block(3,3,3,3) = Matrix3s::Identity();          // dDo'/ddo = I

    // compose deltas -- done here to avoid aliasing when calling with input `d1` and result `sum` referencing the same variable
    compose(d1, d2, dt, sum);
}

template<typename D1, typename D2, typename D3, typename D4, typename D5, typename D6, typename D7, typename D8, typename D9, class T>
inline void between(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1, const MatrixBase<D3>& dv1,
                    const MatrixBase<D4>& dp2, const QuaternionBase<D5>& dq2, const MatrixBase<D6>& dv2,
                    const T dt,
                    MatrixBase<D7>& diff_p, QuaternionBase<D8>& diff_q, MatrixBase<D9>& diff_v )
{
        MatrixSizeCheck<3, 1>::check(dp1);
        MatrixSizeCheck<3, 1>::check(dv1);
        MatrixSizeCheck<3, 1>::check(dp2);
        MatrixSizeCheck<3, 1>::check(dv2);
        MatrixSizeCheck<3, 1>::check(diff_p);
        MatrixSizeCheck<3, 1>::check(diff_v);

        diff_p = dq1.conjugate() * ( dp2 - dp1 - dv1*dt );
        diff_q = dq1.conjugate() *   dq2;
        diff_v = dq1.conjugate() * ( dv2 - dv1 );
}

template<typename D1, typename D2, typename D3, class T>
inline void between(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    T dt,
                    MatrixBase<D3>& d2_minus_d1)
{
    MatrixSizeCheck<10, 1>::check(d1);
    MatrixSizeCheck<10, 1>::check(d2);
    MatrixSizeCheck<10, 1>::check(d2_minus_d1);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1(3) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( & d1(7) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2(0) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( & d2(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( & d2(7) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_p ( & d2_minus_d1(0) );
    Map<Quaternion<typename D3::Scalar> >           diff_q ( & d2_minus_d1(3) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_v ( & d2_minus_d1(7) );

    between(dp1, dq1, dv1, dp2, dq2, dv2, dt, diff_p, diff_q, diff_v);
}


template<typename D1, typename D2, class T>
inline Matrix<typename D1::Scalar, 10, 1> between(const MatrixBase<D1>& d1,
                                                  const MatrixBase<D2>& d2,
                                                  T dt)
{
    Matrix<typename D1::Scalar, 10, 1> diff;
    between(d1, d2, dt, diff);
    return diff;
}

template<typename D1, typename D2, typename D3, class T>
inline void composeOverState(const MatrixBase<D1>& x,
                             const MatrixBase<D2>& d,
                             T dt,
                             MatrixBase<D3>& x_plus_d)
{
    MatrixSizeCheck<10, 1>::check(x);
    MatrixSizeCheck<10, 1>::check(d);
    MatrixSizeCheck<10, 1>::check(x_plus_d);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   p         ( & x( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     q         ( & x( 3 ) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   v         ( & x( 7 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp        ( & d( 0 ) );
    Map<const Quaternion<typename D2::Scalar> >     dq        ( & d( 3 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv        ( & d( 7 ) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         p_plus_d  ( & x_plus_d( 0 ) );
    Map<Quaternion<typename D3::Scalar> >           q_plus_d  ( & x_plus_d( 3 ) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         v_plus_d  ( & x_plus_d( 7 ) );

    p_plus_d = p + v*dt + 0.5*gravity()*dt*dt + q*dp;
    v_plus_d = v +            gravity()*dt    + q*dv;
    q_plus_d =                                  q*dq; // dq here to avoid possible aliasing between x and x_plus_d
}

template<typename D1, typename D2, class T>
inline Matrix<typename D1::Scalar, 10, 1> composeOverState(const MatrixBase<D1>& x,
                                                           const MatrixBase<D2>& d,
                                                           T dt)
{
    Matrix<typename D1::Scalar, 10, 1>  ret;
    composeOverState(x, d, dt, ret);
    return ret;
}

template<typename D1, typename D2, typename D3, typename D4, typename D5, typename D6, typename D7, typename D8, typename D9, class T>
inline void betweenStates(const MatrixBase<D1>& p1, const QuaternionBase<D2>& q1, const MatrixBase<D3>& v1,
                          const MatrixBase<D4>& p2, const QuaternionBase<D5>& q2, const MatrixBase<D6>& v2,
                          const T dt,
                          MatrixBase<D7>& dp, QuaternionBase<D8>& dq, MatrixBase<D9>& dv )
{
        MatrixSizeCheck<3, 1>::check(p1);
        MatrixSizeCheck<3, 1>::check(v1);
        MatrixSizeCheck<3, 1>::check(p2);
        MatrixSizeCheck<3, 1>::check(v2);
        MatrixSizeCheck<3, 1>::check(dp);
        MatrixSizeCheck<3, 1>::check(dv);

        dp = q1.conjugate() * ( p2 - p1 - v1*dt - (T)0.5*gravity().cast<T>()*(T)dt*(T)dt );
        dq = q1.conjugate() *   q2;
        dv = q1.conjugate() * ( v2 - v1         -     gravity().cast<T>()*(T)dt );
}

template<typename D1, typename D2, typename D3, class T>
inline void betweenStates(const MatrixBase<D1>& x1,
                          const MatrixBase<D2>& x2,
                          T dt,
                          MatrixBase<D3>& x2_minus_x1)
{
    MatrixSizeCheck<10, 1>::check(x1);
    MatrixSizeCheck<10, 1>::check(x2);
    MatrixSizeCheck<10, 1>::check(x2_minus_x1);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   p1  ( & x1(0) );
    Map<const Quaternion<typename D1::Scalar> >     q1  ( & x1(3) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   v1  ( & x1(7) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   p2  ( & x2(0) );
    Map<const Quaternion<typename D2::Scalar> >     q2  ( & x2(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   v2  ( & x2(7) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         dp  ( & x2_minus_x1(0) );
    Map<Quaternion<typename D3::Scalar> >           dq  ( & x2_minus_x1(3) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         dv  ( & x2_minus_x1(7) );

    betweenStates(p1, q1, v1, p2, q2, v2, dt, dp, dq, dv);
}

template<typename D1, typename D2, class T>
inline Matrix<typename D1::Scalar, 10, 1> betweenStates(const MatrixBase<D1>& x1,
                                                        const MatrixBase<D2>& x2,
                                                        T dt)
{
    Matrix<typename D1::Scalar, 10, 1> ret;
    betweenStates(x1, x2, dt, ret);
    return ret;
}

template<typename Derived>
Matrix<typename Derived::Scalar, 9, 1> lift(const MatrixBase<Derived>& delta_in)
{
    MatrixSizeCheck<10, 1>::check(delta_in);

    Matrix<typename Derived::Scalar, 9, 1> ret;

    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dp_in  ( & delta_in(0) );
    Map<const Quaternion<typename Derived::Scalar> >     dq_in  ( & delta_in(3) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dv_in  ( & delta_in(7) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dp_ret ( & ret(0) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         do_ret ( & ret(3) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dv_ret ( & ret(6) );

    dp_ret = dp_in;
    do_ret = log_q(dq_in);
    dv_ret = dv_in;

    return ret;
}

template<typename Derived>
Matrix<typename Derived::Scalar, 10, 1> retract(const MatrixBase<Derived>& d_in)
{
    MatrixSizeCheck<9, 1>::check(d_in);

    Matrix<typename Derived::Scalar, 10, 1> ret;

    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dp_in  ( & d_in(0) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   do_in  ( & d_in(3) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dv_in  ( & d_in(6) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dp     ( &  ret(0) );
    Map<Quaternion<typename Derived::Scalar> >           dq     ( &  ret(3) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dv     ( &  ret(7) );

    dp = dp_in;
    dq = exp_q(do_in);
    dv = dv_in;

    return ret;
}

template<typename D1, typename D2, typename D3, typename D4, typename D5, typename D6, typename D7, typename D8, typename D9>
inline void plus(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1, const MatrixBase<D3>& dv1,
                 const MatrixBase<D4>& dp2, const MatrixBase<D5>& do2, const MatrixBase<D6>& dv2,
                 MatrixBase<D7>& plus_p, QuaternionBase<D8>& plus_q, MatrixBase<D9>& plus_v )
{
        plus_p = dp1 + dp2;
        plus_q = dq1 * exp_q(do2);
        plus_v = dv1 + dv2;
}

template<typename D1, typename D2, typename D3>
inline void plus(const MatrixBase<D1>& d1,
                 const MatrixBase<D2>& d2,
                 MatrixBase<D3>& d_pert)
{
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1(3) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( & d1(7) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2(0) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   do2    ( & d2(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( & d2(6) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         dp_p ( & d_pert(0) );
    Map<Quaternion<typename D3::Scalar> >           dq_p ( & d_pert(3) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         dv_p ( & d_pert(7) );

    plus(dp1, dq1, dv1, dp2, do2, dv2, dp_p, dq_p, dv_p);
}

template<typename D1, typename D2>
inline Matrix<typename D1::Scalar, 10, 1> plus(const MatrixBase<D1>& d1,
                                               const MatrixBase<D2>& d2)
{
    Matrix<typename D1::Scalar, 10, 1> ret;
    plus(d1, d2, ret);
    return ret;
}

template<typename D1, typename D2, typename D3, typename D4, typename D5, typename D6, typename D7, typename D8, typename D9>
inline void diff(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1, const MatrixBase<D3>& dv1,
                 const MatrixBase<D4>& dp2, const QuaternionBase<D5>& dq2, const MatrixBase<D6>& dv2,
                 MatrixBase<D7>& diff_p, MatrixBase<D8>& diff_o, MatrixBase<D9>& diff_v )
{
        diff_p = dp2 - dp1;
        diff_o = log_q(dq1.conjugate() * dq2);
        diff_v = dv2 - dv1;
}


template<typename D1, typename D2, typename D3>
inline void diff(const MatrixBase<D1>& d1,
                 const MatrixBase<D2>& d2,
                 MatrixBase<D3>& err)
{
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1(3) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( & d1(7) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2(0) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( & d2(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( & d2(7) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_p ( & err(0) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_o ( & err(3) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_v ( & err(6) );

    diff(dp1, dq1, dv1, dp2, dq2, dv2, diff_p, diff_o, diff_v);
}

//template<typename D1, typename D2, typename D3, typename D4, typename D5>
//inline void diff(const MatrixBase<D1>& d1,
//                 const MatrixBase<D2>& d2,
//                 MatrixBase<D3>& err,
//                 MatrixBase<D4>& J_diff_d1,
//                 MatrixBase<D5>& J_diff_d2)
//{
//    diff(d1, d2, err);
//
//    typedef typename D4::Scalar T4;
//    typedef typename D5::Scalar T5;
//    J_diff_d1.setIdentity();
//    J_diff_d1.block<3,3>(3,3) =
//}

template<typename D1, typename D2>
inline Matrix<typename D1::Scalar, 9, 1> diff(const MatrixBase<D1>& d1,
                                              const MatrixBase<D2>& d2)
{
    Matrix<typename D1::Scalar, 9, 1> ret;
    diff(d1, d2, ret);
    return ret;
}


template<typename D1, typename D2, typename D3, typename D4, typename D5>
inline void body2delta(const MatrixBase<D1>& a,
                       const MatrixBase<D2>& w,
                       const typename D1::Scalar& dt,
                       MatrixBase<D3>& dp,
                       QuaternionBase<D4>& dq,
                       MatrixBase<D5>& dv)
{
    MatrixSizeCheck<3,1>::check(a);
    MatrixSizeCheck<3,1>::check(w);
    MatrixSizeCheck<3,1>::check(dp);
    MatrixSizeCheck<3,1>::check(dv);

    dp = 0.5 * a * dt * dt;
    dq = exp_q(w * dt);
    dv =       a * dt;
}

template<typename D1>
inline Matrix<typename D1::Scalar, 10, 1> body2delta(const MatrixBase<D1>& body,
                                                     const typename D1::Scalar& dt)
{
    MatrixSizeCheck<6,1>::check(body);

    typedef typename D1::Scalar T;

    Matrix<T, 10, 1> delta;

    Map< Matrix<T, 3, 1>> dp ( & delta(0) );
    Map< Quaternion<T>>   dq ( & delta(3) );
    Map< Matrix<T, 3, 1>> dv ( & delta(7) );

    body2delta(body.block(0,0,3,1), body.block(3,0,3,1), dt, dp, dq, dv);

    return delta;
}

template<typename D1, typename D2, typename D3>
inline void body2delta(const MatrixBase<D1>& body,
                       const typename D1::Scalar& dt,
                       MatrixBase<D2>& delta,
                       MatrixBase<D3>& jac_body)
{
    MatrixSizeCheck<6,1>::check(body);
    MatrixSizeCheck<9,6>::check(jac_body);

    typedef typename D1::Scalar T;

    delta = body2delta(body, dt);

    Matrix<T, 3, 1> w = body.block(3,0,3,1);

    jac_body.setZero();
    jac_body.block(0,0,3,3) = 0.5 * dt * dt * Matrix<T, 3, 3>::Identity();
    jac_body.block(3,3,3,3) =            dt * jac_SO3_right(w * dt);
    jac_body.block(6,0,3,3) =            dt * Matrix<T, 3, 3>::Identity();
}

} // namespace imu
} // namespace wolf

#endif /* IMU_TOOLS_H_ */
