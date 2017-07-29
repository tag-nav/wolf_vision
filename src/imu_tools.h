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

template<typename T = wolf::Scalar>
inline Matrix<T, 10, 1> identity()
{
    Matrix<T, 10, 1> ret;
    ret.setZero();
    ret(6) = 1.0;
    return ret;
}

template<typename D1, typename D2>
inline void inverse(const MatrixBase<D1>& d,
                    typename D1::Scalar dt,
                    MatrixBase<D2>& id)
{
    MatrixSizeCheck<10, 1>::check(d);
    MatrixSizeCheck<10, 1>::check(id);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp    ( &d( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     dq    ( &d( 3 ) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv    ( &d( 7 ) );
    Map<Matrix<typename D2::Scalar, 3, 1> >         idp  ( &id( 0 ));
    Map<Quaternion<typename D2::Scalar> >           idq  ( &id( 3 ));
    Map<Matrix<typename D2::Scalar, 3, 1> >         idv  ( &id( 7 ));

    idp = - ( dq.conjugate() * (dp - dv * dt) );
    idv = - ( dq.conjugate() * dv );
    idq =     dq.conjugate();
}

template<typename D>
inline Matrix<typename D::Scalar, 10, 1> inverse(const MatrixBase<D>& d,
                                                 typename D::Scalar dt)
{
    Matrix<typename D::Scalar, 10, 1> id;

    inverse(d, dt, id);

    return id;
}


template<typename D1, typename D2, typename D3>
inline void compose(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    Scalar dt,
                    MatrixBase<D3>& sum)
{
    MatrixSizeCheck<10, 1>::check(d1);
    MatrixSizeCheck<10, 1>::check(d2);
    MatrixSizeCheck<10, 1>::check(sum);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( &d1( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( &d1( 3 ) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( &d1( 7 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( &d2( 0 ) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( &d2( 3 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( &d2( 7 ) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_p  ( &sum( 0 ));
    Map<Quaternion<typename D3::Scalar> >           sum_q  ( &sum( 3 ));
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_v  ( &sum( 7 ));

    sum_p = dp1 + dv1*dt + dq1*dp2;
    sum_v = dv1 +          dq1*dv2;
    sum_q =                dq1*dq2; // dq here to avoid possible aliasing between d1 and sum
}


template<typename D1, typename D2, typename D3, typename D4, typename D5>
inline void compose(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    Scalar dt,
                    MatrixBase<D3>& sum,
                    MatrixBase<D4>& J_sum_d1,
                    MatrixBase<D5>& J_sum_d2)
{
    MatrixSizeCheck<10, 1>::check(d1);
    MatrixSizeCheck<10, 1>::check(d2);
    MatrixSizeCheck<10, 1>::check(sum);
    MatrixSizeCheck< 9, 1>::check(J_sum_d1);
    MatrixSizeCheck< 9, 1>::check(J_sum_d2);

    // Maps over provided data
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( &d1( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( &d1( 3 ) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( &d1( 7 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( &d2( 0 ) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( &d2( 3 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( &d2( 7 ) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_p  ( &sum( 0 ));
    Map<Quaternion<typename D3::Scalar> >           sum_q  ( &sum( 3 ));
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_v  ( &sum( 7 ));

    // Some useful temporaries
    Matrix<typename D1::Scalar, 3, 3> dR1 = dq1.matrix(); // First  Delta, DR
    Matrix<typename D2::Scalar, 3, 3> dR2 = dq2.matrix(); // Second delta, dR

    // Jac wrt first delta
    J_sum_d1.setIdentity();                                     // dDp'/dDp = dDv'/dDv = I
    J_sum_d1.block(0,3,3,3).noalias() = - dR1 * skew(dp2) ;     // dDp'/dDo
    J_sum_d1.block(0,6,3,3) = Matrix3s::Identity() * dt;        // dDp'/dDv = I*dt
    J_sum_d1.block(3,3,3,3) = dR2.transpose();                  // dDo'/dDo
    J_sum_d1.block(6,3,3,3).noalias() = - dR1 * skew(dv2) ;     // dDv'/dDo

    // Jac wrt second delta
    J_sum_d2.setIdentity();                                     //
    J_sum_d2.block(0,0,3,3) = dR1;                              // dDp'/ddp
    J_sum_d2.block(6,6,3,3) = dR1;                              // dDv'/ddv
    // J_sum_d2.block(3,3,3,3) = Matrix3s::Identity();          // dDo'/ddo = I

    // compose deltas -- done here to avoid aliasing when calling with `d1` and `sum` pointing to the same variable
    compose(d1, d2, dt, sum);
}

template<typename D1, typename D2, typename D3>
inline void between(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    Scalar dt,
                    MatrixBase<D3>& d2_minus_d1)
{
    MatrixSizeCheck<10, 1>::check(d1);
    MatrixSizeCheck<10, 1>::check(d2);
    MatrixSizeCheck<10, 1>::check(d2_minus_d1);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( &d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( &d1(3) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( &d1(7) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( &d2(0) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( &d2(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( &d2(7) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_p (&d2_minus_d1(0));
    Map<Quaternion<typename D3::Scalar> >           diff_q (&d2_minus_d1(3));
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_v (&d2_minus_d1(7));

    diff_p = dq1.conjugate() * ( dp2 - dp1 - dv1*dt );
    diff_q = dq1.conjugate() *   dq2;
    diff_v = dq1.conjugate() * ( dv2 - dv1 );
}

template<typename Derived>
Matrix<typename Derived::Scalar, 9, 1> lift(const MatrixBase<Derived>& delta_in)
{
    MatrixSizeCheck<10, 1>::check(delta_in);

    Matrix<typename Derived::Scalar, 9, 1> ret;

    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dp_in  ( &delta_in(0) );
    Map<const Quaternion<typename Derived::Scalar> >     dq_in  ( &delta_in(3) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dv_in  ( &delta_in(7) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dp_ret     ( & ret(0) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         do_ret     ( & ret(3) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dv_ret     ( & ret(6) );

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

    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dp_in  ( &d_in(0) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   do_in  ( &d_in(3) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dv_in  ( &d_in(6) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dp     ( & ret(0) );
    Map<Quaternion<typename Derived::Scalar> >           dq     ( & ret(3) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dv     ( & ret(7) );

    dp = dp_in;
    dq = exp_q(do_in);
    dv = dv_in;

    return ret;
}

template<typename D1, typename D2, typename D3>
inline void compare(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    MatrixBase<D3>& err)
{
    Matrix<typename D3::Scalar, 10, 1> delta_err;

    between(d1, d2, 0.0, delta_err);

    err = lift(delta_err);
}



} // namespace imu
} // namespace wolf

#endif /* IMU_TOOLS_H_ */
