/*
 * three_D_tools.h
 *
 *  Created on: Mar 15, 2018
 *      Author: jsola
 */

#ifndef THREE_D_TOOLS_H_
#define THREE_D_TOOLS_H_


#include "wolf.h"
#include "rotations.h"

/*
 * The functions in this file are related to manipulations of Delta motion magnitudes used in 3D motion.
 *
 * The Delta is defined as a simple 3D pose with the rotation expressed in quaternion form,
 *     Delta = [Dp, Dq]
 * with
 *     Dp : position delta
 *     Dq : quaternion delta
 *
 * The functions are listed below:
 *
 *   - compose:     Dc = D1 (+) D2
 *   - identity:    I  = Delta at the origin, with Dp = [0,0,0]; Dq = [0,0,0,1], so that D (+) I = I (+) D = D
 *   - inverse:     so that D (+) D.inv = D.inv (+) D = I
 *   - between:     Db = D2 (-) D1 := D1.inv (+) D2, so that D2 = D1 (+) Db
 *   - lift:        go from Delta manifold to tangent space (equivalent to log() in rotations)
 *   - retract:     go from tangent space to delta manifold (equivalent to exp() in rotations)
 *   - plus:        D2 = D1 (+) retract(d)
 *   - diff:        d  = lift( D2 (-) D1 )
 */



namespace wolf
{
namespace three_D {
using namespace Eigen;

template<typename D1, typename D2>
inline void identity(MatrixBase<D1>& p, QuaternionBase<D2>& q)
{
    p = MatrixBase<D1>::Zero(3,1);
    q = QuaternionBase<D2>::Identity();
}

template<typename D1, typename D2>
inline void identity(MatrixBase<D1>& p, MatrixBase<D2>& q)
{
    typedef typename D1::Scalar T1;
    typedef typename D2::Scalar T2;
    p << T1(0), T1(0), T1(0);
    q << T2(0), T2(0), T2(0), T2(1);
}

template<typename T = wolf::Scalar>
inline Matrix<T, 7, 1> identity()
{
    Matrix<T, 7, 1> ret;
    ret<< T(0), T(0), T(0),
          T(0), T(0), T(0), T(1);
    return ret;
}

template<typename D1, typename D2, typename D4, typename D5>
inline void inverse(const MatrixBase<D1>& dp, const QuaternionBase<D2>& dq,
                    MatrixBase<D4>& idp, QuaternionBase<D5>& idq)
{
    MatrixSizeCheck<3, 1>::check(dp);
    MatrixSizeCheck<3, 1>::check(idp);

    idp = - dq.conjugate() * dp ;
    idq =   dq.conjugate() ;
}

template<typename D1, typename D2>
inline void inverse(const MatrixBase<D1>& d,
                    MatrixBase<D2>& id)
{
    MatrixSizeCheck<7, 1>::check(d);
    MatrixSizeCheck<7, 1>::check(id);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp   ( & d( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     dq   ( & d( 3 ) );
    Map<Matrix<typename D2::Scalar, 3, 1> >         idp  ( & id( 0 ) );
    Map<Quaternion<typename D2::Scalar> >           idq  ( & id( 3 ) );

    inverse(dp, dq, idp, idq);
}


template<typename D>
inline Matrix<typename D::Scalar, 7, 1> inverse(const MatrixBase<D>& d)
{
    Matrix<typename D::Scalar, 7, 1> id;
    inverse(d, id);
    return id;
}

template<typename D1, typename D2, typename D4, typename D5, typename D7, typename D8>
inline void compose(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1,
                    const MatrixBase<D4>& dp2, const QuaternionBase<D5>& dq2,
                    MatrixBase<D7>& sum_p, QuaternionBase<D8>& sum_q )
{
        MatrixSizeCheck<3, 1>::check(dp1);
        MatrixSizeCheck<3, 1>::check(dp2);
        MatrixSizeCheck<3, 1>::check(sum_p);

        sum_p = dp1 + dq1*dp2;
        sum_q =       dq1*dq2; // dq here to avoid possible aliasing between d1 and sum
}

template<typename D1, typename D2, typename D3>
inline void compose(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    MatrixBase<D3>& sum)
{
    MatrixSizeCheck<7, 1>::check(d1);
    MatrixSizeCheck<7, 1>::check(d2);
    MatrixSizeCheck<7, 1>::check(sum);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1( 3 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2( 0 ) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( & d2( 3 ) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_p  ( & sum( 0 ) );
    Map<Quaternion<typename D3::Scalar> >           sum_q  ( & sum( 3 ) );

    compose(dp1, dq1, dp2, dq2, sum_p, sum_q);
}

template<typename D1, typename D2>
inline Matrix<typename D1::Scalar, 7, 1> compose(const MatrixBase<D1>& d1,
                                                  const MatrixBase<D2>& d2 )
{
    Matrix<typename D1::Scalar, 7, 1>  ret;
    compose(d1, d2, ret);
    return ret;
}

template<typename D1, typename D2, typename D3, typename D4, typename D5>
inline void compose(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    MatrixBase<D3>& sum,
                    MatrixBase<D4>& J_sum_d1,
                    MatrixBase<D5>& J_sum_d2)
{
    MatrixSizeCheck<7, 1>::check(d1);
    MatrixSizeCheck<7, 1>::check(d2);
    MatrixSizeCheck<7, 1>::check(sum);
    MatrixSizeCheck< 6, 6>::check(J_sum_d1);
    MatrixSizeCheck< 6, 6>::check(J_sum_d2);

    // Some useful temporaries
    Matrix<typename D1::Scalar, 3, 3> dR1 = q2R(d1.segment(3,4)); //dq1.matrix(); // First  Delta, DR
    Matrix<typename D2::Scalar, 3, 3> dR2 = q2R(d2.segment(3,4)); //dq2.matrix(); // Second delta, dR

    // Jac wrt first delta
    J_sum_d1.setIdentity();                                     // dDp'/dDp = dDv'/dDv = I
    J_sum_d1.block(0,3,3,3).noalias() = - dR1 * skew(d2.head(3)) ;     // dDp'/dDo
    J_sum_d1.block(3,3,3,3) = dR2.transpose();                  // dDo'/dDo

    // Jac wrt second delta
    J_sum_d2.setIdentity();                                     //
    J_sum_d2.block(0,0,3,3) = dR1;                              // dDp'/ddp
    // J_sum_d2.block(3,3,3,3) = Matrix3s::Identity();          // dDo'/ddo = I

    // compose deltas -- done here to avoid aliasing when calling with input `d1` and result `sum` referencing the same variable
    compose(d1, d2, sum);
}

template<typename D1, typename D2, typename D4, typename D5, typename D7, typename D8>
inline void between(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1,
                    const MatrixBase<D4>& dp2, const QuaternionBase<D5>& dq2,
                    MatrixBase<D7>& dp12, QuaternionBase<D8>& dq12)
{
        MatrixSizeCheck<3, 1>::check(dp1);
        MatrixSizeCheck<3, 1>::check(dp2);
        MatrixSizeCheck<3, 1>::check(dp12);

        dp12 = dq1.conjugate() * ( dp2 - dp1 );
        dq12 = dq1.conjugate() *   dq2;
}

template<typename D1, typename D2, typename D3>
inline void between(const MatrixBase<D1>& d1,
                    const MatrixBase<D2>& d2,
                    MatrixBase<D3>& d2_minus_d1)
{
    MatrixSizeCheck<7, 1>::check(d1);
    MatrixSizeCheck<7, 1>::check(d2);
    MatrixSizeCheck<7, 1>::check(d2_minus_d1);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2(0) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( & d2(3) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         dp12 ( & d2_minus_d1(0) );
    Map<Quaternion<typename D3::Scalar> >           dq12 ( & d2_minus_d1(3) );

    between(dp1, dq1, dp2, dq2, dp12, dq12);
}


template<typename D1, typename D2>
inline Matrix<typename D1::Scalar, 7, 1> between(const MatrixBase<D1>& d1,
                                                 const MatrixBase<D2>& d2 )
{
    Matrix<typename D1::Scalar, 7, 1> d12;
    between(d1, d2, d12);
    return d12;
}

template<typename Derived>
Matrix<typename Derived::Scalar, 6, 1> lift(const MatrixBase<Derived>& delta_in)
{
    MatrixSizeCheck<7, 1>::check(delta_in);

    Matrix<typename Derived::Scalar, 6, 1> ret;

    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dp_in  ( & delta_in(0) );
    Map<const Quaternion<typename Derived::Scalar> >     dq_in  ( & delta_in(3) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dp_ret ( & ret(0) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         do_ret ( & ret(3) );

    dp_ret = dp_in;
    do_ret = log_q(dq_in);

    return ret;
}

template<typename Derived>
Matrix<typename Derived::Scalar, 7, 1> retract(const MatrixBase<Derived>& d_in)
{
    MatrixSizeCheck<6, 1>::check(d_in);

    Matrix<typename Derived::Scalar, 7, 1> ret;

    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dp_in  ( & d_in(0) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   do_in  ( & d_in(3) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dp     ( &  ret(0) );
    Map<Quaternion<typename Derived::Scalar> >           dq     ( &  ret(3) );

    dp = dp_in;
    dq = exp_q(do_in);

    return ret;
}

template<typename D1, typename D2, typename D4, typename D5, typename D7, typename D8>
inline void plus(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1,
                 const MatrixBase<D4>& dp2, const MatrixBase<D5>& do2,
                 MatrixBase<D7>& plus_p, QuaternionBase<D8>& plus_q)
{
        plus_p = dp1 + dp2;
        plus_q = dq1 * exp_q(do2);
}

template<typename D1, typename D2, typename D3>
inline void plus(const MatrixBase<D1>& d1,
                 const MatrixBase<D2>& d2,
                 MatrixBase<D3>& d_plus)
{
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2(0) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   do2    ( & d2(3) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         dp_p ( & d_plus(0) );
    Map<Quaternion<typename D3::Scalar> >           dq_p ( & d_plus(3) );

    plus(dp1, dq1, dp2, do2, dp_p, dq_p);
}

template<typename D1, typename D2>
inline Matrix<typename D1::Scalar, 7, 1> plus(const MatrixBase<D1>& d1,
                                              const MatrixBase<D2>& d2)
{
    Matrix<typename D1::Scalar, 7, 1> d_plus;
    plus(d1, d2, d_plus);
    return d_plus;
}

template<typename D1, typename D2, typename D4, typename D5, typename D7, typename D8>
inline void diff(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1,
                 const MatrixBase<D4>& dp2, const QuaternionBase<D5>& dq2,
                 MatrixBase<D7>& diff_p, MatrixBase<D8>& diff_o )
{
        diff_p = dp2 - dp1;
        diff_o = log_q(dq1.conjugate() * dq2);
}

template<typename D1, typename D2, typename D4, typename D5, typename D6, typename D7, typename D8, typename D9>
inline void diff(const MatrixBase<D1>& dp1, const QuaternionBase<D2>& dq1,
                 const MatrixBase<D4>& dp2, const QuaternionBase<D5>& dq2,
                 MatrixBase<D6>& diff_p, MatrixBase<D7>& diff_o,
                 MatrixBase<D8>& J_do_dq1, MatrixBase<D9>& J_do_dq2)
{
    diff(dp1, dq1, dp2, dq2, diff_p, diff_o);

    J_do_dq1    = - jac_SO3_left_inv(diff_o);
    J_do_dq2    =   jac_SO3_right_inv(diff_o);
}


template<typename D1, typename D2, typename D3>
inline void diff(const MatrixBase<D1>& d1,
                 const MatrixBase<D2>& d2,
                 MatrixBase<D3>& err)
{
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2(0) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( & d2(3) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_p ( & err(0) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_o ( & err(3) );

    diff(dp1, dq1, dp2, dq2, diff_p, diff_o);
}

template<typename D1, typename D2, typename D3, typename D4, typename D5>
inline void diff(const MatrixBase<D1>& d1,
                 const MatrixBase<D2>& d2,
                 MatrixBase<D3>& dif,
                 MatrixBase<D4>& J_diff_d1,
                 MatrixBase<D5>& J_diff_d2)
{
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( & d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( & d1(3) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( & d1(7) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( & d2(0) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( & d2(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( & d2(7) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_p ( & dif(0) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_o ( & dif(3) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_v ( & dif(6) );

    Matrix<typename D4::Scalar, 3, 3> J_do_dq1, J_do_dq2;

    diff(dp1, dq1, dv1, dp2, dq2, dv2, diff_p, diff_o, diff_v, J_do_dq1, J_do_dq2);

    /* d = diff(d1, d2) is
     *   dp = dp2 - dp1
     *   do = Log(dq1.conj * dq2)
     *   dv = dv2 - dv1
     *
     * With trivial Jacobians for dp and dv, and:
     *   J_do_dq1 = - J_l_inv(theta)
     *   J_do_dq2 =   J_r_inv(theta)
     */

    J_diff_d1 = - Matrix<typename D4::Scalar, 6, 6>::Identity();// d(p2  - p1) / d(p1) = - Identity
    J_diff_d1.block(3,3,3,3) = J_do_dq1;       // d(R1.tr*R2) / d(R1) = - J_l_inv(theta)

    J_diff_d2.setIdentity(6,6);                                    // d(R1.tr*R2) / d(R2) =   Identity
    J_diff_d2.block(3,3,3,3) = J_do_dq2;      // d(R1.tr*R2) / d(R1) =   J_r_inv(theta)
}

template<typename D1, typename D2>
inline Matrix<typename D1::Scalar, 6, 1> diff(const MatrixBase<D1>& d1,
                                              const MatrixBase<D2>& d2)
{
    Matrix<typename D1::Scalar, 6, 1> ret;
    diff(d1, d2, ret);
    return ret;
}


} // namespace three_d
} // namespace wolf


#endif /* THREE_D_TOOLS_H_ */
