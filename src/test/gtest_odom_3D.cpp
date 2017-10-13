/**
 * \file gtest_odom_3D.cpp
 *
 *  Created on: Nov 11, 2016
 *      \author: jsola
 */




#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processor_odom_3D.h"

#include <iostream>


#define JAC_NUMERIC(prc_ptr, D, d, dt, J_D, J_d, dx) \
{   VectorXs Do(7); \
    prc_ptr->deltaPlusDelta(D, d, dt, Do); \
    VectorXs dd(7); \
    VectorXs DD(7); \
    VectorXs DDo(7); \
    for (int i = 0; i< 7; i++) {\
        dd = d;\
        DD = D; \
        dd(i) += dx;\
        prc_ptr->deltaPlusDelta(D, dd, dt, DDo);\
        J_d.col(i) = (DDo - Do)/dx; \
        dd = d;\
        DD = D; \
        DD(i) += dx; \
        prc_ptr->deltaPlusDelta(DD, d, dt, DDo); \
        J_D.col(i) = (DDo - Do)/dx; \
    }\
}

using namespace Eigen;
using namespace std;
using namespace wolf;

/** Gain access to members of ProcessorOdom3D
 */
class ProcessorOdom3DTest : public ProcessorOdom3D
{
    public:
        ProcessorOdom3DTest();

        // getters :-D !!
        VectorXs& delta() {return delta_;}
        VectorXs& deltaInt() {return delta_integrated_;}
        MatrixXs& deltaCov() {return delta_cov_;}
        MatrixXs& deltaIntCov() {return delta_integrated_cov_;}
        Scalar& kdd() {return k_disp_to_disp_;}
        Scalar& kdr() {return k_disp_to_rot_;}
        Scalar& krr() {return k_rot_to_rot_;}
        Scalar& dvar_min() {return min_disp_var_;}
        Scalar& rvar_min() {return min_rot_var_;}
};
ProcessorOdom3DTest::ProcessorOdom3DTest() : ProcessorOdom3D()
{
    dvar_min() = 0.5;
    rvar_min() = 0.25;
}

TEST(ProcessorOdom3D, computeCurrentDelta)
{
    // One instance of the processor to test
    ProcessorOdom3DTest prc;

    // input data
    Vector6s data; data.setRandom();
    Scalar dt = 1; // irrelevant, just for the API.

    // Build delta from Eigen tools
    Vector3s data_dp = data.head<3>();
    Vector3s data_do = data.tail<3>();
    Vector3s delta_dp = data_dp;
    Quaternions delta_dq = v2q(data_do);
    Vector7s delta;
    delta.head<3>() = delta_dp;
    delta.tail<4>() = delta_dq.coeffs();

    // construct covariance from processor parameters and motion magnitudes
    Scalar disp = data_dp.norm();
    Scalar rot = data_do.norm();
    Scalar dvar = prc.dvar_min() + prc.kdd()*disp;
    Scalar rvar = prc.rvar_min() + prc.kdr()*disp + prc.krr()*rot;
    Vector6s diag; diag << dvar, dvar, dvar, rvar, rvar, rvar;
    Matrix6s data_cov = diag.asDiagonal();
    Matrix6s delta_cov = data_cov;

    // return values for data2delta()
    VectorXs delta_ret(7);
    MatrixXs delta_cov_ret(6,6);
    MatrixXs jac_delta_calib(6,0);

    // call the function under test
    prc.computeCurrentDelta(data, data_cov, dt, delta_ret, delta_cov_ret, VectorXs::Zero(0), jac_delta_calib);

    ASSERT_MATRIX_APPROX(delta_ret , delta, Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(delta_cov_ret , delta_cov, Constants::EPS_SMALL);

}

TEST(ProcessorOdom3D, deltaPlusDelta)
{
    ProcessorOdom3DTest prc;

    VectorXs D(7); D.setRandom(); D.tail<4>().normalize();
    VectorXs d(7); d.setRandom(); d *= 1; d.tail<4>().normalize();

    // Integrated delta value to check aginst
    // Dp_int <-- Dp + Dq * dp
    // Dq_int <-- Dq * dq
    VectorXs D_int_check(7);
    D_int_check.head<3>() = D.head<3>() + Quaternions(D.data()+3) * d.head<3>();
    D_int_check.tail<4>() = (Quaternions(D.data()+3) * Quaternions(d.data()+3)).coeffs();

    Scalar dt = 1; // dummy, not used in Odom3D

    VectorXs D_int(7);

    prc.deltaPlusDelta(D, d, dt, D_int);

    ASSERT_MATRIX_APPROX(D_int , D_int_check, 1e-10);
//        << "\nDpd  : " << D_int.transpose()
//        << "\ncheck: " << D_int_check.transpose();
}

TEST(ProcessorOdom3D, deltaPlusDelta_Jac)
{
    std::shared_ptr<ProcessorOdom3DTest> prc_ptr = std::make_shared<ProcessorOdom3DTest>();

    VectorXs D(7); D.setRandom(); D.tail<4>().normalize();
    VectorXs d(7); d.setRandom(); d *= 1; d.tail<4>().normalize();
    Scalar dt = 1;
    VectorXs Do(7);
    MatrixXs D_D(6,6);
    MatrixXs D_d(6,6);

    prc_ptr->deltaPlusDelta(D, d, dt, Do, D_D, D_d);

    WOLF_DEBUG("DD:\n ", D_D);
    WOLF_DEBUG("Dd:\n ", D_d);

    MatrixXs J_D(7,7), J_d(7,7);

    JAC_NUMERIC(prc_ptr, D, d, dt, J_D, J_d, Constants::EPS);
    WOLF_DEBUG("J_D:\n ", J_D);
    WOLF_DEBUG("J_d:\n ", J_d);

}


TEST(ProcessorOdom3D, Interpolate0) // basic test
{
    /* Conditions:
     * ref d = id
     * ref D = id
     * fin d = pos
     * fin D = id
     */

    ProcessorOdom3D prc;

    Motion ref(0.0,6,7,6,0), final(0.0,6,7,6,0), interpolated(0.0,6,7,6,0);

    // set ref
    ref.ts_ = 1;
    ref.delta_          << 0,0,0, 0,0,0,1;
    ref.delta_integr_   << 0,0,0, 0,0,0,1;

    WOLF_DEBUG("ref delta= ", ref.delta_.transpose());
    WOLF_DEBUG("ref Delta= ", ref.delta_integr_.transpose());


    // set final
    final.ts_ = 5;
    final.delta_        << 1,0,0, 0,0,0,1;
    final.delta_integr_ << 0,0,0, 0,0,0,1;
    prc.deltaPlusDelta(ref.delta_integr_, final.delta_, (final.ts_ - ref.ts_), final.delta_integr_);

    WOLF_DEBUG("final delta= ", final.delta_.transpose());
    WOLF_DEBUG("final Delta= ", final.delta_integr_.transpose());

    // interpolate!
    Motion second = final;
    TimeStamp t; t = 2;
    // +--+--------+---> time(s)
    // 1  2  3  4  5   // 2 = 25% into interpolated, 75% into second
    interpolated = prc.interpolate(ref, second, t);

    WOLF_DEBUG("interpolated delta= ", interpolated.delta_.transpose());
    WOLF_DEBUG("interpolated Delta= ", interpolated.delta_integr_.transpose());

    // delta
    ASSERT_MATRIX_APPROX(interpolated.delta_.head<3>() , 0.25 * final.delta_.head<3>(), Constants::EPS);
    ASSERT_MATRIX_APPROX(second.delta_.head<3>()       , 0.75 * final.delta_.head<3>(), Constants::EPS);

}

TEST(ProcessorOdom3D, Interpolate1) // delta algebra test
{
    ProcessorOdom3D prc;

    /*
     * We create several poses: origin, ref, int, second, final, as follows:
     *
     *   +---+---+---+---->
     *   o   r   i  s,f
     *
     * We compute all deltas between them: d_or, d_ri, d_is, d_rf
     * We create the motions R, F
     * We interpolate, and get I, S
     */

    // absolute poses: origin, ref, interp, second=final
    Vector7s    x_o, x_r, x_i, x_s, x_f;
    Map<Vector3s>       p_o(x_o.data(), 3);
    Map<Quaternions>    q_o(x_o.data() +3);
    Map<Vector3s>       p_r(x_r.data(), 3);
    Map<Quaternions>    q_r(x_r.data() +3);
    Map<Vector3s>       p_i(x_i.data(), 3);
    Map<Quaternions>    q_i(x_i.data() +3);
    Map<Vector3s>       p_s(x_s.data(), 3);
    Map<Quaternions>    q_s(x_s.data() +3);
    Map<Vector3s>       p_f(x_f.data(), 3);
    Map<Quaternions>    q_f(x_f.data() +3);

    // deltas -- referred to previous delta
    //         o-r    r-i    i-s    s-f
    Vector7s dx_or, dx_ri, dx_is, dx_rf;
    Map<Vector3s>       dp_or(dx_or.data(), 3);
    Map<Quaternions>    dq_or(dx_or.data() +3);
    Map<Vector3s>       dp_ri(dx_ri.data(), 3);
    Map<Quaternions>    dq_ri(dx_ri.data() +3);
    Map<Vector3s>       dp_is(dx_is.data(), 3);
    Map<Quaternions>    dq_is(dx_is.data() +3);
    Map<Vector3s>       dp_rf(dx_rf.data(), 3);
    Map<Quaternions>    dq_rf(dx_rf.data() +3);
    Map<Vector7s>       dx_rs(dx_rf.data(), 7); // this ensures dx_rs = dx_rf

    // Deltas -- always referred to origin
    //         o-r    o-i    o-s    o-f
    Vector7s Dx_or, Dx_oi, Dx_os, Dx_of;
    Map<Vector3s>       Dp_or(Dx_or.data(), 3);
    Map<Quaternions>    Dq_or(Dx_or.data() +3);
    Map<Vector3s>       Dp_oi(Dx_oi.data(), 3);
    Map<Quaternions>    Dq_oi(Dx_oi.data() +3);
    Map<Vector3s>       Dp_os(Dx_os.data(), 3);
    Map<Quaternions>    Dq_os(Dx_os.data() +3);
    Map<Vector3s>       Dp_of(Dx_of.data(), 3);
    Map<Quaternions>    Dq_of(Dx_of.data() +3);

    // time stamps and intervals
    TimeStamp t_o(0), t_r(1), t_i(2.3), t_f(5); // t_i=2: 25% of motion; t_i=2.3: a general interpolation point
    Scalar dt_ri = t_i - t_r;
    Scalar dt_rf = t_f - t_r;

    WOLF_DEBUG("t_o: ", t_o.get(), "; t_r: ", t_r.get(), "; t_i: ", t_i.get(), "; t_f: ", t_f.get());
    WOLF_DEBUG("dt_ri: ", dt_ri, "; dt_rf ", dt_rf)

    // Constant velocity model
    Vector3s v;
    Vector3s w;

    // Motion structures
    Motion R(0.0,6,7,6,0), I(0.0,6,7,6,0), S(0.0,6,7,6,0), F(0.0,6,7,6,0);


    /////////// start experiment ///////////////

    // set origin and ref states
    x_o << 1,2,3, 4,5,6,7; q_o.normalize();
    x_r << 7,6,5, 4,3,2,1; q_r.normalize();

    // set constant velocity params
    v << 3,2,1; // linear velocity
    w << .1,.2,.3; // angular velocity

    // compute other poses from model
    p_i = p_r +      v * dt_ri;
    q_i = q_r * v2q (w * dt_ri);
    p_f = p_r +      v * dt_rf;
    q_f = q_r * v2q (w * dt_rf);
    x_s = x_f;

    WOLF_DEBUG("o   = ", x_o.transpose());
    WOLF_DEBUG("r   = ", x_r.transpose());
    WOLF_DEBUG("i   = ", x_i.transpose());
    WOLF_DEBUG("s   = ", x_s.transpose());
    WOLF_DEBUG("f   = ", x_f.transpose());

    // deltas -- referred to previous delta
    dp_or = q_o.conjugate() * (p_r - p_o);
    dq_or = q_o.conjugate() *  q_r;
    dp_ri = q_r.conjugate() * (p_i - p_r);
    dq_ri = q_r.conjugate() *  q_i;
    dp_is = q_i.conjugate() * (p_s - p_i);
    dq_is = q_i.conjugate() *  q_s;
    dp_rf = q_r.conjugate() * (p_f - p_r);
    dq_rf = q_r.conjugate() *  q_f;

    // Deltas -- always referred to origin
    Dp_or = q_o.conjugate() * (p_r - p_o);
    Dq_or = q_o.conjugate() *  q_r;
    Dp_oi = q_o.conjugate() * (p_i - p_o);
    Dq_oi = q_o.conjugate() *  q_i;
    Dp_os = q_o.conjugate() * (p_s - p_o);
    Dq_os = q_o.conjugate() *  q_s;
    Dp_of = q_o.conjugate() * (p_f - p_o);
    Dq_of = q_o.conjugate() *  q_f;


    // set ref
    R.ts_           = t_r;
    R.delta_        = dx_or; // origin to ref
    R.delta_integr_ = Dx_or; // origin to ref

    WOLF_DEBUG("* R.d = ", R.delta_.transpose());
    WOLF_DEBUG("  or  = ", dx_or.transpose());
    ASSERT_MATRIX_APPROX(R.delta_        , dx_or, Constants::EPS);

    WOLF_DEBUG("  R.D = ", R.delta_integr_.transpose());
    WOLF_DEBUG("  or  = ", Dx_or.transpose());
    ASSERT_MATRIX_APPROX(R.delta_integr_ , Dx_or, Constants::EPS);

    // set final
    F.ts_           = t_f;
    F.delta_        = dx_rf; // ref to final
    F.delta_integr_ = Dx_of; // origin to final

    WOLF_DEBUG("* F.d = ", F.delta_.transpose());
    WOLF_DEBUG("  rf  = ", dx_rf.transpose());
    ASSERT_MATRIX_APPROX(F.delta_        , dx_rf, Constants::EPS);

    WOLF_DEBUG("  F.D = ", F.delta_integr_.transpose());
    WOLF_DEBUG("  of  = ", Dx_of.transpose());
    ASSERT_MATRIX_APPROX(F.delta_integr_ , Dx_of, Constants::EPS);

    S = F; // avoid overwriting final
    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  rs  = ", dx_rs.transpose());
    ASSERT_MATRIX_APPROX(S.delta_        , dx_rs, Constants::EPS);

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_os.transpose());
    ASSERT_MATRIX_APPROX(S.delta_integr_ , Dx_os, Constants::EPS);

    // interpolate!
    WOLF_DEBUG("*** INTERPOLATE *** I has been computed; S has changed.");
    I = prc.interpolate(R, S, t_i);

    WOLF_DEBUG("* I.d = ", I.delta_.transpose());
    WOLF_DEBUG("  ri  = ", dx_ri.transpose());
    ASSERT_MATRIX_APPROX(I.delta_        , dx_ri, Constants::EPS);

    WOLF_DEBUG("  I.D = ", I.delta_integr_.transpose());
    WOLF_DEBUG("  oi  = ", Dx_oi.transpose());
    ASSERT_MATRIX_APPROX(I.delta_integr_ , Dx_oi, Constants::EPS);

    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  is  = ", dx_is.transpose());
    ASSERT_MATRIX_APPROX(S.delta_        , dx_is, Constants::EPS);

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_os.transpose());
    ASSERT_MATRIX_APPROX(S.delta_integr_ , Dx_os, Constants::EPS);

}

TEST(ProcessorOdom3D, Interpolate2) // timestamp out of bounds test
{
    ProcessorOdom3D prc;

    /*
     * We create several poses: origin, ref, int, second, final, as follows:
     *
     *   +---+---+---+---->
     *   o   r   i  s,f
     *
     * We compute all deltas between them: d_or, d_ri, d_is, d_rf
     * We create the motions R, F
     * We interpolate, and get I, S
     */

    // deltas -- referred to previous delta
    //         o-r    r-i    i-s    s-f
    VectorXs dx_or(7), dx_ri(7), dx_is(7), dx_rf(7);
    Map<VectorXs>       dx_rs(dx_rf.data(), 7); // this ensures dx_rs = dx_rf

    // Deltas -- always referred to origin
    //         o-r    o-i    o-s    o-f
    VectorXs Dx_or(7), Dx_oi(7), Dx_os(7), Dx_of(7);

    // time stamps and intervals
    TimeStamp t_o(0), t_r(1), t_i, t_f(5); // we'll set t_i later

    WOLF_DEBUG("t_o: ", t_o.get(), "; t_r: ", t_r.get(), "; t_f: ", t_f.get());

    // Motion structures
    Motion R(0.0,6,7,6,0), I(0.0,6,7,6,0), S(0.0,6,7,6,0), F(0.0,6,7,6,0);


    /////////// start experiment ///////////////

    // set final and ref deltas
    dx_or << 1,2,3, 4,5,6,7; dx_or.tail<4>().normalize();
    dx_rf << 7,6,5, 4,3,2,1; dx_rf.tail<4>().normalize();
    Dx_or = dx_or;
    prc.deltaPlusDelta(Dx_or, dx_rf, t_f - t_r, Dx_of);
    Dx_os = Dx_of;

    // set ref
    R.ts_           = t_r;
    R.delta_        = dx_or; // origin to ref
    R.delta_integr_ = Dx_or; // origin to ref

    WOLF_DEBUG("* R.d = ", R.delta_.transpose());
    WOLF_DEBUG("  or  = ", dx_or.transpose());
    ASSERT_MATRIX_APPROX(R.delta_        , dx_or, Constants::EPS);

    WOLF_DEBUG("  R.D = ", R.delta_integr_.transpose());
    WOLF_DEBUG("  or  = ", Dx_or.transpose());
    ASSERT_MATRIX_APPROX(R.delta_integr_ , Dx_or, Constants::EPS);

    // set final
    F.ts_           = t_f;
    F.delta_        = dx_rf; // ref to final
    F.delta_integr_ = Dx_of; // origin to final

    WOLF_DEBUG("* F.d = ", F.delta_.transpose());
    WOLF_DEBUG("  rf  = ", dx_rf.transpose());
    ASSERT_MATRIX_APPROX(F.delta_        , dx_rf, Constants::EPS);

    WOLF_DEBUG("  F.D = ", F.delta_integr_.transpose());
    WOLF_DEBUG("  of  = ", Dx_of.transpose());
    ASSERT_MATRIX_APPROX(F.delta_integr_ , Dx_of, Constants::EPS);

    S = F; // avoid overwriting final
    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  rs  = ", dx_rs.transpose());
    ASSERT_MATRIX_APPROX(S.delta_        , dx_rs, Constants::EPS);

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_os.transpose());
    ASSERT_MATRIX_APPROX(S.delta_integr_ , Dx_os, Constants::EPS);

    // interpolate!
    t_i = 0.5; /// before ref!
    WOLF_DEBUG("*** INTERPOLATE *** I has been computed; S has changed.");
    I = prc.interpolate(R, S, t_i);

    WOLF_DEBUG("* I.d = ", I.delta_.transpose());
    WOLF_DEBUG("  ri  = ", prc.deltaZero().transpose());
    ASSERT_MATRIX_APPROX(I.delta_  , prc.deltaZero(), Constants::EPS);

    WOLF_DEBUG("  I.D = ", I.delta_integr_.transpose());
    WOLF_DEBUG("  oi  = ", Dx_or.transpose());
    ASSERT_MATRIX_APPROX(I.delta_integr_ , Dx_or, Constants::EPS);

    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  is  = ", dx_rf.transpose());
    ASSERT_MATRIX_APPROX(S.delta_        , dx_rf, Constants::EPS);

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_of.transpose());
    ASSERT_MATRIX_APPROX(S.delta_integr_ , Dx_of, Constants::EPS);

    // interpolate!
    t_i = 5.5; /// after ref!
    S = F;
    WOLF_DEBUG("*** INTERPOLATE *** I has been computed; S has changed.");
    I = prc.interpolate(R, S, t_i);

    WOLF_DEBUG("* I.d = ", I.delta_.transpose());
    WOLF_DEBUG("  ri  = ", dx_rf.transpose());
    ASSERT_MATRIX_APPROX(I.delta_  , dx_rf, Constants::EPS);

    WOLF_DEBUG("  I.D = ", I.delta_integr_.transpose());
    WOLF_DEBUG("  oi  = ", Dx_of.transpose());
    ASSERT_MATRIX_APPROX(I.delta_integr_ , Dx_of, Constants::EPS);

    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  is  = ", prc.deltaZero().transpose());
    ASSERT_MATRIX_APPROX(S.delta_ , prc.deltaZero(), Constants::EPS);

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_of.transpose());
    ASSERT_MATRIX_APPROX(S.delta_integr_ , Dx_of, Constants::EPS);

}




int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


