/**
 * \file gtest_odom_3D.cpp
 *
 *  Created on: Nov 11, 2016
 *      \author: jsola
 */




#include "utils_gtest.h"
#include "../src/logging.h"

#include "../src/processor_odom_3D.h"

#include "../src/wolf.h"

#include <iostream>

#define JAC_NUMERIC(prc_ptr, D, d, dt, J_D, J_d, dx) \
{   VectorXs Do(7); \
    prc_ptr->deltaPlusDelta(D, d, dt, Do); \
    VectorXs dd(7); \
    VectorXs DD(7); \
    VectorXs DDo(7); \
    for (int i = 0; i< 7; i++) {\
        dd = d;\
        dd(i) += dx;\
        prc_ptr->deltaPlusDelta(D, dd, dt, DDo);\
        J_d.col(i) = (DDo - Do)/dx; \
        DD = D; \
        prc_ptr->deltaPlusDelta(DD, d, dt, DDo); \
        J_D.col(i) = (DDo - Do)/dx; \
    }\
}

using namespace Eigen;
using namespace std;
using namespace wolf;


TEST(TestOdom3D, Interpolate0) // basic test
{
    /* Conditions:
     * ref d = id
     * ref D = id
     * fin d = pos
     * fin D = id
     */

    ProcessorOdom3D prc;

    Motion ref(0,7,6), final(0,7,6), interpolated(0,7,6);

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
    ASSERT_TRUE((interpolated.delta_.head<3>() - 0.25 * final.delta_.head<3>()).isMuchSmallerThan(1.0, Constants::EPS));
    ASSERT_TRUE((second.delta_.head<3>()       - 0.75 * final.delta_.head<3>()).isMuchSmallerThan(1.0, Constants::EPS));

}

TEST(TestOdom3D, Interpolate1) // delta algebra test
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
    Map<Quaternions>    q_o(x_o.data()+3);
    Map<Vector3s>       p_r(x_r.data(), 3);
    Map<Quaternions>    q_r(x_r.data()+3);
    Map<Vector3s>       p_i(x_i.data(), 3);
    Map<Quaternions>    q_i(x_i.data()+3);
    Map<Vector3s>       p_s(x_s.data(), 3);
    Map<Quaternions>    q_s(x_s.data()+3);
    Map<Vector3s>       p_f(x_f.data(), 3);
    Map<Quaternions>    q_f(x_f.data()+3);

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
    Motion R(0,7,6), I(0,7,6), S(0,7,6), F(0,7,6);


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


    R.resize(7,6);
    F.resize(7,6);
    I.resize(7,6);

    // set ref
    R.ts_           = t_r;
    R.delta_        = dx_or; // origin to ref
    R.delta_integr_ = Dx_or; // origin to ref

    WOLF_DEBUG("* R.d = ", R.delta_.transpose());
    WOLF_DEBUG("  or  = ", dx_or.transpose());
    ASSERT_TRUE((R.delta_        - dx_or).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  R.D = ", R.delta_integr_.transpose());
    WOLF_DEBUG("  or  = ", Dx_or.transpose());
    ASSERT_TRUE((R.delta_integr_ - Dx_or).isMuchSmallerThan(1.0, Constants::EPS));

    // set final
    F.ts_           = t_f;
    F.delta_        = dx_rf; // ref to final
    F.delta_integr_ = Dx_of; // origin to final

    WOLF_DEBUG("* F.d = ", F.delta_.transpose());
    WOLF_DEBUG("  rf  = ", dx_rf.transpose());
    ASSERT_TRUE((F.delta_        - dx_rf).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  F.D = ", F.delta_integr_.transpose());
    WOLF_DEBUG("  of  = ", Dx_of.transpose());
    ASSERT_TRUE((F.delta_integr_ - Dx_of).isMuchSmallerThan(1.0, Constants::EPS));

    S = F; // avoid overwriting final
    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  rs  = ", dx_rs.transpose());
    ASSERT_TRUE((S.delta_        - dx_rs).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_os.transpose());
    ASSERT_TRUE((S.delta_integr_ - Dx_os).isMuchSmallerThan(1.0, Constants::EPS));

    // interpolate!
    WOLF_DEBUG("*** INTERPOLATE *** I has been computed; S has changed.");
    I = prc.interpolate(R, S, t_i);

    WOLF_DEBUG("* I.d = ", I.delta_.transpose());
    WOLF_DEBUG("  ri  = ", dx_ri.transpose());
    ASSERT_TRUE((I.delta_        - dx_ri).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  I.D = ", I.delta_integr_.transpose());
    WOLF_DEBUG("  oi  = ", Dx_oi.transpose());
    ASSERT_TRUE((I.delta_integr_ - Dx_oi).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  is  = ", dx_is.transpose());
    ASSERT_TRUE((S.delta_        - dx_is).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_os.transpose());
    ASSERT_TRUE((S.delta_integr_ - Dx_os).isMuchSmallerThan(1.0, Constants::EPS));

}

TEST(TestOdom3D, Interpolate2) // timestamp out of bounds test
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
    Motion R(0,7,6), I(0,7,6), S(0,7,6), F(0,7,6);


    /////////// start experiment ///////////////

    // set final and ref deltas
    dx_or << 1,2,3, 4,5,6,7; dx_or.tail<4>().normalize();
    dx_rf << 7,6,5, 4,3,2,1; dx_rf.tail<4>().normalize();
    Dx_or = dx_or;
    prc.deltaPlusDelta(Dx_or, dx_rf, t_f - t_r, Dx_of);
    Dx_os = Dx_of;

    R.resize(7,6);
    I.resize(7,6);
    S.resize(7,6);
    F.resize(7,6);

    // set ref
    R.ts_           = t_r;
    R.delta_        = dx_or; // origin to ref
    R.delta_integr_ = Dx_or; // origin to ref

    WOLF_DEBUG("* R.d = ", R.delta_.transpose());
    WOLF_DEBUG("  or  = ", dx_or.transpose());
    ASSERT_TRUE((R.delta_        - dx_or).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  R.D = ", R.delta_integr_.transpose());
    WOLF_DEBUG("  or  = ", Dx_or.transpose());
    ASSERT_TRUE((R.delta_integr_ - Dx_or).isMuchSmallerThan(1.0, Constants::EPS));

    // set final
    F.ts_           = t_f;
    F.delta_        = dx_rf; // ref to final
    F.delta_integr_ = Dx_of; // origin to final

    WOLF_DEBUG("* F.d = ", F.delta_.transpose());
    WOLF_DEBUG("  rf  = ", dx_rf.transpose());
    ASSERT_TRUE((F.delta_        - dx_rf).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  F.D = ", F.delta_integr_.transpose());
    WOLF_DEBUG("  of  = ", Dx_of.transpose());
    ASSERT_TRUE((F.delta_integr_ - Dx_of).isMuchSmallerThan(1.0, Constants::EPS));

    S = F; // avoid overwriting final
    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  rs  = ", dx_rs.transpose());
    ASSERT_TRUE((S.delta_        - dx_rs).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_os.transpose());
    ASSERT_TRUE((S.delta_integr_ - Dx_os).isMuchSmallerThan(1.0, Constants::EPS));

    // interpolate!
    t_i = 0.5; /// before ref!
    WOLF_DEBUG("*** INTERPOLATE *** I has been computed; S has changed.");
    I = prc.interpolate(R, S, t_i);

    WOLF_DEBUG("* I.d = ", I.delta_.transpose());
    WOLF_DEBUG("  ri  = ", prc.deltaZero());
    ASSERT_TRUE((I.delta_  - prc.deltaZero()).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  I.D = ", I.delta_integr_.transpose());
    WOLF_DEBUG("  oi  = ", Dx_or.transpose());
    ASSERT_TRUE((I.delta_integr_ - Dx_or).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  is  = ", dx_rf.transpose());
    ASSERT_TRUE((S.delta_        - dx_rf).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_of.transpose());
    ASSERT_TRUE((S.delta_integr_ - Dx_of).isMuchSmallerThan(1.0, Constants::EPS));

    // interpolate!
    t_i = 5.5; /// after ref!
    S = F;
    WOLF_DEBUG("*** INTERPOLATE *** I has been computed; S has changed.");
    I = prc.interpolate(R, S, t_i);

    WOLF_DEBUG("* I.d = ", I.delta_.transpose());
    WOLF_DEBUG("  ri  = ", dx_rf.transpose());
    ASSERT_TRUE((I.delta_  - dx_rf).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  I.D = ", I.delta_integr_.transpose());
    WOLF_DEBUG("  oi  = ", Dx_of.transpose());
    ASSERT_TRUE((I.delta_integr_ - Dx_of).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("* S.d = ", S.delta_.transpose());
    WOLF_DEBUG("  is  = ", prc.deltaZero());
    ASSERT_TRUE((S.delta_ - prc.deltaZero()).isMuchSmallerThan(1.0, Constants::EPS));

    WOLF_DEBUG("  S.D = ", S.delta_integr_.transpose());
    WOLF_DEBUG("  os  = ", Dx_of.transpose());
    ASSERT_TRUE((S.delta_integr_ - Dx_of).isMuchSmallerThan(1.0, Constants::EPS));

}




int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


