/*
 * gtest_imu_tools.cpp
 *
 *  Created on: Jul 29, 2017
 *      Author: jsola
 */

#include <IMU_tools.h>
#include "utils_gtest.h"


using namespace Eigen;
using namespace wolf;
using namespace imu;

TEST(IMU_tools, identity)
{
    VectorXs id_true;
    id_true.setZero(10);
    id_true(6) = 1.0;

    VectorXs id = identity<Scalar>();
    ASSERT_MATRIX_APPROX(id, id_true, 1e-10);
}

TEST(IMU_tools, identity_split)
{
    VectorXs p(3), qv(4), v(3);
    Quaternions q;

    identity(p,q,v);
    ASSERT_MATRIX_APPROX(p, Vector3s::Zero(), 1e-10);
    ASSERT_QUATERNION_APPROX(q, Quaternions::Identity(), 1e-10);
    ASSERT_MATRIX_APPROX(v, Vector3s::Zero(), 1e-10);

    identity(p,qv,v);
    ASSERT_MATRIX_APPROX(p , Vector3s::Zero(), 1e-10);
    ASSERT_MATRIX_APPROX(qv, (Vector4s()<<0,0,0,1).finished(), 1e-10);
    ASSERT_MATRIX_APPROX(v , Vector3s::Zero(), 1e-10);
}

TEST(IMU_tools, inverse)
{
    VectorXs d(10), id(10), iid(10), iiid(10), I(10);
    Vector4s qv;
    Scalar dt = 0.1;

    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d << 0, 1, 2, qv, 7, 8, 9;

    id   = inverse(d, dt);

    compose(id, d, dt, I);
    ASSERT_MATRIX_APPROX(I, identity(), 1e-10);
    compose(d, id, -dt, I); // Observe -dt is reversed !!
    ASSERT_MATRIX_APPROX(I, identity(), 1e-10);

    inverse(id, -dt, iid); // Observe -dt is reversed !!
    ASSERT_MATRIX_APPROX( d,  iid, 1e-10);
    iiid = inverse(iid, dt);
    ASSERT_MATRIX_APPROX(id, iiid, 1e-10);
}

TEST(IMU_tools, compose_between)
{
    VectorXs dx1(10), dx2(10), dx3(10);
    Matrix<Scalar, 10, 1> d1, d2, d3;
    Vector4s qv;
    Scalar dt = 0.1;

    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    dx1 << 0, 1, 2, qv, 7, 8, 9;
    d1 = dx1;
    qv = (Vector4s() << 6, 5, 4, 3).finished().normalized();
    dx2 << 9, 8, 7, qv, 2, 1, 0;
    d2 = dx2;

    // combinations of templates for sum()
    compose(dx1, dx2, dt, dx3);
    compose(d1, d2, dt, d3);
    ASSERT_MATRIX_APPROX(d3, dx3, 1e-10);

    compose(d1, dx2, dt, dx3);
    d3 = compose(dx1, d2, dt);
    ASSERT_MATRIX_APPROX(d3, dx3, 1e-10);

    // minus(d1, d3) should recover delta_2
    VectorXs diffx(10);
    Matrix<Scalar,10,1> diff;
    between(d1, d3, dt, diff);
    ASSERT_MATRIX_APPROX(diff, d2, 1e-10);

    // minus(d3, d1, -dt) should recover inverse(d2, dt)
    diff = between(d3, d1, -dt);
    ASSERT_MATRIX_APPROX(diff, inverse(d2, dt), 1e-10);
}

TEST(IMU_tools, compose_between_with_state)
{
    VectorXs x(10), d(10), x2(10), x3(10), d2(10), d3(10);
    Vector4s qv;
    Scalar dt = 0.1;

    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    x << 0, 1, 2, qv, 7, 8, 9;
    qv = (Vector4s() << 6, 5, 4, 3).finished().normalized();
    d << 9, 8, 7, qv, 2, 1, 0;

    composeOverState(x, d, dt, x2);
    x3 = composeOverState(x, d, dt);
    ASSERT_MATRIX_APPROX(x3, x2, 1e-10);

    // betweenStates(x, x2) should recover d
    betweenStates(x, x2, dt, d2);
    d3 = betweenStates(x, x2, dt);
    ASSERT_MATRIX_APPROX(d2, d, 1e-10);
    ASSERT_MATRIX_APPROX(d3, d, 1e-10);
    ASSERT_MATRIX_APPROX(betweenStates(x, x2, dt), d, 1e-10);

    // x + (x2 - x) = x2
    ASSERT_MATRIX_APPROX(composeOverState(x, betweenStates(x, x2, dt), dt), x2, 1e-10);

    // (x + d) - x = d
    ASSERT_MATRIX_APPROX(betweenStates(x, composeOverState(x, d, dt), dt), d, 1e-10);
}

TEST(IMU_tools, lift_retract)
{
    VectorXs d_min(9); d_min << 0, 1, 2, .3, .4, .5, 6, 7, 8; // use angles in the ball theta < pi

    // transform to delta
    VectorXs delta = retract(d_min);

    // expected delta
    Vector3s dp = d_min.head(3);
    Quaternions dq = v2q(d_min.segment(3,3));
    Vector3s dv = d_min.tail(3);
    VectorXs delta_true(10); delta_true << dp, dq.coeffs(), dv;
    ASSERT_MATRIX_APPROX(delta, delta_true, 1e-10);

    // transform to a new tangent -- should be the original tangent
    VectorXs d_from_delta = lift(delta);
    ASSERT_MATRIX_APPROX(d_from_delta, d_min, 1e-10);

    // transform to a new delta -- should be the previous delta
    VectorXs delta_from_d = retract(d_from_delta);
    ASSERT_MATRIX_APPROX(delta_from_d, delta, 1e-10);
}

TEST(IMU_tools, plus)
{
    VectorXs d1(10), d2(10), d3(10);
    VectorXs err(9);
    Vector4s qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d1 << 0, 1, 2, qv, 7, 8, 9;
    err << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09;

    d3.head(3) = d1.head(3) + err.head(3);
    d3.segment(3,4) = (Quaternions(qv.data()) * exp_q(err.segment(3,3))).coeffs();
    d3.tail(3) = d1.tail(3) + err.tail(3);

    plus(d1, err, d2);
    ASSERT_MATRIX_APPROX(diff(d3, d2), VectorXs::Zero(9), 1e-10);
}

TEST(IMU_tools, diff)
{
    VectorXs d1(10), d2(10);
    Vector4s qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d1 << 0, 1, 2, qv, 7, 8, 9;
    d2 = d1;

    VectorXs err(9);
    diff(d1, d2, err);
    ASSERT_MATRIX_APPROX(err, VectorXs::Zero(9), 1e-10);
    ASSERT_MATRIX_APPROX(diff(d1, d2), VectorXs::Zero(9), 1e-10);

    VectorXs d3(10);
    d3.setRandom(); d3.segment(3,4).normalize();
    err.head(3) = d3.head(3) - d1.head(3);
    err.segment(3,3) = log_q(Quaternions(d1.data()+3).conjugate()*Quaternions(d3.data()+3));
    err.tail(3) = d3.tail(3) - d1.tail(3);

    ASSERT_MATRIX_APPROX(err, diff(d1, d3), 1e-10);

}

TEST(IMU_tools, compose_jacobians)
{
    VectorXs d1(10), d2(10), d3(10), d1_pert(10), d2_pert(10), d3_pert(10); // deltas
    VectorXs t1(9), t2(9); // tangents
    Matrix<Scalar, 9, 9> J1_a, J2_a, J1_n, J2_n;
    Vector4s qv1, qv2;
    Scalar dt = 0.1;
    Scalar dx = 1e-6;
    qv1 = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d1 << 0, 1, 2, qv1, 7, 8, 9;
    qv2 = (Vector4s() << 1, 2, 3, 4).finished().normalized();
    d2 << 9, 8, 7, qv2, 2, 1, 0;

    // analytical jacobians
    compose(d1, d2, dt, d3, J1_a, J2_a);

    // numerical jacobians
    for (unsigned int i = 0; i<9; i++)
    {
        t1      . setZero();
        t1(i)   = dx;

        // Jac wrt first delta
        d1_pert = plus(d1, t1);                 //     (d1 + t1)
        d3_pert = compose(d1_pert, d2, dt);     //     (d1 + t1) + d2
        t2      = diff(d3, d3_pert);            //   { (d2 + t1) + d2 } - { d1 + d2 }
        J1_n.col(i) = t2/dx;                    // [ { (d2 + t1) + d2 } - { d1 + d2 } ] / dx

        // Jac wrt second delta
        d2_pert = plus(d2, t1);                 //          (d2 + t1)
        d3_pert = compose(d1, d2_pert, dt);     //     d1 + (d2 + t1)
        t2      = diff(d3, d3_pert);            //   { d1 + (d2 + t1) } - { d1 + d2 }
        J2_n.col(i) = t2/dx;                    // [ { d1 + (d2 + t1) } - { d1 + d2 } ] / dx
    }

    // check that numerical and analytical match
    ASSERT_MATRIX_APPROX(J1_a, J1_n, 1e-4);
    ASSERT_MATRIX_APPROX(J2_a, J2_n, 1e-4);
}

TEST(IMU_tools, diff_jacobians)
{
    VectorXs d1(10), d2(10), d3(9), d1_pert(10), d2_pert(10), d3_pert(9); // deltas
    VectorXs t1(9), t2(9); // tangents
    Matrix<Scalar, 9, 9> J1_a, J2_a, J1_n, J2_n;
    Vector4s qv1, qv2;
    Scalar dx = 1e-6;
    qv1 = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d1 << 0, 1, 2, qv1, 7, 8, 9;
    qv2 = (Vector4s() << 1, 2, 3, 4).finished().normalized();
    d2 << 9, 8, 7, qv2, 2, 1, 0;

    // analytical jacobians
    diff(d1, d2, d3, J1_a, J2_a);

    // numerical jacobians
    for (unsigned int i = 0; i<9; i++)
    {
        t1      . setZero();
        t1(i)   = dx;

        // Jac wrt first delta
        d1_pert = plus(d1, t1);                 //          (d1 + t1)
        d3_pert = diff(d1_pert, d2);            //     d2 - (d1 + t1)
        t2      = d3_pert - d3;                 //   { d2 - (d1 + t1) } - { d2 - d1 }
        J1_n.col(i) = t2/dx;                    // [ { d2 - (d1 + t1) } - { d2 - d1 } ] / dx

        // Jac wrt second delta
        d2_pert = plus(d2, t1);                 //     (d2 + t1)
        d3_pert = diff(d1, d2_pert);            //     (d2 + t1) - d1
        t2      = d3_pert - d3;                 //   { (d2 + t1) - d1 } - { d2 - d1 }
        J2_n.col(i) = t2/dx;                    // [ { (d2 + t1) - d1 } - { d2 - d1 } ] / dx
    }

    // check that numerical and analytical match
    ASSERT_MATRIX_APPROX(J1_a, J1_n, 1e-4);
    ASSERT_MATRIX_APPROX(J2_a, J2_n, 1e-4);
}

TEST(IMU_tools, body2delta_jacobians)
{
    VectorXs delta(10), delta_pert(10); // deltas
    VectorXs body(6), pert(6);
    VectorXs tang(9); // tangents
    Matrix<Scalar, 9, 6> J_a, J_n; // analytic and numerical jacs
    Vector4s qv;;
    Scalar dt = 0.1;
    Scalar dx = 1e-6;
    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    delta << 0, 1, 2,   qv,   7, 8, 9;
    body << 1, 2, 3,   3, 2, 1;

    // analytical jacobians
    body2delta(body, dt, delta, J_a);

    // numerical jacobians
    for (unsigned int i = 0; i<6; i++)
    {
        pert      . setZero();
        pert(i)   = dx;

        // Jac wrt first delta
        delta_pert = body2delta(body + pert, dt);   //     delta(body + pert)
        tang       = diff(delta, delta_pert);       //   delta(body + pert) -- delta(body)
        J_n.col(i) = tang/dx;                       // ( delta(body + pert) -- delta(body) ) / dx
    }

    // check that numerical and analytical match
    ASSERT_MATRIX_APPROX(J_a, J_n, 1e-4);
}

TEST(motion2data, zero)
{
    Vector6s motion;
    Vector6s bias;
    Quaternions q;

    motion  .setZero();
    bias    .setZero();
    q       .setIdentity();

    Vector6s data = imu::motion2data(motion, q, bias);

    Vector6s data_true; data_true << -gravity(), Vector3s::Zero();

    ASSERT_MATRIX_APPROX(data, data_true, 1e-12);
}

TEST(motion2data, motion)
{
    Vector6s motion, g_extended;
    Vector6s bias;
    Quaternions q;

    g_extended << gravity() , Vector3s::Zero();

    motion  << 1,2,3, 4,5,6;
    bias    .setZero();
    q       .setIdentity();

    Vector6s data = imu::motion2data(motion, q, bias);

    Vector6s data_true; data_true = motion - g_extended;

    ASSERT_MATRIX_APPROX(data, data_true, 1e-12);
}

TEST(motion2data, bias)
{
    Vector6s motion, g_extended;
    Vector6s bias;
    Quaternions q;

    g_extended << gravity() , Vector3s::Zero();

    motion  .setZero();
    bias    << 1,2,3, 4,5,6;
    q       .setIdentity();

    Vector6s data = imu::motion2data(motion, q, bias);

    Vector6s data_true; data_true = bias - g_extended;

    ASSERT_MATRIX_APPROX(data, data_true, 1e-12);
}

TEST(motion2data, orientation)
{
    Vector6s motion, g_extended;
    Vector6s bias;
    Quaternions q;

    g_extended << gravity() , Vector3s::Zero();

    motion  .setZero();
    bias    .setZero();
    q       = v2q(Vector3s(M_PI/2, 0, 0)); // turn 90 deg in X axis

    Vector6s data = imu::motion2data(motion, q, bias);

    Vector6s data_true; data_true << 0,-gravity()(2),0, 0,0,0;

    ASSERT_MATRIX_APPROX(data, data_true, 1e-12);
}

TEST(motion2data, AllRandom)
{
    Vector6s motion, g_extended;
    Vector6s bias;
    Quaternions q;


    motion      .setRandom();
    bias        .setRandom();
    q.coeffs()  .setRandom().normalize();

    g_extended << q.conjugate()*gravity() , Vector3s::Zero();

    Vector6s data = imu::motion2data(motion, q, bias);

    Vector6s data_true; data_true = motion + bias - g_extended;

    ASSERT_MATRIX_APPROX(data, data_true, 1e-12);
}


/* Integrate acc and angVel motion, obtain Delta_preintegrated
 * Input:
 *   N: number of steps
 *   q0: initial orientaiton
 *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body brame
 *   bias_real: the real bias of the IMU
 *   bias_preint: the bias used for Delta pre-integration
 * Output:
 *   return: the preintegrated delta
 */
VectorXs integrateDelta(int N, const Quaternions& q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt)
{
    VectorXs data(6);
    VectorXs body(6);
    VectorXs delta(10);
    VectorXs Delta(10), Delta_plus(10);
    Delta << 0,0,0, 0,0,0,1, 0,0,0;
    Quaternions q(q0);
    for (int n = 0; n<N; n++)
    {
        data        = motion2data(motion, q, bias_real);
        q           = q*exp_q(motion.tail(3)*dt);
        body        = data - bias_preint;
        delta       = body2delta(body, dt);
        Delta_plus  = compose(Delta, delta, dt);
        Delta       = Delta_plus;
    }
    return Delta;
}

/* Integrate acc and angVel motion, obtain Delta_preintegrated
 * Input:
 *   N: number of steps
 *   q0: initial orientaiton
 *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body brame
 *   bias_real: the real bias of the IMU
 *   bias_preint: the bias used for Delta pre-integration
 * Output:
 *   J_D_b: the Jacobian of the preintegrated delta wrt the bias
 *   return: the preintegrated delta
 */
VectorXs integrateDelta(int N, const Quaternions& q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, Matrix<Scalar, 9, 6>& J_D_b)
{
    VectorXs data(6);
    VectorXs body(6);
    VectorXs delta(10);
    Matrix<Scalar, 9, 6> J_d_d, J_d_b;
    Matrix<Scalar, 9, 9> J_D_D, J_D_d;
    VectorXs Delta(10), Delta_plus(10);
    Quaternions q;

    Delta << 0,0,0, 0,0,0,1, 0,0,0;
    J_D_b.setZero();
    q = q0;
    for (int n = 0; n<N; n++)
    {
        // Simulate data
        data = motion2data(motion, q, bias_real);
        q    = q*exp_q(motion.tail(3)*dt);
        // Motion::integrateOneStep()
        {   // IMU::computeCurrentDelta
            body  = data - bias_preint;
            body2delta(body, dt, delta, J_d_d);
            J_d_b = - J_d_d;
        }
        {   // IMU::deltaPlusDelta
            compose(Delta, delta, dt, Delta_plus, J_D_D, J_D_d);
        }
        // Motion:: jac calib
        J_D_b = J_D_D*J_D_b + J_D_d*J_d_b;
        // Motion:: buffer
        Delta = Delta_plus;
    }
    return Delta;
}

TEST(IMU_tools, integral_jacobian_bias)
{
    VectorXs Delta(10), Delta_pert(10); // deltas
    VectorXs bias_real(6), pert(6), bias_pert(6), bias_preint(6);
    VectorXs body(6), data(6), motion(6);
    VectorXs tang(9); // tangents
    Matrix<Scalar, 9, 6> J_a, J_n; // analytic and numerical jacs
    Scalar dt = 0.001;
    Scalar dx = 1e-4;
    Quaternions q0(3, 4, 5, 6); q0.normalize();
    motion << .1, .2, .3,   .3, .2, .1;
    bias_real   << .002, .004, .001,   .003, .002, .001;
    bias_preint << .004, .005, .006,   .001, .002, .003;

    int N = 500; // # steps of integration

    // Analytical Jacobians
    Delta = integrateDelta(N, q0, motion, bias_real, bias_preint, dt, J_a);

    // numerical jacobians
    for (unsigned int i = 0; i<6; i++)
    {
        pert       . setZero();
        pert(i)    = dx;

        bias_pert  = bias_preint + pert;
        Delta_pert = integrateDelta(N, q0, motion, bias_real, bias_pert, dt);
        tang       = diff(Delta, Delta_pert);       //   Delta(bias + pert) -- Delta(bias)
        J_n.col(i) = tang/dx;                       // ( Delta(bias + pert) -- Delta(bias) ) / dx
    }

    // check that numerical and analytical match
    ASSERT_MATRIX_APPROX(J_a, J_n, 1e-4);
}

TEST(IMU_tools, delta_correction)
{
    VectorXs Delta(10), Delta_preint(10), Delta_corr; // deltas
    VectorXs bias(6), pert(6), bias_preint(6);
    VectorXs body(6), motion(6);
    VectorXs tang(9); // tangents
    Matrix<Scalar, 9, 6> J_b; // analytic and numerical jacs
    Vector4s qv;;
    Scalar dt = 0.001;
    Quaternions q0(3, 4, 5, 6); q0.normalize();
    motion << 1, 2, 3,   3, 2, 1; motion *= .1;

    int N = 1000; // # steps of integration

    // preintegration with correct bias
    bias << .004, .005, .006,   .001, .002, .003;
    Delta = integrateDelta(N, q0, motion, bias, bias, dt);

    // preintegration with wrong bias
    pert << .002, -.001, .003,   -.0003, .0002, -.0001;
    bias_preint = bias + pert;
    Delta_preint = integrateDelta(N, q0, motion, bias, bias_preint, dt, J_b);

    // correct perturbated
    Vector9s step = J_b*(bias-bias_preint);
    Delta_corr = plus(Delta_preint, step);

    // Corrected delta should match real delta
    ASSERT_MATRIX_APPROX(Delta, Delta_corr, 1e-5);

    // diff between real and corrected should be zero
    ASSERT_MATRIX_APPROX(diff(Delta, Delta_corr), Vector9s::Zero(), 1e-5);

    // diff between preint and corrected deltas should be the jacobian-computed step
    ASSERT_MATRIX_APPROX(diff(Delta_preint, Delta_corr), step, 1e-5);
}

TEST(IMU_tools, full_delta_residual)
{
    VectorXs x1(10), x2(10);
    VectorXs Delta(10), Delta_preint(10), Delta_corr(10);
    VectorXs Delta_real(9), Delta_err(9), Delta_diff(10), Delta_exp(10);
    VectorXs bias(6), pert(6), bias_preint(6), bias_null(6);
    VectorXs body(6), motion(6);
    VectorXs tang(9); // tangents
    Matrix<Scalar, 9, 6> J_b; // analytic and numerical jacs
    Scalar dt = 0.001;
    Quaternions q0; q0.setIdentity();

    x1          << 0,0,0, 0,0,0,1, 0,0,0;
    motion      <<  .3,    .2,    .1,      .1,     .2,     .3; // acc and gyro
//    motion      <<  .3,    .2,    .1,      .0,     .0,     .0; // only acc
//    motion      <<  .0,    .0,    .0,      .1,     .2,     .3; // only gyro
    bias        << 0.01, 0.02, 0.003,   0.002, 0.005, 0.01;
    bias_null   << 0,     0,     0,       0,      0,      0;
//    bias_preint << 0.003, 0.002, 0.001,   0.001,  0.002,  0.003;
    bias_preint = bias_null;

    int N = 1000; // # steps of integration

    // preintegration with no bias
    Delta_real = integrateDelta(N, q0, motion, bias_null, bias_null, dt);

    // final state
    x2 = composeOverState(x1, Delta_real, 1000*dt);

    // preintegration with the correct bias
    Delta = integrateDelta(N, q0, motion, bias, bias, dt);

    ASSERT_MATRIX_APPROX(Delta, Delta_real, 1e-4);

    // preintegration with wrong bias
    Delta_preint = integrateDelta(N, q0, motion, bias, bias_preint, dt, J_b);

    // compute correction step
    Vector9s step = J_b*(bias-bias_preint);

    // correct preintegrated delta
    Delta_corr = plus(Delta_preint, step);

    // Corrected delta should match real delta
    ASSERT_MATRIX_APPROX(Delta_real, Delta_corr, 1e-3);

    // diff between real and corrected should be zero
    ASSERT_MATRIX_APPROX(diff(Delta_real, Delta_corr), Vector9s::Zero(), 1e-3);

    // expected delta
    Delta_exp = betweenStates(x1, x2, N*dt);

    ASSERT_MATRIX_APPROX(Delta_exp, Delta_corr, 1e-3);

    // compute diff between expected and corrected
//    Matrix<Scalar, 9, 9> J_err_exp, J_err_corr;
    diff(Delta_exp, Delta_corr, Delta_err); //, J_err_exp, J_err_corr);

    ASSERT_MATRIX_APPROX(Delta_err, Vector9s::Zero(), 1e-3);

    // compute error between expected and preintegrated
    Delta_err = diff(Delta_preint, Delta_exp);

    // diff between preint and corrected deltas should be the jacobian-computed step
    ASSERT_MATRIX_APPROX(diff(Delta_preint, Delta_corr), step, 1e-3);
    /* This implies:
     *   - Since D_corr is expected to be similar to D_exp
     *      step ~ diff(Delta_preint, Delta_exp)
     *   - the residual can be computed with a regular '-' :
     *      residual = diff(D_preint, D_exp) - J_bias * (bias - bias_preint)
     */

//    WOLF_TRACE("Delta real: ", Delta_real.transpose());
//    WOLF_TRACE("x2: ", x2.transpose());
//    WOLF_TRACE("Delta: ", Delta.transpose());
//    WOLF_TRACE("Delta pre: ", Delta_preint.transpose());
//    WOLF_TRACE("Jac bias: \n", J_b);
//    WOLF_TRACE("Delta step: ", step.transpose());
//    WOLF_TRACE("Delta cor: ", Delta_corr.transpose());
//    WOLF_TRACE("Delta exp: ", Delta_exp.transpose());
//    WOLF_TRACE("Delta err: ", Delta_err.transpose());
//    WOLF_TRACE("Delta err exp-pre: ", Delta_err.transpose());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
//  ::testing::GTEST_FLAG(filter) = "IMU_tools.delta_correction";
  return RUN_ALL_TESTS();
}

