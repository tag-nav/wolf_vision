/**
 * \file test_rotation.cpp
 *
 *  Created on: Oct 13, 2016
 *      \author: AtDinesh
 */

//Eigen
#include <eigen3/Eigen/Geometry>

//Wolf
#include "wolf.h"
#include "../rotations.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include "utils_gtest.h"

//#define write_results

// THESE ARE UNITARY TESTS FOR METHODS IN ROTATION.H

using namespace wolf;
using namespace Eigen;

TEST(rotations, pi2pi)
{
    ASSERT_NEAR(M_PI_2, pi2pi((Scalar)M_PI_2), 1e-10);
    ASSERT_NEAR(-M_PI_2, pi2pi(3.0*M_PI_2), 1e-10);
    ASSERT_NEAR(-M_PI_2, pi2pi(-M_PI_2), 1e-10);
    ASSERT_NEAR(M_PI_2, pi2pi(-3.0*M_PI_2), 1e-10);
    //    ASSERT_NEAR(M_PI, pi2pi(M_PI), 1e-10); // Exact PI is not safely testable because of numeric issues.
    ASSERT_NEAR(M_PI-.01, pi2pi(M_PI-.01), 1e-10);
    ASSERT_NEAR(-M_PI+.01, pi2pi(M_PI+.01), 1e-10);
}

TEST(rotations, Skew_vee)
{
    using namespace wolf;
    Vector3s vec3 = Vector3s::Random();
    Matrix3s skew_mat;
    skew_mat = skew(vec3);

    // vee
    Vector3s vec3_bis;
    vec3_bis = vee(skew_mat);

    ASSERT_TRUE(vec3_bis == vec3);
}

TEST(rotations, v2q_q2v)
{
    using namespace wolf;
    //defines scalars
    wolf::Scalar deg_to_rad = M_PI/180.0;

    Vector4s vec0, vec1;

    //v2q
    Vector3s rot_vector0, rot_vector1;
    rot_vector0 = Vector3s::Random();
    rot_vector1 = rot_vector0 * 100 *deg_to_rad; //far from origin
    rot_vector0 = rot_vector0*deg_to_rad;

    Quaternions quat0, quat1;
    quat0 = v2q(rot_vector0);
    quat1 = v2q(rot_vector1);

    //q2v
    Vector3s quat_to_v0, quat_to_v1;
    VectorXs quat_to_v0x, quat_to_v1x;

    quat_to_v0 = q2v(quat0);
    quat_to_v1 = q2v(quat1);
    quat_to_v0x = q2v(quat0);
    quat_to_v1x = q2v(quat1);

    ASSERT_MATRIX_APPROX(rot_vector0, quat_to_v0, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(rot_vector1, quat_to_v1, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(rot_vector0, quat_to_v0x, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(rot_vector1, quat_to_v1x, wolf::Constants::EPS);
}

TEST(rotations, v2R_R2v)
{
    using namespace wolf;
    //First test is to verify we get the good result with v -> v2R -> R2v -> v
    //test 2 : how small can angles in rotation vector be ?

    //definition
    wolf::Scalar deg_to_rad = M_PI/180.0;
    Vector3s rot_vector0, rot_vector1;

    rot_vector0 = Vector3s::Random();
    rot_vector1 = rot_vector0 * 100 *deg_to_rad; //far from origin
    rot_vector0 = rot_vector0*deg_to_rad;

    Matrix3s rot0, rot1;
    rot0 = v2R(rot_vector0);
    rot1 = v2R(rot_vector1);

    //R2v
    Vector3s rot0_vec, rot1_vec;
    rot0_vec = R2v(rot0);
    rot1_vec = R2v(rot1);

    //check now
    ASSERT_MATRIX_APPROX(rot0_vec, rot_vector0, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(rot1_vec, rot_vector1, wolf::Constants::EPS);
}

TEST(rotations, R2v_v2R_limits)
{
    using namespace wolf;
    //test 2 : how small can angles in rotation vector be ?
    wolf::Scalar scale = 1;
    Matrix3s v_to_R, initial_matrix;
    Vector3s  R_to_v;

    //Vector3s rv;
    for(int i = 0; i<8; i++){
        initial_matrix = v2R(Vector3s::Random() * scale);

        R_to_v = R2v(initial_matrix);     
        v_to_R = v2R(R_to_v);

        ASSERT_MATRIX_APPROX(v_to_R, initial_matrix, wolf::Constants::EPS);
        scale = scale*0.1;
    }
}

TEST(rotations, R2v_v2R_AAlimits)
{
    using namespace wolf;
    //let's see how small the angles can be here : limit reached at scale/10 =  1e-16
    wolf::Scalar scale = 1;
    Matrix3s rotation_mat;
    Vector3s rv;

    for(int i = 0; i<8; i++){
        rotation_mat = v2R(Vector3s::Random() * scale);
        //rotation_mat(0,0) = 1.0;
        //rotation_mat(1,1) = 1.0;
        //rotation_mat(2,2) = 1.0;

        //rv = R2v(rotation_mat); //decomposing R2v below
        AngleAxis<wolf::Scalar> aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa0.axis() * aa0.angle();
        //std::cout << "aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;

        ASSERT_FALSE(rv == Vector3s::Zero());
        scale = scale*0.1;
    }
}

TEST(rotations, v2q2R2v)
{
    using namespace wolf;
    wolf::Scalar scale = 1;
    // testing new path : vector -> quaternion -> matrix -> vector

    for(int i = 0; i< 8; i++){
        Vector3s vector_ = Vector3s::Random()*scale;
        Quaternions quat_ = v2q(vector_);
        Matrix3s mat_ = quat_.matrix();
        Vector3s vector_bis = R2v(mat_);

        ASSERT_MATRIX_APPROX(vector_, vector_bis, wolf::Constants::EPS);
        scale = scale*0.1;
    }
}

TEST(rotations, AngleAxis_limits)
{
    using namespace wolf;
    //Hypothesis : problem with construction of AngleAxis objects.
    // Example : if R = I + [delta t]_x (happenning in the IMU case with delta t = 0.001). Then angle mays be evaluated as 0 (due to cosinus definition ?) 
    // Here we try to get the AngleAxis object from a random rotation matrix, then get back to the rotation matrix using AngleAxis::toRotationMatrix()

    wolf::Scalar scale = 1;
    Matrix3s res, res_i, rotation_mati, rotation_mat;
    Vector3s rv;

    for(int i = 0; i<8; i++){ //FIX ME : Random() will not create a rotation matrix. Then, R2v(Random_matrix()) makes no sense at all.
        rotation_mat = v2R(Vector3s::Random() * scale);
        rotation_mati = rotation_mat;

        //rv = R2v(rotation_mat); //decomposing R2v below
        AngleAxis<wolf::Scalar> aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa0.axis() * aa0.angle();
        //std::cout << "aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;
        res = aa0.toRotationMatrix();

        // now we set the diagonal to identity
        AngleAxis<wolf::Scalar> aa1 = AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa1.axis() * aa1.angle();
        //std::cout << "aa1.axis : " << aa0.axis().transpose() << ",\t aa1.angles :" << aa0.angle() <<std::endl;
        res_i = aa1.toRotationMatrix();

        ASSERT_MATRIX_APPROX(res, rotation_mat, wolf::Constants::EPS);
        ASSERT_MATRIX_APPROX(res_i, rotation_mati, wolf::Constants::EPS);
        scale = scale*0.1;
    }
}

TEST(rotations, AngleAxis_limits2)
{
    using namespace wolf;

    Matrix3s rotation_mat;
    Vector3s rv;
    AngleAxis<wolf::Scalar> aa0;

    //FIX ME : 5. Building a rot mat doing this is not safe; You cannot guarantee that R is valid.
    // Highlight limitation of AngleAxis
    rotation_mat = skew(Vector3s::Random()) *0.0001;
    rotation_mat(0,0) = 1;
    rotation_mat(1,1) = 0.999999;
    rotation_mat(2,2) = 1;
    aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();
    //checking if rv is 0 vector
    ASSERT_FALSE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    rotation_mat(0,0) = 1.0;
    rotation_mat(1,1) = 1.0;
    rotation_mat(2,2) = 1.0;
    aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();
    //checking if rv is 0 vector
    ASSERT_TRUE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    rotation_mat = skew(Vector3s::Random()) *0.1;
    rotation_mat(0,0) = 1;
    rotation_mat(1,1) = 0.9999999;
    rotation_mat(2,2) = 1;
    aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();
    //checking if rv is 0 vector
    ASSERT_FALSE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    rotation_mat(0,0) = 1.0;
    rotation_mat(1,1) = 1.0;
    rotation_mat(2,2) = 1.0;
    aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();
    //checking if rv is 0 vector
    ASSERT_TRUE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
}

TEST(rotations, Quat_compos_const_rateOfTurn)
{
    using namespace wolf;

    // ********* constant rate of turn *********
    wolf::Scalar deg_to_rad = M_PI/180.0;
    Quaternions q0;
    q0.setIdentity();
    Vector3s v0, v1, v2;
    VectorXs const_diff_ox, const_diff_oy, const_diff_oz, ox, oy, oz, qox, qoy, qoz;
    VectorXs cdox_abs, cdoy_abs, cdoz_abs, vector0, t_vec; //= const_diff_## with absolute values
    const wolf::Scalar dt = 0.001;
    const wolf::Scalar N = 100;

    v0 << 30.0*deg_to_rad, 5.0*deg_to_rad, 10.0*deg_to_rad; //constant rate-of-turn in rad/s
    const_diff_ox.resize(N/dt);
    const_diff_oy.resize(N/dt);
    const_diff_oz.resize(N/dt);
    cdox_abs.resize(N/dt);
    cdoy_abs.resize(N/dt);
    cdoz_abs.resize(N/dt);
    vector0 = VectorXs::Zero(N/dt);
    t_vec.resize(N/dt);
    ox.resize(N/dt);
    oy.resize(N/dt);
    oz.resize(N/dt);
    qox.resize(N/dt);
    qoy.resize(N/dt);
    qoz.resize(N/dt);

    for(wolf::Scalar n=0; n<N/dt; n++){
        v2 = q2v(v2q(v0*n*dt));
        ox(n) = v2(0);
        oy(n) = v2(1);
        oz(n) = v2(2);
        /*ox(n) = pi2pi(v0(0)*n*dt);
        oy(n) = pi2pi(v0(1)*n*dt);
        oz(n) = pi2pi(v0(2)*n*dt);*/
        t_vec(n) = n*dt;
    }

    for(wolf::Scalar n=0; n<N/dt; n++){
        if(n!=0)
            q0 = q0 * v2q(v0*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)
        v1 = q2v(q0);   //corresponding rotation vector of current quaternion
        qox(n) = v1(0); //angle X component
        qoy(n) = v1(1); //angle Y component
        qoz(n) = v1(2); //angle Z component
    }

    //Compute difference between orientation vectors (expected - real)
    const_diff_ox = ox - qox;
    const_diff_oy = oy - qoy;
    const_diff_oz = oz - qoz;

    //get absolute difference
    cdox_abs = const_diff_ox.array().abs();
    cdoy_abs = const_diff_oy.array().abs();
    cdoz_abs = const_diff_oz.array().abs();

    ASSERT_TRUE((ox - qox).isMuchSmallerThan(1,0.000001) && (oy - qoy).isMuchSmallerThan(1,0.000001) && (oz - qoz).isMuchSmallerThan(1,0.000001)) <<
            "max orientation error in abs value (x, y, z) : " << cdox_abs.maxCoeff() << "\t" << cdoy_abs.maxCoeff() << "\t" << cdoz_abs.maxCoeff() << std::endl;

#ifdef write_results
    std::ofstream const_rot;
    const_rot.open("const_rot.dat");
    if(const_rot){
        const_rot << "%%timestamp\t" << "ox\t" << "oy\t" << "oz\t" << "qox\t" << "qoy\t" << "qoz\t" << "\n";
        for(int i = 0; i<N/dt; i++)
            const_rot << t_vec(i) << "\t" << ox(i) << "\t" << oy(i) << "\t" << oz(i) << "\t" << qox(i) << "\t" << qoy(i) << "\t" << qoz(i) << "\n";
        const_rot.close();
    }
    else
        PRINTF("could not open file const_rot");
#endif

}

TEST(rotations, Quat_compos_var_rateOfTurn)
{
    using namespace wolf;

    //********* changing rate of turn - same freq for all axis *********
    wolf::Scalar deg_to_rad = M_PI/180.0;
    Quaternions q0;
    q0.setIdentity();
    Vector3s v0, v1, v2;
    VectorXs const_diff_ox, const_diff_oy, const_diff_oz, ox, oy, oz, qox, qoy, qoz;
    VectorXs cdox_abs, cdoy_abs, cdoz_abs, vector0, t_vec; //= const_diff_## with absolute values
    const wolf::Scalar dt = 0.001;
    const wolf::Scalar N = 100;

    const_diff_ox.resize(N/dt);
    const_diff_oy.resize(N/dt);
    const_diff_oz.resize(N/dt);
    cdox_abs.resize(N/dt);
    cdoy_abs.resize(N/dt);
    cdoz_abs.resize(N/dt);
    vector0 = VectorXs::Zero(N/dt);
    t_vec.resize(N/dt);
    ox.resize(N/dt);
    oy.resize(N/dt);
    oz.resize(N/dt);
    qox.resize(N/dt);
    qoy.resize(N/dt);
    qoz.resize(N/dt);

    wolf::Scalar alpha, beta, gamma;
    alpha = 10;
    beta = 10;
    gamma = 10;
    v0 << alpha*deg_to_rad, beta*deg_to_rad, gamma*deg_to_rad;

    for(wolf::Scalar n=0; n<N/dt; n++){
        v1 << sin(v0(0)*n*dt), sin(v0(1)*n*dt), sin(v0(2)*n*dt);
        v1 = q2v(v2q(v1));
        ox(n) = v1(0);
        oy(n) = v1(1);
        oz(n) = v1(2);
        /*ox(n) = pi2pi(v0(0)*n*dt);
        oy(n) = pi2pi(v0(1)*n*dt);
        oz(n) = pi2pi(v0(2)*n*dt);*/
        t_vec(n) = n*dt;
    }

    for(wolf::Scalar n=0; n<N/dt; n++){
        if(n!=0){
            v2 << v0(0)*cos(v0(0)*n*dt), v0(1)*cos(v0(1)*n*dt), v0(2)*cos(v0(2)*n*dt);
            q0 = q0 * v2q(v2*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)
        }
        v1 = q2v(q0);   //corresponding rotation vector of current quaternion
        qox(n) = v1(0); //angle X component
        qoy(n) = v1(1); //angle Y component
        qoz(n) = v1(2); //angle Z component
    }

    //Compute difference between orientation vectors (expected - real)
    const_diff_ox = ox - qox;
    const_diff_oy = oy - qoy;
    const_diff_oz = oz - qoz;

    //get absolute difference
    cdox_abs = const_diff_ox.array().abs();
    cdoy_abs = const_diff_oy.array().abs();
    cdoz_abs = const_diff_oz.array().abs();

    ASSERT_FALSE(cdox_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoy_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoz_abs.isMuchSmallerThan(1,wolf::Constants::EPS)) <<
            "max orientation error in abs value (x, y, z) : " << cdox_abs.maxCoeff() << "\t" << cdoy_abs.maxCoeff() << "\t" << cdoz_abs.maxCoeff() << std::endl;

#ifdef write_results
    std::ofstream sin_rot0;
    sin_rot0.open("sin_rot0.dat");
    if(sin_rot0){
        sin_rot0 << "%%timestamp\t" << "ox\t" << "oy\t" << "oz\t" << "qox\t" << "qoy\t" << "qoz\t" << "\n";
        for(int i = 0; i<N/dt; i++)
            sin_rot0 << t_vec(i) << "\t" << ox(i) << "\t" << oy(i) << "\t" << oz(i) << "\t" << qox(i) << "\t" << qoy(i) << "\t" << qoz(i) << "\n";
        sin_rot0.close();
    }
    else
        PRINTF("could not open file sin_rot0");
#endif
}

TEST(rotations, Quat_compos_var_rateOfTurn_diff)
{
    using namespace wolf;

    //      ********* changing rate of turn - different freq for 1 axis *********
    wolf::Scalar deg_to_rad = M_PI/180.0;
    Quaternions q0;
    q0.setIdentity();
    Vector3s v0, v1, v2;
    VectorXs const_diff_ox, const_diff_oy, const_diff_oz, ox, oy, oz, qox, qoy, qoz;
    VectorXs cdox_abs, cdoy_abs, cdoz_abs, vector0, t_vec; //= const_diff_## with absolute values
    const wolf::Scalar dt = 0.001;
    const wolf::Scalar N = 100;

    const_diff_ox.resize(N/dt);
    const_diff_oy.resize(N/dt);
    const_diff_oz.resize(N/dt);
    cdox_abs.resize(N/dt);
    cdoy_abs.resize(N/dt);
    cdoz_abs.resize(N/dt);
    vector0 = VectorXs::Zero(N/dt);
    t_vec.resize(N/dt);
    ox.resize(N/dt);
    oy.resize(N/dt);
    oz.resize(N/dt);
    qox.resize(N/dt);
    qoy.resize(N/dt);
    qoz.resize(N/dt);

    wolf::Scalar alpha, beta, gamma;
    alpha = 10;
    beta = 5;
    gamma = 10;
    v0 << alpha*deg_to_rad, beta*deg_to_rad, gamma*deg_to_rad;

    for(wolf::Scalar n=0; n<N/dt; n++){
        v1 << sin(v0(0)*n*dt), sin(v0(1)*n*dt), sin(v0(2)*n*dt);
        v1 = q2v(v2q(v1));
        ox(n) = v1(0);
        oy(n) = v1(1);
        oz(n) = v1(2);
        /*ox(n) = pi2pi(v0(0)*n*dt);
        oy(n) = pi2pi(v0(1)*n*dt);
        oz(n) = pi2pi(v0(2)*n*dt);*/
        t_vec(n) = n*dt;
    }

    for(wolf::Scalar n=0; n<N/dt; n++){
        if(n!=0){
            v2 << v0(0)*cos(v0(0)*n*dt), v0(1)*cos(v0(1)*n*dt), v0(2)*cos(v0(2)*n*dt);
            q0 = q0 * v2q(v2*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)
        }
        v1 = q2v(q0);   //corresponding rotation vector of current quaternion
        qox(n) = v1(0); //angle X component
        qoy(n) = v1(1); //angle Y component
        qoz(n) = v1(2); //angle Z component
    }

    //Compute difference between orientation vectors (expected - real)
    const_diff_ox = ox - qox;
    const_diff_oy = oy - qoy;
    const_diff_oz = oz - qoz;

    //get absolute difference
    cdox_abs = const_diff_ox.array().abs();
    cdoy_abs = const_diff_oy.array().abs();
    cdoz_abs = const_diff_oz.array().abs();

    ASSERT_FALSE(cdox_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoy_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoz_abs.isMuchSmallerThan(1,wolf::Constants::EPS)) <<
            "max orientation error in abs value (x, y, z) : " << cdox_abs.maxCoeff() << "\t" << cdoy_abs.maxCoeff() << "\t" << cdoz_abs.maxCoeff() << std::endl;
    //std::cout << "\t quaternion composition with constant rate of turn is NOT OK\n" << std::endl;
    //std::cout << "max orientation error in abs value (x, y, z) : " << cdox_abs.maxCoeff() << "\t" << cdoy_abs.maxCoeff() << "\t" << cdoz_abs.maxCoeff() << std::endl;
#ifdef write_results
    std::ofstream sin_rot;
    sin_rot.open("sin_rot.dat");
    if(sin_rot){
        sin_rot << "%%timestamp\t" << "ox\t" << "oy\t" << "oz\t" << "qox\t" << "qoy\t" << "qoz\t" << "\n";
        for(int i = 0; i<N/dt; i++)
            sin_rot << t_vec(i) << "\t" << ox(i) << "\t" << oy(i) << "\t" << oz(i) << "\t" << qox(i) << "\t" << qoy(i) << "\t" << qoz(i) << "\n";
        sin_rot.close();
    }
    else
        PRINTF("could not open file sin_rot");
#endif
}

TEST(rotations, q2R_R2q)
{
    Vector3s v; v.setRandom();
    Quaternions q = v2q(v);
    Matrix3s R = v2R(v);

    Quaternions q_R = R2q(R);
    Quaternions qq_R(R);

    ASSERT_NEAR(q.norm(),    1, wolf::Constants::EPS);
    ASSERT_NEAR(q_R.norm(),  1, wolf::Constants::EPS);
    ASSERT_NEAR(qq_R.norm(), 1, wolf::Constants::EPS);

    ASSERT_MATRIX_APPROX(q.coeffs(), R2q(R).coeffs(), wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(q.coeffs(), qq_R.coeffs(),   wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(R,          q2R(q),          wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(R,          qq_R.matrix(),   wolf::Constants::EPS);
}

TEST(rotations, Jr)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;

    // Check the main Jr property for q and R
    // exp( theta + d_theta ) \approx exp(theta) * exp(Jr * d_theta)
    Matrix3s Jr = jac_SO3_right(theta);
    ASSERT_QUATERNION_APPROX(exp_q(theta+dtheta), exp_q(theta) * exp_q(Jr*dtheta), 1e-8);
    ASSERT_MATRIX_APPROX(exp_R(theta+dtheta), (exp_R(theta) * exp_R(Jr*dtheta)), 1e-8);
}

TEST(rotations, Jl)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;

    // Check the main Jl property for q and R
    // exp( theta + d_theta ) \approx exp(Jl * d_theta) * exp(theta)
    Matrix3s Jl = jac_SO3_left(theta);
    ASSERT_QUATERNION_APPROX(exp_q(theta+dtheta), exp_q(Jl*dtheta) * exp_q(theta), 1e-8);
    ASSERT_MATRIX_APPROX(exp_R(theta+dtheta), (exp_R(Jl*dtheta) * exp_R(theta)), 1e-8);
}

TEST(rotations, Jr_inv)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;
    Quaternions q = v2q(theta);
    Matrix3s    R = v2R(theta);

    // Check the main Jr_inv property for q and R
    // log( R * exp(d_theta) ) \approx log( R ) + Jrinv * d_theta
    Matrix3s Jr_inv = jac_SO3_right_inv(theta);
    ASSERT_MATRIX_APPROX(log_q(q * exp_q(dtheta)), log_q(q) + Jr_inv*dtheta, 1e-8);
    ASSERT_MATRIX_APPROX(log_R(R * exp_R(dtheta)), log_R(R) + Jr_inv*dtheta, 1e-8);
}

TEST(rotations, Jl_inv)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;
    Quaternions q = v2q(theta);
    Matrix3s    R = v2R(theta);

    // Check the main Jl_inv property for q and R
    // log( exp(d_theta) * R ) \approx log( R ) + Jlinv * d_theta
    Matrix3s Jl_inv = jac_SO3_left_inv(theta);
    ASSERT_MATRIX_APPROX(log_q(exp_q(dtheta) * q), log_q(q) + Jl_inv*dtheta, 1e-8);
    ASSERT_MATRIX_APPROX(log_R(exp_R(dtheta) * R), log_R(R) + Jl_inv*dtheta, 1e-8);
}

TEST(rotations, assert_sizes)
{
    typedef Matrix<Scalar, 3, 3> SS;
    typedef Matrix<Scalar, 3, Dynamic> SD;
    typedef Matrix<Scalar, Dynamic, Dynamic> DD;

    SS ss;
    SD sd(3,3);
    DD dd(3,3);

    MatrixSizeCheck<3,3>::check(ss);
    MatrixSizeCheck<3,3>::check(sd);
    MatrixSizeCheck<3,3>::check(dd);

    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(SS, 3, 3);
    //    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DS, 3, 3); // static check fails
    //    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DD, 3, 3); // static check fails


    VectorXs v(6); // A 6-vector of dynamic size

    MatrixSizeCheck<3,1>::check(v.head<3>());
    //    MatrixSizeCheck<3,1>::check(v.head<2>()); // static check fails
    MatrixSizeCheck<3,1>::check(v.head<3>() + v.tail(3)); // static and dynamic check pass
    //    MatrixSizeCheck<3,1>::check(v.head<3>() + v.tail(2)); // dynamic check should fail

    Quaternions q = v2q(v.head<3>());
    q = v2q(v.tail(3));

}

int main(int argc, char **argv)
{
    using namespace wolf;


    /*
        LIST OF FUNCTIONS : 
        - pi2pi                                                            
        - skew -> Skew_vee                                                        OK
        - vee  -> Skew_vee                                                        OK
        - v2q                                                v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - Matrix<T, 3, 1> q2v(const Quaternion<T>& _q)       v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - VectorXs q2v(const Quaternions& _q)                v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - v2R
        - R2v
        - jac_SO3_right
        - jac_SO3_right_inv
        - jac_SO3_left
        - jac_SO3_left_inv
        - quaternion composition
     */

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
