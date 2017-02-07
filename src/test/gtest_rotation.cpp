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

namespace wolf
{
//these are initial rotation methods
//A problem has been detected when using jets : computing the norm results in NaNs
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> v2q_o(const Eigen::MatrixBase<Derived>& _v)
{

    MatrixSizeCheck<3, 1>::check(_v);
    typedef typename Derived::Scalar T;

    Eigen::Quaternion<T> q;
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
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> q2v_o(const Eigen::QuaternionBase<Derived>& _q)
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

inline Eigen::VectorXs q2v_aa(const Eigen::Quaternions& _q)
{
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_q);
    return aa.axis() * aa.angle();
}

//here is an alternative version to be tested
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> v2q_new(const Eigen::MatrixBase<Derived>& _v)
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
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> q2v_new(const Eigen::QuaternionBase<Derived>& _q)
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
    
}

TEST(rotations, v2q_o_VS_v2q_new) //this test will use functions defined above
{
    using namespace wolf;
    //defines scalars
    wolf::Scalar deg_to_rad = M_PI/180.0;

    Eigen::Vector4s vec0, vec1;

        //v2q
    Eigen::Vector3s rot_vector0, rot_vector1;
    Eigen::Quaternions quat_o, quat_o1, quat_new, quat_new1;
    Eigen::Vector4s vec_o, vec_o1, vec_new, vec_new1;
    Eigen::Vector3s qvec_o, qvec_o1, qvec_new, qvec_new1, qvec_aao, qvec_aa1;
    for (unsigned int iter = 0; iter < 10000; iter ++)
    {
        rot_vector0 = Eigen::Vector3s::Random();
        rot_vector1 = rot_vector0 * 100 *deg_to_rad; //far from origin
        rot_vector0 = rot_vector0 *0.001*deg_to_rad; //close to origin

        quat_o = v2q_o(rot_vector0);
        quat_new = v2q_new(rot_vector0);
        quat_o1 = v2q_o(rot_vector1);
        quat_new1 = v2q_new(rot_vector1);

        //now we do the checking
     vec_o << quat_o.w(), quat_o.x(), quat_o.y(), quat_o.z();
     vec_new << quat_new.w(), quat_new.x(), quat_new.y(), quat_new.z();
     vec_o1 << quat_o1.w(), quat_o1.x(), quat_o1.y(), quat_o1.z();
     vec_new1 << quat_new1.w(), quat_new1.x(), quat_new1.y(), quat_new1.z();

     ASSERT_TRUE((vec_o - vec_new).isMuchSmallerThan(1,wolf::Constants::EPS));
     ASSERT_TRUE((vec_o1 - vec_new1).isMuchSmallerThan(1,wolf::Constants::EPS));
    

        //q2v
    qvec_o     = q2v_o(quat_o);
    qvec_o1    = q2v_o(quat_o1);
    qvec_aao   = q2v_aa(quat_o);
    qvec_aa1   = q2v_aa(quat_o1);
    qvec_new   = q2v_new(quat_new);
    qvec_new1  = q2v_new(quat_new1);

    // 'New' version of q2v is working, result with template version gives the same that the regular version with Eigen::Quaternions argument
    ASSERT_TRUE((qvec_aao - qvec_new).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_aao : " << qvec_aao.transpose() << "\n qvec_new : " << qvec_new.transpose() << std::endl;
    ASSERT_TRUE((qvec_aa1 - qvec_new1).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_aa1 : " << qvec_aa1.transpose() << "\n qvec_new1 : " << qvec_new1.transpose() << std::endl;
    EXPECT_TRUE((qvec_new - rot_vector0).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_new : " << qvec_new.transpose() << "\n rot_vector0 : " << rot_vector0.transpose() << std::endl;
    EXPECT_TRUE((qvec_new1 - rot_vector1).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_new1 : " << qvec_new1.transpose() << "\n rot_vector1 : " << rot_vector1.transpose() << std::endl;

    //Something went wrong with old version of 'q2v' : values in vector are twice that expected.
    EXPECT_FALSE((qvec_o1 - qvec_new1).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_o1 : " << qvec_o1.transpose() << "\n qvec_new1 : " << qvec_new1.transpose() << std::endl;
    EXPECT_FALSE((qvec_o - qvec_new).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_o : " << qvec_o.transpose() << "\n qvec_new : " << qvec_new.transpose() << std::endl;
    EXPECT_FALSE((qvec_o1 - rot_vector0).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_o1 : " << qvec_o1.transpose() << "\n rot_vector0 : " << rot_vector0.transpose() << std::endl;
    EXPECT_FALSE((qvec_o - rot_vector1).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_o : " << qvec_o.transpose() << "\n rot_vector1 : " << rot_vector1.transpose() << std::endl;
    }
}

TEST(rotations, Skew_vee)
{
    using namespace wolf;
    Eigen::Vector3s vec3 = Eigen::Vector3s::Random();
    Eigen::Matrix3s skew_mat;
    skew_mat = skew(vec3);

        // vee
    Eigen::Vector3s vec3_bis;
    vec3_bis = vee(skew_mat);

    ASSERT_TRUE(vec3_bis == vec3);
}

TEST(rotations, v2q_q2v)
{
    using namespace wolf;
    //defines scalars
    wolf::Scalar deg_to_rad = M_PI/180.0;

    Eigen::Vector4s vec0, vec1;

        //v2q
    Eigen::Vector3s rot_vector0, rot_vector1;
    rot_vector0 = Eigen::Vector3s::Random();
    rot_vector1 = rot_vector0 * 100 *deg_to_rad; //far from origin
    rot_vector0 = rot_vector0*deg_to_rad;

    Eigen::Quaternions quat0, quat1;
    quat0 = v2q(rot_vector0);
    quat1 = v2q(rot_vector1);

        //q2v
    Eigen::Vector3s quat_to_v0, quat_to_v1;
    Eigen::VectorXs quat_to_v0x, quat_to_v1x;

    quat_to_v0 = q2v(quat0);
    quat_to_v1 = q2v(quat1);
    quat_to_v0x = q2v(quat0);
    quat_to_v1x = q2v(quat1);

        //now we do the checking
     vec0 << quat0.w(), quat0.x(), quat0.y(), quat0.z();
     vec1 << quat1.w(), quat1.x(), quat1.y(), quat1.z();

    //std::cout << "\n quaternion near origin : \n" << vec0 << "\n quaternion far from origin : \n" << vec1 << std::endl;

    ASSERT_TRUE((rot_vector0 - quat_to_v0).isMuchSmallerThan(1,wolf::Constants::EPS));
    ASSERT_TRUE(rot_vector1.isApprox(quat_to_v1, wolf::Constants::EPS));
    ASSERT_TRUE(rot_vector0.isApprox(quat_to_v0x, wolf::Constants::EPS));
    ASSERT_TRUE(rot_vector1.isApprox(quat_to_v1x, wolf::Constants::EPS));
}

TEST(rotations, v2R_R2v)
{
    using namespace wolf;
    //First test is to verify we get the good result with v -> v2R -> R2v -> v
    //test 2 : how small can angles in rotation vector be ?

    //definition
    wolf::Scalar deg_to_rad = M_PI/180.0;
    Eigen::Vector3s rot_vector0, rot_vector1;

    rot_vector0 = Eigen::Vector3s::Random();
    rot_vector1 = rot_vector0 * 100 *deg_to_rad; //far from origin
    rot_vector0 = rot_vector0*deg_to_rad;

    Eigen::Matrix3s rot0, rot1;
    rot0 = v2R(rot_vector0);
    rot1 = v2R(rot_vector1);

        //R2v
    Eigen::Vector3s rot0_vec, rot1_vec;
    rot0_vec = R2v(rot0);
    rot1_vec = R2v(rot1);

        //check now
    ASSERT_TRUE(rot0_vec.isApprox(rot_vector0, wolf::Constants::EPS));
    ASSERT_TRUE(rot1_vec.isApprox(rot_vector1, wolf::Constants::EPS));
}

TEST(rotations, R2v_v2R_limits)
{
    using namespace wolf;
    //test 2 : how small can angles in rotation vector be ?
    wolf::Scalar scale = 1;
    Eigen::Matrix3s v_to_R, initial_matrix;
    Eigen::Vector3s  R_to_v;

    //Eigen::Vector3s rv;
    for(int i = 0; i<8; i++){
        initial_matrix = v2R(Eigen::Vector3s::Random() * scale);

        R_to_v = R2v(initial_matrix);     
        v_to_R = v2R(R_to_v);

        EXPECT_TRUE((v_to_R-initial_matrix).isMuchSmallerThan(1,wolf::Constants::EPS)); //<< "R2v_v2R_limits : reached at scale " << scale << std::endl;
        scale = scale*0.1;
    }
}

TEST(rotations, R2v_v2R_AAlimits)
{
    using namespace wolf;
    //let's see how small the angles can be here : limit reached at scale/10 =  1e-16
    wolf::Scalar scale = 1;
    Eigen::Matrix3s rotation_mat;
    Eigen::Vector3s rv;

    for(int i = 0; i<8; i++){
        rotation_mat = v2R(Eigen::Vector3s::Random() * scale);
        //rotation_mat(0,0) = 1.0;
        //rotation_mat(1,1) = 1.0;
        //rotation_mat(2,2) = 1.0;

        //rv = R2v(rotation_mat); //decomposing R2v below
        Eigen::AngleAxis<wolf::Scalar> aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa0.axis() * aa0.angle();
        //std::cout << "aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;
        
        EXPECT_FALSE(rv == Eigen::Vector3s::Zero());
        scale = scale*0.1;
    }
}

TEST(rotations, v2q2R2v)
{
    using namespace wolf;
    wolf::Scalar scale = 1;
    // testing new path : vector -> quaternion -> matrix -> vector

    for(int i = 0; i< 8; i++){
    Eigen::Vector3s vector_ = Eigen::Vector3s::Random()*scale;
    Eigen::Quaternions quat_ = v2q(vector_);
    Eigen::Matrix3s mat_ = quat_.matrix();
    Eigen::Vector3s vector_bis = R2v(mat_);

    EXPECT_TRUE((vector_-vector_bis).isMuchSmallerThan(1, wolf::Constants::EPS)) << 
    "problem in vector -> quaternion -> matrix -> vector at scale " << scale << "\t Diff (returned_vector - input vector) = \n" << vector_bis - vector_ << std::endl;
    scale = scale*0.1;
    }
}

TEST(rotations, AngleAxis_limits)
{
    using namespace wolf;
    //Hypothesis : problem with construction of AngleAxis objects.
    // Example : if R = I + [delta t]_x (happenning in the IMU case with delta t = 0.001). Then angle mays be evaluated as 0 (due to cosinus definition ?) 
    // Here we try to get the AngleAxis object from a random rotation matrix, then get back to the rotation matrix using Eigen::AngleAxis::toRotationMatrix()

    wolf::Scalar scale = 1;
    Eigen::Matrix3s res, res_i, rotation_mati, rotation_mat;
    Eigen::Vector3s rv;

    for(int i = 0; i<8; i++){ //FIX ME : Random() will not create a rotation matrix. Then, R2v(Random_matrix()) makes no sense at all.
        rotation_mat = v2R(Eigen::Vector3s::Random() * scale);
        rotation_mati = rotation_mat;

        //rv = R2v(rotation_mat); //decomposing R2v below
        Eigen::AngleAxis<wolf::Scalar> aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa0.axis() * aa0.angle();
        //std::cout << "aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;
        res = aa0.toRotationMatrix();
        
        // now we set the diagonal to identity
        Eigen::AngleAxis<wolf::Scalar> aa1 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa1.axis() * aa1.angle();
        //std::cout << "aa1.axis : " << aa0.axis().transpose() << ",\t aa1.angles :" << aa0.angle() <<std::endl;
        res_i = aa1.toRotationMatrix();

        EXPECT_TRUE(res.isApprox(rotation_mat,wolf::Constants::EPS)) << "limit reached at scale " << scale << std::endl;
        EXPECT_TRUE(res_i.isApprox(rotation_mati,wolf::Constants::EPS)) << "res_i : limit reached at scale " << scale << std::endl;
        scale = scale*0.1;
    }
}

TEST(rotations, AngleAxis_limits2)
{
    using namespace wolf;

    Eigen::Matrix3s rotation_mat;
    Eigen::Vector3s rv;
    Eigen::AngleAxis<wolf::Scalar> aa0;

    //FIX ME : 5. Building a rot mat doing this is not safe; You cannot guarantee that R is valid.
    // Highlight limitation of Eigen::AngleAxis
    rotation_mat = skew(Eigen::Vector3s::Random()) *0.0001;
    rotation_mat(0,0) = 1;
    rotation_mat(1,1) = 0.999999;
    rotation_mat(2,2) = 1;
    aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();
    //checking if rv is 0 vector
    EXPECT_FALSE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    rotation_mat(0,0) = 1.0;
    rotation_mat(1,1) = 1.0;
    rotation_mat(2,2) = 1.0;
    aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();
    //checking if rv is 0 vector
    EXPECT_TRUE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    rotation_mat = skew(Eigen::Vector3s::Random()) *0.1;
    rotation_mat(0,0) = 1;
    rotation_mat(1,1) = 0.9999999;
    rotation_mat(2,2) = 1;
    aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();
    //checking if rv is 0 vector
    EXPECT_FALSE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    rotation_mat(0,0) = 1.0;
    rotation_mat(1,1) = 1.0;
    rotation_mat(2,2) = 1.0;
    aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();
    //checking if rv is 0 vector
    EXPECT_TRUE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
}

TEST(rotations, Quat_compos_const_rateOfTurn)
{
    using namespace wolf;

                                // ********* constant rate of turn *********
    /* First idea was to integrate data on SO3 (succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)) and in R3
    (q2v(v2q(v0*n*dt))). with v0 << 30.0*deg_to_rad, 5.0*deg_to_rad, 10.0*deg_to_rad : constant rate-of-turn in rad/s and dt the time step.
    But this is not OK, we cannot expect those 2 rotation integration to be equal.
    The whole point of the SO3 thing is that we cannot integrate rotation in the R3 space and expect it to work. This is why we must integrate it in the manifold of SO3
    
    more specifically : 
    - At a constant velocity, because we keep a constant rotation axis, the integral is the same.
    - for non-contant velocities, especially if we change the axis of rotation, then it’s not the same, and the only good method is the SO3.

    We change the idea :
    define orientation and derive ox, oy, oz so that we get the rate of turn wx, wy, wz.
    Then compare the final orientation from ox, oy, oz and quaternion we get by data integration
    */

    wolf::Scalar deg_to_rad = M_PI/180.0;
    Eigen::Quaternions q0;
    q0.setIdentity();
    Eigen::Vector3s tmp_vec; //will be used to store rate of turn data

    const unsigned int x_rot_vel = 5;   // deg/s
    const unsigned int y_rot_vel = 2;   // deg/s
    const unsigned int z_rot_vel = 10;  // deg/s

    wolf::Scalar tmpx, tmpy, tmpz;
    /*
        ox oy oz evolution in degrees (for understanding) --> converted in rad
        with * pi/180
        ox = x_rot_vel * t; %express angle in rad before using sinus
        oy = y_rot_vel * t;
        oz = z_rot_vel * t;

        corresponding rate of turn
        %rate of turn expressed in radians/s
        wx = x_rot_vel;
        wy = y_rot_vel;
        wz = z_rot_vel;
     */

     //there is no need to compute the rate of turn at each time because it is constant here : 
    tmpx = deg_to_rad * x_rot_vel;  // rad/s
    tmpy = deg_to_rad * y_rot_vel;
    tmpz = deg_to_rad * z_rot_vel;
    tmp_vec << tmpx, tmpy, tmpz;
    const wolf::Scalar dt = 0.1;

    for(unsigned int data_iter = 0; data_iter < 100; data_iter ++)
    {   
        q0 = q0 * v2q(tmp_vec*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)

    }

    /* We focus on orientation here. position is supposed not to have moved
     * we integrated on 10s. After 10s, the orientation state is supposed to be :
     * ox = x_rot_vel * t = 50; (in degree ! -> *M_PI/180 to get rad)
     * oy = y_rot_vel * t = 20;
     * oz = z_rot_vel * t = 100;
     */

     Eigen::Vector3s final_orientation((Eigen::Vector3s()<< deg_to_rad*50, deg_to_rad*20, deg_to_rad*100).finished());
     ASSERT_TRUE((final_orientation - wolf::q2v(q0)).isMuchSmallerThan(1,wolf::Constants::EPS)) << "final orientation expected : " << final_orientation.transpose() << 
     "\n computed final orientation : " << wolf::q2v(q0).transpose() << std::endl;
}

TEST(rotations, Quat_compos_var_rateOfTurn)
{
    using namespace wolf;

                                //********* changing rate of turn - same freq for all axis *********
    wolf::Scalar deg_to_rad = M_PI/180.0;
    Eigen::Quaternions q0;
    q0.setIdentity();
    Eigen::Vector3s v0, v1, v2;
    Eigen::VectorXs const_diff_ox, const_diff_oy, const_diff_oz, ox, oy, oz, qox, qoy, qoz;
    Eigen::VectorXs cdox_abs, cdoy_abs, cdoz_abs, vector0, t_vec; //= const_diff_## with absolute values
    const wolf::Scalar dt = 0.001;
    const wolf::Scalar N = 100;

    const_diff_ox.resize(N/dt);
    const_diff_oy.resize(N/dt);
    const_diff_oz.resize(N/dt);
    cdox_abs.resize(N/dt);
    cdoy_abs.resize(N/dt);
    cdoz_abs.resize(N/dt);
    vector0 = Eigen::VectorXs::Zero(N/dt);
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

    EXPECT_FALSE(cdox_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoy_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoz_abs.isMuchSmallerThan(1,wolf::Constants::EPS)) << 
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
    Eigen::Quaternions q0;
    q0.setIdentity();
    Eigen::Vector3s v0, v1, v2;
    Eigen::VectorXs const_diff_ox, const_diff_oy, const_diff_oz, ox, oy, oz, qox, qoy, qoz;
    Eigen::VectorXs cdox_abs, cdoy_abs, cdoz_abs, vector0, t_vec; //= const_diff_## with absolute values
    const wolf::Scalar dt = 0.001;
    const wolf::Scalar N = 100;

    const_diff_ox.resize(N/dt);
    const_diff_oy.resize(N/dt);
    const_diff_oz.resize(N/dt);
    cdox_abs.resize(N/dt);
    cdoy_abs.resize(N/dt);
    cdoz_abs.resize(N/dt);
    vector0 = Eigen::VectorXs::Zero(N/dt);
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

    EXPECT_FALSE(cdox_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoy_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoz_abs.isMuchSmallerThan(1,wolf::Constants::EPS)) << 
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

int main(int argc, char **argv)
{
    using namespace wolf;
                        

    /*
        LIST OF FUNCTIONS : 
        - pi2pi                                                            
        - skew -> Skew_vee                                                            OK
        - vee  -> Skew_vee                                                            OK
        - v2q                                                              v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - Eigen::Matrix<T, 3, 1> q2v(const Eigen::Quaternion<T>& _q)       v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - Eigen::VectorXs q2v(const Eigen::Quaternions& _q)                v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - v2R
        - R2v
        - jac_SO3_right
        - jac_SO3_right_inv
        - jac_SO3_left
        - jac_SO3_left_inv
        - quaternion composition
     */

     testing::InitGoogleTest(&argc, argv);
     ::testing::GTEST_FLAG(filter) = "*Quat_compos_const_rateOfTurn";
     return RUN_ALL_TESTS();
}