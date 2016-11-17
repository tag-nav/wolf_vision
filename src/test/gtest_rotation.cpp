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

    ASSERT_TRUE(rot_vector0.isApprox(quat_to_v0, wolf::Constants::EPS));
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
        initial_matrix = Eigen::Matrix3s::Random() * scale; //FIX ME : Random() will not create a rotation matrix. Then, R2v(initial_matrix) makes no sense at all.

        R_to_v = R2v(initial_matrix);
        // now we set the diagonal to identity
        //rotation_mati(0,0) = 1.0;
        //rotation_mati(1,1) = 1.0;
        //rotation_mati(2,2) = 1.0;
        
        v_to_R = v2R(R_to_v);

        EXPECT_TRUE(v_to_R.isApprox(initial_matrix,wolf::Constants::EPS)); //<< "R2v_v2R_limits : reached at scale " << scale << std::endl;
        scale = scale*0.1;
    }
}

TEST(rotations, R2v_v2R_limits2)
{
    using namespace wolf;
    //let's see how small the angles can be here : limit reached at scale/10 =  1e-16
    wolf::Scalar scale = 1;
    Eigen::Matrix3s rotation_mat;
    Eigen::Vector3s rv;

    for(int i = 0; i<8; i++){
        rotation_mat = Eigen::Matrix3s::Random() * scale; //FIX ME : Random() will not create a rotation matrix. Then, R2v(initial_matrix) makes no sense at all.
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
        rotation_mati(0,0) = 1.0;
        rotation_mati(1,1) = 1.0;
        rotation_mati(2,2) = 1.0;
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
    EXPECT_FALSE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

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
    EXPECT_FALSE(rv.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
}

TEST(rotations, Quat_compos_const_rateOfTurn)
{
    using namespace wolf;

                                // ********* constant rate of turn *********
    wolf::Scalar deg_to_rad = M_PI/180.0;
    Eigen::Quaternions q0;
    q0.setIdentity();
    Eigen::Vector3s v0, v1, v2;
    Eigen::VectorXs const_diff_ox, const_diff_oy, const_diff_oz, ox, oy, oz, qox, qoy, qoz;
    Eigen::VectorXs cdox_abs, cdoy_abs, cdoz_abs, vector0, t_vec; //= const_diff_## with absolute values
    const wolf::Scalar dt = 0.001;
    const wolf::Scalar N = 100;

    v0 << 30.0*deg_to_rad, 5.0*deg_to_rad, 10.0*deg_to_rad; //constant rate-of-turn in rad/s
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

    for(wolf::Scalar t=0; t<N/dt; t++){
        v2 = q2v(v2q(v0*t*dt));
        ox(t) = v2(0);
        oy(t) = v2(1);
        oz(t) = v2(2);
        /*ox(t) = pi2pi(v0(0)*t*dt);
        oy(t) = pi2pi(v0(1)*t*dt);
        oz(t) = pi2pi(v0(2)*t*dt);*/
        t_vec(t) = t*dt;
    }
    
    for(wolf::Scalar t=0; t<N/dt; t++){
        if(t!=0)
            q0 = q0 * v2q(v0*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)
        v1 = q2v(q0);   //corresponding rotation vector of current quaternion
        qox(t) = v1(0); //angle X component
        qoy(t) = v1(1); //angle Y component
        qoz(t) = v1(2); //angle Z component
    }

    //Compute difference between orientation vectors (expected - real)
    const_diff_ox = ox - qox;
    const_diff_oy = oy - qoy;
    const_diff_oz = oz - qoz;

    //get absolute difference
    cdox_abs = const_diff_ox.array().abs();
    cdoy_abs = const_diff_oy.array().abs();
    cdoz_abs = const_diff_oz.array().abs();

    EXPECT_TRUE(cdox_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoy_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoz_abs.isMuchSmallerThan(1,wolf::Constants::EPS)) << 
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

    for(wolf::Scalar t=0; t<N/dt; t++){
        v1 << sin(v0(0)*t*dt), sin(v0(1)*t*dt), sin(v0(2)*t*dt);
        v1 = q2v(v2q(v1));
        ox(t) = v1(0);
        oy(t) = v1(1);
        oz(t) = v1(2);
        /*ox(t) = pi2pi(v0(0)*t*dt);
        oy(t) = pi2pi(v0(1)*t*dt);
        oz(t) = pi2pi(v0(2)*t*dt);*/
        t_vec(t) = t*dt;
    }

    for(wolf::Scalar t=0; t<N/dt; t++){
        if(t!=0){
            v2 << v0(0)*cos(v0(0)*t*dt), v0(1)*cos(v0(1)*t*dt), v0(2)*cos(v0(2)*t*dt);
            q0 = q0 * v2q(v2*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)
        }
        v1 = q2v(q0);   //corresponding rotation vector of current quaternion
        qox(t) = v1(0); //angle X component
        qoy(t) = v1(1); //angle Y component
        qoz(t) = v1(2); //angle Z component
    }

    //Compute difference between orientation vectors (expected - real)
    const_diff_ox = ox - qox;
    const_diff_oy = oy - qoy;
    const_diff_oz = oz - qoz;

    //get absolute difference
    cdox_abs = const_diff_ox.array().abs();
    cdoy_abs = const_diff_oy.array().abs();
    cdoz_abs = const_diff_oz.array().abs();

    EXPECT_TRUE(cdox_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoy_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoz_abs.isMuchSmallerThan(1,wolf::Constants::EPS)) << 
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

    for(wolf::Scalar t=0; t<N/dt; t++){
        v1 << sin(v0(0)*t*dt), sin(v0(1)*t*dt), sin(v0(2)*t*dt);
        v1 = q2v(v2q(v1));
        ox(t) = v1(0);
        oy(t) = v1(1);
        oz(t) = v1(2);
        /*ox(t) = pi2pi(v0(0)*t*dt);
        oy(t) = pi2pi(v0(1)*t*dt);
        oz(t) = pi2pi(v0(2)*t*dt);*/
        t_vec(t) = t*dt;
    }

    for(wolf::Scalar t=0; t<N/dt; t++){
        if(t!=0){
            v2 << v0(0)*cos(v0(0)*t*dt), v0(1)*cos(v0(1)*t*dt), v0(2)*cos(v0(2)*t*dt);
            q0 = q0 * v2q(v2*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)
        }
        v1 = q2v(q0);   //corresponding rotation vector of current quaternion
        qox(t) = v1(0); //angle X component
        qoy(t) = v1(1); //angle Y component
        qoz(t) = v1(2); //angle Z component
    }

    //Compute difference between orientation vectors (expected - real)
    const_diff_ox = ox - qox;
    const_diff_oy = oy - qoy;
    const_diff_oz = oz - qoz;

    //get absolute difference
    cdox_abs = const_diff_ox.array().abs();
    cdoy_abs = const_diff_oy.array().abs();
    cdoz_abs = const_diff_oz.array().abs();

    EXPECT_TRUE(cdox_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoy_abs.isMuchSmallerThan(1,wolf::Constants::EPS) && cdoz_abs.isMuchSmallerThan(1,wolf::Constants::EPS)) << 
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
     return RUN_ALL_TESTS();
}