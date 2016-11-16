/**
 * \file test_rotation.cpp
 *
 *  Created on: Oct 13, 2016
 *      \author: AtDinesh
 */

//std
#include <iostream>

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

//#define write_results

int main()
{
    using namespace wolf;
                        // THESE ARE UNITARY TESTS FOR METHODS IN ROTATION.H

    /*
        LIST OF FUNCTIONS : 
        - pi2pi                                                            
        - skew                                                             OK
        - vee                                                              OK
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
     wolf::Scalar scale = 0;

     //pi2pi
     
     /**********************************************************************************************/
     /// skew + vee
        //skew
     std::cout<< "\n\n######################################### Test Skew + vee ################################################\n" << std::endl;

     Eigen::Vector3s vec3 = Eigen::Vector3s::Random();
     Eigen::Matrix3s skew_mat;
     skew_mat = skew(vec3);

     std::cout << "Input Vector :\n " << vec3 << "\n corresponding skew matrix : \n" << skew_mat << "\n" <<std::endl; 

        // vee
     Eigen::Vector3s vec3_bis;
     vec3_bis = vee(skew_mat);
     if(vec3_bis == vec3)
        std::cout << "vee() checked \n" << std::endl;
    else
        std::cout << "vee() false \n used matrix : " << skew_mat << "\n returned vee vector : \n" << vec3_bis << "\n" << std::endl;
    
    /**********************************************************************************************/
    ///v2q + q2v
    std::cout<< "\n\n######################################### Test v2q + q2v ################################################\n" << std::endl;

    Eigen::Vector4s vec0, vec1;

        //v2q
    wolf::Scalar deg_to_rad = 3.14159265359/180.0;;
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

     std::cout << "\n quaternion near origin : \n" << vec0 << "\n quaternion far from origin : \n" << vec1 << std::endl;

    if(rot_vector0.isApprox(quat_to_v0, wolf::Constants::EPS))
        std::cout << "fixed = q2v() is ok near origin \n " << std::endl;
    else{
        std::cout << "fixed = q2v() is NOT ok near origin  -  input rotation vector : \n" << rot_vector0 <<
        "\n returned rotation vector: \n" << quat_to_v0 << std::endl;
        std::cout << "Diff between vectors (rot_vector0 - quat_to_v0) : \n" << rot_vector0 - quat_to_v0 << std::endl;
    }

    if(rot_vector1.isApprox(quat_to_v1, wolf::Constants::EPS))
        std::cout << "fixed = q2v() is ok far from origin \n " << std::endl;
    else{
        std::cout << "fixed = q2v() is NOT ok far from origin  -  input rotation vector \n: " << rot_vector1 <<
         "\n returned rotation vector : \n" << quat_to_v1 << std::endl;
         std::cout << "Diff between vectors (rot_vector1 - quat_to_v1) : \n" << rot_vector1 - quat_to_v1 << std::endl;
    }

    if(rot_vector0.isApprox(quat_to_v0x, wolf::Constants::EPS))
        std::cout << "Dynamic = q2v() is ok near origin \n " << std::endl;
    else{
        std::cout << "Dynamic = q2v() is NOT ok near origin  -  input rotation vector : \n" << rot_vector0 <<
         "\n returned rotation vector : \n" << quat_to_v0x << std::endl;
         std::cout << "Diff between vectors (rot_vector0 - quat_to_v0x) : \n" << rot_vector0 - quat_to_v0x << std::endl;
    }

    if(rot_vector1.isApprox(quat_to_v1x, wolf::Constants::EPS))
        std::cout << "Dynamic = q2v() is ok far from origin \n " << std::endl;
    else{
        std::cout << "Dynamic = q2v() is NOT ok far from origin  -  input rotation vector: \n" << rot_vector1 <<
        "\n returned rotation vector : \n" << quat_to_v1x << std::endl;
         std::cout << "Diff between vectors (rot_vector1 - quat_to_v1x) : \n" << rot_vector1 - quat_to_v1x << std::endl;
    }

    /**********************************************************************************************/
    std::cout<< "\n\n######################################### Test v2R + R2v ################################################\n" << std::endl;

    ///v2R, R2v
                                        //First test is to verify we get the good result with v -> v2R -> R2v -> v
                                        //test 2 : how small can angles in rotation vector be ?
        //v2R
    //we re-use rot_vector0 and rot_vector1 defined above
    Eigen::Matrix3s rot0, rot1;
    rot0 = v2R(rot_vector0);
    rot1 = v2R(rot_vector1);

        //R2v
    Eigen::Vector3s rot0_vec, rot1_vec;
    rot0_vec = R2v(rot0);
    rot1_vec = R2v(rot1);

        //check now
    if(rot0_vec.isApprox(rot_vector0, wolf::Constants::EPS))
        std::cout << "v2R, R2v ok with small angles \n" << std::endl;
    else{
        std::cout << "v2R, R2v NOT ok with small angles - intput rotation vector : \n" << rot_vector0 << "\n corresponding matrix : \n " <<
        rot0 << "\n returned rotation vector : \n"<< rot0_vec << std::endl;
        std::cout << "Diff between vectors (rot_vector - rot_vec) : " << rot_vector0 - rot0_vec << std::endl;
    }

    if(rot1_vec.isApprox(rot_vector1, wolf::Constants::EPS))
        std::cout << "v2R, R2v ok with large angles \n" << std::endl;
    else{
        std::cout << "v2R, R2v NOT ok with large angles - intput rotation vector : \n" << rot_vector1 << "\n corresponding matrix : \n " <<
        rot1 << "\n returned rotation vector : \n"<< rot1_vec << std::endl;
        std::cout << "Diff between vectors (rot_vector - rot_vec) : " << rot_vector1 - rot1_vec << std::endl;
    }

    std::cout<< "\n\n######################################### Test R2v --> v2R limits ################################################\n" << std::endl;

    scale = 1;
    Eigen::Matrix3s v_to_R, initial_matrix;
    Eigen::Vector3s  R_to_v;

    //Eigen::Vector3s rv;
    for(int i = 0; i<8; i++){
        initial_matrix = Eigen::Matrix3s::Random() * scale;

        R_to_v = R2v(initial_matrix); //decomposing R2v below
        Eigen::AngleAxis<wolf::Scalar> angleAxis_R_to_v = Eigen::AngleAxis<wolf::Scalar>(initial_matrix);
        std::cout << "angleAxis_R_to_v.axis : " << angleAxis_R_to_v.axis().transpose() << ",\t angleAxis_R_to_v.angles :" << angleAxis_R_to_v.angle() <<std::endl;
        // now we set the diagonal to identity
        //rotation_mati(0,0) = 1.0;
        //rotation_mati(1,1) = 1.0;
        //rotation_mati(2,2) = 1.0;
        
        v_to_R = v2R(R_to_v);

        if(!v_to_R.isApprox(initial_matrix,wolf::Constants::EPS)){
            std::cout << "\n limit reached at scale " << scale << ", rotation matrix is : \n" << initial_matrix << "\n v_to_R is : \n" << v_to_R << std::endl;
            //std::cout << "aa0.axis : \n" << aa0.axis() << "\n aa0.angles \n:" << aa0.angle() <<std::endl;
            break;
        }   
        scale = scale*0.1;
    }

    /**********************************************************************************************/
    std::cout<< "\n\n######################################### Test R2v limits ################################################\n" << std::endl;

                                        //let's see how small the angles can be here : limit reached at scale/10 =  1e-16
    scale = 1;
    Eigen::Matrix3s rotation_mat;
    Eigen::Vector3s rv;
    for(int i = 0; i<8; i++){
        rotation_mat = Eigen::Matrix3s::Random() * scale;
        //rotation_mat(0,0) = 1.0;
        //rotation_mat(1,1) = 1.0;
        //rotation_mat(2,2) = 1.0;

        //rv = R2v(rotation_mat); //decomposing R2v below
        Eigen::AngleAxis<wolf::Scalar> aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa0.axis() * aa0.angle();
        std::cout << "aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;
        
        if(rv == Eigen::Vector3s::Zero()){
            std::cout << "\n limit reached at scale " << scale << ", rotation matrix is : \n" << rotation_mat << "\n rv is : \n" << rv << std::endl;
            //std::cout << "aa0.axis : \n" << aa0.axis() << "\n aa0.angles \n:" << aa0.angle() <<std::endl;
            break;
        }
        scale = scale*0.1;
    }
    /**********************************************************************************************/
    std::cout<< "\n\n######################################### Test vector -> quaternion -> matrix -> vector ################################################\n" << std::endl;

                                      // testing new path : vector -> quaternion -> matrix -> vector
    scale = 1;
    for(int i = 0; i< 8; i++){
    Eigen::Vector3s vector_ = Eigen::Vector3s::Random()*scale;
    Eigen::Quaternions quat_ = v2q(vector_);
    Eigen::Matrix3s mat_ = quat_.matrix();
    Eigen::Vector3s vector_bis = R2v(mat_);

    if(!vector_bis.isApprox(vector_, wolf::Constants::EPS)){
        std::cout << "problem in vector -> quaternion -> matrix -> vector at scale " << scale << "\n input vector : \n" << vector_ << "\n returned vector : \n" << vector_bis << std::endl;
        std::cout << "Diff (returned_vector - input vector) = \n" << vector_bis - vector_ << std::endl;
        break;
    }
    scale = scale*0.1;
    }

    /**********************************************************************************************/
    std::cout<< "\n\n######################################### Test Eigen::AngleAxis limits ################################################\n" << std::endl;

    //Hypothesis : problem with construction of AngleAxis objects.
    // Example : if R = I + [delta t]_x (happenning in the IMU case with delta t = 0.001). Then angle mays be evaluated as 0 (due to cosinus definition ?) 
    // Here we try to get the AngleAxis object from a random rotation matrix, then get back to the rotation matrix using Eigen::AngleAxis::toRotationMatrix()

    scale = 1;
    Eigen::Matrix3s res, res_i, rotation_mati; //rotation_mat already declared
    //Eigen::Vector3s rv;
    for(int i = 0; i<8; i++){
        rotation_mat = Eigen::Matrix3s::Random() * scale;
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

        if(!res.isApprox(rotation_mat,wolf::Constants::EPS)){
            std::cout << "\n limit reached at scale " << scale << ", rotation matrix is : \n" << rotation_mat << "\n res is : \n" << res << std::endl;
            //std::cout << "aa0.axis : \n" << aa0.axis() << "\n aa0.angles \n:" << aa0.angle() <<std::endl;
            //break;
        }
        if(!res_i.isApprox(rotation_mati,wolf::Constants::EPS)){
            std::cout << "\n limit reached at scale " << scale << ", rotation matrix is : \n" << rotation_mati << "\n res_i is : \n" << res_i << std::endl;
            break;
        }
        
        scale = scale*0.1;
    }

    std::cout<< "\n\n>>>>>>>>>>>>>>>>> Highlight limitation of Eigen::AngleAxis <<<<<<<<<<<<<<<<<<<<<\n" << std::endl;
    rotation_mat = skew(Eigen::Vector3s::Random()) *0.0001;
    rotation_mat(0,0) = 1;
    rotation_mat(1,1) = 0.999999;
    rotation_mat(2,2) = 1;
    Eigen::AngleAxis<wolf::Scalar> aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();

    std::cout << "\n initial matrix : \n" << rotation_mat << "\nassociated angle axis : " << rv.transpose() << "\t aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;

    rotation_mat(0,0) = 1.0;
    rotation_mat(1,1) = 1.0;
    rotation_mat(2,2) = 1.0;
    aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();

    std::cout << "\n initial matrix : \n" << rotation_mat << "\nassociated angle axis : " << rv.transpose() << "\t aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;

    rotation_mat = skew(Eigen::Vector3s::Random()) *0.1;
    rotation_mat(0,0) = 1;
    rotation_mat(1,1) = 0.9999999;
    rotation_mat(2,2) = 1;
    aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();

    std::cout << "\n initial matrix : \n" << rotation_mat << "\nassociated angle axis : " << rv.transpose() << "\t aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;

    rotation_mat(0,0) = 1.0;
    rotation_mat(1,1) = 1.0;
    rotation_mat(2,2) = 1.0;
    aa0 = Eigen::AngleAxis<wolf::Scalar>(rotation_mat);
    rv = aa0.axis() * aa0.angle();

    std::cout << "\n initial matrix : \n" << rotation_mat << "\nassociated angle axis : " << rv.transpose() << "\t aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;

    /**********************************************************************************************/
    ///Quaternion composition
    std::cout<< "\n\n######################################### Quaternion composition ################################################\n" << std::endl;

    Eigen::Quaternions q0;
    q0.setIdentity();
    Eigen::Vector3s v0, v1, v2;
    Eigen::VectorXs const_diff_ox, const_diff_oy, const_diff_oz, ox, oy, oz, qox, qoy, qoz;
    Eigen::VectorXs cdox_abs, cdoy_abs, cdoz_abs, vector0, t_vec; //= const_diff_## with absolute values
    const wolf::Scalar dt = 0.001;
    const wolf::Scalar N = 100;

    std::cout << "\t\t********* constant rate of turn *********\n" << std::endl;

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

    std::cout << "pi2pi..." << std::endl;
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
    
    std::cout << "composing..." << std::endl;
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

    std::cout << "checking..." << std::endl;
    if(cdox_abs.isApprox(vector0,wolf::Constants::EPS) && cdoy_abs.isApprox(vector0,wolf::Constants::EPS) && cdoz_abs.isApprox(vector0,wolf::Constants::EPS))
        std::cout << "\t quaternion composition with constant rate of turn is OK\n" << std::endl;
    else{
        std::cout << "\t quaternion composition with constant rate of turn is NOT OK\n" << std::endl;
        std::cout << "max orientation error in abs value (x, y, z) : " << cdox_abs.maxCoeff() << "\t" << cdoy_abs.maxCoeff() << "\t" << cdoz_abs.maxCoeff() << std::endl;
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
                std::cout << "could not open file const_rot" << std::endl;
        #endif
    }
}