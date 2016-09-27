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
     */

     //pi2pi
     
     /// skew + vee
        //skew
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
    
    ///v2q + q2v
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

                                        //let's see how small the angles can be here
    wolf::Scalar scale = 1;
    Eigen::Matrix3s rotation_mat;
    Eigen::Vector3s rv;
    for(int i = 0; i<8; i++){
        rotation_mat = Eigen::Matrix3s::Random() * scale;
        //rotation_mat(0,0) = 1.0;
        //rotation_mat(1,1) = 1.0;
        //rotation_mat(2,2) = 1.0;

        rv = R2v(rotation_mat);
        if(rv == Eigen::Vector3s::Zero()){
            std::cout << "\n limit reached at scale " << scale << ", rotation matrix is : \n" << rotation_mat << "\n rv is : \n" << rv << std::endl;
            break;
        }
        scale = scale*0.1;
    }
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
}