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
        - v2q                                                              OK (precision wolf::Constants::EPS)
        - Eigen::Matrix<T, 3, 1> q2v(const Eigen::Quaternion<T>& _q)       OK (precision wolf::Constants::EPS)
        - Eigen::VectorXs q2v(const Eigen::Quaternions& _q)                OK (precision wolf::Constants::EPS)
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

    //v2R, R2v
    
}