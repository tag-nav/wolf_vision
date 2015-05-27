
//std
#include <iostream>

//Eigen
#include <eigen3/Eigen/Geometry>

//Wolf
#include "wolf.h"
#include "state_quaternion.h"

int main()
{
    std::cout << std::endl << "Eigen Quatenrnion test" << std::endl;
    
    WolfScalar q1[4]; 
    Eigen::Map<Eigen::Quaternions> q1_map(q1);

    //try to find out how eigen sorts storage (real part tail or head ? )    
    std::cout << std::endl << "************************** TEST #1 ***************************" << std::endl;    
    q1_map.w() = -1; 
    q1_map.x() = 2; 
    q1_map.y() = 5; 
    q1_map.z() = 9; 
    std::cout << "q1[0]=" << q1[0] << "; q1_map.x()=" << q1_map.x() << std::endl;
    std::cout << "q1[1]=" << q1[1] << "; q1_map.y()=" << q1_map.y() << std::endl;
    std::cout << "q1[2]=" << q1[2] << "; q1_map.z()=" << q1_map.z() << std::endl;
    std::cout << "q1[3]=" << q1[3] << "; q1_map.w()=" << q1_map.w() << std::endl;
    std::cout << std::endl << "RESULT: Eigen stores REAL part in the LAST memory position of the quaternion." << std::endl;
    
    //rot matrix
    std::cout << std::endl << "************************** TEST #2 ***************************" << std::endl;    
    StateQuaternion sq(q1);
    Eigen::Matrix3s RM; 
    sq.normalize(); 
    std::cout << "Rot matrix through StateQuaternion class" << std::endl;
    RM = sq.getRotationMatrix(); 
    std::cout << RM << std::endl;
    
    std::cout << "Rot matrix through Eigen::Map class" << std::endl;
    RM = q1_map.toRotationMatrix();
    std::cout << RM << std::endl; 
    
    std::cout << std::endl << "End of Eigen Quatenrnion tests" << std::endl;
    return 0;
}

