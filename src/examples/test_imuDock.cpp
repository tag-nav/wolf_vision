/**
 * \file test_imuDock.cpp
 *
 *  Created on: July 18, 2017
 *      \author: Dinesh Atchuthan
 */

#include "wolf.h"
#include "problem.h"
#include "ceres_wrapper/ceres_manager.h"
#include "sensor_imu.h"
#include "processor_imu.h"
#include "sensor_odom_3D.h"
#include "processor_odom_3D.h"

/*
    In this test, we use the experimental conditions needed for Humanoids 2017.
    IMU data are acquired using the docking station. 

    Constraints are (supposing to KeyFrames, stateblocks or first Keyframe are noted *1 and second Keyframes's are *2) :
    invar       : P1, V1, V2
    var         : Q1,B1,P2,Q2,B2
    constraints : Odometry constraint between KeyFrames
                  IMU constraint
                  FixBias constraint --> makes the problem observable (adding a big covariance on all part but a smaller one on yaw)
                  Fic3D constraint

*/
int main(int argc, char** argv)
{
    using namespace wolf;


}