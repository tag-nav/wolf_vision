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

//std
#include <iostream>
#include <fstream>

/*                              OFFLINE VERSION
    In this test, we use the experimental conditions needed for Humanoids 2017.
    IMU data are acquired using the docking station. 

    Constraints are (supposing to KeyFrames, stateblocks or first Keyframe are noted *1 and second Keyframes's are *2) :
    invar       : P1, V1, V2
    var         : Q1,B1,P2,Q2,B2
    constraints : Odometry constraint between KeyFrames
                  IMU constraint
                  FixBias constraint --> makes the problem observable (adding a big covariance on all part but a smaller one on yaw)
                  Fix3D constraint

    What we expect  : Estimate biases (this will strongly depend on the actual trajectory of the IMU)
                      Estimate the position and orienttion of the IMU (check with the standard deviation using covariance matrix)
*/
int main(int argc, char** argv)
{
    using namespace wolf;

    //we expect 1 file giving imu measurements
    std::ifstream imu_data_input;
    const char * filename_imu;
    if (argc < 02)embed
    {
        WOLF_ERROR("Missing 1 imput argument (path to imu data file).")
        return 1; //return with error
    }
    else
    {
        filename_imu = argv[1];

        imu_data_input.open(filename_imu);
        WOLF_INFO("imu file : ", filename_imu)
    }

    // Check if the file is correctly opened

    //#################################################### SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // Create the WOLF Problem + set origin of the problem

    // Configure Ceres

    // Create sensors + processors

    //

    //#################################################### PROCESS DATA
    // process IMU and odometry

    //#################################################### OPTIMIZATION PART
    // Create needed constraints

    // Fix/Unfix stateblocks

    // Solve + compute covariances

    // Get standard deviation from covariances

    // Check results
        // Define expected values

        // Are expected values in the range of estimated +/- 2*stdev ?
}