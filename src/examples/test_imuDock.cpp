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
    if (argc < 02)
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

    // ___Check if the file is correctly opened___
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data file ! Exiting" << std::endl;
        return 1;
    }

    //#################################################### SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // ___Create the WOLF Problem + define origin of the problem___
    ProblemPtr problem = Problem::create("PQVBB 3D");
    Eigen::VectorXs problem_origin(16);
    problem_origin << 0,0,0, 0,0,0,1, 0,0,0, 0.8291,0.8291,0.8291, 0.1875,0.1875,0.1875; //using values of initial bias here
    //CeresManager* ceres_manager_wolf_diff = new CeresManager(problem);

    // ___Configure Ceres if needed___

    // ___Create sensors + processors___
    SensorIMUPtr sensorIMU = std::static_pointer_cast<SensorIMU>(problem->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml"));
    ProcessorIMUPtr processorIMU = std::static_pointer_cast<ProcessorIMU>(problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "/src/examples/processor_imu.yaml"));

    SensorOdom3DPtr sensorOdom = std::static_pointer_cast<SensorOdom3D>(problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml"));
    ProcessorOdom3DPtr processorOdom = std::static_pointer_cast<ProcessorOdom3D>(problem->installProcessor("ODOM 3D", "odom", "odom", "/src/examples/processor_odom_3D.yaml"));
    
    // ___set origin of processors to the problem's origin___
    TimeStamp ts(0);
    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(processorIMU->setOrigin(problem_origin, ts));
    processorOdom->setOrigin(origin_KF);

    //#################################################### PROCESS DATA
    // ___process IMU and odometry___

    //#################################################### OPTIMIZATION PART
    // ___Create needed constraints___

    // ___Fix/Unfix stateblocks___

    // ___Solve + compute covariances___

    // ___Get standard deviation from covariances___

    // ___Check results___
        // ___Define expected values___

        // ___Are expected values in the range of estimated +/- 2*stdev ?___
}