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

//Constraints headers
#include "constraint_fix_3D.h"
#include "constraint_fix_bias.h"

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

    Representation of the application:

                        KF1----------◼----------KF2
                   /----P1----------\ /----------P2             invar       : P1, V1, V2
        Abs|------◼                 ◼                           var         : Q1,B1,P2,Q2,B2
                   \----Q1----------/ \----------Q2
                        V1
        Abs|------◼-----B1
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
        WOLF_ERROR("Failed to open data file ! Exiting")
        return 1;
    }

    //#################################################### SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // ___Create the WOLF Problem + define origin of the problem___
    ProblemPtr problem = Problem::create("PQVBB 3D");
    Eigen::VectorXs problem_origin(16);
    problem_origin << 0,0,0, 0,0,0,1, 0,0,0, 0.8291,0.8291,0.8291, 0.1875,0.1875,0.1875; //using values of initial bias here
    CeresManager* ceres_manager = new CeresManager(problem);

    // ___Configure Ceres if needed___

    // ___Create sensors + processors___
    SensorIMUPtr sensorIMU = std::static_pointer_cast<SensorIMU>(problem->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml"));
    ProcessorIMUPtr processorIMU = std::static_pointer_cast<ProcessorIMU>(problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml"));
    
    SensorOdom3DPtr sensorOdom = std::static_pointer_cast<SensorOdom3D>(problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml"));
    ProcessorOdom3DPtr processorOdom = std::static_pointer_cast<ProcessorOdom3D>(problem->installProcessor("ODOM 3D", "odom", "odom", wolf_root + "/src/examples/processor_odom_3D.yaml"));
    // ___set origin of processors to the problem's origin___
    TimeStamp ts(0);
    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(processorIMU->setOrigin(problem_origin, ts));
    processorOdom->setOrigin(origin_KF);

    //#################################################### PROCESS DATA
    // ___process IMU and odometry___

    //Create vectors to store data and time
    Eigen::Vector6s data_imu, data_odom;
    Scalar clock;

    //Create captures
    CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sensorIMU, data_imu); //ts is set at 0
    CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sensorOdom, data_odom, 6, 6);

    //while we do not reach the end of file, read IMU input (ts, Ax, Ay, Az, Wx, Wy, Wz) and process data through capture
    while(!imu_data_input.eof())
    {
        //read
        imu_data_input >> clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5];

        //set capture
        ts.set(clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        //process
        sensorIMU->process(imu_ptr);
    }

    //All IMU data have been processed, close the file
    imu_data_input.close();

    //now impose final odometry using last timestamp of imu
    data_odom << 0,-0.06,0, 0,0,0;
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom);
    sensorOdom->process(mot_ptr);

    //A KeyFrame should have been created (depending on time_span in processors). get the last KeyFrame
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(problem->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //#################################################### OPTIMIZATION PART
    // ___Create needed constraints___

    //Add Fix3D constraint on first KeyFrame (with large covariance except for yaw)
    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFix_cov(5,5) = 0.1;
    CaptureBasePtr cap_fix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 6));
    FeatureBasePtr featureFix = cap_fix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), featureFix_cov));
    ConstraintFix3DPtr ctr_fix = std::static_pointer_cast<ConstraintFix3D>(featureFix->addConstraint(std::make_shared<ConstraintFix3D>(featureFix)));

    // Add Bias constraint on first KeyFrame
    Eigen::MatrixXs featureFixBias_cov(6,6);
    featureFixBias_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFixBias_cov.topLeftCorner(3,3) *= 0.005;        // sqrt(0.005) = 0.0707 m/s2
    featureFixBias_cov.bottomRightCorner(3,3) *= 0.003;  // sqrt(0.003) = 0.0548 rad/s
    CaptureBasePtr cap_fixbias = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector6s() << 0,0,0, 0,0,0).finished(), featureFixBias_cov, 6, 6));
    //create a FeatureBase to constraint biases
    FeatureBasePtr featureFixBias = cap_fixbias->addFeature(std::make_shared<FeatureBase>("FIX BIAS", (Eigen::Vector6s() << 0,0,0, 0,0,0).finished(), featureFixBias_cov));
    ConstraintFixBiasPtr ctr_fixBias = std::static_pointer_cast<ConstraintFixBias>(featureFixBias->addConstraint(std::make_shared<ConstraintFixBias>(featureFixBias)));

    // ___Fix/Unfix stateblocks___
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->fix();
    origin_KF->getAccBiasPtr()->unfix();
    origin_KF->getGyroBiasPtr()->unfix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();
    last_KF->getAccBiasPtr()->unfix();
    last_KF->getGyroBiasPtr()->unfix();

    // ___Solve + compute covariances___
    problem->print(4,0,1,0);
    ceres::Solver::Summary summary = ceres_manager->solve();
    ceres_manager->computeCovariances(ALL_MARGINALS);
    problem->print(4,0,1,0);

    //#################################################### RESULTS PART
    // ___Define expected values___
    Eigen::Vector7s expected_initial_pose, expected_final_pose;
    expected_initial_pose << 0,0,0,0,0,0,1;
    expected_final_pose << 0,-0.06,0,0,0,0,1;

    // ___Get standard deviation from covariances___
    Eigen::MatrixXs cov_KF1(16,16), cov_KF2(16,16);

    problem->getFrameCovariance(origin_KF, cov_KF1);
    problem->getFrameCovariance(last_KF, cov_KF2);

    Eigen::Matrix<wolf::Scalar, 16, 1> stdev_KF1, stdev_KF2;

    stdev_KF1 = 2*(cov_KF1.diagonal().array().sqrt());
    stdev_KF2 = 2*(cov_KF2.diagonal().array().sqrt());

    WOLF_DEBUG("stdev KF1 : ", stdev_KF1.transpose());
    WOLF_DEBUG("stdev KF2 : ", stdev_KF2.transpose());
    
    // ___Are expected values in the range of estimated +/- 2*stdev ?___

    return 0;
}