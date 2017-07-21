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

#define OUTPUT_RESULTS

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

                                    Imu
                        KF1----------◼----------KF2
                   /----P1----------\ /----------P2             invar       : P1, V1, V2
        Abs|------◼                 ◼                          var         : Q1,B1,P2,Q2,B2
                   \----Q1----------/ \----------Q2
                        V1          Odom
        Abs|------◼-----B1
*/
int main(int argc, char** argv)
{
    //#################################################### INITIALIZATION
    using namespace wolf;

    //___get input file for imu data___
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

    #ifdef OUTPUT_RESULTS
        //define output file
        std::ofstream output_results_before, output_results_after, checking;
        output_results_before.open("imu_dock_beforeOptim.dat");
        output_results_after.open("imu_dock_afterOptim.dat");
        checking.open("KF2_pose_stdev.dat");
    #endif

    // ___initialize variabes that will be used through the code___
    Eigen::VectorXs problem_origin(16);
    problem_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0.;
    Eigen::Vector7s imu_pose((Eigen::Vector7s()<<0,0,0,0,0,0,1).finished()), odom_pose((Eigen::Vector7s()<<0,0,0,0,0,0,1).finished());
    
    //Create vectors to store data and time
    Eigen::Vector6s data_imu, data_odom;
    Scalar clock;
    TimeStamp ts(0), ts_output(0); //will be used to store the data timestamps and set timestamps in captures

    // ___Define expected values___
    Eigen::Vector7s expected_KF1_pose((Eigen::Vector7s()<<0,0,0,0,0,0,1).finished()), expected_KF2_pose((Eigen::Vector7s()<<0,-0.06,0,0,0,0,11).finished());

    //#################################################### SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // ___Create the WOLF Problem + define origin of the problem___
    ProblemPtr problem = Problem::create("PQVBB 3D");
    CeresManager* ceres_manager = new CeresManager(problem);
 
    // ___Configure Ceres if needed___

    // ___Create sensors + processors___
    SensorIMUPtr sensorIMU = std::static_pointer_cast<SensorIMU>(problem->installSensor("IMU", "Main IMU", imu_pose, wolf_root + "/src/examples/sensor_imu.yaml"));
    ProcessorIMUPtr processorIMU = std::static_pointer_cast<ProcessorIMU>(problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml"));
    
    SensorOdom3DPtr sensorOdom = std::static_pointer_cast<SensorOdom3D>(problem->installSensor("ODOM 3D", "odom", odom_pose, wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml"));
    ProcessorOdom3DPtr processorOdom = std::static_pointer_cast<ProcessorOdom3D>(problem->installProcessor("ODOM 3D", "odom", "odom", wolf_root + "/src/examples/processor_odom_3D.yaml"));
    
    // ___set origin of processors to the problem's origin___
    FrameIMUPtr KF1 = std::static_pointer_cast<FrameIMU>(processorIMU->setOrigin(problem_origin, ts)); // XXX JS: setting ts to zero, and then reading clock from data, is inconsistent.
    processorOdom->setOrigin(KF1);

    //#################################################### PROCESS DATA
    // ___process IMU and odometry___

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
    // XXX JS: in my  opinion, we should control the KF creation better, not using time span. Is it possible?
    FrameIMUPtr KF2 = std::static_pointer_cast<FrameIMU>(problem->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //#################################################### OPTIMIZATION PART
    // ___Create needed constraints___

    //Add Fix3D constraint on first KeyFrame (with large covariance except for yaw)
    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFix_cov(5,5) = 0.01;
    CaptureBasePtr cap_fix = KF1->addCapture(std::make_shared<CaptureMotion>(0, nullptr, problem_origin.head(7), 7, 6));
    FeatureBasePtr featureFix = cap_fix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", problem_origin.head(7), featureFix_cov));
    ConstraintFix3DPtr ctr_fix = std::static_pointer_cast<ConstraintFix3D>(featureFix->addConstraint(std::make_shared<ConstraintFix3D>(featureFix)));

    Eigen::MatrixXs featureFixBias_cov(6,6);
    featureFixBias_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFixBias_cov.topLeftCorner(3,3) *= sensorIMU->getAbInitialStdev() * sensorIMU->getAbInitialStdev();
    featureFixBias_cov.bottomRightCorner(3,3) *= sensorIMU->getWbInitialStdev() * sensorIMU->getWbInitialStdev();
    CaptureBasePtr cap_fixbias = KF1->addCapture(std::make_shared<CaptureMotion>(0, nullptr, problem_origin.tail(6), featureFixBias_cov, 6, 6));
    //create a FeatureBase to constraint biases
    FeatureBasePtr featureFixBias = cap_fixbias->addFeature(std::make_shared<FeatureBase>("FIX BIAS", problem_origin.tail(6), featureFixBias_cov));
    ConstraintFixBiasPtr ctr_fixBias = std::static_pointer_cast<ConstraintFixBias>(featureFixBias->addConstraint(std::make_shared<ConstraintFixBias>(featureFixBias)));

    // ___Fix/Unfix stateblocks___
    KF1->getPPtr()->fix();
    KF1->getOPtr()->unfix();
    KF1->getVPtr()->fix();
    KF1->getAccBiasPtr()->unfix();
    KF1->getGyroBiasPtr()->unfix();

    KF2->getPPtr()->unfix();
    KF2->getOPtr()->unfix();
    KF2->getVPtr()->fix();
    KF2->getAccBiasPtr()->unfix();
    KF2->getGyroBiasPtr()->unfix();

    #ifdef OUTPUT_RESULTS
        // ___OUTPUT___
        /* Produce output file for matlab visualization
         * first output : estimated trajectory BEFORE optimization (getting the states each millisecond)
         */

        unsigned int time_iter(0);
        Scalar ms(0.001);
        ts_output.set(0);
        while(ts_output.get() < ts.get() + ms)
        {
            output_results_before << ts_output.get() << "\t" << problem->getState(ts_output).transpose() << std::endl;
            time_iter++;
            ts_output.set(time_iter * ms);
        }
    #endif

    // ___Solve + compute covariances___
    problem->print(4,0,1,0);
    ceres::Solver::Summary summary = ceres_manager->solve();
    ceres_manager->computeCovariances(ALL_MARGINALS);
    problem->print(1,0,1,0);

    //#################################################### RESULTS PART

    // ___Get standard deviation from covariances___
    Eigen::MatrixXs cov_KF1(16,16), cov_KF2(16,16);

    problem->getFrameCovariance(KF1, cov_KF1);
    problem->getFrameCovariance(KF2, cov_KF2);

    Eigen::Matrix<wolf::Scalar, 16, 1> stdev_KF1, stdev_KF2;

    stdev_KF1 = 2*(cov_KF1.diagonal().array().sqrt());
    stdev_KF2 = 2*(cov_KF2.diagonal().array().sqrt());

    WOLF_DEBUG("stdev KF1 : ", stdev_KF1.transpose());
    WOLF_DEBUG("stdev KF2 : ", stdev_KF2.transpose());

    #ifdef OUTPUT_RESULTS
        // ___OUTPUT___
        /* Produce output file for matlab visualization
         * Second output:   KF2 position standard deviation computed
         *                  estimated trajectory AFTER optimization 
         *                  + get KF2 timestamp + state just in case the loop is not working as expected
         */

        //KF2 position stdev
        checking << KF2->getTimeStamp().get() << stdev_KF2.transpose() << std::endl;

        //estimated trajectort
        time_iter = 0;
        ts_output.set(0);
        while(ts_output.get() < ts.get() + ms)
        {
            output_results_after << ts_output.get() << "\t" << problem->getState(ts_output).transpose() << std::endl;
            time_iter++;
            ts_output.set(time_iter * ms);
        }

        //finally, output the timestamp and state associated to KF2
        checking << KF2->getTimeStamp().get() << "\t" << KF2->getState().transpose() << std::endl;
    #endif
    
    // ___Are expected values in the range of estimated +/- 2*stdev ?___

    #ifdef OUTPUT_RESULTS
        output_results_before.close();
        output_results_after.close();
        checking.close();
    #endif

    return 0;
}
