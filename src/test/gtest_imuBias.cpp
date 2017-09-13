/**
 * \file gtest_imuBias.cpp
 *
 *  Created on: May 15, 2017
 *      \author: Dinsh Atchuthan
 */

//Wolf
#include "wolf.h"
#include "problem.h"
#include "feature_fix.h"
#include "constraint_fix_3D.h"
#include "constraint_fix_bias.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "capture_fix.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"
#include "processor_odom_3D.h"
#include "ceres_wrapper/ceres_manager.h"

#include "utils_gtest.h"
#include "../src/logging.h"

#include <iostream>
#include <fstream>

//#define OUTPUT_DATA

using namespace Eigen;
using namespace std;
using namespace wolf;


// ProcessorIMU_Real_CaptureFix is similar to ProcessorIMU_Real_CaptureFix_odom
// the difference is that the second one gets the odometry measurements from a file while the first one imposes only one odometry
// between first and last KF
class ProcessorIMU_Real_CaptureFix : public testing::Test
{
    public:
        wolf::TimeStamp t;
        wolf::Scalar dt;
        SensorIMUPtr sen_imu;
        SensorOdom3DPtr sen_odom3D;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        ProcessorOdom3DPtr processor_ptr_odom3D;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs expected_origin_state;
        std::ofstream debug_results;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create("PQVBB 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


        // SENSOR + PROCESSOR IMU
        //We want a processorIMU with a specific max_time_span (1s) forour test
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
        prc_imu_params->max_time_span = 10;
        prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_imu_params->dist_traveled = 1000000000;
        prc_imu_params->angle_turned = 1000000000;

        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 1.99999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    //===================================================== END{SETTING PROBLEM}

        char* imu_filepath;
        //std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/M1.txt");
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/bias_plateformRotateX.txt");
        imu_filepath   = new char[imu_filepath_string.length() + 1];
        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        //WOLF_INFO("imu file: ", imu_filepath)
        ASSERT_TRUE(imu_data_input.is_open()) << "Failed to open data files... Exiting";

        #ifdef OUTPUT_DATA
        //std::ofstream debug_results;
        debug_results.open("KFO_cfix3D.dat");
        #endif

        //===================================================== SETTING PROBLEM

        // reset origin of problem
        Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

        t.set(0);
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
        data_odom3D << 0,-0.06,0, 0,0,0;
        expected_final_state.resize(16);
        expected_final_state << 0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        expected_origin_state.resize(16);
        expected_origin_state << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D, 6, 7, 6, 0);

        // process all IMu data and then finish with the odom measurement that will create a new constraint

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            imu_ptr->getTimeStamp();
            sen_imu->process(imu_ptr);
        }

        // PROCESS ODOM 3D DATA
        mot_ptr->setTimeStamp(ts);
        mot_ptr->setData(data_odom3D);
        sen_odom3D->process(mot_ptr);

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    }

    virtual void TearDown(){}
};

class ProcessorIMU_Real_CaptureFix_odom : public testing::Test
{
    public:
        wolf::TimeStamp t;
        wolf::Scalar dt;
        SensorIMUPtr sen_imu;
        SensorOdom3DPtr sen_odom3D;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        ProcessorOdom3DPtr processor_ptr_odom3D;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs expected_origin_state;
        std::ofstream debug_results;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create("PQVBB 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


        // SENSOR + PROCESSOR IMU
        //We want a processorIMU with a specific max_time_span (1s) forour test
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
        prc_imu_params->max_time_span = 10;
        prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_imu_params->dist_traveled = 1000000000;
        prc_imu_params->angle_turned = 1000000000;

        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 0.49999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    //===================================================== END{SETTING PROBLEM}

        char* imu_filepath;
        char * odom_filepath;
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_testPatternRot.txt");  //imu_testPattern3, imu_testPattern3_biased
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_testPatternRot.txt"); //odom_testPattern3_biased,
        imu_filepath   = new char[imu_filepath_string.length() + 1];
        odom_filepath   = new char[odom_filepath_string.length() + 1];
        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::strcpy(odom_filepath, odom_filepath_string.c_str());
        std::ifstream imu_data_input;
        std::ifstream odom_data_input;

        imu_data_input.open(imu_filepath);
        odom_data_input.open(odom_filepath);
        //WOLF_INFO("imu file: ", imu_filepath)
        ASSERT_TRUE(imu_data_input.is_open() && odom_data_input.is_open()) << "Failed to open data files... Exiting";

        #ifdef OUTPUT_DATA
        debug_results.open(wolf_root + "/KFO_cfix3D_odom.dat");
        if (debug_results.is_open()) std::cout << "debug results file opened!" << wolf_root + "KFO_cfix3D_odom.dat" << std::endl;
        else std::cout << "debug results file open failed!" << wolf_root + "KFO_cfix3D_odom.dat" << std::endl;
        #endif

        //===================================================== SETTING PROBLEM

        // reset origin of problem
        Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

        t.set(0);
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;

        expected_final_state.resize(16);
        expected_origin_state.resize(16);

        imu_data_input >> expected_origin_state[0] >> expected_origin_state[1] >> expected_origin_state[2] >> expected_origin_state[6] >> expected_origin_state[3] >> expected_origin_state[4] >> expected_origin_state[5] >> expected_origin_state[7] >> expected_origin_state[8] >> expected_origin_state[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        expected_origin_state.tail(6) = origin_bias;  

        wolf::Scalar input_clock;
        TimeStamp ts(0);
        TimeStamp t_odom(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D, 6, 7, 6, 0);

        // process all IMu data and then finish with the odom measurement that will create a new constraint
        //read first odom data from file
        odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
        t_odom.set(input_clock);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            //imu_ptr->getTimeStamp();
            sen_imu->process(imu_ptr);

            if(ts.get() == t_odom.get())
            {
                WOLF_DEBUG("ts : ", ts.get())
                // PROCESS ODOM 3D DATA
                mot_ptr->setTimeStamp(t_odom);
                mot_ptr->setData(data_odom3D);
                sen_odom3D->process(mot_ptr);

                //prepare next odometry measurement if there is any
                odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
                t_odom.set(input_clock);
            }
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

        //closing files
        imu_data_input.close();
        odom_data_input.close();
    }

    virtual void TearDown(){}
};

TEST_F(ProcessorIMU_Real_CaptureFix_odom,M1_VarQ1B1P2Q2B2_InvarP1V1V2_initOK_ConstrO_KF0_cfixem6To100)
{
    // Create a ConstraintFix that will constraint the initial pose
    // give it a big small covariance on yaw and big on the other parts of the diagonal.
    // This is needed to make the problem observable, otherwise the jacobian would be rank deficient -> cannot compute covariances

    expected_origin_state.tail(6) << 0,0,0, 0,0,0;

    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFix_cov(5,5) = 0.1;
    CaptureBasePtr capfix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 7, 6, 0));
    FeatureBasePtr ffix = capfix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), featureFix_cov));
    ConstraintFix3DPtr ctr_fix = std::static_pointer_cast<ConstraintFix3D>(ffix->addConstraint(std::make_shared<ConstraintFix3D>(ffix)));

    // Create a ConstraintFixBias for origin KeyFrame
    Eigen::MatrixXs featureFixBias_cov(6,6);
    featureFixBias_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFixBias_cov.topLeftCorner(3,3) *= 0.0007;        // sqrt(0.0007) = 0.0265 m/s2
    featureFixBias_cov.bottomRightCorner(3,3) *= 0.0007;  // sart(0.0007) = 0.005 rad/s
    CaptureBasePtr capfixbias = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector6s() << 0,0,0, 0,0,0).finished(), featureFixBias_cov, 6, 7, 6, 0));
    //create a FeatureBase to constraint biases
    FeatureBasePtr ffixbias = capfixbias->addFeature(std::make_shared<FeatureBase>("FIX BIAS", (Eigen::Vector6s() << 0,0,0, 0,0,0).finished(), featureFixBias_cov));
    ConstraintFixBiasPtr ctr_fixBias = std::static_pointer_cast<ConstraintFixBias>(ffixbias->addConstraint(std::make_shared<ConstraintFixBias>(ffixbias)));

    //unfix / fix stateblocks
    origin_KF->getPPtr()->fix();
    origin_KF->getVPtr()->fix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getAccBiasPtr()->unfix();
    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();

    //last_KF->setState(expected_final_state);
    FrameBaseList frameList = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

    //Fix velocity to [0 0 0] in all frames
    for(auto frame : frameList)
    {
        frame->getVPtr()->setState((Eigen::Vector3s()<<0,0,0).finished());
        frame->getVPtr()->fix();
    }

    //vary the covariance in odometry position displacement + solve + output result
    for (wolf::Scalar p_var = 0.000001; p_var <= 0.04; p_var=p_var*10)
//        for (wolf::Scalar p_var = 0.0001; p_var <= 0.0001; p_var=p_var*10)
    {
        for(auto frame : frameList)
        {
            ConstraintBaseList ctr_list = frame->getConstrainedByList();
            //std::cout << "ctr_list size : " << ctr_list.size() << std::endl;

            for(auto ctr : ctr_list)
            {
                //std::cout << "ctr ID : " << (*ctr_it)->getTypeId() << std::endl;
                if (ctr->getTypeId() == CTR_ODOM_3D) //change covariances in features to constraint only position
                {
                    Eigen::MatrixXs meas_cov(ctr->getMeasurementCovariance());
                    meas_cov.topLeftCorner(3,3) = (Eigen::Matrix3s() << p_var, 0, 0, 0, p_var, 0, 0, 0, p_var).finished();
                    ctr->getFeaturePtr()->setMeasurementCovariance(meas_cov);
                }
            }
        }

        //reset origin to its initial value (value it had before solving any problem) for the new solve + perturbate
        Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.0001);
        origin_KF->setState(expected_origin_state);
        Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
        Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
        origin_KF->getAccBiasPtr()->setState(accBias + random_err);
        origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);
        
        wolf_problem_ptr_->print(4,0,1,0);

        //solve + compute covariance
        ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
        ceres_manager_wolf_diff->computeCovariances(ALL);

        wolf_problem_ptr_->print(4,0,1,0);

        //format output : get states + associated covariances
        Eigen::MatrixXs cov_AB1(3,3), cov_GB1(3,3), cov_P1(3,3);
        Eigen::MatrixXs cov_Q1(4,4);
        wolf_problem_ptr_->getCovarianceBlock(origin_KF->getAccBiasPtr(), origin_KF->getAccBiasPtr(), cov_AB1);
        wolf_problem_ptr_->getCovarianceBlock(origin_KF->getGyroBiasPtr(), origin_KF->getGyroBiasPtr(), cov_GB1);
        wolf_problem_ptr_->getCovarianceBlock(origin_KF->getPPtr(), origin_KF->getPPtr(), cov_P1);
        wolf_problem_ptr_->getCovarianceBlock(origin_KF->getOPtr(), origin_KF->getOPtr(), cov_Q1);

        Eigen::MatrixXs cov_AB2(3,3), cov_GB2(3,3), cov_P2(3,3);
        Eigen::MatrixXs cov_Q2(4,4);
        wolf_problem_ptr_->getCovarianceBlock(last_KF->getAccBiasPtr(), last_KF->getAccBiasPtr(), cov_AB2);
        wolf_problem_ptr_->getCovarianceBlock(last_KF->getGyroBiasPtr(), last_KF->getGyroBiasPtr(), cov_GB2);
        wolf_problem_ptr_->getCovarianceBlock(last_KF->getPPtr(), last_KF->getPPtr(), cov_P2);
        wolf_problem_ptr_->getCovarianceBlock(last_KF->getOPtr(), last_KF->getOPtr(), cov_Q2);
        std::cout << p_var << "\n\tcov_AB1 : " << sqrt(cov_AB1(0,0)) << ", " << sqrt(cov_AB1(1,1)) << ", " << sqrt(cov_AB1(2,2))
                << "\n\t cov_GB1 : " << sqrt(cov_GB1(0,0)) << ", " << sqrt(cov_GB1(1,1)) << ", " << sqrt(cov_GB1(2,2))
                << "\n\t cov_P2 : " << sqrt(cov_P2(0,0)) << ", " << sqrt(cov_P2(1,1)) << ", " << sqrt(cov_P2(2,2)) << std::endl;

        #ifdef OUTPUT_DATA

        debug_results << sqrt(p_var) << "\t" << origin_KF->getPPtr()->getState().transpose() << "\t" << origin_KF->getOPtr()->getState().transpose() << "\t" << origin_KF->getAccBiasPtr()->getState().transpose() << "\t" << origin_KF->getGyroBiasPtr()->getState().transpose() << "\t"
                    << sqrt(cov_P1(0,0)) << "\t" << sqrt(cov_P1(1,1)) << "\t" << sqrt(cov_P1(2,2)) << "\t" 
                    << sqrt(cov_Q1(0,0)) << "\t" << sqrt(cov_Q1(1,1)) << "\t" << sqrt(cov_Q1(2,2)) << "\t" 
                    << sqrt(cov_AB1(0,0)) << "\t" << sqrt(cov_AB1(1,1)) << "\t" << sqrt(cov_AB1(2,2)) << "\t" 
                    << sqrt(cov_GB1(0,0)) << "\t" << sqrt(cov_GB1(1,1)) << "\t" << sqrt(cov_GB1(2,2)) << "\t"
                    << last_KF->getPPtr()->getState().transpose() << "\t" << last_KF->getOPtr()->getState().transpose() << "\t" << last_KF->getAccBiasPtr()->getState().transpose() << "\t" << last_KF->getGyroBiasPtr()->getState().transpose() << "\t"
                    << sqrt(cov_P2(0,0)) << "\t" << sqrt(cov_P2(1,1)) << "\t" << sqrt(cov_P2(2,2)) << "\t" 
                    << sqrt(cov_Q2(0,0)) << "\t" << sqrt(cov_Q2(1,1)) << "\t" << sqrt(cov_Q2(2,2)) << "\t" 
                    << sqrt(cov_AB2(0,0)) << "\t" << sqrt(cov_AB2(1,1)) << "\t" << sqrt(cov_AB2(2,2)) << "\t" 
                    << sqrt(cov_GB2(0,0)) << "\t" << sqrt(cov_GB2(1,1)) << "\t" << sqrt(cov_GB2(2,2)) << std::endl;

        /*debug_results << sqrt(p_var) << "\t" << last_KF->getPPtr()->getState().transpose() << "\t" << last_KF->getOPtr()->getState().transpose() << "\t" << last_KF->getAccBiasPtr()->getState().transpose() << "\t" << last_KF->getGyroBiasPtr()->getState().transpose() << "\t"
                    << sqrt(cov_P2(0,0)) << "\t" << sqrt(cov_P2(1,1)) << "\t" << sqrt(cov_P2(2,2)) << "\t" 
                    << sqrt(cov_Q2(0,0)) << "\t" << sqrt(cov_Q2(1,1)) << "\t" << sqrt(cov_Q2(2,2)) << "\t" 
                    << sqrt(cov_AB2(0,0)) << "\t" << sqrt(cov_AB2(1,1)) << "\t" << sqrt(cov_AB2(2,2)) << "\t" 
                    << sqrt(cov_GB2(0,0)) << "\t" << sqrt(cov_GB2(1,1)) << "\t" << sqrt(cov_GB2(2,2)) << std::endl;*/
        #endif
     }

    #ifdef OUTPUT_DATA
    debug_results.close();
    #endif

     //just print measurement covariances of IMU and odometry :
    ConstraintBaseList ctr_list = origin_KF->getConstrainedByList();
    for (auto ctr_it = ctr_list.begin(); ctr_it != ctr_list.end(); ctr_it++)
    {
        if ((*ctr_it)->getTypeId() == CTR_ODOM_3D) //change covariances in features to constraint only position
            {
                Eigen::MatrixXs meas_cov((*ctr_it)->getMeasurementCovariance());
                std::cout << "\n Odom3D meas cov : " << meas_cov.diagonal().transpose() << std::endl;
            }

            else if ((*ctr_it)->getTypeId() == CTR_IMU)
            {
                Eigen::MatrixXs IMUmeas_cov((*ctr_it)->getMeasurementCovariance());
                std::cout << "\n imu meas cov : " << IMUmeas_cov.diagonal().transpose() << std::endl;
            }
    }

    //Assertions : estimations we expect in the ideal case
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Pos : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Ori : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_final_state.segment(3,4).transpose() << std::endl;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "ProcessorIMU_Real_CaptureFix_odom*";
  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
