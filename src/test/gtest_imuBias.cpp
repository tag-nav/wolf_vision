/**
 * \file gtest_imuBias.cpp
 *
 *  Created on: May 15, 2017
 *      \author: Dinsh Atchuthan
 */

//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"
#include "processor_odom_3D.h"
#include "ceres_wrapper/ceres_manager.h"

#include "utils_gtest.h"
#include "../src/logging.h"

#include <iostream>
#include <fstream>

//#define DEBUG_RESULTS
//#define DEBUG_RESULTS_BIAS
//#define GET_RESIDUALS

using namespace Eigen;
using namespace std;
using namespace wolf;

//Global Variables
Eigen::Vector6s M1_bias, M2_bias, M3_bias, M4_bias; 

class ProcessorIMU_Bias : public testing::Test
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


    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
        t.set(0);

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
        prc_odom3D_params->max_time_span = 0.39999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    //===================================================== END{SETTING PROBLEM}
    }

    virtual void TearDown(){}
};

class ProcessorIMU_Bias_LowQualityOdom : public testing::Test
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


    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
        t.set(0);

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
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_LowQPos.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 0.39999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    //===================================================== END{SETTING PROBLEM}
    }

    virtual void TearDown(){}
};

class ProcessorIMU_Real : public testing::Test
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

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
        t.set(0);

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
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/M1.txt");
        imu_filepath   = new char[imu_filepath_string.length() + 1];
        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        //WOLF_INFO("imu file: ", imu_filepath)
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }

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
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D, 6, 6);

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

TEST_F(ProcessorIMU_Bias,Set1)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set1.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set1_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set1Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set2)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set2.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set2_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,-0.06,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set2Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set3)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set3.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set3_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,-0.06,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set3Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set4)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set4.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set4_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set4Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set5)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set5.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set5_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,-0.06,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set5Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set6)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set6.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set6_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,-0.06,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set6Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set7)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set7.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set7_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set7Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set8)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set8.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set8_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set8Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set9)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set9.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set9_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set9Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set10)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set10.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set10_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
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

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set10Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Bias,Set11)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Set11.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Set11_odom.txt");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() || !odom_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        sen_imu->process(imu_ptr);

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_Set11Cov.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();

}

TEST_F(ProcessorIMU_Real,M1_VarB1V2B2_InvarP1Q1V1P2Q2_initOK)
{
        //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    //last_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    wolf_problem_ptr_->print(4,1,1,1);
}

TEST_F(ProcessorIMU_Real,M1_VarP1Q1B1V2B2_InvarV1P2Q2_PosinitError)
{
    //change initial guess to a wrong value
    Eigen::VectorXs originState(origin_KF->getState());
    Eigen::VectorXs perturbed_OriginState(originState);
    perturbed_OriginState.head(3) = originState.head(3) + (Eigen::Vector3s() << 0.15, 0.5, 1).finished();
    origin_KF->setState(perturbed_OriginState);

    //prepare problem for solving
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_TRUE((origin_KF->getPPtr()->getState() - originState.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Pos : " << origin_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << originState.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getOPtr()->getState() - originState.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Ori : " << origin_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << originState.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarP1Q1B1V2B2_InvarV1P2Q2_QuatInitError)
{
    //change initial guess to a wrong value
    Eigen::VectorXs originState(origin_KF->getState());
    Eigen::VectorXs perturbed_OriginState(originState);

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    Eigen::Map<Eigen::Quaternions> quat_map(perturbed_OriginState.data() + 3);

    orientation_perturbation(0) = 1.02;
    orientation_perturbation(1) = 0.53;
    orientation_perturbation(2) = 2.09;

    //introduce the perturbation directly in the quaternion StateBlock
    quat_map = quat_map * v2q(orientation_perturbation);
    origin_KF->setState(perturbed_OriginState);
    
    //prepare problem for solving
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE((origin_KF->getPPtr()->getState() - originState.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Pos : " << origin_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << originState.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getOPtr()->getState() - originState.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Ori : " << origin_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << originState.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarP1Q1V1B1V2B2_InvarP2Q2_PosinitError)
{
    //change initial guess to a wrong value
    Eigen::VectorXs originState(origin_KF->getState());
    Eigen::VectorXs perturbed_OriginState(originState);
    perturbed_OriginState.head(3) = originState.head(3) + (Eigen::Vector3s() << 0.15, 0.5, 1).finished();
    origin_KF->setState(perturbed_OriginState);

    //prepare problem for solving

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_TRUE((origin_KF->getPPtr()->getState() - originState.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Pos : " << origin_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << originState.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getOPtr()->getState() - originState.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Ori : " << origin_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << originState.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarP1Q1V1B1V2B2_InvarP2Q2_QuatInitError)
{
    //change initial guess to a wrong value
    Eigen::VectorXs originState(origin_KF->getState());
    Eigen::VectorXs perturbed_OriginState(originState);

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    Eigen::Map<Eigen::Quaternions> quat_map(perturbed_OriginState.data() + 3);

    orientation_perturbation(0) = 1.02;
    orientation_perturbation(1) = 0.53;
    orientation_perturbation(2) = 2.09;

    //introduce the perturbation directly in the quaternion StateBlock
    quat_map = quat_map * v2q(orientation_perturbation);
    origin_KF->setState(perturbed_OriginState);
    
    //prepare problem for solving

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE((origin_KF->getPPtr()->getState() - originState.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Pos : " << origin_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << originState.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getOPtr()->getState() - originState.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Ori : " << origin_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << originState.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarV1B1P2Q2V2B2_InvarP1Q1_PosinitError)
{
    //change initial guess to a wrong value
    Eigen::VectorXs perturbed_FinalState(expected_final_state);
    perturbed_FinalState.head(3) = expected_final_state.head(3) + (Eigen::Vector3s() << 0.15, 0.5, 1).finished();

    //prepare problem for solving
    last_KF->setState(perturbed_FinalState);

    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Pos : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_final_state.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Ori : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_final_state.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarV1B1P2Q2V2B2_InvarP1Q1_QuatInitError)
{
    //change initial guess to a wrong value
    Eigen::VectorXs perturbed_FinalState(expected_final_state);
    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    Eigen::Map<Eigen::Quaternions> quat_map(perturbed_FinalState.data() + 3);

    orientation_perturbation(0) = 1.02;
    orientation_perturbation(1) = 0.53;
    orientation_perturbation(2) = 2.09;

    //introduce the perturbation directly in the quaternion StateBlock
    quat_map = quat_map * v2q(orientation_perturbation);
    last_KF->setState(perturbed_FinalState);
    
    //prepare problem for solving

    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Pos : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_final_state.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Ori : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_final_state.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarP1Q1B1V2B2_InvarV1P2Q2_initOK_ConstrP_KF0)
{
    //prepare problem for solving
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1);
    ConstraintBaseList ctr_list = origin_KF->getConstrainedByList();
    //std::cout << "ctr_list size : " << ctr_list.size() << std::endl;
    for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
    {
        //std::cout << "ctr ID : " << (*ctr_it)->getTypeId() << std::endl;
        if ((*ctr_it)->getTypeId() == CTR_ODOM_3D) //change covariances in features to constraint only position
        {
            Eigen::MatrixXs meas_cov((*ctr_it)->getMeasurementCovariance());
            //std::cout << "meas_cov : \n" << meas_cov << std::endl;
            meas_cov.bottomRightCorner(3,3) = Eigen::Matrix3s::Identity()*50;
            (*ctr_it)->getFeaturePtr()->setMeasurementCovariance(meas_cov);
            //std::cout << "new meas_cov : \n" << (*ctr_it)->getMeasurementCovariance() << std::endl;
        }
    }
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE((origin_KF->getPPtr()->getState() - expected_origin_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Pos : " << origin_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_origin_state.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getOPtr()->getState() - expected_origin_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Ori : " << origin_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_origin_state.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarP2Q2B1V2B2_InvarV1P1Q1_initOK_ConstrP_KF0)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    //wolf_problem_ptr_->print(4,1,1,1);
    ConstraintBaseList ctr_list = origin_KF->getConstrainedByList();
    //std::cout << "ctr_list size : " << ctr_list.size() << std::endl;
    for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
    {
        //std::cout << "ctr ID : " << (*ctr_it)->getTypeId() << std::endl;
        if ((*ctr_it)->getTypeId() == CTR_ODOM_3D) //change covariances in features to constraint only position
        {
            Eigen::MatrixXs meas_cov((*ctr_it)->getMeasurementCovariance());
            //std::cout << "meas_cov : \n" << meas_cov << std::endl;
            meas_cov.bottomRightCorner(3,3) = (Eigen::Matrix3s() << 50, 20, 10, 20, 50, 20, 10, 20, 50).finished();
            (*ctr_it)->getFeaturePtr()->setMeasurementCovariance(meas_cov);
            //std::cout << "new meas_cov : \n" << (*ctr_it)->getMeasurementCovariance() << std::endl;
        }
    }
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Pos : " << origin_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_final_state.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Ori : " << origin_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_final_state.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarP1Q1B1V2B2_InvarV1P2Q2_initOK_ConstrO_KF0)
{
    //prepare problem for solving
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();

    ConstraintBaseList ctr_list = origin_KF->getConstrainedByList();
    //std::cout << "ctr_list size : " << ctr_list.size() << std::endl;
    for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
    {
        //std::cout << "ctr ID : " << (*ctr_it)->getTypeId() << std::endl;
        if ((*ctr_it)->getTypeId() == CTR_ODOM_3D) //change covariances in features to constraint only position
        {
            Eigen::MatrixXs meas_cov((*ctr_it)->getMeasurementCovariance());
            meas_cov.topLeftCorner(3,3) =  (Eigen::Matrix3s() << 50, 20, 10, 20, 50, 20, 10, 20, 50).finished();
            (*ctr_it)->getFeaturePtr()->setMeasurementCovariance(meas_cov);
            //std::cout << "new meas_cov : \n" << (*ctr_it)->getMeasurementCovariance() << std::endl;
        }
    }
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE((origin_KF->getPPtr()->getState() - expected_origin_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Pos : " << origin_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_origin_state.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getOPtr()->getState() - expected_origin_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Ori : " << origin_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_origin_state.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarP2Q2B1V2B2_InvarV1P1Q1_initOK_ConstrO_KF0)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    ConstraintBaseList ctr_list = origin_KF->getConstrainedByList();
    //std::cout << "ctr_list size : " << ctr_list.size() << std::endl;
    for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
    {
        //std::cout << "ctr ID : " << (*ctr_it)->getTypeId() << std::endl;
        if ((*ctr_it)->getTypeId() == CTR_ODOM_3D) //change covariances in features to constraint only position
        {
            Eigen::MatrixXs meas_cov((*ctr_it)->getMeasurementCovariance());
            //std::cout << "meas_cov : \n" << meas_cov << std::endl;
            meas_cov.topLeftCorner(3,3) = (Eigen::Matrix3s() << 50, 20, 10, 20, 50, 20, 10, 20, 50).finished();
            (*ctr_it)->getFeaturePtr()->setMeasurementCovariance(meas_cov);
            //std::cout << "new meas_cov : \n" << (*ctr_it)->getMeasurementCovariance() << std::endl;
        }
    }
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Pos : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_final_state.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Ori : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_final_state.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarQ1P2B1V2B2_InvarV1P1Q2_initOK_ConstrO_KF0)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);
    last_KF->getOPtr()->fix();

    ConstraintBaseList ctr_list = origin_KF->getConstrainedByList();
    //std::cout << "ctr_list size : " << ctr_list.size() << std::endl;
    for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
    {
        //std::cout << "ctr ID : " << (*ctr_it)->getTypeId() << std::endl;
        if ((*ctr_it)->getTypeId() == CTR_ODOM_3D) //change covariances in features to constraint only position
        {
            Eigen::MatrixXs meas_cov((*ctr_it)->getMeasurementCovariance());
            //std::cout << "meas_cov : \n" << meas_cov << std::endl;
            meas_cov.topLeftCorner(3,3) = (Eigen::Matrix3s() << 50, 20, 10, 20, 50, 20, 10, 20, 50).finished();
            (*ctr_it)->getFeaturePtr()->setMeasurementCovariance(meas_cov);
            //std::cout << "new meas_cov : \n" << (*ctr_it)->getMeasurementCovariance() << std::endl;
        }
    }
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Pos : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_final_state.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getOPtr()->getState() - expected_origin_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Ori : " << origin_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_origin_state.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Real,M1_VarQ1P2Q2B1V2B2_InvarV1P1V2_initOK_ConstrO_KF0)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    ConstraintBaseList ctr_list = origin_KF->getConstrainedByList();
    //std::cout << "ctr_list size : " << ctr_list.size() << std::endl;
    for(auto ctr_it = ctr_list.begin(); ctr_it!=ctr_list.end(); ctr_it++)
    {
        //std::cout << "ctr ID : " << (*ctr_it)->getTypeId() << std::endl;
        if ((*ctr_it)->getTypeId() == CTR_ODOM_3D) //change covariances in features to constraint only position
        {
            Eigen::MatrixXs meas_cov((*ctr_it)->getMeasurementCovariance());
            //std::cout << "meas_cov : \n" << meas_cov << std::endl;
            meas_cov.topLeftCorner(3,3) = (Eigen::Matrix3s() << 50, 20, 10, 20, 50, 20, 10, 20, 50).finished();
            (*ctr_it)->getFeaturePtr()->setMeasurementCovariance(meas_cov);
            //std::cout << "new meas_cov : \n" << (*ctr_it)->getMeasurementCovariance() << std::endl;
        }
    }
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Pos : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Pos : " << expected_final_state.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Ori : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected Ori : " << expected_final_state.segment(3,4).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Bias,M1_VarP2Q2B1V2B2_InvarV1P1Q1_initOK)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    //===================================================== END{SETTING PROBLEM}

    char* imu_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/M1.txt");
    imu_filepath   = new char[imu_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

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
    Eigen::VectorXs expected_final_state(16);
    expected_final_state << 0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D, 6, 6);

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
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;
    M1_bias << origin_KF->getAccBiasPtr()->getState(), origin_KF->getGyroBiasPtr()->getState(); 
}

TEST_F(ProcessorIMU_Bias,M2_VarP2Q2B1V2B2_InvarV1P1Q1_initOK)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    //===================================================== END{SETTING PROBLEM}

    char* imu_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/M2.txt");
    imu_filepath   = new char[imu_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

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
    Eigen::VectorXs expected_final_state(16);
    expected_final_state << 0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D, 6, 6);

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
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    M2_bias << origin_KF->getAccBiasPtr()->getState(), origin_KF->getGyroBiasPtr()->getState(); 
    std::cout << "M2_bias - M1_bias : " << (M2_bias - M1_bias).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Bias,M3_VarP2Q2B1V2B2_InvarV1P1Q1_initOK)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    //===================================================== END{SETTING PROBLEM}

    char* imu_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/M3.txt");
    imu_filepath   = new char[imu_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

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
    Eigen::VectorXs expected_final_state(16);
    expected_final_state << 0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D, 6, 6);

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
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    M3_bias << origin_KF->getAccBiasPtr()->getState(), origin_KF->getGyroBiasPtr()->getState(); 
    std::cout << "M3_bias - M1_bias : " << (M3_bias - M1_bias).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Bias,M4_VarP2Q2B1V2B2_InvarV1P1Q1_initOK)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    //===================================================== END{SETTING PROBLEM}

    char* imu_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/M4.txt");
    imu_filepath   = new char[imu_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

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
    Eigen::VectorXs expected_final_state(16);
    expected_final_state << 0,-0.06,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D, 6, 6);

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
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected O : " << expected_final_state.segment(3,4).transpose() << std::endl;

    M4_bias << origin_KF->getAccBiasPtr()->getState(), origin_KF->getGyroBiasPtr()->getState(); 
    std::cout << "M4_bias - M1_bias : " << (M4_bias - M1_bias).transpose() << std::endl;
}

TEST_F(ProcessorIMU_Bias_LowQualityOdom,Foot_VarQ1P2Q2B1V2B2_InvarV1P1_initOK)
{
    std::string wolf_root = _WOLF_ROOT_DIR;

    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/pattern20s.dat");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/pattern20s_odom.dat");

    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    //WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open() | !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0, 0,0,0,1, 0,0,0).finished());
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() && !odom_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        sen_imu->process(imu_ptr);

        if(ts.get() == t_odom.get()) //every 100 ms
        {
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

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    //origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    wolf_problem_ptr_->print(4,1,1,1);

    std::ofstream framesCov;
    framesCov.open("gtest_imuBias_foot.dat");
    if(framesCov)
        framesCov   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;

    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBaseList frame_listCov = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_listCov)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            TimeStamp ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getVPtr(), frmIMU_ptr->getVPtr(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBiasPtr(), frmIMU_ptr->getAccBiasPtr(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBiasPtr(), frmIMU_ptr->getGyroBiasPtr(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)


            framesCov << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "|\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "|\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "|\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "|\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "||\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "|\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "|\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "|\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "|\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }
    framesCov.close();
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "ProcessorIMU_Real.M*";
  //::testing::GTEST_FLAG(filter) = "ProcessorIMU_Bias_LowQualityOdom.Foot_VarQ1P2Q2B1V2B2_InvarV1P1_initOK";
  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}