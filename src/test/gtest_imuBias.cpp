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
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 1.99999;
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

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
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

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
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

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
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

    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected P : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getOPtr()->getState().transpose() <<
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

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  //::testing::GTEST_FLAG(filter) = "";
  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}