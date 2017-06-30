/**
 * \file gtest_constraint_imu.cpp
 *
 *  Created on: Jan 01, 2017
 *      \author: Dinesh Atchuthan
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

/*
 * This test is designed to test IMU biases in a particular case : perfect IMU, not moving.
 * var(b1,b2,p2,v2,q2), inv(p1,q1,v1); fac1: imu+(b1=b2)
 * So there is no odometry data.
 * IMU data file should first contain initial conditions, then the time_step between each new imu and the last one (in seconds) data,
 * and finally the last stateafter integration and the last timestamp, Then it should contain all IMU data and related timestamps
 */

class ConstraintIMU_biasTest_Static_NullBias : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_static_biasNull.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Static_NonNullBias : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_static_biasNonNull.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        
        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Move_NullBias : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_move_biasNull.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Move_NonNullBias : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_move_BiasNonNull.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Move_NonNullBiasFreeFalling : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_move_AbxNonNull_FreeFalling.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_MoveTR_NonNullBiasAccCst : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_moveTR_AbxNonNull_AccCst.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Move_NonNullBiasRotCst : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_move_AbxNonNull_RotCst.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Move_NonNullBiasRotAndVCst : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_move_BiasNonNull_RotVCst.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Move_NonNullBiasRot : public testing::Test
{
    public:
        wolf::TimeStamp t;
        SensorIMUPtr sen_imu;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Vector6s origin_bias;
        Eigen::VectorXs expected_final_state;
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;

        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/data_bias_check_Rot.txt");

        imu_filepath = new char[imu_filepath_string.length() + 1];

        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::ifstream imu_data_input;

        imu_data_input.open(imu_filepath);
        if(!imu_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }
        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( !imu_data_input.eof())
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_testBase : public testing::Test
{
    public:
        wolf::ProblemPtr wolf_problem_ptr_;
        wolf::TimeStamp ts;
        wolf::CaptureIMUPtr imu_ptr;
        Eigen::VectorXs state_vec;
        Eigen::VectorXs delta_preint;
        Eigen::Matrix<wolf::Scalar,9,9> delta_preint_cov;
        std::shared_ptr<wolf::FeatureIMU> feat_imu;
        wolf::FrameIMUPtr last_frame;
        wolf::FrameIMUPtr origin_frame;
        Eigen::Matrix<wolf::Scalar,9,6> dD_db_jacobians;
    
    virtual void SetUp()
    {
        using namespace wolf;
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;
        
        // Wolf problem
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs IMU_extrinsics(7);
        IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
        SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, shared_ptr<IntrinsicsBase>());
        wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Time and data variables
        TimeStamp t;
        Eigen::Vector6s data_;
        state_vec.resize(16);
        t.set(0.01);

    // Set the origin
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases
        wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    //create a keyframe at origin
        ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
        Eigen::VectorXs origin_state = x0;
        origin_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, origin_state);
        wolf_problem_ptr_->getTrajectoryPtr()->addFrame(origin_frame);
    
    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
        imu_ptr = std::make_shared<CaptureIMU>(t, sensor_ptr, data_);
        imu_ptr->setFramePtr(origin_frame);

    //process data
        data_ << 2, 0, 9.8, 0, 0, 0;
        t.set(0.1);
        // Expected state after one integration
        //x << 0.01,0,0, 0,0,0,1, 0.2,0,0, 0,0,0, 0,0,0; // advanced at a=2m/s2 during 0.1s ==> dx = 0.5*2*0.1^2 = 0.01; dvx = 2*0.1 = 0.2
    // assign data to capture
        imu_ptr->setData(data_);
        imu_ptr->setTimeStamp(t);
    // process data in capture
        sensor_ptr->process(imu_ptr);
    }

    virtual void TearDown()
    {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
        /*
            You can do deallocation of resources in TearDown or the destructor routine. 
                However, if you want exception handling you must do it only in the TearDown code because throwing an exception 
                from the destructor results in undefined behavior.
            The Google assertion macros may throw exceptions in platforms where they are enabled in future releases. 
                Therefore, it's a good idea to use assertion macros in the TearDown code for better maintenance.
        */
    }

};


// var(b1,b2), inv(p1,q1,v1,p2,q2,v2); fac1: imu(p,q,v)+(b1=b2)
class ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot : public testing::Test
{
    public:
        wolf::TimeStamp t;
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
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;
        char* odom_filepath;
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/data_bias_check_Rot.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_bias_check_Rot.txt");

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

        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 5.9999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

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
            //std::cout << "input_clock : " << input_clock << std::endl;
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
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();
        odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Move_NonNullBiasTr_initOK : public testing::Test
{
    public:
        wolf::TimeStamp t;
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
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;
        char* odom_filepath;
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/data_bias_check_Tr.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_bias_check_Tr.txt");

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

        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 5.9999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

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
            //std::cout << "input_clock : " << input_clock << std::endl;
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
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();
        odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex : public testing::Test
{
    public:
        wolf::TimeStamp t;
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
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;
        char* odom_filepath;
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/data_bias_check_Complex.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_bias_check_Complex.txt");

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

        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 5.999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        //x_origin.tail(6) = origin_bias;

        /*std::cout << "x_origin : " << x_origin.transpose() << std::endl;
        std::cout << "origin_bias : " << origin_bias.transpose() << std::endl;
        std::cout << "expected_final_state : " << expected_final_state.transpose() << std::endl;*/
        

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

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
            //std::cout << "input_clock : " << input_clock << std::endl;
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
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();
        odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK : public testing::Test
{
    public:
        wolf::TimeStamp t;
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
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;
        char* odom_filepath;
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/data_bias_check_Complex.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_bias_check_Complex.txt");

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

        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 5.9999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;    

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

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
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();
        odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Static_NullBiasNoisyComplex_initOK : public testing::Test
{
    public:
        wolf::TimeStamp t;
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
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;
        char* odom_filepath;
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/data_bias_check_StaticNullBiasNoisy.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_bias_check_StaticNullBiasNoisy.txt");

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

        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 2.9999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;    

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

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
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();
        odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Move_BiasedNoisyComplex_initOK : public testing::Test
{
    public:
        wolf::TimeStamp t;
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
        Eigen::VectorXs x_origin;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================================== INPUT FILES

        char* imu_filepath;
        char* odom_filepath;
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/data_check_BiasedNoisyComplex.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_check_BiasedNoisyComplex.txt");

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

        //===================================================== END{INPUT FILES}
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 0.04999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(16);
        x_origin.resize(16);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
        t.set(0);

        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];
        imu_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                    expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;    

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;

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
        last_KF->setState(expected_final_state);

        //closing file
        imu_data_input.close();
        odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    origin_KF->unfix();
    last_KF->unfix();
    }

    virtual void TearDown(){}
};

/*TEST_F(ConstraintIMU_testBase, constructorIMU)
{   
    using namespace wolf;

    //create FrameIMU
    ts = 0.1;
    state_vec << 0.01,0,0, 0,0,0,1, 0.2,0,0, 0,0,0, 0,0,0;
   	last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, state_vec);
    //create a feature
    delta_preint_cov = Eigen::MatrixXs::Identity(9,9);
    delta_preint.resize(10);
    delta_preint << 0.01,0,0.049, 0,0,0,1, 0.2,0,0.98;
    dD_db_jacobians = Eigen::Matrix<wolf::Scalar,9,6>::Random();
    feat_imu = std::make_shared<FeatureIMU>(delta_preint, delta_preint_cov);

    //create the constraint
    ConstraintIMU constraint_imu(feat_imu,last_frame);
}*/

// tests with following conditions :
//  var(b1,b2),        invar(p1,q1,v1,p2,q2,v2),    factor : imu(p,q,v)

TEST_F(ConstraintIMU_biasTest_Static_NullBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    origin_KF->getAccBiasPtr()->setState((Vector3s()<<1,2,3).finished());
    origin_KF->getGyroBiasPtr()->setState((Vector3s()<<1,2,3).finished());

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();
    last_KF->getAccBiasPtr()->setState((Vector3s()<<-1,-2,-3).finished());
    last_KF->getGyroBiasPtr()->setState((Vector3s()<<-1,-2,-3).finished());

    wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
}

TEST_F(ConstraintIMU_biasTest_Static_NullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{

    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_StaticNullBias.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{

    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_StaticNonNullBias.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
}

TEST_F(ConstraintIMU_biasTest_Move_NullBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_biasTest_Move_NullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{

    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_MoveNullBias_VarB1B2.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();
    last_KF->getAccBiasPtr()->fix();
    last_KF->getGyroBiasPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //wolf_problem_ptr_->print(4,1,1,1);

    #ifdef DEBUG_RESULTS_BIAS
        std::ofstream debug_results;
        debug_results.open("save_gtest_CTRIMU_bias_MoveNonNullBias_VarB1B2.dat");
        if(debug_results)
        {
            debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << 0 << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        }
    #else

        //Only biases are unfixed
        ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
        "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
        ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
        "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

        ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
        "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
        ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
        "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    #endif
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{

    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_MoveNonNullBias_VarB1B2.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasFreeFalling, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasFreeFalling,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{

    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_MoveNonNullBiasFreeFalling_VarB1B2.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;
}

TEST_F(ConstraintIMU_biasTest_MoveTR_NonNullBiasAccCst, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
}

TEST_F(ConstraintIMU_biasTest_MoveTR_NonNullBiasAccCst,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_NonNullBiasAccCst_VarB1B2.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotCst, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals before CERES : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals after CERES: " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotCst,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_TRNonNullBiasRotCst_VarB1B2.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, 0.0001 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, 0.0001 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, 0.0001 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, 0.0001 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, 0.00001 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, 0.00001 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, 0.00001 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, 0.00001 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, 0.00001 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, 0.00001 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, 0.00001 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, 0.00001 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotAndVCst, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotAndVCst,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{

    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_TRNonNullBiasRotVCst_VarB1B2.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        //last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotAndVCst, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    //last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    //last_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRot, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getOPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V2_InvarP1Q1V1P2Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    //last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V1V2_InvarP1Q1P2Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    //last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V1Q2V2_InvarP1Q1P2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();

    last_KF->getPPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    //last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();

    last_KF->getOPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    last_KF->setState(expected_final_state);

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V1P2Q2V2_InvarP1Q1_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    last_KF->setState(expected_final_state);

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2P2Q2V2_InvarP1Q1V1_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    last_KF->setState(expected_final_state);

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarAll_initOK)
{

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasTr_initOK, VarB1B2_InvarP1Q1V1P2Q2V2)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasTr_initOK, VarAll)
{
    //prepare problem for solving

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK, VarB1B2_InvarP1Q1V1P2Q2V2)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    //wolf_problem_ptr_->print(4,1,1,1); 

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK, VarB1B2P2Q2_InvarP1Q1V1V2)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK, VarB1B2P2Q2V2_InvarP1Q1V1)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    WOLF_WARN("LAST_KF V assertion precision : ", wolf::Constants::EPS*1000)
    EXPECT_TRUE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK, VarAll)
{
    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex, VarB1B2_InvarP1Q1V1P2Q2V2_initZero) //falling in local minimum
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);
    //wolf_problem_ptr_->print(4,1,1,1); 

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Fallin in local minimum
    EXPECT_FALSE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_FALSE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex, VarB1B2P2Q2V2_InvarP1Q1V1_initZero) //falling in local minimum
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Falling in local minimum
    EXPECT_FALSE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_FALSE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_FALSE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex, VarAll_initBiasZero) //falling in local minimum
{
    //prepare problem for solving
    //origin_KF->getPPtr()->fix();
    //origin_KF->getOPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1); 

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //ceres_manager_wolf_diff->computeCovariances(ALL);
    //std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Falling in local minimum
    EXPECT_FALSE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, 0.0001 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_FALSE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, 0.0001 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{

    #ifdef DEBUG_RESULTS_BIAS
    std::ofstream debug_results;
    debug_results.open("save_gtest_CTRIMU_bias_Complex_VarB1B2.dat");
    if(debug_results)
        debug_results   << "%%introduced_error\t"
                        << "exp_KF0_Abx\t" << "exp_KF0_Aby\t" << "exp_KF0_Abz\t" << "exp_KF0_Wbx\t" << "exp_KF0_Wby\t" << "exp_KF0_Wbz\t"
                        << "init_KF0_Abx\t" << "init_KF0_Aby\t" << "init_KF0_Abz\t" << "init_KF0_Wbx\t" << "init_KF0_Wby\t" << "init_KF0_Wbz\t"
                        << "res_KF0_Abx\t" << "res_KF0_Aby\t" << "res_KF0_Abz\t" << "res_KF0_Wbx\t" << "res_KF0_Wby\t" << "res_KF0_Wbz\t"
                        << "res_KF1_Abx\t" << "res_KF1_Aby\t" << "res_KF1_Abz\t" << "res_KF1_Wbx\t" << "res_KF1_Wby\t" << "res_KF1_Wbz\t" << std::endl;
    #endif

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-7")

    for(int i = 0; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-5")
    epsilon_bias = 0.00001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            EXPECT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            EXPECT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            EXPECT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            EXPECT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-3")
    epsilon_bias = 0.001;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    for(int i = 1; i<9; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<90; i++)
    {
        perturbated_origin_state[10] = x_origin(10) + i * epsilon_bias;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        last_KF->getPPtr()->fix();
        last_KF->getOPtr()->fix();
        last_KF->getVPtr()->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        #ifdef DEBUG_RESULTS_BIAS
            Eigen::VectorXs KF0_frm_state(16), KF1_frm_state(16);
            KF0_frm_state = origin_KF->getState();
            KF1_frm_state = last_KF->getState();

            debug_results << std::setprecision(16) << i * epsilon_bias << "\t" << origin_bias(0) << "\t"  << origin_bias(1) << "\t"  << origin_bias(2) << "\t"  << origin_bias(3) << "\t"  << origin_bias(4) << "\t"  << origin_bias(5) << "\t"  
            << perturbated_origin_state(10) << "\t" << perturbated_origin_state(11) << "\t" << perturbated_origin_state(12) << "\t" << perturbated_origin_state(13) << "\t" << perturbated_origin_state(14) << "\t" << perturbated_origin_state(15) << "\t" 
            << KF0_frm_state(10) << "\t" << KF0_frm_state(11) << "\t" << KF0_frm_state(12) << "\t" << KF0_frm_state(13) << "\t" << KF0_frm_state(14) << "\t" << KF0_frm_state(15) << "\t" 
                << KF1_frm_state(10) << "\t" << KF1_frm_state(11) << "\t" << KF1_frm_state(12) << "\t" << KF1_frm_state(13) << "\t" << KF1_frm_state(14) << "\t" << KF1_frm_state(15) << std::endl;
        #else

            //Only biases are unfixed
            ASSERT_TRUE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

            ASSERT_TRUE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
            "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
            ASSERT_TRUE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
            "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
        #endif
    }
    //std::cout << summary.FullReport() << std::endl;
}

//Tests related to noise

TEST_F(ConstraintIMU_ODOM_biasTest_Static_NullBiasNoisyComplex_initOK, varV1B1P2Q2V2B2_invarP1Q1)
{
    origin_KF->getOPtr()->fix();
    origin_KF->getPPtr()->fix();

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);
    std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    //Noisy data, this will certainly be false excepted for position and orientation due to precise odometry
    EXPECT_FALSE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_FALSE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_FALSE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    std::ofstream framesCov;
    framesCov.open("framesCovariances.dat");
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

    #endif
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_BiasedNoisyComplex_initOK, varV1B1P2Q2V2B2_invarP1Q1)
{
    origin_KF->getOPtr()->fix();
    origin_KF->getPPtr()->fix();

    #ifdef GET_RESIDUALS
        wolf::FrameBaseList frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();

        //trials to print all constraintIMUs' residuals
        Eigen::Matrix<wolf::Scalar,15,1> IMU_residuals;
        Eigen::Vector3s p1(Eigen::Vector3s::Zero());
        Eigen::Vector4s q1_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q1(q1_vec.data());
        Eigen::Vector3s v1(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab1(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb1(Eigen::Vector3s::Zero());
        Eigen::Vector3s p2(Eigen::Vector3s::Zero());
        Eigen::Vector4s q2_vec(Eigen::Vector4s::Zero());
        Eigen::Map<Quaternions> q2(q2_vec.data());
        Eigen::Vector3s v2(Eigen::Vector3s::Zero());
        Eigen::Vector3s ab2(Eigen::Vector3s::Zero());
        Eigen::Vector3s wb2(Eigen::Vector3s::Zero());

        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    #endif

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);
    std::cout << summary.BriefReport() << std::endl;

   // wolf_problem_ptr_->print(4,1,1,1);

    //Noisy data, this will certainly be false excepted for position and orientation due to precise odometry
    EXPECT_FALSE((origin_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((origin_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "origin_KF Gyro bias : " << origin_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;

    EXPECT_FALSE((last_KF->getAccBiasPtr()->getState() - origin_bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getState().transpose() <<
    "\n expected Acc bias : " << origin_bias.head(3).transpose() << std::endl;
    EXPECT_FALSE((last_KF->getGyroBiasPtr()->getState() - origin_bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getState().transpose() <<
    "\n expected Gyro bias : " << origin_bias.tail(3).transpose() << std::endl;
    WOLF_WARN("position precision in expactation test : ", 0.0001)
    EXPECT_TRUE((last_KF->getPPtr()->getState() - expected_final_state.head(3)).isMuchSmallerThan(1, 0.0001 )) << "last_KF P : " << last_KF->getPPtr()->getState().transpose() <<
    "\n expected Position : " << expected_final_state.head(3).transpose() << std::endl;
    EXPECT_TRUE((last_KF->getOPtr()->getState() - expected_final_state.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS*1000 )) << "last_KF Q : " << last_KF->getOPtr()->getState().transpose() <<
    "\n expected orientation : " << expected_final_state.segment(3,4).transpose() << std::endl;
    EXPECT_FALSE((last_KF->getVPtr()->getState() - expected_final_state.segment(7,3)).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << "last_KF V : " << last_KF->getVPtr()->getState().transpose() <<
    "\n expected velocity : " << expected_final_state.segment(7,3).transpose() << std::endl;

    #ifdef GET_RESIDUALS
        frame_list = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList();
        for(FrameBasePtr frm_ptr : frame_list)
        {
            if(frm_ptr->isKey())
            {
                ConstraintBaseList ctr_list =  frm_ptr->getConstrainedByList();
                for(ConstraintBasePtr ctr_ptr : ctr_list)
                {
                    if(ctr_ptr->getTypeId() == CTR_IMU)
                    {
                        p1      = ctr_ptr->getFrameOtherPtr()->getPPtr()->getState();
                        q1_vec  = ctr_ptr->getFrameOtherPtr()->getOPtr()->getState();
                        v1      = ctr_ptr->getFrameOtherPtr()->getVPtr()->getState();
                        ab1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getAccBiasPtr()->getState();
                        wb1     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFrameOtherPtr())->getGyroBiasPtr()->getState();

                        p2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getPPtr()->getState();
                        q2_vec  = ctr_ptr->getFeaturePtr()->getFramePtr()->getOPtr()->getState();
                        v2      = ctr_ptr->getFeaturePtr()->getFramePtr()->getVPtr()->getState();
                        ab2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getAccBiasPtr()->getState();
                        wb2     = std::static_pointer_cast<FrameIMU>(ctr_ptr->getFeaturePtr()->getFramePtr())->getGyroBiasPtr()->getState();

                        std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, IMU_residuals);
                        std::cout << "IMU residuals : " << IMU_residuals.transpose() << std::endl;
                    }
                }
            }
        }

    std::ofstream framesCov;
    framesCov.open("framesCovariances121kf_new.dat");
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

    #endif
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "ConstraintIMU_ODOM_biasTest_Move_BiasedNoisyComplex_initOK.*:ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex*:";
  //::testing::GTEST_FLAG(filter) = "ConstraintIMU_biasTest_Static_NullBias.VarB1B2_InvarP1Q1V1P2Q2V2_initOK";
  return RUN_ALL_TESTS();
}
