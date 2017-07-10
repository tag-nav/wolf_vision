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
#include "constraint_fix_3D.h"

#include "utils_gtest.h"
#include "../src/logging.h"

#include <iostream>
#include <fstream>

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

        expected_final_state = x_origin; //null bias + static

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << -wolf::gravity(), 0,0,0;

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu); //set data on IMU (measure only gravity here)

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t));
        last_KF->setState(expected_final_state); //We expect to find this solution, this can be perturbated in following tests

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Static_NonNullAccBias : public testing::Test
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
        origin_bias << 0.002, 0.005, 0.1, 0,0,0;
        //origin_bias << 0.002, 0.005, 0.1, 0.07,-0.035,-0.1;

        expected_final_state = x_origin; //null bias + static

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << -wolf::gravity(), 0,0,0;
        data_imu = data_imu + origin_bias;

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu); //set data on IMU (measure only gravity here)

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t));
        last_KF->setState(expected_final_state); //We expect to find this solution, this can be perturbated in following tests

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest_Static_NonNullGyroBias : public testing::Test
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
        origin_bias << 0, 0,0, 0.07,-0.035,-0.1;
        //origin_bias << 0.002, 0.005, 0.1, 0.07,-0.035,-0.1;

        expected_final_state = x_origin; //null bias + static, 
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << -wolf::gravity(), 0,0,0;
        data_imu = data_imu + origin_bias;

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu); //set data on IMU (measure only gravity here)

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t));
        last_KF->setState(expected_final_state); //We expect to find this solution, this can be perturbated in following tests

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
        origin_bias << 0.002, 0.005, 0.1, 0.07,-0.035,-0.1;

        expected_final_state = x_origin; //null bias + static, 
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << -wolf::gravity(), 0,0,0;
        data_imu = data_imu + origin_bias;

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu); //set data on IMU (measure only gravity here)

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t));
        last_KF->setState(expected_final_state); //We expect to find this solution, this can be perturbated in following tests

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

        Eigen::Vector6s data_imu;
        data_imu << 0,10,-wolf::gravity()(2), 0,0,0; //10m/s on y direction
        expected_final_state << 0,5,0, 0,0,0,1, 0,10,0, 0,0,0, 0,0,0; // advanced at a=10m/s2 during 1s ==> dx = 0.5*10*1^2 = 5; dvx = 10*1 = 10

        origin_bias<< 0,0,0,0,0,0;

        expected_final_state = x_origin;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu);

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t));
        last_KF->setState(expected_final_state);

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

        Eigen::Vector6s data_imu;
        origin_bias = Eigen::Vector6s::Random();
        data_imu << 0,10,-wolf::gravity()(2), 0,0,0; //10m/s on y direction
        data_imu = data_imu + origin_bias;
        expected_final_state << 0,5,0, 0,0,0,1, 0,10,0, 0,0,0, 0,0,0; // advanced at a=10m/s2 during 1s ==> dx = 0.5*10*1^2 = 5; dvx = 10*1 = 10

        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu);

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t));
        last_KF->setState(expected_final_state);

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

        Eigen::Vector6s data_imu, data_imu_initial;
        origin_bias = Eigen::Vector6s::Random();
        wolf::Scalar rate_of_turn = 5 * M_PI/180.0; // rad/s
        data_imu << -wolf::gravity(), rate_of_turn,0,0; //rotation only
        data_imu_initial = data_imu;

        // Expected state after one second integration
        Eigen::Quaternions quat_comp(Eigen::Quaternions::Identity());
        quat_comp = quat_comp * wolf::v2q(data_imu.tail(3)*1);
        expected_final_state << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0, 0,0,0, 0,0,0; // rotated at 5 deg/s for 0.1s = 5 deg => 5 * M_PI/180

        data_imu = data_imu + origin_bias; // bias measurements
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu);

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            //gravity measure depends on current IMU orientation + bias
            //use data_imu_initial to measure gravity from real orientation of IMU then add biases
            data_imu.head(3) = (v2q(data_imu_initial.tail(3) * t.get()).conjugate() * data_imu_initial.head(3)) + origin_bias.head(3);
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setData(data_imu);
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t));
        last_KF->setState(expected_final_state);

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
        x_origin << 0,0,0, 0,0,0,1, 10,-3,4, 0,0,0, 0,0,0;
        t.set(0);

        Eigen::Vector6s data_imu, data_imu_initial;
        origin_bias = Eigen::Vector6s::Random();
        origin_bias << 0,0,0, 0,0,0;
        wolf::Scalar rate_of_turn = 5 * M_PI/180.0; // rad/s
        data_imu << -wolf::gravity(), rate_of_turn,0,0; //rotation only
        data_imu_initial = data_imu;

        // Expected state after one second integration
        Eigen::Quaternions quat_comp(Eigen::Quaternions::Identity());
        quat_comp = quat_comp * wolf::v2q(data_imu.tail(3)*1);

        // rotated at 5 deg/s for 1s = 5 deg => 5 * M_PI/180
        // no other acceleration : we should still be moving at initial velocity
        // position = V*dt, dt = 1s
        expected_final_state << 10,-3,4, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 10,-3,4, 0,0,0, 0,0,0;

        data_imu = data_imu + origin_bias; // bias measurements
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu);

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            //gravity measure depends on current IMU orientation + bias
            //use data_imu_initial to measure gravity from real orientation of IMU then add biases
            data_imu.head(3) = (v2q(data_imu_initial.tail(3) * t.get()).conjugate() * data_imu_initial.head(3)) + origin_bias.head(3);
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setData(data_imu);
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t));
        last_KF->setState(expected_final_state);

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

// var(b1,b2), inv(p1,q1,v1,p2,q2,v2); fac1: imu(p,q,v)+(b1=b2)

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
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
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
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
        t.set(0);
        Eigen::Quaternions current_quatState(Eigen::Quaternions::Identity());

        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn(Eigen::Vector3s::Zero()); //deg/s

        Scalar dt(0.001);
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);

        while( ts.get() < 1 )
        {
            // PROCESS IMU DATA
            // Time and data variables
            ts.set(ts.get() + dt);
            
            rateOfTurn = Eigen::Vector3s::Random()*10; //to have rate of turn > 0 deg/s
            data_imu.tail(3) = rateOfTurn* M_PI/180.0;
            data_imu.head(3) =  current_quatState.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement

            //compute current orientaton taking this measure into account
            current_quatState = current_quatState * wolf::v2q(data_imu.tail(3)*dt);

            //set timestamp, add bias, set data and process
            imu_ptr->setTimeStamp(ts);
            data_imu = data_imu + origin_bias;
            imu_ptr->setData(data_imu);
            sen_imu->process(imu_ptr);
        }

        expected_final_state.head(10) << 0,0,0, current_quatState.x(), current_quatState.y(), current_quatState.z(), current_quatState.w(), 0,0,0;
        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

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
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
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
        prc_odom3D_params->max_time_span = 0.9999;
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
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
        t.set(0);
        Eigen::Quaternions odom_quat(Eigen::Quaternions::Identity());
        Eigen::Quaternions current_quatState(Eigen::Quaternions::Identity());

        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero()), data_odom3D(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn(Eigen::Vector3s::Zero()); //deg/s

        Scalar dt(0.0010), dt_odom(1.0);
        TimeStamp ts(0.0), t_odom(0.0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
        sen_odom3D->process(mot_ptr);
        //first odometry data will be processed at this timestamp
        t_odom.set(t_odom.get() + dt_odom);

        //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

        for(unsigned int i = 1; i<=1000; i++)
        {
            // PROCESS IMU DATA
            // Time and data variables
            ts.set(i*dt);
            
            rateOfTurn = Eigen::Vector3s::Random()*10; //to have rate of turn > 0.99 deg/s
            data_imu.tail<3>() = rateOfTurn* M_PI/180.0;
            data_imu.head<3>() =  current_quatState.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement

            //compute odometry + current orientaton taking this measure into account
            odom_quat = odom_quat * wolf::v2q(data_imu.tail(3)*dt);
            current_quatState = current_quatState * wolf::v2q(data_imu.tail(3)*dt);

            //set timestamp, add bias, set data and process
            imu_ptr->setTimeStamp(ts);
            data_imu = data_imu + origin_bias;
            imu_ptr->setData(data_imu);
            sen_imu->process(imu_ptr);

            if(ts.get() >= t_odom.get())
            {
                // PROCESS ODOM 3D DATA
                data_odom3D.head(3) << 0,0,0;
                data_odom3D.tail(3) = q2v(odom_quat);
                mot_ptr->setTimeStamp(t_odom);
                mot_ptr->setData(data_odom3D);
                sen_odom3D->process(mot_ptr);

                //prepare next odometry measurement
                odom_quat = Eigen::Quaternions::Identity(); //set to identity to have next odom relative to this last KF
                t_odom.set(t_odom.get() + dt_odom);
            }
        }

        expected_final_state.head(10) << 0,0,0, current_quatState.x(), current_quatState.y(), current_quatState.z(), current_quatState.w(), 0,0,0;
        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot2 : public testing::Test
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
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
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
        prc_odom3D_params->max_time_span = 0.4999;
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
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
        t.set(0);
        Eigen::Quaternions odom_quat(Eigen::Quaternions::Identity());
        Eigen::Quaternions current_quatState(Eigen::Quaternions::Identity());

        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero()), data_odom3D(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn(Eigen::Vector3s::Zero()); //deg/s

        Scalar dt(0.0010), dt_odom(0.5);
        TimeStamp ts(0.0), t_odom(0.0);
        int odom_count(1);

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
        sen_odom3D->process(mot_ptr);
        //first odometry data will be processed at this timestamp
        t_odom.set(odom_count*dt_odom);

        //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

        for(unsigned int i = 1; i<=1000; i++)
        {
            // PROCESS IMU DATA
            // Time and data variables
            ts.set(i*dt);
            
            rateOfTurn = Eigen::Vector3s::Random()*10; //to have rate of turn > 0.99 deg/s
            data_imu.tail<3>() = rateOfTurn* M_PI/180.0;
            data_imu.head<3>() =  current_quatState.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement

            //compute odometry + current orientaton taking this measure into account
            odom_quat = odom_quat * wolf::v2q(data_imu.tail(3)*dt);
            current_quatState = current_quatState * wolf::v2q(data_imu.tail(3)*dt);

            //set timestamp, add bias, set data and process
            imu_ptr->setTimeStamp(ts);
            data_imu = data_imu + origin_bias;
            imu_ptr->setData(data_imu);
            sen_imu->process(imu_ptr);

            if(ts.get() >= t_odom.get())
            {
                // PROCESS ODOM 3D DATA
                data_odom3D.head(3) << 0,0,0;
                data_odom3D.tail(3) = q2v(odom_quat);
                mot_ptr->setTimeStamp(t_odom);
                mot_ptr->setData(data_odom3D);
                sen_odom3D->process(mot_ptr);

                //prepare next odometry measurement
                odom_quat = Eigen::Quaternions::Identity(); //set to identity to have next odom relative to this last KF
                odom_count++;
                t_odom.set(odom_count*dt_odom);
            }
        }

        expected_final_state.head(10) << 0,0,0, current_quatState.x(), current_quatState.y(), current_quatState.z(), current_quatState.w(), 0,0,0;
        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY : public testing::Test
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
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
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
        prc_odom3D_params->max_time_span = 0.9999;
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
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
        t.set(0);
        Eigen::Quaternions odom_quat(Eigen::Quaternions::Identity());
        Eigen::Quaternions current_quatState(Eigen::Quaternions::Identity());

        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero()), data_odom3D(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn; //deg/s
        rateOfTurn << 0,90,0;

        Scalar dt(0.0010), dt_odom(1.0);
        TimeStamp ts(0.0), t_odom(0.0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
        sen_odom3D->process(mot_ptr);
        //first odometry data will be processed at this timestamp
        t_odom.set(t_odom.get() + dt_odom);

        data_imu.tail<3>() = rateOfTurn* M_PI/180.0; //constant rotation =

        //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement
        for(unsigned int i = 1; i<=1000; i++)
        {
            // PROCESS IMU DATA
            // Time and data variables
            ts.set(i*dt);
            data_imu.head<3>() =  current_quatState.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement

            //compute odometry + current orientaton taking this measure into account
            odom_quat = odom_quat * wolf::v2q(data_imu.tail(3)*dt);
            current_quatState = current_quatState * wolf::v2q(data_imu.tail(3)*dt);

            //set timestamp, add bias, set data and process
            imu_ptr->setTimeStamp(ts);
            data_imu = data_imu + origin_bias;
            imu_ptr->setData(data_imu);
            sen_imu->process(imu_ptr);

            if(ts.get() >= t_odom.get())
            {
                // PROCESS ODOM 3D DATA
                data_odom3D.head(3) << 0,0,0;
                data_odom3D.tail(3) = q2v(odom_quat);
                mot_ptr->setTimeStamp(t_odom);
                mot_ptr->setData(data_odom3D);
                sen_odom3D->process(mot_ptr);

                //prepare next odometry measurement
                odom_quat = Eigen::Quaternions::Identity(); //set to identity to have next odom relative to this last KF
                t_odom.set(t_odom.get() + dt_odom);
            }
        }

        expected_final_state.head(10) << 0,0,0, current_quatState.x(), current_quatState.y(), current_quatState.z(), current_quatState.w(), 0,0,0;
        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotXY : public testing::Test
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
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
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
        prc_odom3D_params->max_time_span = 0.9999;
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
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
        t.set(0);
        Eigen::Quaternions odom_quat(Eigen::Quaternions::Identity());
        Eigen::Quaternions current_quatState(Eigen::Quaternions::Identity());

        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero()), data_odom3D(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn; //deg/s
        rateOfTurn << 45,90,0;

        Scalar dt(0.0010), dt_odom(1.0);
        TimeStamp ts(0.0), t_odom(1.0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
        sen_odom3D->process(mot_ptr);
        //first odometry data will be processed at this timestamp
        //t_odom.set(t_odom.get() + dt_odom);

        Eigen::Vector2s randomPart;
        data_imu.tail<3>() = rateOfTurn* M_PI/180.0; //constant rotation =

        //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement
        for(unsigned int i = 1; i<=500; i++)
        {
            // PROCESS IMU DATA
            // Time and data variables
            ts.set(i*dt);

            /*data_imu.tail<3>() = rateOfTurn* M_PI/180.0;
            randomPart = Eigen::Vector2s::Random(); //to have rate of turn > 0.99 deg/s
            data_imu.segment<2>(3) += randomPart* M_PI/180.0;*/
            data_imu.head<3>() =  current_quatState.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement

            //compute odometry + current orientaton taking this measure into account
            odom_quat = odom_quat * wolf::v2q(data_imu.tail(3)*dt);
            current_quatState = current_quatState * wolf::v2q(data_imu.tail(3)*dt);

            //set timestamp, add bias, set data and process
            imu_ptr->setTimeStamp(ts);
            data_imu = data_imu + origin_bias;
            imu_ptr->setData(data_imu);
            sen_imu->process(imu_ptr);

            if(ts.get() >= t_odom.get())
            {
                // PROCESS ODOM 3D DATA
                data_odom3D.head(3) << 0,0,0;
                data_odom3D.tail(3) = q2v(odom_quat);
                mot_ptr->setTimeStamp(t_odom);
                mot_ptr->setData(data_odom3D);
                sen_odom3D->process(mot_ptr);

                //prepare next odometry measurement
                odom_quat = Eigen::Quaternions::Identity(); //set to identity to have next odom relative to this last KF
                t_odom.set(t_odom.get() + dt_odom);
            }
        }

        rateOfTurn << 30,10,0;
        data_imu.tail<3>() = rateOfTurn* M_PI/180.0;

        for(unsigned int j = 1; j<=500; j++)
        {
            // PROCESS IMU DATA
            // Time and data variables
            ts.set((500 + j)*dt);

            /*data_imu.tail<3>() = rateOfTurn* M_PI/180.0;
            randomPart = Eigen::Vector2s::Random(); //to have rate of turn > 0.99 deg/s
            data_imu.segment<2>(3) += randomPart* M_PI/180.0;*/
            data_imu.head<3>() =  current_quatState.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement

            //compute odometry + current orientaton taking this measure into account
            odom_quat = odom_quat * wolf::v2q(data_imu.tail(3)*dt);
            current_quatState = current_quatState * wolf::v2q(data_imu.tail(3)*dt);

            //set timestamp, add bias, set data and process
            imu_ptr->setTimeStamp(ts);
            data_imu = data_imu + origin_bias;
            imu_ptr->setData(data_imu);
            sen_imu->process(imu_ptr);

            if(ts.get() >= t_odom.get())
            {
                // PROCESS ODOM 3D DATA
                data_odom3D.head(3) << 0,0,0;
                data_odom3D.tail(3) = q2v(odom_quat);
                mot_ptr->setTimeStamp(t_odom);
                mot_ptr->setData(data_odom3D);
                sen_odom3D->process(mot_ptr);

                //prepare next odometry measurement
                odom_quat = Eigen::Quaternions::Identity(); //set to identity to have next odom relative to this last KF
                t_odom.set(t_odom.get() + dt_odom);
            }
        }

        expected_final_state.head(10) << 0,0,0, current_quatState.x(), current_quatState.y(), current_quatState.z(), current_quatState.w(), 0,0,0;
        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotYRandom : public testing::Test
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
        
        //===================================================== SETTING PROBLEM
        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
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
        prc_odom3D_params->max_time_span = 0.4999;
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
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
        t.set(0);
        Eigen::Quaternions odom_quat(Eigen::Quaternions::Identity());
        Eigen::Quaternions current_quatState(Eigen::Quaternions::Identity());

        expected_final_state.tail(6) = origin_bias;
        x_origin.tail(6) = origin_bias;

        //set origin of the problem
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero()), data_odom3D(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn; //deg/s
        Eigen::Vector1s randomPart;
        rateOfTurn << 0,90,0;

        Scalar dt(0.001), dt_odom(0.5);
        TimeStamp ts(0.0), t_odom(0.0);
        int odom_count(1);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6);
        sen_odom3D->process(mot_ptr);
        //first odometry data will be processed at this timestamp
        t_odom.set(odom_count*dt_odom);

        data_imu.tail<3>() = rateOfTurn* M_PI/180.0; //constant rotation =
        
        //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement
        for(unsigned int i = 1; i<=1000; i++)
        {
            // PROCESS IMU DATA
            // Time and data variables
            ts.set(i*dt);

            data_imu.tail<3>() = rateOfTurn* M_PI/180.0;
            randomPart = Eigen::Vector1s::Random();
            data_imu.segment<1>(4) += randomPart* M_PI/180.0;

            data_imu.head<3>() =  current_quatState.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement

            //compute odometry + current orientaton taking this measure into account
            odom_quat = odom_quat * wolf::v2q(data_imu.tail(3)*dt);
            current_quatState = current_quatState * wolf::v2q(data_imu.tail(3)*dt);

            //set timestamp, add bias, set data and process
            imu_ptr->setTimeStamp(ts);
            data_imu = data_imu + origin_bias;
            imu_ptr->setData(data_imu);
            sen_imu->process(imu_ptr);

            if(ts.get() >= t_odom.get())
            {
                // PROCESS ODOM 3D DATA
                data_odom3D.head(3) << 0,0,0;
                data_odom3D.tail(3) = q2v(odom_quat);
                mot_ptr->setTimeStamp(t_odom);
                mot_ptr->setData(data_odom3D);
                sen_odom3D->process(mot_ptr);

                //prepare next odometry measurement
                odom_quat = Eigen::Quaternions::Identity(); //set to identity to have next odom relative to this last KF
                odom_count++;
                t_odom.set(odom_count*dt_odom);
            }
        }

        expected_final_state.head(10) << 0,0,0, current_quatState.x(), current_quatState.y(), current_quatState.z(), current_quatState.w(), 0,0,0;
        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
        last_KF->setState(expected_final_state);

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
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_ComplexBiased.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_Complex.txt");

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
        prc_odom3D_params->max_time_span = 1.9999;
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
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_ComplexBiased.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_Complex.txt");

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
        prc_odom3D_params->max_time_span = 1.9999;
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
        data_imu << -wolf::gravity(), 0,0,0;

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

            if(ts.get() == t_odom.get())
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
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_ComplexNoisy.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_Complex.txt");

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
        prc_odom3D_params->max_time_span = 1.9999;
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
        data_imu << -wolf::gravity(), 0,0,0;

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
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/imu_ComplexBiasedNoisy.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/odom_Complex.txt");

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
        prc_odom3D_params->max_time_span = 1.99999;
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

    //wolf_problem_ptr_->print(1,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //wolf_problem_ptr_->print(1,1,1,1);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_biasTest_Static_NullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    }
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullAccBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
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
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullAccBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    }
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullGyroBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
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
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullGyroBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    }
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
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
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_biasTest_Move_NullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
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

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

   // wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    }
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotCst,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotCst,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    }
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotAndVCst,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotAndVCst,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    }
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotAndVCst, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRot, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*1000)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*1000)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));

    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY, VarB1B2V2_InvarP1Q1V1P2Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));

    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

//jacobian matrix rank deficient here - estimating both initial and final velocity
TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY, VarB1B2V1V2_InvarP1Q1P2Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)

    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
}

//jacobian matrix rank deficient here - estimating both initial and final velocity
TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY, VarB1B2V1Q2V2_InvarP1Q1P2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
}

//jacobian matrix rank deficient here - estimating both initial and final velocity
TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
}

//jacobian matrix rank deficient here - estimating both initial and final velocity
TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY, VarB1B2V1P2Q2V2_InvarP1Q1_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.0001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    //wolf_problem_ptr_->print(4,1,1,1);
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*10000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.0001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.0001)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*10000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.0001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.0001)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY, VarB1B2P2Q2V2_InvarP1Q1V1_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));

    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY, VarQ1B1B2P2Q2_InvarP1V1V2_initOK)
{
    //Add fix constraint on yaw to make the problem observable
    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFix_cov(5,5) = 0.1;
    CaptureBasePtr capfix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 6));
    FeatureBasePtr ffix = capfix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), featureFix_cov));
    ConstraintFix3DPtr ctr_fix = std::static_pointer_cast<ConstraintFix3D>(ffix->addConstraint(std::make_shared<ConstraintFix3D>(ffix)));
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
    
    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotXY, VarQ1B1B2P2Q2_InvarP1V1V2_initOK)
{
    //Add fix constraint on yaw to make the problem observable
    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFix_cov(5,5) = 0.1;
    CaptureBasePtr capfix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 6));
    FeatureBasePtr ffix = capfix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), featureFix_cov));
    ConstraintFix3DPtr ctr_fix = std::static_pointer_cast<ConstraintFix3D>(ffix->addConstraint(std::make_shared<ConstraintFix3D>(ffix)));
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
    
    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotYRandom, VarQ1B1B2P2Q2_InvarP1V1V2_initOK)
{
    //Add fix constraint on yaw to make the problem observable
    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFix_cov(5,5) = 0.1;
    CaptureBasePtr capfix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 6));
    FeatureBasePtr ffix = capfix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), featureFix_cov));
    ConstraintFix3DPtr ctr_fix = std::static_pointer_cast<ConstraintFix3D>(ffix->addConstraint(std::make_shared<ConstraintFix3D>(ffix)));
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);
    //wolf_problem_ptr_->print(4,0,1,0);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
    
    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    //wolf_problem_ptr_->print(4,1,1,1);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*1000)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*1000)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));

    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V2_InvarP1Q1V1P2Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));

    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V1V2_InvarP1Q1P2Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)

    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V1Q2V2_InvarP1Q1P2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2V1P2Q2V2_InvarP1Q1_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.0001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    //wolf_problem_ptr_->print(4,1,1,1);
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*10000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.0001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.0001)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*10000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.0001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.0001)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarB1B2P2Q2V2_InvarP1Q1V1_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));

    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot2, VarQ1B1B2P2Q2_InvarP1V1V2_initOK)
{
    //Add fix constraint on yaw to make the problem observable
    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFix_cov(5,5) = 0.1;
    CaptureBasePtr capfix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 6));
    FeatureBasePtr ffix = capfix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), featureFix_cov));
    ConstraintFix3DPtr ctr_fix = std::static_pointer_cast<ConstraintFix3D>(ffix->addConstraint(std::make_shared<ConstraintFix3D>(ffix)));
    
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->fix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();

    //perturbation of origin bias
    Eigen::Vector3s random_err(Eigen::Vector3s::Random() * 0.00001);
    Eigen::Vector3s accBias = origin_KF->getAccBiasPtr()->getState();
    Eigen::Vector3s gyroBias = origin_KF->getGyroBiasPtr()->getState();
    origin_KF->getAccBiasPtr()->setState(accBias + random_err);
    origin_KF->getGyroBiasPtr()->setState(gyroBias + random_err);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.00001)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
    
    if(cov_stdev.tail(6).maxCoeff()>=1)
        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

//not constrained enough, thus, estimation fails
/*TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot, VarAll_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->unfix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_MATRIX_APPROX(origin_KF->getPPtr()->getState(), x_origin.head(3), wolf::Constants::EPS*100)
    Eigen::Map<const Eigen::Quaternions> estimatedOriginQuat(origin_KF->getOPtr()->getState().data()), expectedOriginQuat(x_origin.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedOriginQuat, expectedOriginQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*100)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}*/

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK, VarB1B2_InvarP1Q1V1P2Q2V2)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    WOLF_WARN("Precision set to ", 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)
    
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK, VarB1B2P2Q2_InvarP1Q1V1V2)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    WOLF_WARN("Precision set to ", 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), 0.001) //biased + noisy imu, consider an error of 1mm
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, 0.00001)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK, VarB1B2P2Q2V2_InvarP1Q1V1)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    WOLF_WARN("Precision set to ", 0.0001)

    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.0001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), 0.0001)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), 0.0001)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.0001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK, VarAll)
{
    //prepare problem for solving
    origin_KF->getPPtr()->unfix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->unfix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    WOLF_WARN("Precision set to ", 0.001)

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getPPtr()->getState(), x_origin.head(3), 0.0001)
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.0001)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), 0.0001)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.0001)
}

//Commented tests below are falling in local minimum
/*
TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex, VarB1B2_InvarP1Q1V1P2Q2V2_initZero) //Falling in local minimum
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    //ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    
    //ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    //ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero()), cov4(Eigen::Matrix4s::Zero());
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex, VarB1B2P2Q2V2_InvarP1Q1V1_initZero) //Falling in local minimum
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero()), cov4(Eigen::Matrix4s::Zero());
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
}
*/
TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);

    /*ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)
    
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), wolf::Constants::EPS*100)*/

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero()), cov4(Eigen::Matrix4s::Zero());
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_NonNullBiasComplex_initOK,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbated_origin_state(x_origin);
    ceres::Solver::Summary summary;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();
    WOLF_WARN("Precision set to ", 0.001)

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    summary = ceres_manager_wolf_diff->solve();

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(expected_final_state);

        summary = ceres_manager_wolf_diff->solve();

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
        ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)

        ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.001)
        ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.001)
    }
}

//Tests related to noise

TEST_F(ConstraintIMU_ODOM_biasTest_Static_NullBiasNoisyComplex_initOK, varB1B2P2Q2B2_invarP1Q1V1V2)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);
    std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), 0.005)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero()), cov4(Eigen::Matrix4s::Zero());
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
}

TEST_F(ConstraintIMU_ODOM_biasTest_Static_NullBiasNoisyComplex_initOK, varQ1B1P2Q2B2_invarP1V1V2) // added a Fix3D constraint on 1st KF
{
    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFix_cov(5,5) = 0.1;
    CaptureBasePtr capfix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 6));
    FeatureBasePtr ffix = capfix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), featureFix_cov));
    ConstraintFix3DPtr ctr_fix = std::static_pointer_cast<ConstraintFix3D>(ffix->addConstraint(std::make_shared<ConstraintFix3D>(ffix)));

    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->fix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);
    std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), 0.005)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, 0.001)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero()), cov4(Eigen::Matrix4s::Zero());
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
}

TEST_F(ConstraintIMU_ODOM_biasTest_Static_NullBiasNoisyComplex_initOK, varB1B2_invarP1Q1V1P2Q2V2)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    origin_KF->getAccBiasPtr()->unfix();
    origin_KF->getGyroBiasPtr()->unfix();

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    //wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);
    std::cout << summary.BriefReport() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.01)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.01)

    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.01)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.01)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero()), cov4(Eigen::Matrix4s::Zero());
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
}

TEST_F(ConstraintIMU_ODOM_biasTest_Move_BiasedNoisyComplex_initOK, varB1P2Q2V2B2_invarP1Q1V1)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->unfix();
    last_KF->getVPtr()->unfix();

    //perturbatte a little the bias of origin state
    Eigen::VectorXs perturbated_origin_state(x_origin);
    wolf::Scalar epsilon_bias = 0.0001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbated_origin_state.tail(6) = x_origin.tail(6) + err;
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(expected_final_state);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    ceres_manager_wolf_diff->computeCovariances(ALL);
    std::cout << summary.BriefReport() << std::endl;

    //These ASSERTS can be removed since we are more interested in using covariances to make sure that expected values are inside estimated +/- 2*std
    WOLF_WARN("Precision set to ", 0.01)

    ASSERT_MATRIX_APPROX(origin_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.01)
    ASSERT_MATRIX_APPROX(origin_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.01)

    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), 0.0001)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), 0.01)
    ASSERT_MATRIX_APPROX(last_KF->getAccBiasPtr()->getState(), origin_bias.head(3), 0.01)
    ASSERT_MATRIX_APPROX(last_KF->getGyroBiasPtr()->getState(), origin_bias.tail(3), 0.01)

    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(16,16);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<16; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<16; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot2.*";
  return RUN_ALL_TESTS();
}
