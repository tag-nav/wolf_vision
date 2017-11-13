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
        FrameBasePtr KF0;
        FrameBasePtr KF1;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        t.set(0);

        origin_bias << 0,0,0, 0,0,0;
        expected_final_state = x_origin; //null bias + static

        //set origin of the problem
        KF0 = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << -wolf::gravity(), 0,0,0;

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu, sen_imu->getNoiseCov(), origin_bias); //set data on IMU (measure only gravity here)

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);

        }

        KF1 = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
        KF1->setState(expected_final_state); //We expect to find this solution, this can be perturbated in following tests

        //===================================================== END{PROCESS DATA}
        KF0->unfix();
        KF1->unfix();
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
        FrameBasePtr KF0;
        FrameBasePtr KF1;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}
        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        t.set(0);
        origin_bias << 0.02, 0.05, 0.1, 0,0,0;

        expected_final_state = x_origin; //null bias + static

        //set origin of the problem
        KF0 = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << -wolf::gravity(), 0,0,0;
        data_imu = data_imu + origin_bias;

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu, sen_imu->getNoiseCov(), origin_bias); //set data on IMU (measure only gravity here)

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        KF1 = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
        KF1->setState(expected_final_state); //We expect to find this solution, this can be perturbated in following tests

        //===================================================== END{PROCESS DATA}
        KF0->unfix();
        KF1->unfix();
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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
//        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
//        ceres_options.max_line_search_step_contraction = 1e-3;
//        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}
        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        t.set(0);
        origin_bias << 0, 0, 0, 0.0002, 0.0005, 0.001;

        expected_final_state = x_origin; //null bias + static,

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << -wolf::gravity(), 0,0,0;
        data_imu = data_imu + origin_bias;

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu, sen_imu->getNoiseCov() );//, origin_bias); //set data on IMU (measure only gravity here)

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}
        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        t.set(0);
        origin_bias << 0.002, 0.005, 0.1, 0.07,-0.035,-0.1;
        origin_bias *= .01;

        expected_final_state = x_origin; //null bias + static
        

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu;
        data_imu << -wolf::gravity(), 0,0,0;
        data_imu = data_imu + origin_bias;

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu, sen_imu->getNoiseCov(), Eigen::Vector6s::Zero()); //set data on IMU (measure only gravity here)

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}

        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        t.set(0);

        Eigen::Vector6s data_imu;
        data_imu << 0,10,-wolf::gravity()(2), 0,0,0; //10m/s on y direction
        expected_final_state << 0,5,0, 0,0,0,1, 0,10,0; // advanced at a=10m/s2 during 1s ==> dx = 0.5*10*1^2 = 5; dvx = 10*1 = 10

        origin_bias<< 0,0,0,0,0,0;

        expected_final_state = x_origin;

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}


        //===================================================== PROCESS DATA
        // PROCESS DATA

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu, sen_imu->getNoiseCov(), Eigen::Vector6s::Zero());

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}
        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        t.set(0);

        Eigen::Vector6s data_imu;
        origin_bias = Eigen::Vector6s::Random() * 0.001;
        data_imu << 0,10,-wolf::gravity()(2), 0,0,0; //10m/s on y direction
        data_imu = data_imu + origin_bias;
        expected_final_state << 0,5,0, 0,0,0,1, 0,10,0; // advanced at a=10m/s2 during 1s ==> dx = 0.5*10*1^2 = 5; dvx = 10*1 = 10

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu, sen_imu->getNoiseCov(), Eigen::Vector6s::Zero());

        for(unsigned int i = 0; i < 1000; i++) //integrate during 1 second
        {
            t.set(t.get() + 0.001); //increment of 1 ms
            imu_ptr->setTimeStamp(t);

            // process data in capture
            sen_imu->process(imu_ptr);
        }

        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}
        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        t.set(0);

        Eigen::Vector6s data_imu, data_imu_initial;
        origin_bias = Eigen::Vector6s::Random() * 0.001;
        wolf::Scalar rate_of_turn = 5 * M_PI/180.0; // rad/s
        data_imu << -wolf::gravity(), rate_of_turn,0,0; //rotation only
        data_imu_initial = data_imu;

        // Expected state after one second integration
        Eigen::Quaternions quat_comp(Eigen::Quaternions::Identity());
        quat_comp = quat_comp * wolf::v2q(data_imu.tail(3)*1);
        expected_final_state << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0; // rotated at 5 deg/s for 0.1s = 5 deg => 5 * M_PI/180

        data_imu = data_imu + origin_bias; // bias measurements

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu, sen_imu->getNoiseCov(), Eigen::Vector6s::Zero());

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

        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}
        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 10,-3,4;
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
        expected_final_state << 10,-3,4, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 10,-3,4;

        data_imu = data_imu + origin_bias; // bias measurements

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(t, sen_imu, data_imu, sen_imu->getNoiseCov(), Eigen::Vector6s::Zero());

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

        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t1.yaml");
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);
    
        //===================================================== END{SETTING PROBLEM}
        //===================================================== INITIALIZATION

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        origin_bias << 0.01, 0.02, 0.003, 0.002, 0.005, 0.01;
        t.set(0);
        Eigen::Quaternions quat_current(Eigen::Quaternions::Identity());

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn(Eigen::Vector3s::Zero()); //deg/s

        Scalar dt(0.001);
        TimeStamp ts(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu, sen_imu->getNoiseCov(), Eigen::Vector6s::Zero());

        while( ts.get() < 1 )
        {
            // PROCESS IMU DATA
            // Time and data variables
            ts += dt;
            
            rateOfTurn << .1, .2, .3; //to have rate of turn > 0 deg/s
            data_imu.head(3) = quat_current.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement
            data_imu.tail(3) = rateOfTurn;

            //compute current orientaton taking this measure into account
            quat_current = quat_current * wolf::v2q(rateOfTurn*dt);

            //set timestamp, add bias, set data and process
            imu_ptr->setTimeStamp(ts);
            data_imu = data_imu + origin_bias;
            imu_ptr->setData(data_imu);
            sen_imu->process(imu_ptr);

        }

        expected_final_state << 0,0,0, quat_current.x(), quat_current.y(), quat_current.z(), quat_current.w(), 0,0,0;
        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts);
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
        ProblemPtr problem;
        CeresManagerPtr ceres_manager;
        SensorBasePtr sensor;
        SensorIMUPtr sensor_imu;
        SensorOdom3DPtr sensor_odo;
        ProcessorBasePtr processor;
        ProcessorIMUPtr processor_imu;
        ProcessorOdom3DPtr processor_odo;
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
        CaptureIMUPtr    capture_imu;
        CaptureOdom3DPtr capture_odo;
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
        problem = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
        ceres_manager = std::make_shared<CeresManager>(problem, ceres_options);

        // SENSOR + PROCESSOR IMU
        sensor          = problem->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        sensor_imu      = std::static_pointer_cast<SensorIMU>(sensor);
        processor       = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_t6.yaml");
        processor_imu   = std::static_pointer_cast<ProcessorIMU>(processor);

        // SENSOR + PROCESSOR ODOM 3D
        sensor          = problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        sensor_odo      = std::static_pointer_cast<SensorOdom3D>(sensor);

        sensor_imu->setNoiseStd((sensor_imu->getNoiseStd()/10.0).eval());
        sensor_odo->setNoiseStd((sensor_odo->getNoiseStd()/10.0).eval());
        WOLF_TRACE("IMU cov: ", sensor_imu->getNoiseCov().diagonal().transpose());
        WOLF_TRACE("ODO cov: ", sensor_odo->getNoiseCov().diagonal().transpose());

        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span    = 0.0099;
        prc_odom3D_params->max_buff_length  = 1000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled    = 1000;
        prc_odom3D_params->angle_turned     = 1000;

        processor       = problem->installProcessor("ODOM 3D", "odom", sensor_odo, prc_odom3D_params);
        processor_odo   = std::static_pointer_cast<ProcessorOdom3D>(processor);

        //===================================================== END{SETTING PROBLEM}
        //===================================================== INITIALIZATION

        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
//        origin_bias /= 100;
        t.set(0);

        //set origin of the problem

        origin_KF = processor_imu->setOrigin(x_origin, t);
        processor_odo->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero()),
                        data_odo(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn(Eigen::Vector3s::Zero()); //deg/s

        TimeStamp t_imu(0.0),    t_odo(0.0);
        Scalar   dt_imu(0.001), dt_odo(.01);

        capture_imu = std::make_shared<CaptureIMU>   (t_imu, sensor_imu, data_imu, sensor_imu->getNoiseCov(), sensor_imu->getCalibration(), nullptr);

        capture_odo = std::make_shared<CaptureOdom3D>(t_odo, sensor_odo, data_odo, sensor_odo->getNoiseCov(), nullptr);
        sensor_odo->process(capture_odo);
        t_odo += dt_odo;        //first odometry data will be processed at this timestamp

        // ground truth for quaternion:
        Eigen::Quaternions quat_odo(Eigen::Quaternions::Identity());
        Eigen::Quaternions quat_imu(Eigen::Quaternions::Identity());

        WOLF_TRACE("last delta preint: ", processor_imu->getLastPtr()->getDeltaPreint().transpose());
        WOLF_TRACE("last jacoob bias : ", processor_imu->getLastPtr()->getJacobianCalib().row(0));

        for(unsigned int i = 1; i<=1000; i++)
        {

            // PROCESS IMU DATA
            // Time and data variables
            t_imu += dt_imu;
            
//            rateOfTurn = Eigen::Vector3s::Random()*10; //to have rate of turn > 0.99 deg/s
            rateOfTurn << 5, 10, 15; // deg/s
            data_imu.tail<3>() = rateOfTurn * M_PI/180.0;
            data_imu.head<3>() =  quat_imu.conjugate() * (- wolf::gravity()); //gravity measured, we have no other translation movement

            //compute odometry + current orientaton taking this measure into account
            quat_odo = quat_odo * wolf::v2q(data_imu.tail(3)*dt_imu);
            quat_imu = quat_imu * wolf::v2q(data_imu.tail(3)*dt_imu);

            //set timestamp, add bias, set data and process
            capture_imu->setTimeStamp(t_imu);
            data_imu = data_imu + origin_bias;
            capture_imu->setData(data_imu);
            sensor_imu->process(capture_imu);

            WOLF_TRACE("last delta preint: ", processor_imu->getLastPtr()->getDeltaPreint().transpose());
            WOLF_TRACE("last jacoob bias : ", processor_imu->getLastPtr()->getJacobianCalib().row(0));

            //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement
            if(t_imu.get() >= t_odo.get())
            {
                WOLF_TRACE("====== create ODOM KF ========");
//                WOLF_TRACE("Jac calib: ", processor_imu->getLastPtr()->getJacobianCalib().row(0));
//                WOLF_TRACE("last calib: ", processor_imu->getLastPtr()->getCalibration().transpose());
//                WOLF_TRACE("last calib preint: ", processor_imu->getLastPtr()->getCalibrationPreint().transpose());

                // PROCESS ODOM 3D DATA
                data_odo.head(3) << 0,0,0;
                data_odo.tail(3) = q2v(quat_odo);
                capture_odo->setTimeStamp(t_odo);
                capture_odo->setData(data_odo);

//                WOLF_TRACE("Jac calib: ", processor_imu->getLastPtr()->getJacobianCalib().row(0));
//                WOLF_TRACE("last calib: ", processor_imu->getLastPtr()->getCalibration().transpose());
//                WOLF_TRACE("last calib preint: ", processor_imu->getLastPtr()->getCalibrationPreint().transpose());

                sensor_odo->process(capture_odo);

//                WOLF_TRACE("Jac calib: ", std::static_pointer_cast<CaptureMotion>(processor_imu->getOriginPtr())->getJacobianCalib().row(0));
//                WOLF_TRACE("orig calib: ", processor_imu->getOriginPtr()->getCalibration().transpose());
//                WOLF_TRACE("orig calib preint: ", std::static_pointer_cast<CaptureMotion>(processor_imu->getOriginPtr())->getCalibrationPreint().transpose());

                //prepare next odometry measurement
                quat_odo = Eigen::Quaternions::Identity(); //set to identity to have next odom relative to this last KF
                t_odo += dt_odo;
                break;
            }
        }

        expected_final_state.resize(10);
        expected_final_state << 0,0,0, quat_imu.x(), quat_imu.y(), quat_imu.z(), quat_imu.w(), 0,0,0;

        last_KF = problem->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t_imu);
        last_KF->setState(expected_final_state);

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();

        CaptureBasePtr origin_CB = origin_KF->getCaptureOf(sensor_imu);
        CaptureMotionPtr last_CM   = std::static_pointer_cast<CaptureMotion>(last_KF  ->getCaptureOf(sensor_imu));

//        WOLF_TRACE("KF1 calib    : ", origin_CB->getCalibration().transpose());
//        WOLF_TRACE("KF2 calib pre: ", last_CM  ->getCalibrationPreint().transpose());
//        WOLF_TRACE("KF2 calib    : ", last_CM  ->getCalibration().transpose());
//        WOLF_TRACE("KF2 delta pre: ", last_CM  ->getDeltaPreint().transpose());
//        WOLF_TRACE("KF2 delta cor: ", last_CM  ->getDeltaCorrected(origin_CB->getCalibration()).transpose());
//        WOLF_TRACE("KF2 jacob    : ", last_CM  ->getJacobianCalib().row(0));


        // ==================================================== show problem status

        problem->print(4,1,1,1);

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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

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

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
        origin_bias *= .1;
        t.set(0);
        Eigen::Quaternions odom_quat(Eigen::Quaternions::Identity());
        Eigen::Quaternions current_quatState(Eigen::Quaternions::Identity());

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero()), data_odom3D(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn; //deg/s
        rateOfTurn << 0,90,0;
        VectorXs D_cor(10);

        Scalar dt(0.0010), dt_odom(1.0);
        TimeStamp ts(0.0), t_odom(0.0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu, sen_imu->getNoiseCov(), Eigen::Vector6s::Zero());
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, sen_odom3D->getNoiseCov(), 7, 6, nullptr);
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

            D_cor = processor_ptr_imu->getLastPtr()->getDeltaCorrected(origin_bias);

            if(ts.get() >= t_odom.get())
            {
                WOLF_TRACE("X_preint(t)  : ", wolf_problem_ptr_->getState(ts).transpose());

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

        expected_final_state << 0,0,0, current_quatState.x(), current_quatState.y(), current_quatState.z(), current_quatState.w(), 0,0,0;
        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts);
        last_KF->setState(expected_final_state);

        WOLF_TRACE("X_correct(t) : ", imu::composeOverState(x_origin, D_cor, ts - t).transpose());
        WOLF_TRACE("X_true(t)    : ", expected_final_state.transpose());

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
        FrameBasePtr origin_KF;
        FrameBasePtr last_KF;
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
        wolf_problem_ptr_ = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

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

        expected_final_state.resize(10);
        x_origin.resize(10);
        x_origin << 0,0,0, 0,0,0,1, 0,0,0;
        origin_bias << 0.0015, 0.004, -0.002, 0.005, -0.0074, -0.003;
        t.set(0);
        Eigen::Quaternions odom_quat(Eigen::Quaternions::Identity());
        Eigen::Quaternions current_quatState(Eigen::Quaternions::Identity());

        //set origin of the problem
        origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
        processor_ptr_odom3D->setOrigin(origin_KF);

        //===================================================== END{INITIALIZATION}
        //===================================================== PROCESS DATA
        // PROCESS DATA

        Eigen::Vector6s data_imu(Eigen::Vector6s::Zero()), data_odom3D(Eigen::Vector6s::Zero());
        Eigen::Vector3s rateOfTurn; //deg/s
        rateOfTurn << 45,90,0;

        Scalar dt(0.0010), dt_odom(1.0);
        TimeStamp ts(0.0), t_odom(1.0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu, sen_imu->getNoiseCov(), Eigen::Vector6s::Zero());
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 7, 6, nullptr);
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

        expected_final_state << 0,0,0, current_quatState.x(), current_quatState.y(), current_quatState.z(), current_quatState.w(), 0,0,0;
        last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts);
        last_KF->setState(expected_final_state);

        //===================================================== END{PROCESS DATA}
        origin_KF->unfix();
        last_KF->unfix();
    }

    virtual void TearDown(){}
};

class ConstraintIMU_biasTest : public testing::Test
{
    public:
        ProblemPtr problem;
        SensorIMUPtr sensor_imu;
        ProcessorIMUPtr processor_imu;
        FrameBasePtr KF_0, KF_1;
        CaptureBasePtr   C_0;
        CaptureMotionPtr CM_0, CM_1;
        CaptureIMUPtr capture_imu;
        CeresManagerPtr ceres_manager;

        TimeStamp t0, t;
        Scalar dt, DT;
        int num_integrations;

        VectorXs x0;
        Vector3s p0, v0;
        Quaternions q0, q;
        Vector6s motion, data, bias_real, bias_preint, bias_null;
        Vector3s a, w, am, wm;


    virtual void SetUp( )
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        std::string wolf_root = _WOLF_ROOT_DIR;

        //===================================== SETTING PROBLEM
        problem = Problem::create("POV 3D");

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION;
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager = std::make_shared<CeresManager>(problem, ceres_options);

        // SENSOR + PROCESSOR IMU
        SensorBasePtr       sensor = problem->installSensor   ("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorBasePtr processor = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_no_vote.yaml");
        sensor_imu    = std::static_pointer_cast<SensorIMU>   (sensor);
        processor_imu = std::static_pointer_cast<ProcessorIMU>(processor);

        bias_null.setZero();

        x0.resize(10);

    }

    virtual void TearDown( ) { }

    /* Create IMU data from body motion
     * Input:
     *   motion: [ax, ay, az, wx, wy, wz] the motion in body frame
     *   q: the current orientation wrt horizontal
     *   bias: the bias of the IMU
     * Output:
     *   return: the data vector as created by the IMU (with motion, gravity, and bias)
     */
    VectorXs motion2data(const VectorXs& body, const Quaternions& q, const VectorXs& bias)
    {
        VectorXs data(6);
        data = body;                                // start with body motion
        data.head(3) -= q.conjugate()*gravity();    // add -g
        data = data + bias;                         // add bias
        return data;
    }



    /* Integrate acc and angVel motion, obtain Delta_preintegrated
     * Input:
     *   N: number of steps
     *   q0: initial orientaiton
     *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body brame
     *   bias_real: the real bias of the IMU
     *   bias_preint: the bias used for Delta pre-integration
     * Output:
     *   return: the preintegrated delta
     */
    VectorXs integrateDelta(int N, const Quaternions& q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt)
    {
        VectorXs data(6);
        VectorXs body(6);
        VectorXs delta(10);
        VectorXs Delta(10), Delta_plus(10);
        Delta = imu::identity();
        Quaternions q(q0);
        for (int n = 0; n<N; n++)
        {
            data        = motion2data(motion, q, bias_real);
            q           = q*exp_q(motion.tail(3)*dt);
            body        = data - bias_preint;
            delta       = imu::body2delta(body, dt);
            Delta_plus  = imu::compose(Delta, delta, dt);
            Delta       = Delta_plus;
        }
        return Delta;
    }

    /* Integrate acc and angVel motion, obtain Delta_preintegrated
     * Input:
     *   N: number of steps
     *   q0: initial orientaiton
     *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body brame
     *   bias_real: the real bias of the IMU
     *   bias_preint: the bias used for Delta pre-integration
     * Output:
     *   J_D_b: the Jacobian of the preintegrated delta wrt the bias
     *   return: the preintegrated delta
     */
    VectorXs integrateDelta(int N, const Quaternions& q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, Matrix<Scalar, 9, 6>& J_D_b)
    {
        VectorXs data(6);
        VectorXs body(6);
        VectorXs delta(10);
        Matrix<Scalar, 9, 6> J_d_d, J_d_b;
        Matrix<Scalar, 9, 9> J_D_D, J_D_d;
        VectorXs Delta(10), Delta_plus(10);
        Quaternions q;

        Delta = imu::identity();
        J_D_b.setZero();
        q = q0;
        for (int n = 0; n<N; n++)
        {
            // Simulate data
            data = motion2data(motion, q, bias_real);
            q    = q*exp_q(motion.tail(3)*dt);
            // Motion::integrateOneStep()
            {   // IMU::computeCurrentDelta
                body  = data - bias_preint;
                imu::body2delta(body, dt, delta, J_d_d);
                J_d_b = - J_d_d;
            }
            {   // IMU::deltaPlusDelta
                imu::compose(Delta, delta, dt, Delta_plus, J_D_D, J_D_d);
            }
            // Motion:: jac calib
            J_D_b = J_D_D*J_D_b + J_D_d*J_d_b;
            // Motion:: buffer
            Delta = Delta_plus;
        }
        return Delta;
    }

};

TEST_F(ConstraintIMU_biasTest, VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{

         // ==================================== INITIAL CONDITIONS
        t0 = 0;
        dt = 0.01;
        num_integrations = 50;
        DT = num_integrations * dt;

        p0 << 0,0,0;
        q0.setIdentity();
        v0 << 0,0,0;
        MatrixXs  P0(9,9); P0.setIdentity() * 0.01;

        x0  << p0, q0.coeffs(), v0;
        KF_0 = problem->setPrior(x0, P0, t0);
        C_0  = processor_imu->getOriginPtr();

        // bias
        bias_real  << .001, .002, .003, -.001, -.002, -.003;
        bias_preint = -bias_real;

        processor_imu->getLastPtr()->setCalibrationPreint(bias_preint);

        // ===================================== MOTION params
        a << 1,2,3;
        w << 1,2,3;

        WOLF_TRACE("w * DT (rather be lower than 1.507 approx) = ", (DT*w).transpose()); // beware if w*DT is large (>~ 1.507) then Jacobian for correction is poor

        motion << a, w;

        // ===================================== INTEGRATE EXACTLY with no bias at all
        VectorXs D_exact = integrateDelta(num_integrations, q0, motion, bias_null, bias_null, dt);
        VectorXs x_exact = imu::composeOverState(x0, D_exact, DT );

        WOLF_TRACE("D_exact      : ", D_exact.transpose() );
        WOLF_TRACE("X_exact      : ", x_exact.transpose() );

        // ===================================== INTEGRATE USING PROCESSOR
        VectorXs D_preint(10), D_corrected(10);

        capture_imu = std::make_shared<CaptureIMU>(t0, sensor_imu, data, sensor_imu->getNoiseCov());
        q = q0;
        t = t0;
        for (int i= 0; i < num_integrations; i++)
        {
            t   += dt;
            data = motion2data(motion, q, bias_real);
            q    = q * exp_q(w*dt);

            capture_imu->setTimeStamp(t);
            capture_imu->setData(data);

            sensor_imu->process(capture_imu);

            D_preint    = processor_imu->getLastPtr()->getDeltaPreint();
            D_corrected = processor_imu->getLastPtr()->getDeltaCorrected(bias_real);

        }

        // ========================================== INTEGRATE USING IMU_TOOLS
        VectorXs D_preint_imu, D_corrected_imu;
        Matrix<Scalar, 9, 6> J_b;
        D_preint_imu = integrateDelta(num_integrations, q0, motion, bias_real, bias_preint, dt, J_b);

        // correct perturbated
        Vector9s step = J_b*(bias_real-bias_preint);
        D_corrected_imu = imu::plus(D_preint_imu, step);

        WOLF_TRACE("D_preint     : ", D_preint       .transpose() );
        WOLF_TRACE("D_preint_imu : ", D_preint_imu   .transpose() );
        ASSERT_MATRIX_APPROX(D_preint, D_preint_imu, 1e-8);

        WOLF_TRACE("D_correct    : ", D_corrected    .transpose() );
        WOLF_TRACE("D_correct_imu: ", D_corrected_imu.transpose() );
        ASSERT_MATRIX_APPROX(D_corrected, D_corrected_imu, 1e-8);

        WOLF_TRACE("X_preint     : ", imu::composeOverState(x0, D_preint       , DT ).transpose() );
        WOLF_TRACE("X_preint_imu : ", imu::composeOverState(x0, D_preint_imu   , DT ).transpose() );

        WOLF_TRACE("X_correct    : ", imu::composeOverState(x0, D_corrected    , DT ).transpose() );
        WOLF_TRACE("X_correct_imu: ", imu::composeOverState(x0, D_corrected_imu, DT ).transpose() );
        ASSERT_MATRIX_APPROX(imu::composeOverState(x0, D_corrected     , DT ), x_exact, 1e-5);
        ASSERT_MATRIX_APPROX(imu::composeOverState(x0, D_corrected_imu , DT ), x_exact, 1e-5);

}

// tests with following conditions :
//  var(b1,b2),        invar(p1,q1,v1,p2,q2,v2),    factor : imu(p,q,v)

TEST_F(ConstraintIMU_biasTest_Static_NullBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    KF0->getPPtr()->fix();
    KF0->getOPtr()->fix();
    KF0->getVPtr()->fix();
    KF0->getCaptureOf(sen_imu)->setCalibration((Vector6s()<<1,2,3,1,2,3).finished());

    KF1->getPPtr()->setState(expected_final_state.head(3));
    KF1->getOPtr()->setState(expected_final_state.segment(3,4));
    KF1->getVPtr()->setState(expected_final_state.segment(7,3));

    KF1->getPPtr()->fix();
    KF1->getOPtr()->fix();
    KF1->getVPtr()->fix();
    KF1->getCaptureOf(sen_imu)->setCalibration((Vector6s()<<-1,-2,-3,-1,-2,-3).finished());

    std::string report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias

    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)
}

TEST_F(ConstraintIMU_biasTest_Static_NullBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    KF0->getPPtr()->fix();
    KF0->getOPtr()->fix();
    KF0->getVPtr()->fix();
    KF1->getPPtr()->fix();
    KF1->getOPtr()->fix();
    KF1->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbed_bias(origin_bias);
    std::string report;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    KF0->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    KF1->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias

    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    KF0->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    KF1->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias

    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    KF0->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    KF1->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias

    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbed_bias = origin_bias + err;
        KF0->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
        KF1->getCaptureOf(sen_imu)->setCalibration(origin_bias);

        report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
        ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias
    
        ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
        ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)
    }
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullAccBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    KF0->getPPtr()->fix();
    KF0->getOPtr()->fix();
    KF0->getVPtr()->fix();

    KF1->getPPtr()->setState(expected_final_state.head(3));
    KF1->getOPtr()->setState(expected_final_state.segment(3,4));
    KF1->getVPtr()->setState(expected_final_state.segment(7,3));

    KF1->getPPtr()->fix();
    KF1->getOPtr()->fix();
    KF1->getVPtr()->fix();

    std::string report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias

    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullAccBias,VarB1B2_InvarP1Q1V1P2Q2V2_ErrBias)
{
    //prepare problem for solving
    KF0->getPPtr()->fix();
    KF0->getOPtr()->fix();
    KF0->getVPtr()->fix();
    KF1->getPPtr()->fix();
    KF1->getOPtr()->fix();
    KF1->getVPtr()->fix();

    wolf::Scalar epsilon_bias = 0.0000001;
    Eigen::VectorXs perturbed_bias(origin_bias);
    std::string report;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    KF0->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    KF1->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias

    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    KF0->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    KF1->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias

    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    KF0->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    KF1->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
    ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias

    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
    ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbed_bias = origin_bias + err;
        KF0->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
        KF1->getCaptureOf(sen_imu)->setCalibration(origin_bias);

        report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS) //Acc bias
        ASSERT_MATRIX_APPROX(KF0->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS) //Gyro bias
    
        ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().head(3), origin_bias.head(3), wolf::Constants::EPS)
        ASSERT_MATRIX_APPROX(KF1->getCaptureOf(sen_imu)->getCalibration().tail(3), origin_bias.tail(3), wolf::Constants::EPS)
    }
}

TEST_F(ConstraintIMU_biasTest_Static_NonNullGyroBias,VarB1B2_InvarP1Q1V1P2Q2V2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    last_KF->getPPtr()->setState(expected_final_state.head(3));
    last_KF->getOPtr()->setState(expected_final_state.segment(3,4));
    last_KF->getVPtr()->setState(expected_final_state.segment(7,3));

    last_KF->getPPtr()->fix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->fix();

    std::string report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
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
    Eigen::VectorXs perturbed_bias(origin_bias);
    std::string report;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;

    origin_KF->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    last_KF  ->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    last_KF->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)


    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
    last_KF->getCaptureOf(sen_imu)->setCalibration(origin_bias);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbed_bias = origin_bias + err;
        origin_KF->getCaptureOf(sen_imu)->setCalibration(perturbed_bias);
        last_KF->getCaptureOf(sen_imu)->setCalibration(origin_bias);

        report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
        ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
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
    Eigen::VectorXs perturbed_bias(origin_bias);
    std::string report;

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed

    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)


    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbed_bias = origin_bias + err;
        origin_KF->setState(x_origin);
        last_KF->setState(expected_final_state);

        report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport;

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
        ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

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

    std::string report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

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
    Eigen::VectorXs perturbed_bias(origin_bias);

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    std::string report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbed_bias = origin_bias + err;
        origin_KF->setState(x_origin);
        last_KF->setState(expected_final_state);

        report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
        
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

    std::string report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

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
    Eigen::VectorXs perturbed_bias(origin_bias);

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    std::string report = ceres_manager_wolf_diff->solve(1); // 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbed_bias = origin_bias + err;
        origin_KF->setState(x_origin);
        last_KF->setState(expected_final_state);

        report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
        ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

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

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

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
    Eigen::VectorXs perturbed_bias(origin_bias);

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbed_bias = origin_bias + err;
        origin_KF->setState(x_origin);
        last_KF->setState(expected_final_state);

        report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
        ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

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

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

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
    Eigen::VectorXs perturbed_bias(origin_bias);

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-6")
    epsilon_bias = 0.000001;
    Eigen::Vector6s err;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-4")
    epsilon_bias = 0.0001;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-2")
    epsilon_bias = 0.01;

    err = Eigen::Vector6s::Random() * epsilon_bias*10;
    perturbed_bias = origin_bias + err;
    origin_KF->setState(x_origin);
    last_KF->setState(expected_final_state);

    report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    //==============================================================
    //WOLF_INFO("Starting error bias 1e-1")
    epsilon_bias = 0.1;

    for(int i = 1; i<10; i++)
    {
        err = Eigen::Vector6s::Random() * epsilon_bias*10;
        perturbed_bias = origin_bias + err;
        origin_KF->setState(x_origin);
        last_KF->setState(expected_final_state);

        report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

        //Only biases are unfixed
        ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
        ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    }
}

TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRotAndVCst, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
{
    //prepare problem for solving
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();

    last_KF->getPPtr()->unfix();
    last_KF->getOPtr()->fix();
    last_KF->getVPtr()->unfix();

    last_KF->setState(expected_final_state);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

}

//TEST_F(ConstraintIMU_biasTest_Move_NonNullBiasRot, VarB1B2V1P2V2_InvarP1Q1Q2_initOK)
//{
//    //prepare problem for solving
//    origin_KF->getPPtr()->fix();
//    origin_KF->getOPtr()->fix();
//    origin_KF->getVPtr()->unfix();
//
//    last_KF->getPPtr()->unfix();
//    last_KF->getOPtr()->fix();
//    last_KF->getVPtr()->unfix();
//
//    last_KF->setState(expected_final_state);
//
//    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
//
//    //Only biases are unfixed
//    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-3)
//    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-3)
//
//}

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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    ceres_manager_wolf_diff->computeCovariances(ALL);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    Eigen::Matrix<wolf::Scalar, 10, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(10,10);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<10; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<10; i++)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)

    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 10, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(10,10);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<10; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<10; i++)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    //ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)

    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.0001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*10000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*10000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 10, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(10,10);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<10; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<10; i++)
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
    CaptureBasePtr capfix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 6, nullptr));
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 10, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(10,10);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<10; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<10; i++)
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
    CaptureBasePtr capfix = origin_KF->addCapture(std::make_shared<CaptureMotion>(0, nullptr, (Eigen::Vector7s() << 0,0,0, 0,0,0,1).finished(), 7, 6, nullptr));
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sen_imu)->getCalibration();
    origin_KF->getCaptureOf(sen_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager_wolf_diff->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    ceres_manager_wolf_diff->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sen_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 10, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(10,10);
        
    //get data from covariance blocks
    wolf_problem_ptr_->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<10; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<10; i++)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sensor_imu)->getCalibration();
    origin_KF->getCaptureOf(sensor_imu)->setCalibration(bias + random_err);

//    WOLF_TRACE("real   bias : ", origin_bias.transpose());
//    WOLF_TRACE("origin bias : ", origin_KF->getCaptureOf(sensor_imu)->getCalibration().transpose());
//    WOLF_TRACE("last   bias : ", last_KF->getCaptureOf(sensor_imu)->getCalibration().transpose());
//    WOLF_TRACE("jacob  bias : ", std::static_pointer_cast<CaptureIMU>(last_KF->getCaptureOf(sensor_imu))->getJacobianCalib().row(0));

    std::string report = ceres_manager->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    std::cout << report << std::endl;
    ceres_manager->computeCovariances(ALL);

    //Only biases are unfixed
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)

    Eigen::Matrix<wolf::Scalar, 10, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(10,10);
        
    //get data from covariance blocks
    problem->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<10; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<10; i++)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sensor_imu)->getCalibration();
    origin_KF->getCaptureOf(sensor_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    ceres_manager->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)

    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 10, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(10,10);
        
    //get data from covariance blocks
    problem->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<10; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<10; i++)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sensor_imu)->getCalibration();
    origin_KF->getCaptureOf(sensor_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)

    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sensor_imu)->getCalibration();
    origin_KF->getCaptureOf(sensor_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sensor_imu)->getCalibration();
    origin_KF->getCaptureOf(sensor_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.0001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sensor_imu)->getCalibration();
    origin_KF->getCaptureOf(sensor_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*10000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*10000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)
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
    Eigen::Vector6s random_err(Eigen::Vector6s::Random() * 0.00001);
    Eigen::Vector6s bias = origin_KF->getCaptureOf(sensor_imu)->getCalibration();
    origin_KF->getCaptureOf(sensor_imu)->setCalibration(bias + random_err);

    std::string report = ceres_manager->solve(1);// 0: nothing, 1: BriefReport, 2: FullReport
    ceres_manager->computeCovariances(ALL);

    ASSERT_MATRIX_APPROX(origin_KF->getVPtr()->getState(), x_origin.segment(7,3), wolf::Constants::EPS*1000)
    ASSERT_MATRIX_APPROX(origin_KF->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    ASSERT_MATRIX_APPROX(last_KF  ->getCaptureOf(sensor_imu)->getCalibration(), origin_bias, 1e-5)
    
    ASSERT_MATRIX_APPROX(last_KF->getPPtr()->getState(), expected_final_state.head(3), wolf::Constants::EPS*100)
    ASSERT_MATRIX_APPROX(last_KF->getVPtr()->getState(), expected_final_state.segment(7,3), wolf::Constants::EPS*1000)
    Eigen::Map<const Eigen::Quaternions> estimatedLastQuat(last_KF->getOPtr()->getState().data()), expectedLastQuat(expected_final_state.segment(3,4).data());
    ASSERT_QUATERNION_APPROX(estimatedLastQuat, expectedLastQuat, wolf::Constants::EPS*100)

    Eigen::Matrix<wolf::Scalar, 10, 1> cov_stdev, actual_state(last_KF->getState());
    Eigen::MatrixXs covX(10,10);
        
    //get data from covariance blocks
    problem->getFrameCovariance(last_KF, covX);

    for(int i = 0; i<10; i++)
        cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)
    
    /*TEST_COUT << "2*std : " << cov_stdev.transpose();
    TEST_COUT << "expect : " << expected_final_state.transpose(); //expected final state
    TEST_COUT << "estim : " << last_KF->getState().transpose(); //estimated final state*/

    for(unsigned int i = 0; i<10; i++)
        assert((expected_final_state(i) <= actual_state(i) + cov_stdev(i)) && (expected_final_state(i) >= actual_state(i) - cov_stdev(i)));

//    if(cov_stdev.tail(6).maxCoeff()>=1)
//        WOLF_WARN("Big 2*stdev on one or more biases! Max coeff :", cov_stdev.tail(6).maxCoeff())
}

//Tests related to noise

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "ConstraintIMU_biasTest.*:ConstraintIMU_biasTest_Move_NonNullBiasRot.*:ConstraintIMU_biasTest_Static_NullBias.*:ConstraintIMU_biasTest_Static_NonNullAccBias.*:ConstraintIMU_biasTest_Static_NonNullGyroBias.*";
//  ::testing::GTEST_FLAG(filter) = "ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRotY.VarB1B2V1V2_InvarP1Q1P2Q2_initOK";
//    ::testing::GTEST_FLAG(filter) = "ConstraintIMU_ODOM_biasTest_Move_NonNullBiasRot.VarB1B2_InvarP1Q1V1P2Q2V2_initOK";
//    ::testing::GTEST_FLAG(filter) = "ConstraintIMU_biasTest_Move_NonNullBiasRot.VarB1B2V1P2V2_InvarP1Q1Q2_initOK";
//  ::testing::GTEST_FLAG(filter) = "ConstraintIMU_biasTest.*";


  return RUN_ALL_TESTS();
}
