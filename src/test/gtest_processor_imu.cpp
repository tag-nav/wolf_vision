/**
 * \file gtest_processor_imu.cpp
 *
 *  Created on: Nov 23, 2016
 *      \author: jsola
 */



#include "utils_gtest.h"
#include "../src/logging.h"

#include "../sensor_imu.h"
#include "../processor_imu.h"
#include "../capture_imu.h"
#include "ceres_wrapper/ceres_manager.h"
#include "constraint_odom_3D.h"

#include <iostream>

class ProcessorIMUt : public testing::Test
{

    public: //These can be accessed in fixtures
        wolf::ProblemPtr problem;
        wolf::SensorBasePtr sensor_ptr;
        wolf::TimeStamp t;
        wolf::Scalar mti_clock, tmp;
        Eigen::Vector6s data;
        Eigen::Matrix6s data_cov;
        Eigen::VectorXs x0;
        std::shared_ptr<wolf::CaptureIMU> cap_imu_ptr;

    //a new of this will be created for each new test
    virtual void SetUp()
    {
        using namespace wolf;
        using namespace Eigen;
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;
        using namespace wolf::Constants;

        std::string wolf_root = _WOLF_ROOT_DIR;

        // Wolf problem
        problem = Problem::create(FRM_PQVBB_3D);
        Vector7s extrinsics = (Vector7s()<<1,0,0, 0,0,0,1).finished();
        sensor_ptr = problem->installSensor("IMU", "Main IMU", extrinsics,  wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorBasePtr processor_ptr = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

        // Time and data variables
        data = Vector6s::Zero();
        data_cov = Matrix6s::Identity();

        // Set the origin
        x0.resize(16);

        // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
        cap_imu_ptr = make_shared<CaptureIMU>(t, sensor_ptr, data);
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


TEST(ProcessorIMU_constructors, ALL)
{
    using namespace wolf;

    //constructor without any argument
    ProcessorIMUPtr prc0 = std::make_shared<ProcessorIMU>();
    ASSERT_EQ(prc0->getMaxTimeSpan(), 1.0);
    ASSERT_EQ(prc0->getMaxBuffLength(), 10000);
    ASSERT_EQ(prc0->getDistTraveled(), 1.0);
    ASSERT_EQ(prc0->getAngleTurned(), 0.2);

    //constructor with ProcessorIMUParamsPtr argument only
    ProcessorIMUParamsPtr param_ptr = std::make_shared<ProcessorIMUParams>();
    param_ptr->max_time_span = 2.0;
    param_ptr->max_buff_length = 20000;
    param_ptr->dist_traveled = 2.0;
    param_ptr->angle_turned = 2.0;

    ProcessorIMUPtr prc1 = std::make_shared<ProcessorIMU>(param_ptr);
    ASSERT_EQ(prc1->getMaxTimeSpan(), param_ptr->max_time_span);
    ASSERT_EQ(prc1->getMaxBuffLength(), param_ptr->max_buff_length);
    ASSERT_EQ(prc1->getDistTraveled(), param_ptr->dist_traveled);
    ASSERT_EQ(prc1->getAngleTurned(), param_ptr->angle_turned);

    //Factory constructor without yaml
    std::string wolf_root = _WOLF_ROOT_DIR;
    ProblemPtr problem = Problem::create(FRM_PQVBB_3D);
    Vector7s extrinsics = (Vector7s()<<1,0,0, 0,0,0,1).finished();
    SensorBasePtr sensor_ptr = problem->installSensor("IMU", "Main IMU", extrinsics, wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorBasePtr processor_ptr = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan(), 1.0);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxBuffLength(), 10000);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getDistTraveled(), 1.0);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getAngleTurned(), 0.2);

    //Factory constructor with yaml
    processor_ptr = problem->installProcessor("IMU", "Sec IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml");
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan(), 2.0);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxBuffLength(), 20000);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getDistTraveled(), 2.0);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getAngleTurned(), 0.2);
}

TEST(ProcessorIMU, voteForKeyFrame)
{
    using namespace wolf;
    using namespace Eigen;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    using namespace wolf::Constants;

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr problem = Problem::create(FRM_PQVBB_3D);
    Eigen::Vector7s extrinsics = (Vector7s()<<1,0,0, 0,0,0,1).finished();
    SensorBasePtr sensor_ptr = problem->installSensor("IMU", "Main IMU", extrinsics,  wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorBasePtr processor_ptr = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml");
    
    //setting origin
    Eigen::VectorXs x0(16);
    TimeStamp t(0);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.000,  0,0,.000; // Try some non-zero biases
    problem->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin

    // Time and data variables
    Scalar dt = std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan() + 0.1;
    Eigen::Vector6s data;
    data << 1,0,0, 0,0,0;
    Eigen::Matrix6s data_cov(Matrix6s::Zero());
    data_cov(0,0) = 0.5;
    t.set(dt);

    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    std::shared_ptr<wolf::CaptureIMU> cap_imu_ptr = make_shared<CaptureIMU>(t, sensor_ptr, data);
    sensor_ptr->process(cap_imu_ptr);

    /*There should be 3 frames :
        - 1 KeyFrame at origin
        - 1 keyframe created by process() in voteForKeyFrame() since conditions to create a keyframe are met
        - 1 frame that would be used by future data
    */
    ASSERT_EQ(problem->getTrajectoryPtr()->getFrameList().size(),3);

    /* if max_time_span == 2,  Wolf tree should be

    Hardware
        S1
            pm5
            o: C2 - F2
            l: C4 - F3
        Trajectory
        KF1
            Estim, ts=0,	 x = ( 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0)
            C1 -> S1
        KF2
            Estim, ts=2.1,	 x = ( 2.2     0       -22     0       0       0       1       2.1     0       -21     0       0       0       0       0       0      )
            C2 -> S1
                f1
                    m = ( 2.21 0   0   0   0   0   1   2.1 0   0  )
                    c1 --> F1
        F3
            Estim, ts=2.1,	 x = ( . . .)
            C4 -> S1
    */
    //TODO : ASSERT TESTS to make sure the constraints are correctly set + check the tree above

}

//replace TEST by TEST_F if SetUp() needed
TEST_F(ProcessorIMUt, acc_x)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    data << 2, 0, 9.8, 0, 0, 0; // only acc_x, but measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.1);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Eigen::VectorXs x(16);
    x << 0.01,0,0, 0,0,0,1, 0.2,0,0, 0,0,0, 0,0,0; // advanced at a=2m/s2 during 0.1s ==> dx = 0.5*2*0.1^2 = 0.01; dvx = 2*0.1 = 0.2

    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
}

TEST_F(ProcessorIMUt, acc_y)
{
    //last part of this test fails with precision wolf::Constants::EPS_SMALL beacause error is in 1e-12

    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    data << 0, 20, 9.8, 0, 0, 0; // only acc_y, but measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Eigen::VectorXs x(16);
    x << 0,0.00001,0, 0,0,0,1, 0,0.02,0, 0,0,0, 0,0,0; // advanced at a=20m/s2 during 0.001s ==> dx = 0.5*20*0.001^2 = 0.00001; dvx = 20*0.001 = 0.02

    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    //do so for 5s
    const unsigned int iter = 5000; //how many ms 
    for(int i = 1; i < iter; i++) //already did one integration above
    {
        cap_imu_ptr->setTimeStamp(i*0.001 + 0.001); //first one will be 0.002 and last will be 5.000
        sensor_ptr->process(cap_imu_ptr);
    }

    // advanced at a=20m/s2 during 5s ==> dx = 0.5*20*5^2 = 250; dvx = 20*5 = 100
    x << 0,250,0, 0,0,0,1, 0,100,0, 0,0,0, 0,0,0;
    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS)) << "current state is " << problem->getCurrentState().transpose() << std::endl;
}

TEST_F(ProcessorIMUt, acc_z)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    data << 0, 0, 9.8 + 2.0, 0, 0, 0; // only acc_z, but measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.1);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Eigen::VectorXs x(16);
    x << 0,0,0.01, 0,0,0,1, 0,0,0.2, 0,0,0, 0,0,0; // advanced at a=2m/s2 during 0.1s ==> dz = 0.5*2*0.1^2 = 0.01; dvz = 2*0.1 = 0.2

    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    //do so for 5s
    const unsigned int iter = 50; //how 0.1s 
    for(int i = 1; i < iter; i++) //already did one integration above
    {
        cap_imu_ptr->setTimeStamp(i*0.1 + 0.1); //first one will be 0.2 and last will be 5.0
        sensor_ptr->process(cap_imu_ptr);
    }

    // advanced at a=2m/s2 during 5s ==> dz = 0.5*2*5^2 = 25; dvz = 2*5 = 10
    x << 0,0,25, 0,0,0,1, 0,0,10, 0,0,0, 0,0,0;
    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL)) << "current state is " << problem->getCurrentState().transpose() << std::endl;
}

TEST_F(ProcessorIMUt, check_Covariance)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    data << 2, 0, 9.8, 0, 0, 0; // only acc_x, but measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.1);
    sensor_ptr->process(cap_imu_ptr);

    Eigen::VectorXs delta_preint(problem->getProcessorMotionPtr()->getMotion().delta_integr_);
    Eigen::Matrix<wolf::Scalar,9,9> delta_preint_cov = problem->getProcessorMotionPtr()->getCurrentDeltaPreintCov();

    ASSERT_FALSE(delta_preint.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
    ASSERT_FALSE(delta_preint_cov.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
}

TEST_F(ProcessorIMUt, Covariances)
{
    data_cov.topLeftCorner(3,3)     *= 0.01; // acc variance
    data_cov.bottomRightCorner(3,3) *= 0.01; // gyro variance
    cap_imu_ptr->setDataCovariance(data_cov);

}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

