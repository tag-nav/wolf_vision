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

#include <iostream>

class ProcessorIMU : public testing::Test
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

        // Wolf problem
        problem = Problem::create(FRM_PQVBB_3D);
        Vector7s extrinsics = (Vector7s()<<1,0,0, 0,0,0,1).finished();
        sensor_ptr = problem->installSensor("IMU", "Main IMU", extrinsics, shared_ptr<IntrinsicsBase>());
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

/*using namespace wolf;
using namespace Eigen;
using std::shared_ptr;
using std::make_shared;
using std::static_pointer_cast;
using namespace wolf::Constants;


// Wolf problem
ProblemPtr problem = Problem::create(FRM_PQVBB_3D);
Vector7s extrinsics = (Vector7s()<<1,0,0, 0,0,0,1).finished();
SensorBasePtr    sensor_ptr     = problem->installSensor("IMU", "Main IMU", extrinsics, IntrinsicsBasePtr());
ProcessorBasePtr processor_ptr  = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

// Time and data variables
TimeStamp t;
Scalar mti_clock, tmp;
Vector6s data = Vector6s::Zero();
Matrix6s data_cov = Matrix6s::Identity();


// Set the origin
VectorXs x0(16);

// Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
shared_ptr<CaptureIMU> cap_imu_ptr = make_shared<CaptureIMU>(t, sensor_ptr, data);
CaptureIMUPtr cap_imu_ptr = make_shared<CaptureIMU>(t, sensor_ptr, data);
*/

//replace TEST by TEST_F
TEST_F(ProcessorIMU, acc_x)
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

TEST_F(ProcessorIMU, acc_y)
{
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
}

TEST_F(ProcessorIMU, acc_z)
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
}

TEST_F(ProcessorIMU, check_Covariance)
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

TEST_F(ProcessorIMU, Covariances)
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

