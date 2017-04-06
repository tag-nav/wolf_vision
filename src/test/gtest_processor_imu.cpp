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
#include "rotations.h"
#include <cmath>
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
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;
    prc_imu_params->voting_active = true;
    ProcessorBasePtr processor_ptr = problem->installProcessor("IMU", "IMU pre-integrator", sensor_ptr, prc_imu_params);
    
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
    // last part of this test fails with precision wolf::Constants::EPS_SMALL beacause error is in 1e-12
    // difference hier is that we integrate over 1ms periods

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
    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS)) << "current state is " << problem->getCurrentState().transpose() << "\n x is : " << x.transpose() << std::endl;
}

TEST_F(ProcessorIMUt, acc_xyz)
{
    /* Here again the error seems to be in the discretization.
     * integrating over 10s with a dt of 0.001s lead to an error in velocity of order 1e-3. 
     * The smaller is the time step the more precise is the integration
     * Same conclusion for position
     */

    Eigen::Vector3s tmp_a_vec; //will be used to store IMU acceleration data
    Eigen::Vector3s w_vec((Eigen::Vector3s()<<0,0,0).finished());
    wolf::Scalar time = 0;
    const wolf::Scalar x_freq = 2;
    const wolf::Scalar y_freq = 1;
    const wolf::Scalar z_freq = 4;

    wolf::Scalar tmp_ax, tmp_ay, tmp_az;
    /*
        From evolution of position we determine the acceleration : 
        x = sin(x_freq*t);
        y = sin(y_freq*t);
        z = sin(z_freq*t);

        corresponding acceleration
        ax = - (x_freq * x_freq) * sin(x_freq * t);
        ay = - (y_freq * y_freq) * sin(y_freq * t);
        az = - (z_freq * z_freq) * sin(z_freq * t);

        Notice that in this case, initial velocity is given exactly as :
        initial_vx = x_freq;
        initial_vy = y_freq;
        initial_vz = z_freq;
     */

    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  x_freq,y_freq,z_freq,  0,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    const wolf::Scalar dt = 0.0001;
    
    // This test is for pure translation -> no rotation ==> constant rate of turn vector = [0,0,0]
    data.tail(3) = w_vec;

    for(unsigned int data_iter = 0; data_iter <= 10000; data_iter ++)
    {   
        tmp_ax = - (x_freq * x_freq) * sin(x_freq * time);
        tmp_ay = - (y_freq * y_freq) * sin(y_freq * time);
        tmp_az = - (z_freq * z_freq) * sin(z_freq * time);
        tmp_a_vec << tmp_ax, tmp_ay, tmp_az;

        Eigen::Quaternions rot(problem->getCurrentState().data()+3); //should always be identity quaternion here...
        data.head(3) =  tmp_a_vec + rot.conjugate() * (- wolf::gravity()); //gravity measured

        cap_imu_ptr->setData(data);
        cap_imu_ptr->setTimeStamp(time);
        sensor_ptr->process(cap_imu_ptr);

        time += dt;
    }
    time -= dt; //to get final time

    /* We should not have turned : final quaternion must be identity quaternion [0,0,0,1]
     * We integrated over 1 s : 
     * x = sin(x_freq * 1)
     * y = sin(y_freq * 1)
     * z = sin(z_freq * 1)

     * Velocity is given by first derivative :
     * vx = x_freq * cos(x_freq * 1)
     * vy = y_freq * cos(y_freq * 1)
     * vz = z_freq * cos(z_freq * 1)
     */

    Eigen::VectorXs x(16);
    wolf::Scalar exp_px = sin(x_freq * time);
    wolf::Scalar exp_py = sin(y_freq * time);
    wolf::Scalar exp_pz = sin(z_freq * time);

    wolf::Scalar exp_vx = x_freq * cos(x_freq * time);
    wolf::Scalar exp_vy = y_freq * cos(y_freq * time);
    wolf::Scalar exp_vz = z_freq * cos(z_freq * time);

    x << exp_px,exp_py,exp_pz, 0,0,0,1, exp_vx,exp_vy,exp_vz, 0,0,0, 0,0,0;

    //check velocity and bias parts
    ASSERT_TRUE((problem->getCurrentState().tail(9) - x.tail(9)).isMuchSmallerThan(1, 0.001)) << "current VBB is : \n" << problem->getCurrentState().tail(9).transpose() <<
    "\n expected is : \n" << x.tail(9).transpose() << std::endl;

    //check orientation part
    ASSERT_TRUE((problem->getCurrentState().segment(3,4) - x.segment(3,4)).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10)) << "current orientation is : \n" << problem->getCurrentState().segment(3,4).transpose() <<
    "\n expected is : \n" << x.segment(3,4).transpose() << std::endl;

    //check position part
    ASSERT_TRUE((problem->getCurrentState().head(3) - x.head(3)).isMuchSmallerThan(1, 0.001)) << "current position is : \n" << problem->getCurrentState().head(3).transpose() <<
    "\n expected is : \n" << x.head(3).transpose() << std::endl;

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

TEST_F(ProcessorIMUt, gyro_x)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0, 0, 9.8, rate_of_turn, 0, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Eigen::Quaternions quat_comp(Eigen::Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    Eigen::VectorXs x(16);
    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0, 0,0,0, 0,0,0; // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180

    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    //do so for 5s
    const unsigned int iter = 5000; //how many ms 
    for(int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Eigen::Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity());

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 5.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0, 0,0,0, 0,0,0;
    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS)) << "current state is : \n" << problem->getCurrentState().transpose() <<
    "\n current x is : \n" << x.transpose() << std::endl;
}

TEST_F(ProcessorIMUt, gyro_x_biasedAbx)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    wolf::Scalar abx(0.002);
    Eigen::Vector3s acc_bias((Eigen::Vector3s()<<abx,0,0).finished());
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  abx,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0+abx, 0, 9.8, rate_of_turn, 0, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Eigen::Quaternions quat_comp(Eigen::Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    Eigen::VectorXs x(16);
    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0, abx,0,0, 0,0,0; // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180

    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL)) << "expected state : " << problem->getCurrentState().transpose() 
    << "\n estimated state : " << x.transpose();

    //do so for 5s
    const unsigned int iter = 5000; //how many ms 
    for(int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Eigen::Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity()) + acc_bias;

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 5.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0, abx,0,0, 0,0,0;
    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS)) << "estimated state is : \n" << problem->getCurrentState().transpose() <<
    "\n expected state is : \n" << x.transpose() << std::endl;
}

TEST_F(ProcessorIMUt, gyro_z)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0, 0, 9.8, 0, 0, rate_of_turn; // measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Eigen::Quaternions quat_comp(Eigen::Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    Eigen::VectorXs x(16);
    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0, 0,0,0, 0,0,0; // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180

    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    //do so for 5s
    const unsigned int iter = 5000; //how many ms 
    for(int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 5.000
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0, 0,0,0, 0,0,0;
    ASSERT_TRUE((problem->getCurrentState() - x).isMuchSmallerThan(1, wolf::Constants::EPS)) << "current state is : \n" << problem->getCurrentState().transpose() <<
    "\n current x is : \n" << x.transpose() << std::endl;
}


TEST_F(ProcessorIMUt, gyro_xyz)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases

    problem->getProcessorMotionPtr()->setOrigin(x0, t);

    Eigen::Vector3s tmp_vec; //will be used to store rate of turn data
    Eigen::Quaternions quat_comp(Eigen::Quaternions::Identity());
    Eigen::Matrix3s R0(Eigen::Matrix3s::Identity());
    wolf::Scalar time = 0;
    const unsigned int x_rot_vel = 5;
    const unsigned int y_rot_vel = 6;
    const unsigned int z_rot_vel = 13;

    wolf::Scalar tmpx, tmpy, tmpz;
    
    /*
        ox oy oz evolution in degrees (for understanding) --> converted in rad
        with * pi/180
        ox = pi*sin(alpha*t*pi/180); %express angle in rad before using sinus
        oy = pi*sin(beta*t*pi/180);
        oz = pi*sin(gamma*t*pi/180);

        corresponding rate of turn
        %rate of turn expressed in radians/s
        wx = pi*alpha*cos(alpha*t*pi/180)*pi/180;
        wy = pi*beta*cos(beta*t*pi/180)*pi/180;
        wz = pi*gamma*cos(gamma*t*pi/180)*pi/180;
     */

     const wolf::Scalar dt = 0.001;

    for(unsigned int data_iter = 0; data_iter <= 1000; data_iter ++)
    {   
        time += dt;

        tmpx = M_PI*x_rot_vel*cos((M_PI/180) * x_rot_vel * time)*M_PI/180;
        tmpy = M_PI*y_rot_vel*cos((M_PI/180) * y_rot_vel * time)*M_PI/180;
        tmpz = M_PI*z_rot_vel*cos((M_PI/180) * z_rot_vel * time)*M_PI/180;
        tmp_vec << tmpx, tmpy, tmpz;

        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(tmp_vec*dt);
        R0 = R0 * wolf::v2R(tmp_vec*dt);
        // use processorIMU
        Eigen::Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity()); //gravity measured
        data.tail(3) = tmp_vec;

        cap_imu_ptr->setData(data);
        cap_imu_ptr->setTimeStamp(time);
        sensor_ptr->process(cap_imu_ptr);
    }

    /* We focus on orientation here. position is supposed not to have moved
     * we integrated using 2 ways : 
        - a direct quaternion composition q = q (x) q(w*dt)
        - using methods implemented in processorIMU

        We check one against the other
     */

     // validating that the quaternion composition and rotation matrix composition actually describe the same rotation.
    Eigen::Quaternions R2quat(wolf::v2q(wolf::R2v(R0)));
    Eigen::Vector4s quat_comp_vec((Eigen::Vector4s() <<quat_comp.x(), quat_comp.y(), quat_comp.z(), quat_comp.w()).finished() );
    Eigen::Vector4s R2quat_vec((Eigen::Vector4s() <<R2quat.x(), R2quat.y(), R2quat.z(), R2quat.w()).finished() );

    ASSERT_TRUE((quat_comp_vec - R2quat_vec).isMuchSmallerThan(1, wolf::Constants::EPS)) << "quat_comp_vec : " << quat_comp_vec.transpose() << "\n R2quat_vec : " << R2quat_vec.transpose() << std::endl;

    Eigen::VectorXs x(16);
    x << 0,0,0, quat_comp.x(), quat_comp.y(), quat_comp.z(), quat_comp.w(), 0,0,0, 0,0,0, 0,0,0;

    Eigen::Quaternions result_quat(problem->getCurrentState().data() + 3);
    //std::cout << "final orientation : " << wolf::q2v(result_quat).transpose() << std::endl;

    //check position part
    ASSERT_TRUE((problem->getCurrentState().head(3) - x.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS)) << "current position is : \n" << problem->getCurrentState().head(3).transpose() <<
    "\n expected is : \n" << x.head(3).transpose() << std::endl;

    //check velocity and bias parts
    ASSERT_TRUE((problem->getCurrentState().tail(9) - x.tail(9)).isMuchSmallerThan(1, wolf::Constants::EPS)) << "current VBB is : \n" << problem->getCurrentState().tail(9).transpose() <<
    "\n expected is : \n" << x.tail(9).transpose() << std::endl;

    //check orientation part
    EXPECT_TRUE((problem->getCurrentState().segment(3,4) - x.segment(3,4) ).isMuchSmallerThan(1, wolf::Constants::EPS)) << "current orientation is : \n" << problem->getCurrentState().segment(3,4).transpose() <<
    "\n expected is : \n" << x.segment(3,4).transpose() << std::endl;
    // expect above fails, look for the actual precision that works
    ASSERT_TRUE((problem->getCurrentState().segment(3,4) - x.segment(3,4) ).isMuchSmallerThan(1, 0.001)) << "current orientation is : \n" << problem->getCurrentState().segment(3,4).transpose() <<
    "\n expected is : \n" << x.segment(3,4).transpose() << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "ProcessorIMUt.gyro_x_biasedAbx";
  return RUN_ALL_TESTS();
}

