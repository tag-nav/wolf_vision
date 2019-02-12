/**
 * \file gtest_processor_imu.cpp
 *
 *  Created on: Nov 23, 2016
 *      \author: jsola
 */

#include "base/capture/capture_IMU.h"
#include "base/processor/processor_IMU.h"
#include "base/sensor/sensor_IMU.h"
#include "base/wolf.h"

#include "utils_gtest.h"
#include "base/logging.h"

#include "base/rotations.h"
#include "base/ceres_wrapper/ceres_manager.h"

#include <cmath>
#include <iostream>

using namespace Eigen;

class ProcessorIMUt : public testing::Test
{

    public: //These can be accessed in fixtures
        wolf::ProblemPtr problem;
        wolf::SensorBasePtr sensor_ptr;
        wolf::TimeStamp t;
        wolf::Scalar mti_clock, tmp;
        wolf::Scalar dt;
        Vector6s data;
        Matrix6s data_cov;
        VectorXs x0;
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
        problem = Problem::create("POV 3D");
        Vector7s extrinsics = (Vector7s() << 0,0,0, 0,0,0,1).finished();
        sensor_ptr = problem->installSensor("IMU", "Main IMU", extrinsics,  wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorBasePtr processor_ptr = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml");

        // Time and data variables
        data = Vector6s::Zero();
        data_cov = Matrix6s::Identity();

        // Set the origin
        x0.resize(10);

        // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
        cap_imu_ptr = make_shared<CaptureIMU>(t, sensor_ptr, data, data_cov, Vector6s::Zero());
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

    //constructor with ProcessorIMUParamsPtr argument only
    ProcessorParamsIMUPtr param_ptr = std::make_shared<ProcessorParamsIMU>();
    param_ptr->max_time_span =   2.0;
    param_ptr->max_buff_length = 20000;
    param_ptr->dist_traveled =   2.0;
    param_ptr->angle_turned =    2.0;

    ProcessorIMUPtr prc1 = std::make_shared<ProcessorIMU>(param_ptr);
    ASSERT_EQ(prc1->getMaxTimeSpan(), param_ptr->max_time_span);
    ASSERT_EQ(prc1->getMaxBuffLength(), param_ptr->max_buff_length);
    ASSERT_EQ(prc1->getDistTraveled(), param_ptr->dist_traveled);
    ASSERT_EQ(prc1->getAngleTurned(), param_ptr->angle_turned);

    //Factory constructor without yaml
    std::string wolf_root = _WOLF_ROOT_DIR;
    ProblemPtr problem = Problem::create("POV 3D");
    Vector7s extrinsics = (Vector7s()<<1,0,0, 0,0,0,1).finished();
    SensorBasePtr sensor_ptr = problem->installSensor("IMU", "Main IMU", extrinsics, wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorBasePtr processor_ptr = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");
    ProcessorParamsIMU params_default;
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan(),   params_default.max_time_span);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxBuffLength(), params_default.max_buff_length);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getDistTraveled(),  params_default.dist_traveled);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getAngleTurned(),   params_default.angle_turned);

    //Factory constructor with yaml
    processor_ptr = problem->installProcessor("IMU", "Sec IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml");
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan(),   2.0);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxBuffLength(), 20000);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getDistTraveled(),  2.0);
    ASSERT_EQ(std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getAngleTurned(),   0.2);
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
    ProblemPtr problem = Problem::create("POV 3D");
    Vector7s extrinsics = (Vector7s()<<1,0,0, 0,0,0,1).finished();
    SensorBasePtr sensor_ptr = problem->installSensor("IMU", "Main IMU", extrinsics,  wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorParamsIMUPtr prc_imu_params = std::make_shared<ProcessorParamsIMU>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;
    prc_imu_params->voting_active = true;
    ProcessorBasePtr processor_ptr = problem->installProcessor("IMU", "IMU pre-integrator", sensor_ptr, prc_imu_params);
    
    //setting origin
    VectorXs x0(10);
    TimeStamp t(0);
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    //data variable and covariance matrix
    // since we integrate only a few times, give the capture a big covariance, otherwise it will be so small that it won't pass following assertions
    Vector6s data;
    data << 1,0,0, 0,0,0;
    Matrix6s data_cov(Matrix6s::Identity());
    data_cov(0,0) = 0.5;

    // Create the captureIMU to store the IMU data arriving from (sensor / callback / file / etc.)
    std::shared_ptr<wolf::CaptureIMU> cap_imu_ptr = make_shared<CaptureIMU>(t, sensor_ptr, data, data_cov, Vector6s::Zero());

    //  Time  
    // we want more than one data to integrate otherwise covariance will be 0
    Scalar dt = std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan() - 0.1;
    t.set(dt);
    cap_imu_ptr->setTimeStamp(t);
    sensor_ptr->process(cap_imu_ptr);

    dt = std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan() + 0.1;
    t.set(dt);
    cap_imu_ptr->setTimeStamp(t);
    sensor_ptr->process(cap_imu_ptr);

    /*There should be 3 frames :
        - 1 KeyFrame at origin
        - 1 keyframe created by process() in voteForKeyFrame() since conditions to create a keyframe are met
        - 1 frame that would be used by future data
    */
    ASSERT_EQ(problem->getTrajectoryPtr()->getFrameList().size(),(unsigned int) 3);

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
TEST_F(ProcessorIMUt, interpolate)
{
    using namespace wolf;

    t.set(0);
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    data << 2, 0, 0, 0, 0, 0; // only acc_x
    cap_imu_ptr->setData(data);

    // make one step to depart from origin
    cap_imu_ptr->setTimeStamp(0.05);
    sensor_ptr->process(cap_imu_ptr);
    Motion mot_ref = problem->getProcessorMotionPtr()->getMotion();

    // make two steps, then pretend it's just one
    cap_imu_ptr->setTimeStamp(0.10);
    sensor_ptr->process(cap_imu_ptr);
    Motion mot_int_gt = problem->getProcessorMotionPtr()->getMotion();

    cap_imu_ptr->setTimeStamp(0.15);
    sensor_ptr->process(cap_imu_ptr);
    Motion mot_final = problem->getProcessorMotionPtr()->getMotion();
    mot_final.delta_ = mot_final.delta_integr_;
    Motion mot_sec = mot_final;

//    problem->getProcessorMotionPtr()->getBuffer().print(0,1,1,0);

class P : public wolf::ProcessorIMU
{
    public:
        P() : ProcessorIMU(std::make_shared<ProcessorParamsIMU>()) {}
        Motion interp(const Motion& ref, Motion& sec, TimeStamp ts)
        {
            return ProcessorIMU::interpolate(ref, sec, ts);
        }
} imu;

Motion mot_int = imu.interp(mot_ref, mot_sec, TimeStamp(0.10));

ASSERT_MATRIX_APPROX(mot_int.data_,  mot_int_gt.data_, 1e-6);
//ASSERT_MATRIX_APPROX(mot_int.delta_, mot_int_gt.delta_, 1e-6); // FIXME: delta_p not correctly interpolated
ASSERT_MATRIX_APPROX(mot_final.delta_integr_,  mot_sec.delta_integr_, 1e-6);

}

TEST_F(ProcessorIMUt, acc_x)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    data << 2, 0, 9.806, 0, 0, 0; // only acc_x, but measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.1);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    VectorXs x(10);
    x << 0.01,0,0, 0,0,0,1, 0.2,0,0; // advanced at a=2m/s2 during 0.1s ==> dx = 0.5*2*0.1^2 = 0.01; dvx = 2*0.1 = 0.2
    Vector6s b; b << 0,0,0, 0,0,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibration() , b, wolf::Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibrationPreint() , b, wolf::Constants::EPS_SMALL);
}

TEST_F(ProcessorIMUt, acc_y)
{
    // last part of this test fails with precision wolf::Constants::EPS_SMALL beacause error is in 1e-12
    // difference hier is that we integrate over 1ms periods

    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    data << 0, 20, 9.806, 0, 0, 0; // only acc_y, but measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    VectorXs x(10);
    x << 0,0.00001,0, 0,0,0,1, 0,0.02,0; // advanced at a=20m/s2 during 0.001s ==> dx = 0.5*20*0.001^2 = 0.00001; dvx = 20*0.001 = 0.02
    Vector6s b; b<< 0,0,0, 0,0,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibration() , b, wolf::Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibrationPreint() , b, wolf::Constants::EPS_SMALL);

    //do so for 5s
    const unsigned int iter = 1000; //how many ms
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        cap_imu_ptr->setTimeStamp(i*0.001 + 0.001); //first one will be 0.002 and last will be 5.000
        sensor_ptr->process(cap_imu_ptr);
    }

    // advanced at a=20m/s2 during 1s ==> dx = 0.5*20*1^2 = 10; dvx = 20*1 = 20
    x << 0,10,0, 0,0,0,1, 0,20,0;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibration() , b, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibrationPreint() , b, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, acc_z)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    data << 0, 0, 9.806 + 2.0, 0, 0, 0; // only acc_z, but measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.1);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    VectorXs x(10);
    x << 0,0,0.01, 0,0,0,1, 0,0,0.2; // advanced at a=2m/s2 during 0.1s ==> dz = 0.5*2*0.1^2 = 0.01; dvz = 2*0.1 = 0.2
    Vector6s b; b<< 0,0,0, 0,0,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibration() , b, wolf::Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibrationPreint() , b, wolf::Constants::EPS_SMALL);

    //do so for 5s
    const unsigned int iter = 50; //how 0.1s 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        cap_imu_ptr->setTimeStamp(i*0.1 + 0.1); //first one will be 0.2 and last will be 5.0
        sensor_ptr->process(cap_imu_ptr);
    }

    // advanced at a=2m/s2 during 5s ==> dz = 0.5*2*5^2 = 25; dvz = 2*5 = 10
    x << 0,0,25, 0,0,0,1, 0,0,10;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibration() , b, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(problem->getProcessorMotionPtr()->getLastPtr()->getCalibrationPreint() , b, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, check_Covariance)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    data << 2, 0, 9.806, 0, 0, 0; // only acc_x, but measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.1);
    sensor_ptr->process(cap_imu_ptr);

    VectorXs delta_preint(problem->getProcessorMotionPtr()->getMotion().delta_integr_);
//    Matrix<wolf::Scalar,9,9> delta_preint_cov = problem->getProcessorMotionPtr()->getCurrentDeltaPreintCov();

    ASSERT_FALSE(delta_preint.isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
//    ASSERT_MATRIX_APPROX(delta_preint_cov, MatrixXs::Zero(9,9), wolf::Constants::EPS_SMALL); // << "delta_preint_cov :\n" << delta_preint_cov; //covariances computed only at keyframe creation
}

TEST_F(ProcessorIMUt, gyro_x)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0, 0, 9.806, rate_of_turn, 0, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0; // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);

    //do so for 5s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity());

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_x_biasedAbx)
{
    // time
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks

    // bias
    wolf::Scalar abx = 0.002;
    Vector6s bias; bias << abx,0,0,  0,0,0;
    Vector3s acc_bias = bias.head(3);
    // state
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();

    // init things
    problem->setPrior(x0, P0, t, 0.01);

    problem->getProcessorMotionPtr()->getOriginPtr()->setCalibration(bias);
    problem->getProcessorMotionPtr()->getLastPtr()->setCalibrationPreint(bias);
//    WOLF_DEBUG("calib: ", cap_imu_ptr->getCalibration().transpose());

    // data
    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << acc_bias - wolf::gravity(), rate_of_turn, 0, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x_true(10);
    x_true << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0; // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180

    VectorXs x_est(10);
    x_est = problem->getCurrentState().head(10);

    ASSERT_MATRIX_APPROX(x_est.transpose() , x_true.transpose(), wolf::Constants::EPS);

    //do so for 5s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity()) + acc_bias;

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    x_true << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x_true, wolf::Constants::EPS);

}

TEST_F(ProcessorIMUt, gyro_xy_biasedAbxy)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    wolf::Scalar abx(0.002), aby(0.01);
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    Vector6s bias; bias << abx,aby,0,  0,0,0;
    Vector3s acc_bias = bias.head(3);

    problem->getProcessorMotionPtr()->getOriginPtr()->setCalibration(bias);
    problem->getProcessorMotionPtr()->getLastPtr()->setCalibrationPreint(bias);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
//    data << 0+abx, 0+aby, 9.806, rate_of_turn, rate_of_turn*1.5, 0; // measure gravity
    data << acc_bias - wolf::gravity(), rate_of_turn*1.5, 0, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0;//, abx,aby,0, 0,0,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);// << "expected state : " << problem->getCurrentState().transpose()
//    << "\n estimated state : " << x.transpose();

    //do so for 5s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity()) + acc_bias;

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);// << "estimated state is : \n" << problem->getCurrentState().transpose() <<
//    "\n expected state is : \n" << x.transpose() << std::endl;
}

TEST_F(ProcessorIMUt, gyro_z)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << -wolf::gravity(), 0, 0, rate_of_turn; // measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0; // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);

    //do so for 5s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 1.000
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 0,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_xyz)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    Vector3s tmp_vec; //will be used to store rate of turn data
    Quaternions quat_comp(Quaternions::Identity());
    Matrix3s R0(Matrix3s::Identity());
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
        Quaternions rot(problem->getCurrentState().data()+3);
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
    Quaternions R2quat(wolf::v2q(wolf::R2v(R0)));
    Vector4s quat_comp_vec((Vector4s() <<quat_comp.x(), quat_comp.y(), quat_comp.z(), quat_comp.w()).finished() );
    Vector4s R2quat_vec((Vector4s() <<R2quat.x(), R2quat.y(), R2quat.z(), R2quat.w()).finished() );

    ASSERT_MATRIX_APPROX(quat_comp_vec , R2quat_vec, wolf::Constants::EPS);// << "quat_comp_vec : " << quat_comp_vec.transpose() << "\n R2quat_vec : " << R2quat_vec.transpose() << std::endl;

    VectorXs x(10);
    x << 0,0,0, quat_comp.x(), quat_comp.y(), quat_comp.z(), quat_comp.w(), 0,0,0;

    Quaternions result_quat(problem->getCurrentState().data() + 3);
    //std::cout << "final orientation : " << wolf::q2v(result_quat).transpose() << std::endl;

    //check position part
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(3) , x.head(3), wolf::Constants::EPS);

    //check orientation part
    ASSERT_MATRIX_APPROX(problem->getCurrentState().segment(3,4) , x.segment(3,4) , wolf::Constants::EPS);

    //check velocity and bias parts
    ASSERT_MATRIX_APPROX(problem->getCurrentState().segment(7,3) , x.segment(7,3), wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_z_ConstVelocity)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  2,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << -wolf::gravity(), 0, 0, rate_of_turn; // measure gravity!

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    x << 0.002,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 2,0,0; // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180, 2m/s * 0.001s = 0.002m

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);

    //do so for 1s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 5.000
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 2,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 2,0,0; //2m/s * 1s = 2m
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_x_ConstVelocity)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  2,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0, 0, 9.806, rate_of_turn, 0, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180, 2m/s * 0.001s = 0.002
    x << 0.002,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 2,0,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);

    //do so for 1s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity());

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 2,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 2,0,0; //2m/s * 1s = 2m
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_xy_ConstVelocity)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  2,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0, 0, 9.806, 0, rate_of_turn, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180, 2m/s * 0.001s = 0.002
    x << 0.002,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 2,0,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);

    //do so for 1s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity());

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 2,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 2,0,0; //2m/s * 1s = 2m
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_y_ConstVelocity)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  2,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0, 0, 9.806, 0, rate_of_turn, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180, 2m/s * 0.001s = 0.002
    x << 0.002,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 2,0,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS_SMALL);

    //do so for 1s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 1; i < iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity());

        cap_imu_ptr->setTimeStamp(i*dt + dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    x << 2,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 2,0,0; //2m/s * 1s = 2m
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_xyz_ConstVelocity)
{
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  2,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    Vector3s tmp_vec; //will be used to store rate of turn data
    Quaternions quat_comp(Quaternions::Identity());
    Matrix3s R0(Matrix3s::Identity());
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

    for(unsigned int data_iter = 0; data_iter < 1000; data_iter ++)
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
        Quaternions rot(problem->getCurrentState().data()+3);
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
    Quaternions R2quat(wolf::v2q(wolf::R2v(R0)));
    Vector4s quat_comp_vec((Vector4s() <<quat_comp.x(), quat_comp.y(), quat_comp.z(), quat_comp.w()).finished() );
    Vector4s R2quat_vec((Vector4s() <<R2quat.x(), R2quat.y(), R2quat.z(), R2quat.w()).finished() );

    ASSERT_MATRIX_APPROX(quat_comp_vec , R2quat_vec, wolf::Constants::EPS); // << "quat_comp_vec : " << quat_comp_vec.transpose() << "\n R2quat_vec : " << R2quat_vec.transpose() << std::endl;

    VectorXs x(10);
    //rotation + translation due to initial velocity
    x << 2,0,0, quat_comp.x(), quat_comp.y(), quat_comp.z(), quat_comp.w(), 2,0,0;

    Quaternions result_quat(problem->getCurrentState().data() + 3);
    //std::cout << "final orientation : " << wolf::q2v(result_quat).transpose() << std::endl;

    //check position part
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(3) , x.head(3), wolf::Constants::EPS);

    //check orientation part
    ASSERT_MATRIX_APPROX(problem->getCurrentState().segment(3,4) , x.segment(3,4) , wolf::Constants::EPS);

    //check velocity
    ASSERT_MATRIX_APPROX(problem->getCurrentState().segment(7,3) , x.segment(7,3), wolf::Constants::EPS);

}

TEST_F(ProcessorIMUt, gyro_x_acc_x)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 1, 0, -wolf::gravity()(2), rate_of_turn, 0, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180 on x axis
    // translation with constant acc : 1 m/s^2 for 0.001 second. Initial velocity : 0, p = 0.5*a*dt^2 + v*dt = 0.5*1*0.001^2 = 0.0000005 on x axis
    // v = a*dt = 0.001
    x << 0.0000005,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0.001,0,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS); // << "1. current state is : \n" << problem->getCurrentState().transpose() <<
//    "\n current x is : \n" << x.transpose() << std::endl;

    //do so for 1s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 2; i <= iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity()) + (Vector3s()<<1,0,0).finished();

        cap_imu_ptr->setTimeStamp(i*dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    // translation with constant acc : 1 m/s for 1 second. Initial velocity : 0, p = 0.5*a*dt + v*dt = 0.5 on x axis
    // v = a*dt = 1
    x << 0.5,0,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 1,0,0;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_y_acc_y)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0, 1, -wolf::gravity()(2), 0, rate_of_turn, 0; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180 on y axis
    // translation with constant acc : 1 m/s^2 for 0.001 second. Initial velocity : 0, p = 0.5*a*dt^2 + v*dt = 0.5*1*0.001^2 = 0.0000005 on y axis
    // v = a*dt = 0.001
    x << 0,0.0000005,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0.001,0;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS); // << "1. current state is : \n" << problem->getCurrentState().transpose() <<
//    "\n current x is : \n" << x.transpose() << std::endl;

    //do so for 1s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 2; i <= iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity()) + (Vector3s()<<0,1,0).finished();

        cap_imu_ptr->setTimeStamp(i*dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    // translation with constant acc : 1 m/s for 1 second. Initial velocity : 0, p = 0.5*a*dt + v*dt = 0.5 on y axis
    // v = a*dt = 1
    x << 0,0.5,0, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,1,0;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

TEST_F(ProcessorIMUt, gyro_z_acc_z)
{
    wolf::Scalar dt(0.001);
    t.set(0); // clock in 0,1 ms ticks
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    MatrixXs P0(9,9); P0.setIdentity();
    problem->setPrior(x0, P0, t, 0.01);

    wolf::Scalar rate_of_turn = 5 * M_PI/180.0;
    data << 0, 0, -wolf::gravity()(2) + 1, 0, 0, rate_of_turn; // measure gravity

    cap_imu_ptr->setData(data);
    cap_imu_ptr->setTimeStamp(0.001);
    sensor_ptr->process(cap_imu_ptr);

    // Expected state after one integration
    Quaternions quat_comp(Quaternions::Identity());
    quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

    VectorXs x(10);
    // rotated at 5 deg/s for 0.001s = 0.005 deg => 0.005 * M_PI/180 on z axis
    // translation with constant acc : 1 m/s^2 for 0.001 second. Initial velocity : 0, p = 0.5*a*dt^2 + v*dt = 0.5*1*0.001^2 = 0.0000005 on z axis
    // v = a*dt = 0.001
    x << 0,0,0.0000005, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,0.001;

    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);

    //do so for 1s
    const unsigned int iter = 1000; //how many ms 
    for(unsigned int i = 2; i <= iter; i++) //already did one integration above
    {
        // quaternion composition
        quat_comp = quat_comp * wolf::v2q(data.tail(3)*dt);

        Quaternions rot(problem->getCurrentState().data()+3);
        data.head(3) =  rot.conjugate() * (- wolf::gravity()) + (Vector3s()<<0,0,1).finished();

        cap_imu_ptr->setTimeStamp(i*dt); //first one will be 0.002 and last will be 1.000
        cap_imu_ptr->setData(data);
        sensor_ptr->process(cap_imu_ptr);
    }

    // translation with constant acc : 1 m/s for 1 second. Initial velocity : 0, p = 0.5*a*dt + v*dt = 0.5 on z axis
    // v = a*dt = 1
    x << 0,0,0.5, quat_comp.x(),quat_comp.y(),quat_comp.z(),quat_comp.w(), 0,0,1;
    ASSERT_MATRIX_APPROX(problem->getCurrentState().head(10) , x, wolf::Constants::EPS);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  //::testing::GTEST_FLAG(filter) = "ProcessorIMUt.check_Covariance";
  return RUN_ALL_TESTS();
}

