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

TEST(ProcessorIMU_constraints, KF_c_KF)
{
    //having a look at how optimizer reacts for a simple case like:
    //   C_o     C_o is a constraintOdom3D
    //  /   \    c is a constraintIMU
    //  *-c-*    * are keyframes
    // The keyframe at origin is fixed and the other one will be estimated given constraints

    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

                            /*********************************** INITIALISATION ***********************************/                         
    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs IMU_extrinsics(7);
    IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, shared_ptr<IntrinsicsBase>());
    wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                            /*********************************** SETTING PROBLEM ***********************************/ 
    // Time and data variables
    Eigen::Vector6s data_;
    Scalar mpu_clock = 0;
    TimeStamp t(mpu_clock);

    // Set the origin
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0;
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix(); //fix the keyframe at origin

    TimeStamp ts(0);
    Eigen::VectorXs origin_state = x0;
    
    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    CaptureIMUPtr imu_ptr( std::make_shared<CaptureIMU>(t, sensor_ptr, data_, Eigen::Matrix6s::Identity()) );
    imu_ptr->setFramePtr(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back());

    Eigen::VectorXs state_vec;
    Eigen::VectorXs delta_preint;
    //FrameIMUPtr last_frame;
    Eigen::Matrix<wolf::Scalar,9,9> delta_preint_cov;

    //process data
    //We can interpret this first data as a noisy and biased data from an IMU that is not moving
    mpu_clock = 0.001000;
    data_ << 0.1256, -0.143701, 9.939331, 0.127445, 0.187814, -0.055003; //imu data AccX, AccY, AccZ, GyrX, GyrY, GyrZ
    t.set(mpu_clock);
    // assign data to capture
    imu_ptr->setData(data_);
    imu_ptr->setTimeStamp(t);
    // process data in capture
    sensor_ptr->process(imu_ptr);

    mpu_clock = 0.002000;
    data_ << 0.1256, -0.143701, 9.939331, 0.127445, 0.187814, -0.055003; //imu data AccX, AccY, AccZ, GyrX, GyrY, GyrZ
    t.set(mpu_clock);
    // assign data to capture
    imu_ptr->setData(data_);
    imu_ptr->setTimeStamp(t);
    // process data in capture
    sensor_ptr->process(imu_ptr);

    /// constraint creation
    //create FrameIMU
    ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
    state_vec = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState();
    FrameIMUPtr last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, state_vec);
    wolf_problem_ptr_->getTrajectoryPtr()->addFrame(last_frame);

        //create a feature
    delta_preint_cov = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentDeltaPreintCov();
    delta_preint = wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_;
    std::shared_ptr<FeatureIMU> feat_imu = std::make_shared<FeatureIMU>(delta_preint, delta_preint_cov);
    feat_imu->setCapturePtr(imu_ptr);

        //create a constraintIMU
    ConstraintIMUPtr constraint_imu = std::make_shared<ConstraintIMU>(feat_imu, last_frame);
    feat_imu->addConstraint(constraint_imu);
    last_frame->addConstrainedBy(constraint_imu);

    FrameIMUPtr ref_frame_ptr(std::static_pointer_cast<FrameIMU>(imu_ptr->getFramePtr()));

    Eigen::Matrix<wolf::Scalar, 10, 1> expect;
    Eigen::Vector3s ref_frame_p = ref_frame_ptr->getPPtr()->getVector();
    Eigen::Quaternions ref_frame_o(ref_frame_ptr->getOPtr()->getVector().data());
    Eigen::Vector3s ref_frame_v = ref_frame_ptr->getVPtr()->getVector();
    Eigen::Vector3s current_frame_p = last_frame->getPPtr()->getVector();
    Eigen::Quaternions current_frame_o(last_frame->getOPtr()->getVector().data());
    Eigen::Vector3s current_frame_v = last_frame->getVPtr()->getVector();
    Eigen::Vector3s acc_bias(ref_frame_ptr->getAccBiasPtr()->getVector()), gyro_bias(ref_frame_ptr->getGyroBiasPtr()->getVector());
    Eigen::Matrix<wolf::Scalar, 9, 1> residu;
    residu << 0,0,0,  0,0,0,  0,0,0;
    
    constraint_imu->expectation(ref_frame_p, ref_frame_o, ref_frame_v, acc_bias, gyro_bias, current_frame_p, current_frame_o, current_frame_v, expect);
    //std::cout << "expectation : " << expect.transpose() << std::endl;
    constraint_imu->getResiduals(ref_frame_p, ref_frame_o, ref_frame_v, acc_bias, gyro_bias, current_frame_p, current_frame_o, current_frame_v,residu);
    //std::cout << "residuals : " << residu.transpose() << std::endl;

    	                                      /*               CREATE CONSTRAINT ODOM 3D               */
    //create a feature
    FeatureBasePtr last_feature = std::make_shared<FeatureBase>("ODOM_3D", origin_state.head(7),Eigen::Matrix7s::Identity()); //first KF and last KF at same position
    last_feature->setCapturePtr(imu_ptr);

    //create an ODOM constraint between first and last keyframes
    ConstraintOdom3DPtr constraintOdom_ptr = std::make_shared<ConstraintOdom3D>(last_feature, last_frame);
    last_feature -> addConstraint(constraintOdom_ptr);
    last_frame -> addConstrainedBy(constraintOdom_ptr);

    Eigen::Vector7s odom_expec;
    odom_expec  = constraintOdom_ptr -> expectation();

                            /*********************************** CALL TO OPTIMIZATION MODULE ***********************************/
    // COMPUTE COVARIANCES
    std::cout << "computing covariances..." << std::endl;
    ceres_manager->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS
    std::cout << "computed!" << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

