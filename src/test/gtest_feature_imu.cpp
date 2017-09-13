//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"

#include "utils_gtest.h"
#include "../src/logging.h"

class FeatureIMU_test : public testing::Test
{

    public: //These can be accessed in fixtures
        wolf::ProblemPtr wolf_problem_ptr_;
        wolf::TimeStamp ts;
        wolf::CaptureIMUPtr imu_ptr;
        Eigen::VectorXs state_vec;
        Eigen::VectorXs delta_preint;
        Eigen::Matrix<wolf::Scalar,9,9> delta_preint_cov;
        std::shared_ptr<wolf::FeatureIMU> feat_imu;
        wolf::FrameIMUPtr last_frame;
        wolf::FrameBasePtr origin_frame;
        Eigen::MatrixXs dD_db_jacobians;
        wolf::ProcessorBasePtr processor_ptr_;

    //a new of this will be created for each new test
    virtual void SetUp()
    {
        using namespace wolf;
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        // Wolf problem
        wolf_problem_ptr_ = Problem::create("PQVBB 3D");
        Eigen::VectorXs IMU_extrinsics(7);
        IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
        IntrinsicsIMUPtr sen_imu_params = std::make_shared<IntrinsicsIMU>();
        SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, sen_imu_params);
        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Time and data variables
        TimeStamp t;
        Eigen::Vector6s data_;

        t.set(0);

    // Set the origin
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases
        origin_frame = wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);  //create a keyframe at origin
    
    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    // give the capture a big covariance, otherwise it will be so small that it won't pass following assertions
        imu_ptr = std::make_shared<CaptureIMU>(t, sensor_ptr, data_, Eigen::Matrix6s::Identity()); 
        imu_ptr->setFramePtr(origin_frame); //to get ptr to Frm ni processorIMU and then get biases

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

    //create FrameIMU
        ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
        state_vec = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState();
   	    last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, state_vec);
        wolf_problem_ptr_->getTrajectoryPtr()->addFrame(last_frame);
        
    //create a feature
        delta_preint = wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_;
        delta_preint_cov = wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_cov_;
        //feat_imu = std::make_shared<FeatureIMU>(delta_preint, delta_preint_cov);
        std::static_pointer_cast<wolf::ProcessorIMU>(wolf_problem_ptr_->getProcessorMotionPtr())->getJacobianCalib(dD_db_jacobians);
        feat_imu = std::make_shared<FeatureIMU>(delta_preint, delta_preint_cov, imu_ptr, dD_db_jacobians);
        feat_imu->setCapturePtr(imu_ptr); //associate the feature to a capture

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

TEST_F(FeatureIMU_test, check_frame)
{
    // set variables
    using namespace wolf;

    FrameBasePtr left_frame = feat_imu->getFramePtr();
    wolf::TimeStamp t;
    left_frame->getTimeStamp(t);
    origin_frame->getTimeStamp(ts);

    ASSERT_EQ(t,ts) << "t != ts \t t=" << t << "\t ts=" << ts << std::endl;
    ASSERT_TRUE(origin_frame->isKey());
    ASSERT_TRUE(left_frame->isKey());

    wolf::StateBlockPtr origin_pptr, origin_optr, origin_vptr, left_pptr, left_optr, left_vptr;
    origin_pptr = origin_frame->getPPtr();
    origin_optr = origin_frame->getOPtr();
    origin_vptr = origin_frame->getVPtr();
    left_pptr = left_frame->getPPtr();
    left_optr = left_frame->getOPtr();
    left_vptr = left_frame->getVPtr();

    ASSERT_MATRIX_APPROX(origin_pptr->getState(), left_pptr->getState(), wolf::Constants::EPS_SMALL);
    Eigen::Map<const Eigen::Quaternions> origin_Quat(origin_optr->getState().data()), left_Quat(left_optr->getState().data());
    ASSERT_QUATERNION_APPROX(origin_Quat, left_Quat, wolf::Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(origin_vptr->getState(), left_vptr->getState(), wolf::Constants::EPS_SMALL);

    ASSERT_EQ(origin_frame->id(), left_frame->id());
}

TEST_F(FeatureIMU_test, access_members)
{
    using namespace wolf;

    Eigen::VectorXs delta(10);
    //dx = 0.5*2*0.1^2 = 0.01; dvx = 2*0.1 = 0.2; dz = 0.5*9.8*0.1^2 = 0.049; dvz = 9.8*0.1 = 0.98
    delta << 0.01,0,0.049, 0,0,0,1, 0.2,0,0.98;
    ASSERT_MATRIX_APPROX(feat_imu->dp_preint_, delta.head<3>(), wolf::Constants::EPS_SMALL);
    ASSERT_MATRIX_APPROX(feat_imu->dv_preint_, delta.tail<3>(), wolf::Constants::EPS);
    EXPECT_MATRIX_APPROX(feat_imu->dv_preint_, delta.tail<3>(), wolf::Constants::EPS_SMALL*10)

    Eigen::Map<const Eigen::Quaternions> delta_quat(delta.segment<4>(3).data());
    ASSERT_QUATERNION_APPROX(feat_imu->dq_preint_, delta_quat, wolf::Constants::EPS_SMALL);
}

TEST_F(FeatureIMU_test, addConstraint)
{
    using namespace wolf;
    
    FrameIMUPtr frm_imu = std::static_pointer_cast<FrameIMU>(last_frame);
    ConstraintIMUPtr constraint_imu = std::make_shared<ConstraintIMU>(feat_imu, std::static_pointer_cast<FrameIMU>(frm_imu), processor_ptr_);
    feat_imu->addConstraint(constraint_imu);
    origin_frame->addConstrainedBy(constraint_imu);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
