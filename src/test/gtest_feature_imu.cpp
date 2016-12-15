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

//#define DEBUG_RESULTS


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
        Eigen::Matrix<wolf::Scalar,9,6> dD_db_jacobians;

    //a new of this will be created for each new test
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

        t.set(0);

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

    //create FrameIMU
        ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
        state_vec = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState();
   	    last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, state_vec);
        wolf_problem_ptr_->getTrajectoryPtr()->addFrame(last_frame);
        

    //create a feature
        delta_preint_cov = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentDeltaPreintCov();
        delta_preint = wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_;
        //feat_imu = std::make_shared<FeatureIMU>(delta_preint, delta_preint_cov);
        std::static_pointer_cast<wolf::ProcessorIMU>(wolf_problem_ptr_->getProcessorMotionPtr())->getJacobians(dD_db_jacobians);
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

    ASSERT_TRUE((origin_pptr->getVector() - left_pptr->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
    ASSERT_TRUE((origin_optr->getVector() - left_optr->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));
    ASSERT_TRUE((origin_vptr->getVector() - left_vptr->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL));

    ASSERT_EQ(origin_frame->id(), left_frame->id());
}

TEST_F(FeatureIMU_test, access_members)
{
    using namespace wolf;

    Eigen::VectorXs delta(10);
    //dx = 0.5*2*0.1^2 = 0.01; dvx = 2*0.1 = 0.2; dz = 0.5*9.8*0.1^2 = 0.049; dvz = 9.8*0.1 = 0.98
    delta << 0.01,0,0.049, 0,0,0,1, 0.2,0,0.98;
    ASSERT_TRUE((feat_imu->dp_preint_ - delta.head<3>()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL)) << "feat_imu->dp_preint_ : " << feat_imu->dp_preint_.transpose() << ",\t delta.head<3>() :" << delta.head<3>().transpose() << 
    ",\n delta_preint : " << delta_preint.transpose() << std::endl;
    EXPECT_TRUE((feat_imu->dv_preint_ - delta.tail<3>()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL)) << "feat_imu->dv_preint_ : " << feat_imu->dv_preint_.transpose() << ",\t delta.tail<3>() :" << delta.tail<3>().transpose() << 
    ",\n (feat_imu->dv_preint_ - delta.tail<3>() : " << (feat_imu->dv_preint_ - delta.tail<3>()).transpose() << std::endl;
    ASSERT_TRUE((feat_imu->dv_preint_ - delta.tail<3>()).isMuchSmallerThan(1, wolf::Constants::EPS)) << "feat_imu->dv_preint_ : " << feat_imu->dv_preint_.transpose() << ",\t delta.tail<3>() :" << delta.tail<3>().transpose() << 
    ",\n delta_preint : " << delta_preint.transpose() << std::endl;

    Eigen::Vector4s quat;
    quat << feat_imu->dq_preint_.x(), feat_imu->dq_preint_.y(), feat_imu->dq_preint_.z(), feat_imu->dq_preint_.w();
    ASSERT_TRUE((quat - delta.segment<4>(3)).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL)) << "feat_imu->dq_preint_ : " << quat.transpose() << ",\t delta.segment<4>(3) :" << delta.segment<4>(3).transpose() << 
    ",\n delta_preint : " << delta_preint.transpose() << std::endl;
}

TEST_F(FeatureIMU_test, addConstraint)
{
    using namespace wolf;

    feat_imu->getFramePtr()->getTimeStamp();
    last_frame->getTimeStamp();
    
    ConstraintIMUPtr constraint_imu = std::make_shared<ConstraintIMU>(feat_imu, last_frame);
    feat_imu->addConstraint(constraint_imu);
    last_frame->addConstrainedBy(constraint_imu);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
