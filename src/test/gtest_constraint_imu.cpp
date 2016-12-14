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
        wolf::FrameBasePtr origin_frame;
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

TEST_F(ConstraintIMU_testBase, constructors)
{   
    using namespace wolf;

    //create FrameIMU
    ts = 0.1;
    state_vec << 0.01,0,0, 0,0,0,1, 0.2,0,0, 0,0,0, 0,0,0;
   	last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, state_vec);
    
    //create a feature
    delta_preint_cov = Eigen::MatrixXs::Random(9,9);
    delta_preint << 0.01,0,0.049, 0,0,0,1, 0.2,0,0.98;
    dD_db_jacobians = Eigen::Matrix<wolf::Scalar,9,6>::Random();
    feat_imu = std::make_shared<FeatureIMU>(delta_preint, delta_preint_cov, imu_ptr, dD_db_jacobians);

    //create the constraint
    ConstraintIMU constraint_imu(feat_imu,last_frame);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
