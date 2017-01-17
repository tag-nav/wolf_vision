//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "constraint_odom_3D.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"
#include "capture_fix.h"
#include "ceres_wrapper/ceres_manager.h"

//#define DEBUG_RESULTS

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::cout << std::endl << "==================== test_constraint_imu ======================" << std::endl;

    bool c0(false), c1(false);// c2(true), c3(true), c4(true);
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
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

    // Time and data variables
    TimeStamp t;
    Eigen::Vector6s data_;
    Scalar mpu_clock = 0;

    t.set(mpu_clock);

    // Set the origin
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.001,  0,0,.002; // Try some non-zero biases
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    //create a keyframe at origin
    TimeStamp ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
    Eigen::VectorXs origin_state = x0;
    FrameIMUPtr origin_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, origin_state);
    wolf_problem_ptr_->getTrajectoryPtr()->addFrame(origin_frame);
    
    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    CaptureIMUPtr imu_ptr( std::make_shared<CaptureIMU>(t, sensor_ptr, data_, Eigen::Matrix6s::Identity()) );
    imu_ptr->setFramePtr(origin_frame);


    // set variables
    using namespace std;
    Eigen::VectorXs state_vec;
    Eigen::VectorXs delta_preint;
    //FrameIMUPtr last_frame;
    Eigen::Matrix<wolf::Scalar,9,9> delta_preint_cov;

    //process data
    mpu_clock = 0.001003;
    data_ << 0.579595, -0.143701, 9.939331, 0.127445, 0.187814, -0.055003;
    t.set(mpu_clock);
    // assign data to capture
    imu_ptr->setData(data_);
    imu_ptr->setTimeStamp(t);
    // process data in capture
    sensor_ptr->process(imu_ptr);

    if(c0){
    /// ******************************************************************************************** ///
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
    std::cout << "expectation : " << expect.transpose() << std::endl;

    constraint_imu->getResiduals(ref_frame_p, ref_frame_o, ref_frame_v, acc_bias, gyro_bias, current_frame_p, current_frame_o, current_frame_v,residu);
    std::cout << "residuals : " << residu.transpose() << std::endl;

    //reset origin of motion to new frame
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(last_frame);
    imu_ptr->setFramePtr(last_frame);
    }
    /// ******************************************************************************************** ///

    mpu_clock = 0.002135;
    data_ << 0.581990, -0.191602, 10.071057, 0.136836, 0.203912, -0.057686;
    t.set(mpu_clock);
    imu_ptr->setData(data_);
    imu_ptr->setTimeStamp(t);
    sensor_ptr->process(imu_ptr);

    mpu_clock = 0.003040;
    data_ << 0.596360, -0.225132, 10.205178, 0.154276, 0.174399, -0.036221;
    t.set(mpu_clock);
    imu_ptr->setData(data_);
    imu_ptr->setTimeStamp(t);
    sensor_ptr->process(imu_ptr);

    if(c1){
    /// ******************************************************************************************** ///
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
    std::cout << "expectation : " << expect.transpose() << std::endl;

    constraint_imu->getResiduals(ref_frame_p, ref_frame_o, ref_frame_v, acc_bias, gyro_bias, current_frame_p, current_frame_o, current_frame_v,residu);
    std::cout << "residuals : " << residu.transpose() << std::endl;

    //reset origin of motion to new frame
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(last_frame);
    imu_ptr->setFramePtr(last_frame);
    }

    mpu_clock = 0.004046;
    data_ << 0.553250, -0.203577, 10.324929, 0.128787, 0.156959, -0.044270;
    t.set(mpu_clock);
    imu_ptr->setData(data_);
    imu_ptr->setTimeStamp(t);
    sensor_ptr->process(imu_ptr);

    mpu_clock = 0.005045;
    data_ << 0.548459, -0.184417, 10.387200, 0.083175, 0.120738, -0.026831;
    t.set(mpu_clock);
    imu_ptr->setData(data_);
    imu_ptr->setTimeStamp(t);
    sensor_ptr->process(imu_ptr);

    //create the constraint
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
    std::cout << "expectation : " << expect.transpose() << std::endl;

    constraint_imu->getResiduals(ref_frame_p, ref_frame_o, ref_frame_v, acc_bias, gyro_bias, current_frame_p, current_frame_o, current_frame_v,residu);
    std::cout << "residuals : " << residu.transpose() << std::endl;

    ///having a look at covariances
    Eigen::MatrixXs predelta_cov;
    predelta_cov.resize(9,9);
    predelta_cov = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentDeltaPreintCov();
    std::cout << "predelta_cov : \n" << predelta_cov << std::endl; 

        ///Optimization
    // PRIOR
    //FrameBasePtr first_frame = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front();
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    //SensorBasePtr sensorbase = std::make_shared<SensorBase>("ABSOLUTE POSE", nullptr, nullptr, nullptr, 0);
    //CaptureFixPtr initial_covariance = std::make_shared<CaptureFix>(TimeStamp(0), sensorbase, first_frame->getState().head(7), Eigen::Matrix6s::Identity() * 0.01);
    //first_frame->addCapture(initial_covariance);
    //initial_covariance->process();
    //std::cout << "initial covariance: constraint " << initial_covariance->getFeatureList().front()->getConstrainedByList().front()->id() << std::endl << initial_covariance->getFeatureList().front()->getMeasurementCovariance() << std::endl;

    // COMPUTE COVARIANCES
    std::cout << "computing covariances..." << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS
    std::cout << "computed!" << std::endl;

    /*
    // SOLVING PROBLEMS
    ceres::Solver::Summary summary_diff;
    std::cout << "solving..." << std::endl;
    Eigen::VectorXs prev_wolf_state = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getState();
    summary_diff = ceres_manager_wolf_diff->solve();
    Eigen::VectorXs post_wolf_state = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getState();
    std::cout << " prev_wolf_state : " << prev_wolf_state.transpose() << "\n post_wolf_state : " << post_wolf_state.transpose() << std::endl;
    //std::cout << summary_wolf_diff.BriefReport() << std::endl;
    std::cout << "solved!" << std::endl;
    */

    /*
    std::cout << "WOLF AUTO DIFF" << std::endl;
    std::cout << "Jacobian evaluation: " << summary_wolf.jacobian_evaluation_time_in_seconds << std::endl;
    std::cout << "Total time: " << summary_wolf.total_time_in_seconds << std::endl;
    */

    return 0;
}