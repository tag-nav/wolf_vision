//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "constraint_odom_3D.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

#define DEBUG_RESULTS

int _kbhit();

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::cout << std::endl << "==================== test_imu_constraintAHP ======================" << std::endl;

    /*load files containing accelerometer and data
    structure is : Timestampt\t Ax\t Ay\t Az\t Wx\t Wy\t Wz
    */
    
    std::ifstream data_file;
    const char * filename;
    if (argc < 2)
        {
            std::cout << "Missing input argument! : needs 1 argument (path to data file)." << std::endl;
            return 1;
        }
        else
        {
            filename = argv[1];
            data_file.open(filename);
            std::cout << "file: " << filename << std::endl;

            std::string dummy; //this is needed only if first line is headers
            getline(data_file, dummy);
        }

        if(!data_file.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            return 1;
        }
        
    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs IMU_extrinsics(7);
    IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, shared_ptr<IntrinsicsBase>());
    wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

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
    wolf::FrameIMUPtr origin_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, origin_state);
    wolf_problem_ptr_->getTrajectoryPtr()->addFrame(origin_frame);
    
    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    CaptureIMUPtr imu_ptr( std::make_shared<CaptureIMU>(t, sensor_ptr, data_, origin_frame) );

    // main loop
    using namespace std;
    clock_t begin = clock();
    const int keyframe_spacing = 10;
    int last_keyframe_dt = 0;
    Eigen::VectorXs state_vec;
    Eigen::VectorXs delta_preint;
    FrameIMUPtr last_frame;
    //FrameIMUPtr previous_frame;
    Eigen::Matrix<wolf::Scalar,9,9> delta_preint_cov;
    Eigen::Matrix<wolf::Scalar,9,6> dD_db;
    int iteration = 0;

    //needed to retrieve jacobians wrt biases
    wolf::ProcessorIMUPtr proc_imu = std::static_pointer_cast<ProcessorIMU>(wolf_problem_ptr_->getProcessorMotionPtr());
    
    while(!data_file.eof() && iteration<100){
        //std::cout << "last_keyframe_dt :  " << last_keyframe_dt << std::endl;
        if(last_keyframe_dt >= keyframe_spacing){
            //previous_frame = std::static_pointer_cast<FrameIMU>(imu_ptr->getFramePtr()); //to constraint the new frame and link it to previous one
            ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
            state_vec = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState();
            last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, state_vec);
            //FrameBasePtr last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts_.get(),std::make_shared<StateBlock>(frame_val.head(3)), std::make_shared<StateQuaternion>(frame_val.tail(4)));
            wolf_problem_ptr_->getTrajectoryPtr()->addFrame(last_frame);

            //create a feature
            delta_preint_cov = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentDeltaPreintCov();
            delta_preint = wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_;
            proc_imu -> getJacobians(dD_db);
            std::shared_ptr<FeatureIMU> feat_imu = std::make_shared<FeatureIMU>(delta_preint, delta_preint_cov, imu_ptr, dD_db);

            //create a constraintIMU
            //wolf_problem_ptr_->getProcessorMotionPtr()->emplaceConstraint(feat_imu, previous_frame);
            ConstraintIMUPtr constraint_imu = std::make_shared<ConstraintIMU>(feat_imu, last_frame);
            feat_imu->addConstraint(constraint_imu);
            last_frame->addConstrainedBy(constraint_imu);

            //reset origin of motion to new frame
            wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(last_frame);
            last_keyframe_dt = 0;
        }
        else
            last_keyframe_dt++;

        // read new data
        data_file >> mpu_clock >> data_[0] >> data_[1] >> data_[2] >> data_[3] >> data_[4] >> data_[5];
        t.set(mpu_clock); //

        // assign data to capture
        imu_ptr->setData(data_);
        imu_ptr->setTimeStamp(t);

        // process data in capture
        sensor_ptr->process(imu_ptr);
        std::cout << "iteration : " << iteration << std::endl;
        iteration++;
    }

    //first close the file no longer needed
    data_file.close();

    //make final a keyframe
    ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
    state_vec = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState();
    last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, state_vec);
    wolf_problem_ptr_->getTrajectoryPtr()->addFrame(last_frame);

    //create a feature
    FeatureBasePtr last_feature = std::make_shared<FeatureBase>("ODOM_3D", origin_state.head(7),Eigen::Matrix7s::Identity()); //first KF and last KF at same position
    last_feature->setCapturePtr(imu_ptr);

    //create an ODOM constraint between first and last keyframes
    ConstraintOdom3DPtr constraintOdom_ptr = std::make_shared<ConstraintOdom3D>(last_feature, last_frame);
    last_feature -> addConstraint(constraintOdom_ptr);
    last_frame -> addConstrainedBy(constraintOdom_ptr);

    Eigen::Vector7s expec;
    expec  = constraintOdom_ptr -> expectation();

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // Final state
    std::cout << "\nIntegration results ----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Initial    state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << x0.head(16).transpose() << std::endl;
    std::cout << "Integrated delta: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_.transpose() << std::endl;
    std::cout << "Integrated state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState().head(16).transpose() << std::endl;
    std::cout << "Integrated std  : " << std::fixed << std::setprecision(3) << std::setw(8)
    << (wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_cov_.diagonal().transpose()).array().sqrt() << std::endl;


    // Print statistics
    std::cout << "\nStatistics -----------------------------------------------------------------------------------" << std::endl;
    std::cout << "If you want meaningful CPU metrics, remove all couts in the loop / remove DEBUG_RESULTS definition variable, and compile in RELEASE mode!" << std::endl;

    TimeStamp t0, tf;
    t0 = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().front().ts_;
    tf = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
    int N = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().size();
    std::cout << "t0        : " << t0.get() << " s" << std::endl;
    std::cout << "tf        : " << tf.get() << " s" << std::endl;
    std::cout << "duration  : " << tf-t0 << " s" << std::endl;
    std::cout << "N samples : " << N << std::endl;
    std::cout << "frequency : " << (N-1)/(tf-t0) << " Hz" << std::endl;
    std::cout << "CPU time  : " << elapsed_secs << " s" << std::endl;
    std::cout << "s/integr  : " << elapsed_secs/(N-1)*1e6 << " us" << std::endl;
    std::cout << "integr/s  : " << (N-1)/elapsed_secs << " ips" << std::endl;

    return 0;

}

int _kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}