//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

//#define DEBUG_RESULTS

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

            //std::string dummy; //this is needed only first line is headers
            //getline(data_file, dummy);
        }

        if(!data_file.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            return 1;
        }
        
    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PVQBB_3D);
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
    x0 << 0,0,0,  0,0,0,  0,0,0,1,  0,0,.001,  0,0,.002; // Try some non-zero biases
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    CaptureIMU::Ptr imu_ptr( std::make_shared<CaptureIMU>(t, sensor_ptr, data_) );

    // main loop
    using namespace std;
    clock_t begin = clock();
    std::cout << "\n\t\t\t\tENTERING MAIN LOOP - Please press ENTER to exit loop\n" << std::endl;

    while(!data_file.eof()){
        // read new data
        data_file >> mpu_clock >> data_[0] >> data_[1] >> data_[2] >> data_[3] >> data_[4] >> data_[5];
        t.set(mpu_clock); //

        // assign data to capture
        imu_ptr->setData(data_);
        imu_ptr->setTimeStamp(t);

        // process data in capture
        sensor_ptr->process(imu_ptr);
    }

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