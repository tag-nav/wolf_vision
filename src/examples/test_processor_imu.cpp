/**
 * \file test_processor_imu.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: dtsbourg
 */

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


int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor IMU test ======================" << std::endl;

    std::ifstream data_file_acc;
    std::ifstream data_file_gyro;
    //load files containing accelerometer and gyroscope data
    const char * filename_acc;
    const char * filename_gyro;
    if (argc < 3)
    {
        std::cout << "missing input argument : needs 2 arguments (path to accelerometer file and path to gyroscope data)." << std::endl;
    }
    else
    {
        filename_acc = argv[1];
        filename_gyro = argv[2];
        data_file_acc.open(filename_acc);
        data_file_gyro.open(filename_gyro);
        std::cout << "Acc  file: " << filename_acc << std::endl;
        std::cout << "Gyro file: " << filename_gyro << std::endl;

        std::string dummy;
        getline(data_file_acc, dummy); getline(data_file_gyro, dummy);

        if(!data_file_acc.is_open() || !data_file_gyro.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            return 1;
        }
    }

    TimeStamp t;
    float mti_clock, tmp;
    Eigen::Vector6s data_;

    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PQVBB_3D);
    Eigen::VectorXs extrinsics(7);
    extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBase* sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", extrinsics, nullptr);
    wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");
    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    // Set the origin
    data_file_acc >> mti_clock >> data_[0] >> data_[1] >> data_[2];
    data_file_gyro >> tmp >> data_[3] >> data_[4] >> data_[5];
    t.set(mti_clock * 0.0001); // clock in 0,1 ms ticks
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0, 0,0,0, 0,0,0;

    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    CaptureIMU* imu_ptr;


    using namespace std;
    clock_t begin = clock();
    while(!data_file_acc.eof()){
        data_file_acc >> mti_clock >> data_[0] >> data_[1] >> data_[2];
        data_file_gyro >> tmp >> data_[3] >> data_[4] >> data_[5];

        t.set(mti_clock * 0.0001); // clock in 0,1 ms ticks

        imu_ptr = new CaptureIMU(t, sensor_ptr, data_);

        imu_ptr ->process();


//        std::cout << "\nCurrent    delta: " << std::fixed << std::setprecision(3) << std::setw(8) << std::right
//        << wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_.transpose();
//        std::cout << "\nIntegrated delta: " << std::fixed << std::setprecision(3) << std::setw(8)
//        << wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_.transpose();
//        std::cout << "\nIntegrated state: " << std::fixed << std::setprecision(3) << std::setw(8)
//        << wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState().transpose();
//        std::cout << std::endl;

    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // Print statistics
    std::cout << std::endl;
    TimeStamp t0, tf;
    t0 = wolf_problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().front().ts_;
    tf = wolf_problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().back().ts_;
    int N = wolf_problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().size();
    std::cout << "t0        : " << t0.get() << " s" << std::endl;
    std::cout << "tf        : " << tf.get() << " s" << std::endl;
    std::cout << "duration  : " << tf-t0 << " s" << std::endl;
    std::cout << "N samples : " << N << std::endl;
    std::cout << "frequency : " << (N-1)/(tf-t0) << " Hz" << std::endl;
    std::cout << "CPU       : " << elapsed_secs << " s" << std::endl;
    std::cout << "integr/s  : " << (N-1)/elapsed_secs << " ips" << std::endl;



    delete wolf_problem_ptr_;

    return 0;
}
