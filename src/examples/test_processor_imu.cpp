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
#include <cmath>

//#define DEBUG_RESULTS


int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::cout << std::endl << "==================== processor IMU test ======================" << std::endl;

    //load files containing accelerometer and gyroscope data
    std::ifstream data_file_acc;
    std::ifstream data_file_gyro;
    const char * filename_acc;
    const char * filename_gyro;

    //prepare creation of file if DEBUG_RESULTS activated
#ifdef DEBUG_RESULTS
    std::ofstream debug_results;
    debug_results.open("debug_results.dat");
    if(debug_results)
        debug_results << "%%TimeStamp\t"
                      << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                      << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                      << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
#endif

    if (argc < 3)
    {
        std::cout << "Missing input argument! : needs 2 arguments (path to accelerometer file and path to gyroscope data)." << std::endl;
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

    // Wolf problem
    ProblemPtr problem_ptr_ = make_shared<Problem>(FRM_PVQBB_3D);
    problem_ptr_->setup();
    Eigen::VectorXs extrinsics(7);
    extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBasePtr sensor_ptr = problem_ptr_->installSensor("IMU", "Main IMU", extrinsics, shared_ptr<IntrinsicsBase>());
    problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Time and data variables
    TimeStamp t;
    Scalar mti_clock, tmp;
    Eigen::Vector6s data_;

    // Get initial data
    data_file_acc >> mti_clock >> data_[0] >> data_[1] >> data_[2];
    data_file_gyro >> tmp >> data_[3] >> data_[4] >> data_[5];
    t.set(mti_clock * 0.0001); // clock in 0,1 ms ticks

    // Set the origin
    Eigen::VectorXs x0(16);
    x0 << 0,1,0,  1,0,0,  0,0,0,1,  0,0,.001,  0,0,.002; // Try some non-zero biases
    problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    shared_ptr<CaptureIMU> imu_ptr = make_shared<CaptureIMU>(t, sensor_ptr, data_);

    problem_ptr_->print();

    std::cout << "Main loop -----------" << std::endl;

    // main loop
    using namespace std;
    clock_t begin = clock();
    while(!data_file_acc.eof()){

        // read new data
        data_file_acc >> mti_clock >> data_[0] >> data_[1] >> data_[2];
        data_file_gyro >> tmp >> data_[3] >> data_[4] >> data_[5];
        t.set(mti_clock * 0.0001); // clock in 0,1 ms ticks

        // assign data to capture
        imu_ptr->setData(data_);
        imu_ptr->setTimeStamp(t);

        // process data in capture
        imu_ptr ->process();

#ifdef DEBUG_RESULTS

        Eigen::VectorXs delta_debug;
        Eigen::VectorXs delta_integr_debug;
        Eigen::VectorXs x_debug;
        TimeStamp ts;

        // std::cout << "Current    delta: " << std::fixed << std::setprecision(3) << std::setw(8) << std::right
        // << wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_.transpose() << std::endl;

        // std::cout << "Integrated delta: " << std::fixed << std::setprecision(3) << std::setw(8)
        // << wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_.transpose() << std::endl;

        // Eigen::VectorXs x = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState();
        // std::cout << "Integrated state: " << std::fixed << std::setprecision(3) << std::setw(8)
        // << x.head(10).transpose() << std::endl;

        // std::cout << std::endl;
        delta_debug = problem_ptr_->getProcessorMotionPtr()->getMotion().delta_;
        delta_integr_debug = problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_;
        x_debug = problem_ptr_->getProcessorMotionPtr()->getCurrentState();
        ts = problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().back().ts_;

        if(debug_results)
            debug_results << ts.get() << "\t" << delta_debug(0) << "\t" << delta_debug(1) << "\t" << delta_debug(2) << "\t" << delta_debug(3) << "\t" << delta_debug(4) << "\t"
            << delta_debug(5) << "\t" << delta_debug(6) << "\t" << delta_debug(7) << "\t" << delta_debug(8) << "\t" << delta_debug(9) << "\t"
            << delta_integr_debug(0) << "\t" << delta_integr_debug(1) << "\t" << delta_integr_debug(2) << "\t" << delta_integr_debug(3) << "\t" << delta_integr_debug(4) << "\t"
            << delta_integr_debug(5) << "\t" << delta_integr_debug(6) << "\t" << delta_integr_debug(7) << "\t" << delta_integr_debug(8) << "\t" << delta_integr_debug(9) << "\t"
            << x_debug(0) << "\t" << x_debug(1) << "\t" << x_debug(2) << "\t" << x_debug(3) << "\t" << x_debug(4) << "\t"
            << x_debug(5) << "\t" << x_debug(6) << "\t" << x_debug(7) << "\t" << x_debug(8) << "\t" << x_debug(9) << "\n";
#endif

    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // Final state
    std::cout << "\nIntegration results ----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Initial    state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << x0.head(16).transpose() << std::endl;
    std::cout << "Integrated delta: " << std::fixed << std::setprecision(3) << std::setw(8)
    << problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_.transpose() << std::endl;
    std::cout << "Integrated state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << problem_ptr_->getProcessorMotionPtr()->getCurrentState().head(16).transpose() << std::endl;
    std::cout << "Integrated std  : " << std::fixed << std::setprecision(3) << std::setw(8)
    << (problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_cov_.diagonal().transpose()).array().sqrt() << std::endl;


    // Print statistics
    std::cout << "\nStatistics -----------------------------------------------------------------------------------" << std::endl;
    std::cout << "If you want meaningful CPU metrics, remove all couts in the loop / remove DEBUG_RESULTS definition variable, and compile in RELEASE mode!" << std::endl;

#ifdef DEBUG_RESULTS
    std::cout << "\t\tWARNING : DEBUG_RESULTS ACTIVATED - slows the process (writing results to result_debugs.dat file)" << std::endl;
    debug_results.close();
#endif

    TimeStamp t0, tf;
    t0 = problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().front().ts_;
    tf = problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().back().ts_;
    int N = problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().size();
    std::cout << "t0        : " << t0.get() << " s" << std::endl;
    std::cout << "tf        : " << tf.get() << " s" << std::endl;
    std::cout << "duration  : " << tf-t0 << " s" << std::endl;
    std::cout << "N samples : " << N << std::endl;
    std::cout << "frequency : " << (N-1)/(tf-t0) << " Hz" << std::endl;
    std::cout << "CPU time  : " << elapsed_secs << " s" << std::endl;
    std::cout << "s/integr  : " << elapsed_secs/(N-1)*1e6 << " us" << std::endl;
    std::cout << "integr/s  : " << (N-1)/elapsed_secs << " ips" << std::endl;

    problem_ptr_->print();

    problem_ptr_.reset();

    return 0;
}
