/**
 * \file test_mpu.cpp
 *
 *  Created on: Oct 4, 2016
 *      \author: AtDinesh
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

    //prepare MPU here
    if (argc < 2)
    {
        std::cout << "Missing input argument! : needs 1 argument : way to MPU device. (usually /dev/ttyACM#)\n 
        Please make sure that you have rights to access the device and that your user belongs to the dialout group." << std::endl;
    }

    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PVQBB_3D);
    Eigen::VectorXs IMU_extrinsics(7);
    IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBase* sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, nullptr);
    wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Time and data variables
    TimeStamp t;
    Eigen::Vector6s data_;
    Scalar mpu_clock, tmp;

    t.set(mpu_clock * 0.0001); // clock in 0,1 ms ticks

    // Set the origin
    Eigen::VectorXs x0(16);
    x0 << 0,1,0,  1,0,0,  0,0,0,1,  0,0,.001,  0,0,.002; // Try some non-zero biases
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    CaptureIMU* imu_ptr( new CaptureIMU(t, sensor_ptr, data_) );

    // main loop
    using namespace std;
    clock_t begin = clock();

    while(!_kbhit()){
        // read new data

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

        delta_debug = wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_;
        delta_integr_debug = wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_;
        x_debug = wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState();
        ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().back().ts_;

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
    << wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_.transpose() << std::endl;
    std::cout << "Integrated state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotionPtr()->getCurrentState().head(16).transpose() << std::endl;
    std::cout << "Integrated std  : " << std::fixed << std::setprecision(3) << std::setw(8)
    << (wolf_problem_ptr_->getProcessorMotionPtr()->getMotion().delta_integr_cov_.diagonal().transpose()).array().sqrt() << std::endl;


    // Print statistics
    std::cout << "\nStatistics -----------------------------------------------------------------------------------" << std::endl;
    std::cout << "If you want meaningful CPU metrics, remove all couts in the loop / remove DEBUG_RESULTS definition variable, and compile in RELEASE mode!" << std::endl;

#ifdef DEBUG_RESULTS
    std::cout << "\t\tWARNING : DEBUG_RESULTS ACTIVATED - slows the process (writing results to result_debugs.dat file)" << std::endl;
    debug_results.close();
#endif

    TimeStamp t0, tf;
    t0 = wolf_problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().front().ts_;
    tf = wolf_problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().back().ts_;
    int N = wolf_problem_ptr_->getProcessorMotionPtr()->getBufferPtr()->get().size();
    std::cout << "t0        : " << t0.get() << " s" << std::endl;
    std::cout << "tf        : " << tf.get() << " s" << std::endl;
    std::cout << "duration  : " << tf-t0 << " s" << std::endl;
    std::cout << "N samples : " << N << std::endl;
    std::cout << "frequency : " << (N-1)/(tf-t0) << " Hz" << std::endl;
    std::cout << "CPU time  : " << elapsed_secs << " s" << std::endl;
    std::cout << "s/integr  : " << elapsed_secs/(N-1)*1e6 << " us" << std::endl;
    std::cout << "integr/s  : " << (N-1)/elapsed_secs << " ips" << std::endl;

    delete imu_ptr;
    delete wolf_problem_ptr_;

    return 0;

}