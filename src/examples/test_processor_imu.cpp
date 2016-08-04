/**
 * \file test_processor_imu.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: dtsbourg
 */

//std
#include <iostream>
#include <fstream>

//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"

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
    SensorBase* sensor_ptr_ = new SensorIMU( new StateBlock(Eigen::VectorXs::Zero(3)),
                                             new StateQuaternion(),
                                             new StateBlock(Eigen::VectorXs::Zero(6)));

    ProcessorIMU* processor_ptr_ = new ProcessorIMU();

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(processor_ptr_);

    CaptureIMU* imu_ptr;

    while(!data_file_acc.eof()){
        data_file_acc >> mti_clock >> data_[0] >> data_[1] >> data_[2];
        data_file_gyro >> tmp >> data_[3] >> data_[4] >> data_[5];

        t.set(mti_clock);

        imu_ptr = new CaptureIMU(t, sensor_ptr_, data_);

        std::cout << "in process" << std::endl;

        imu_ptr ->process();

        std::cout << "out process" << std::endl;

    }

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    delete wolf_problem_ptr_;

    return 0;
}
