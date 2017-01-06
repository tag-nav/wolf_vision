#include "capture_imu.h"

namespace wolf {


CaptureIMU::CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _acc_gyro_data, 
                        FrameBasePtr _origin_frame_ptr) :
        CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, _origin_frame_ptr)
{
    setType("IMU");
    this->setFramePtr(_origin_frame_ptr);
//    std::cout << "constructed    +C-IMU" << id() << std::endl;
}

CaptureIMU::CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _acc_gyro_data, 
                       const Eigen::MatrixXs& _data_cov, FrameBasePtr _origin_frame_ptr) :
        CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, _data_cov, _origin_frame_ptr)
{
    setType("IMU");
    this->setFramePtr(_origin_frame_ptr);
//    std::cout << "constructed    +C-IMU" << id() << std::endl;
}


CaptureIMU::~CaptureIMU()
{
//    std::cout << "destructed     -C-IMU" << id() << std::endl;
    //
}


} //namespace wolf
