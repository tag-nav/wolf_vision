#include "capture_imu.h"

namespace wolf {


CaptureIMU::CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr,
                             const Eigen::Vector6s& _acc_gyro_data) :
        CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data )
{
    setType("IMU");
    std::cout << "constructed    +C-IMU" << id() << std::endl;
}

CaptureIMU::~CaptureIMU()
{
    std::cout << "destructed     -C-IMU" << id() << std::endl;
    //
}


} //namespace wolf
