#include <capture_IMU.h>
#include <sensor_IMU.h>
#include "state_quaternion.h"

namespace wolf {



CaptureIMU::CaptureIMU(const TimeStamp& _init_ts,
                       SensorBasePtr _sensor_ptr,
                       const Eigen::Vector6s& _acc_gyro_data,
                       const Eigen::MatrixXs& _data_cov,
                       FrameBasePtr _origin_frame_ptr) :
                CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, _data_cov, 10, 9, _origin_frame_ptr, nullptr, nullptr, std::make_shared<StateBlock>(6, false))
{
    setType("IMU");
}

CaptureIMU::CaptureIMU(const TimeStamp& _init_ts,
                       SensorBasePtr _sensor_ptr,
                       const Eigen::Vector6s& _acc_gyro_data,
                       const Eigen::MatrixXs& _data_cov,
                       const Vector6s& _bias,
                       FrameBasePtr _origin_frame_ptr) :
                CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, _data_cov, 10, 9, _origin_frame_ptr, nullptr, nullptr, std::make_shared<StateBlock>(_bias, false))
{
    setType("IMU");
}


CaptureIMU::~CaptureIMU()
{
    //
}

} //namespace wolf
