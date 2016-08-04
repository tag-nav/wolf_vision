#include "capture_imu.h"

namespace wolf {


CaptureIMU::CaptureIMU(const TimeStamp& _init_ts, SensorBase* _sensor_ptr,
                             const Eigen::Vector6s& _acc_gyro_data) :
        CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data )
{
    setType("IMU");
}

/*CaptureIMU::CaptureIMU(const TimeStamp& _init_ts, SensorBase* _sensor_ptr,
                             const Eigen::Vector6s& _data, const Eigen::Matrix<Scalar,6,3>& _data_covariance) :
        CaptureMotion(_init_ts, _sensor_ptr, _data, _data_covariance)
{
    setType("IMU");
}*/

CaptureIMU::~CaptureIMU()
{
    //
}


} //namespace wolf
