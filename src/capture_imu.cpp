#include "capture_imu.h"
#include "sensor_imu.h"

namespace wolf {


CaptureIMU::CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _acc_gyro_data, 
                        FrameBasePtr _origin_frame_ptr) :
        CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, 10, 9 , _origin_frame_ptr)
{
    setType("IMU");

    SensorIMUPtr imu_ptr = std::static_pointer_cast<SensorIMU>(_sensor_ptr);

    Scalar acc_var  = imu_ptr->getAccelNoise() * imu_ptr->getAccelNoise();
    Scalar gyro_var = imu_ptr->getGyroNoise()  * imu_ptr->getGyroNoise();

    Vector6s data_vars; data_vars << acc_var, acc_var, acc_var, gyro_var, gyro_var, gyro_var;

    Matrix6s data_cov; data_cov.setZero();

    data_cov.diagonal() = data_vars;

    setDataCovariance(data_cov);

//    std::cout << "constructed    +C-IMU" << id() << std::endl;
}

CaptureIMU::CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _acc_gyro_data, 
                       const Eigen::MatrixXs& _data_cov, FrameBasePtr _origin_frame_ptr) :
        CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, _data_cov, 10, 9 , _origin_frame_ptr)
{
    setType("IMU");
//    std::cout << "constructed    +C-IMU" << id() << std::endl;
}


CaptureIMU::~CaptureIMU()
{
//    std::cout << "destructed     -C-IMU" << id() << std::endl;
    //
}


} //namespace wolf
