#include "capture_imu.h"
#include "sensor_imu.h"
#include "state_quaternion.h"

namespace wolf {


CaptureIMU::CaptureIMU(const TimeStamp& _init_ts,
                       SensorBasePtr _sensor_ptr,
                       const Eigen::Vector6s& _acc_gyro_data,
                       FrameBasePtr _origin_frame_ptr) :
                CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, 10, 9, 6, _origin_frame_ptr)
{
    setType("IMU");

    SensorIMUPtr imu_ptr = std::static_pointer_cast<SensorIMU>(_sensor_ptr);

    Scalar acc_var  = imu_ptr->getAccelNoise() * imu_ptr->getAccelNoise();
    Scalar gyro_var = imu_ptr->getGyroNoise()  * imu_ptr->getGyroNoise();

    Vector6s data_vars; data_vars << acc_var, acc_var, acc_var, gyro_var, gyro_var, gyro_var;

    DiagonalMatrix<wolf::Scalar, 6> data_cov (data_vars);

    setDataCovariance(data_cov);
}

CaptureIMU::CaptureIMU(const TimeStamp& _init_ts,
                       SensorBasePtr _sensor_ptr,
                       const Eigen::Vector6s& _acc_gyro_data,
                       const Eigen::MatrixXs& _data_cov,
                       FrameBasePtr _origin_frame_ptr) :
                CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, _data_cov, 10, 9, 6, _origin_frame_ptr)
{
    setType("IMU");
}



CaptureIMU::CaptureIMU(const TimeStamp& _init_ts,
                       SensorBasePtr _sensor_ptr,
                       const Eigen::Vector6s& _acc_gyro_data,
                       const Eigen::MatrixXs& _data_cov,
                       const VectorXs& _extrinsics,
                       const Vector6s& _bias,
                       FrameBasePtr _origin_frame_ptr) :
                CaptureMotion(_init_ts, _sensor_ptr, _acc_gyro_data, _data_cov, 10, 9, 6, _origin_frame_ptr)
{
    if (!getSensorPtr() || getSensorPtr()->isExtrinsicDynamic())
    {
        setStateBlockPtr(0, std::make_shared<StateBlock>     (_extrinsics.head<3>(), false));
        setStateBlockPtr(1, std::make_shared<StateQuaternion>(_extrinsics.tail<4>(), false));
    }
    if (!getSensorPtr() || getSensorPtr()->isIntrinsicDynamic())
    {
        setStateBlockPtr(2, std::make_shared<StateBlock>(_bias, false));
    }
    setType("IMU");
}


CaptureIMU::~CaptureIMU()
{
//    std::cout << "destructed     -C-IMU" << id() << std::endl;
    //
}


} //namespace wolf
