#ifndef CAPTURE_IMU_H
#define CAPTURE_IMU_H

//Wolf includes
#include "capture_motion.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(CaptureIMU);

class CaptureIMU : public CaptureMotion
{
    public:

        CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _data, 
                    FrameBasePtr _origin_frame_ptr = nullptr);

        CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _data, 
                   const Eigen::MatrixXs& _data_cov, FrameBasePtr _origin_frame_ptr = nullptr);

        CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _data,
                   const Eigen::MatrixXs& _data_cov, const VectorXs& _extrinsics, const Vector6s& _bias, FrameBasePtr _origin_frame_ptr = nullptr);

        virtual ~CaptureIMU();

};

} // namespace wolf

#endif // CAPTURE_IMU_H
