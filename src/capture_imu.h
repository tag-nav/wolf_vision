#ifndef CAPTURE_IMU_H
#define CAPTURE_IMU_H

//Wolf includes
#include "capture_motion.h"

namespace wolf {

class CaptureIMU : public CaptureMotion
{
    public:

        CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _data);

//<<<<<<< e72779277b2cbd56ce81286c43b51ae2b4934110
//        /*CaptureIMU(const TimeStamp& _init_ts, SensorBase* _sensor_ptr, const Eigen::Vector6s& _data, const Eigen::Matrix<Scalar,6,3>& _data_covariance);*/
//=======
//        /*CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _data, const Eigen::Matrix<Scalar,6,3>& _data_covariance);
//
//        CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _data, const Eigen::Matrix<Scalar,6,3>& _data_covariance, FrameBasePtr _origin_frame_ptr);*/
//>>>>>>> typedef all pointers to base classes

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureIMU();

};

} // namespace wolf

#endif // CAPTURE_IMU_H
