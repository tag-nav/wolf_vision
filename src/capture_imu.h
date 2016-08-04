#ifndef CAPTURE_IMU_H
#define CAPTURE_IMU_H

//Wolf includes
#include "capture_motion.h"

namespace wolf {

class CaptureIMU : public CaptureMotion
{
    public:

        CaptureIMU(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::Vector6s& _data);

        CaptureIMU(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::Vector6s& _data, const Eigen::Matrix<Scalar,6,3>& _data_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureIMU();

        virtual void process();

        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now = 0) const;

        virtual void addConstraints();

        virtual void integrateCapture(CaptureMotion* _new_capture);

        virtual CaptureIMU* interpolateCapture(const TimeStamp& _ts);
};

} // namespace wolf

#endif // CAPTURE_IMU_H
