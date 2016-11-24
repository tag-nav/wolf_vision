#ifndef CAPTURE_IMU_H
#define CAPTURE_IMU_H

//Wolf includes
#include "capture_motion.h"

namespace wolf {

class CaptureIMU : public CaptureMotion
{
    public:
        typedef std::shared_ptr<CaptureIMU> Ptr;

    public:

        CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _data);
        virtual ~CaptureIMU();

};

} // namespace wolf

#endif // CAPTURE_IMU_H
