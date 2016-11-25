#ifndef CAPTURE_IMU_H
#define CAPTURE_IMU_H

//Wolf includes
#include "capture_motion.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(CaptureIMU);

class CaptureIMU : public CaptureMotion
{
    public:

        CaptureIMU(const TimeStamp& _init_ts, SensorBasePtr _sensor_ptr, const Eigen::Vector6s& _data);
        virtual ~CaptureIMU();

};

} // namespace wolf

#endif // CAPTURE_IMU_H
