
#ifndef CAPTURE_RELATIVE_H_
#define CAPTURE_RELATIVE_H_

//std includes
//

//Wolf includes
#include "wolf.h"
#include "capture_base.h"

//class CaptureBase
class CaptureRelative : public CaptureBase
{
    public:
        CaptureRelative(const TimeStamp& _ts, SensorBase* _sensor_ptr);
        
        CaptureRelative(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data);

        CaptureRelative(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);

        virtual ~CaptureRelative();

        virtual void integrateCapture(CaptureRelative* _new_capture) = 0;
};
#endif
