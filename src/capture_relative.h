
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
    protected:
        TimeStamp final_time_stamp_; ///< Final Time stamp

    public:
        CaptureRelative(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr);
        
        CaptureRelative(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data);

        CaptureRelative(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);

        virtual ~CaptureRelative();

        virtual void integrateCapture(CaptureRelative* _new_capture) = 0;

        virtual CaptureRelative* interpolateCapture(const TimeStamp& _ts) = 0;

        TimeStamp getInitTimeStamp() const;

        TimeStamp getFinalTimeStamp() const;

        void setInitTimeStamp(const TimeStamp & _ts);

        void setFinalTimeStamp(const TimeStamp & _ts);
};
#endif
