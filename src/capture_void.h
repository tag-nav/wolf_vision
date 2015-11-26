#ifndef CAPTURE_VOID_H_
#define CAPTURE_VOID_H_

//Wolf includes
#include "capture_base.h"

//class CaptureVoid
class CaptureVoid : public CaptureBase
{
    public:
        CaptureVoid(const TimeStamp& _ts, SensorBase* _sensor_ptr);

        virtual ~CaptureVoid();

        virtual Eigen::VectorXs computePrior(const TimeStamp& _now) const;
};
#endif
