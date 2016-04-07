#ifndef CAPTURE_VOID_H_
#define CAPTURE_VOID_H_

//Wolf includes
#include "capture_base.h"


namespace wolf {

//class CaptureVoid
class CaptureVoid : public CaptureBase
{
    public:
        CaptureVoid(const TimeStamp& _ts, SensorBase* _sensor_ptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureVoid();

        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const;
};

} // namespace wolf

#endif
