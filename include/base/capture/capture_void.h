#ifndef CAPTURE_VOID_H_
#define CAPTURE_VOID_H_

//Wolf includes
#include "base/capture/capture_base.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(CaptureVoid);
   
    
//class CaptureVoid
class CaptureVoid : public CaptureBase
{
    public:
        CaptureVoid(const TimeStamp& _ts, SensorBasePtr _sensor_ptr);
        virtual ~CaptureVoid();

};

} // namespace wolf

#endif
