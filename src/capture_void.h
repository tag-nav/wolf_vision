#ifndef CAPTURE_VOID_H_
#define CAPTURE_VOID_H_

//Wolf includes
#include "capture_base.h"


namespace wolf {

//forward declaration to typedef class pointers
class CaptureVoid;
typedef std::shared_ptr<CaptureVoid> CaptureVoidPtr;
typedef std::shared_ptr<const CaptureVoid> CaptureVoidConstPtr;
typedef std::weak_ptr<CaptureVoid> CaptureVoidWPtr;      
   
    
//class CaptureVoid
class CaptureVoid : public CaptureBase
{
    public:
        CaptureVoid(const TimeStamp& _ts, SensorBasePtr _sensor_ptr);
        virtual ~CaptureVoid();

};

} // namespace wolf

#endif
