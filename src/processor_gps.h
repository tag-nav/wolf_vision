//
// Created by ptirindelli on 16/12/15.
//

#ifndef WOLF_PROCESSOR_GPS_H
#define WOLF_PROCESSOR_GPS_H

// Fwd references
class SensorGPS;

// Wolf includes
#include "processor_base.h"
#include "capture_gps.h"

// Std includes

class ProcessorGPS : public ProcessorBase
{
protected:
    SensorGPS* sensor_gps_ptr_; //specific pointer to sensor gps object
    CaptureGPS* capture_gps_ptr_;


public:
    ProcessorGPS();
    virtual ~ProcessorGPS();

        virtual void process(CaptureBase* _capture_ptr);

private:
    virtual void extractFeatures(CaptureBase *_capture_ptr);
    virtual void establishConstraints(CaptureBase *_capture_ptr);

};

inline void ProcessorGPS::process(CaptureBase* _capture_ptr)
{
    extractFeatures(_capture_ptr);
    establishConstraints(_capture_ptr);
}

#endif //WOLF_PROCESSOR_GPS_H
