//
// Created by ptirindelli on 16/12/15.
//

#ifndef WOLF_PROCESSOR_GPS_H
#define WOLF_PROCESSOR_GPS_H

// Wolf includes
#include "processor_base.h"
#include "capture_gps.h"

// Std includes


namespace wolf {
    
WOLF_PTR_TYPEDEFS(ProcessorGPS);
    
//class
class ProcessorGPS : public ProcessorBase
{
    protected:
        CaptureGPSPtr capture_gps_ptr_;
        Scalar gps_covariance_;

    public:
        ProcessorGPS();
        virtual ~ProcessorGPS(Scalar time_tolerance_);
        virtual void init(CaptureBasePtr _capture_ptr);
        virtual void process(CaptureBasePtr _capture_ptr);
        virtual bool voteForKeyFrame();
        virtual bool keyFrameCallback(wolf::FrameBasePtr, const Scalar& _time_tol);

    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);

};

} // namespace wolf

#endif //WOLF_PROCESSOR_GPS_H
