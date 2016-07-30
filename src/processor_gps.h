//
// Created by ptirindelli on 16/12/15.
//

#ifndef WOLF_PROCESSOR_GPS_H
#define WOLF_PROCESSOR_GPS_H

namespace wolf
{
class CaptureGPS;
}

// Wolf includes
#include "processor_base.h"

// Std includes


namespace wolf {

class ProcessorGPS : public ProcessorBase
{
    protected:
        // unused
        //SensorGPS* sensor_gps_ptr_; //specific pointer to sensor gps object
        CaptureGPS* capture_gps_ptr_;

        Scalar gps_covariance_;


    public:
        ProcessorGPS();
        virtual ~ProcessorGPS();
        virtual void init(CaptureBase* _capture_ptr);
        virtual void process(CaptureBase* _capture_ptr);
        virtual bool voteForKeyFrame();
        virtual bool keyFrameCallback(wolf::FrameBase*, const Scalar& _time_tol);

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);

};

} // namespace wolf

#endif //WOLF_PROCESSOR_GPS_H
