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

        virtual bool voteForKeyFrame(){return false;}

        virtual bool keyFrameCallback(wolf::FrameBase*){return false;}

    public:
        static ProcessorBase* create(const std::string & _unique_name, const ProcessorParamsBase* _params);

};

inline void ProcessorGPS::init(CaptureBase* _capture_ptr)
{
}

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_gps = ProcessorFactory::get()->registerCreator("GPS", ProcessorGPS::create);
}
} // namespace wolf


#endif //WOLF_PROCESSOR_GPS_H
