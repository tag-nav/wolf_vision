/*
 * ProcessorRangeBearing.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_PROCESSOR_RANGE_BEARING_H_
#define HELLO_WOLF_PROCESSOR_RANGE_BEARING_H_

#include "processor_base.h"
#include "sensor_range_bearing.h"

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsRangeBearing);

struct ProcessorParamsRangeBearing : public ProcessorParamsBase
{

};



WOLF_PTR_TYPEDEFS(ProcessorRangeBearing);

class ProcessorRangeBearing : public ProcessorBase
{
    public:
        ProcessorRangeBearing(const SensorRangeBearingPtr _sensor_ptr, const Scalar& _time_tolerance = 0);
        virtual ~ProcessorRangeBearing();

        // Implementation of pure virtuals from ProcessorBase
        virtual void process(CaptureBasePtr _capture) override;
        virtual bool voteForKeyFrame() override {return false;}
        virtual bool keyFrameCallback(FrameBasePtr _key_frame, const Scalar& _time_tolerance) override {return true;}

        // Factory method for high level API
        static ProcessorBasePtr create(const std::string& _unique_name,
                                       const ProcessorParamsBasePtr _params,
                                       const SensorBasePtr sensor_ptr = nullptr);

};

} /* namespace wolf */

#endif /* HELLO_WOLF_PROCESSOR_RANGE_BEARING_H_ */
