/*
 * ProcessorRangeBearing.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "processor_range_bearing.h"
#include "capture_range_bearing.h"

namespace wolf
{

ProcessorRangeBearing::ProcessorRangeBearing(const SensorRangeBearingPtr _sensor_ptr, const Scalar& _time_tolerance) :
        ProcessorBase("RANGE BEARING", _time_tolerance)
{
    //
}

ProcessorRangeBearing::~ProcessorRangeBearing()
{
    //
}

void ProcessorRangeBearing::process(CaptureBasePtr _capture)
{
    CaptureRangeBearingPtr capture = std::static_pointer_cast<CaptureRangeBearing>(_capture);
    Size n = capture->getRanges().size();

    for (Size i = 0; i < n; i++)
    {
        Scalar range    = capture->getRange  (i);
        Scalar bearing  = capture->getBearing(i);
    }
}

ProcessorBasePtr ProcessorRangeBearing::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    SensorRangeBearingPtr sensor_rb = std::static_pointer_cast<SensorRangeBearing>(_sen_ptr);

    // construct processor
    ProcessorRangeBearingPtr prc = std::make_shared<ProcessorRangeBearing>(sensor_rb, _params->time_tolerance);

    // setup processor
    prc->setName(_unique_name);

    return prc;
}


} /* namespace wolf */


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf
{
WOLF_REGISTER_PROCESSOR("RANGE BEARING", ProcessorRangeBearing)
} // namespace wolf

