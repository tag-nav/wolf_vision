/*
 * ProcessorRangeBearing.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "processor_range_bearing.h"

namespace wolf
{

ProcessorRangeBearing::ProcessorRangeBearing(const Scalar& _time_tolerance = 0) :
        ProcessorBase("RANGE BEARING", _time_tolerance)
{
    //
}

ProcessorRangeBearing::~ProcessorRangeBearing()
{
    //
}

ProcessorBasePtr ProcessorRangeBearing::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{

    // construct processor
    ProcessorOdom3DPtr prc_odo = std::make_shared<ProcessorOdom3D>(prc_odo_params, sen_odo);

    // setup processor
    prc_odo->setName(_unique_name);

    return prc_odo;
}


} /* namespace wolf */
