#include "processor_base.h"
#include "sensor_base.h"
#include "node_terminus.h"

ProcessorBase::ProcessorBase() :
        NodeLinked(MID, "PROCESSOR")
{
    //
}

ProcessorBase::~ProcessorBase()
{
    //
}
