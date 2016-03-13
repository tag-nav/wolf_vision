#include "processor_base.h"
#include "sensor_base.h"
#include "node_terminus.h"

ProcessorBase::ProcessorBase(ProcessorType _tp) :
        NodeLinked(MID, "PROCESSOR"),
        type_(_tp)
{
    //
}

ProcessorBase::~ProcessorBase()
{
    //
}
