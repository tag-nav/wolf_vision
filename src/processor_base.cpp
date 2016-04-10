#include "processor_base.h"
#include "sensor_base.h"
#include "node_terminus.h"

namespace wolf {

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


} // namespace wolf
