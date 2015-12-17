#include "processor_base.h"

ProcessorBase::ProcessorBase() :
        NodeLinked(MID, "PROCESSOR")
{
    //
}

ProcessorBase::~ProcessorBase()
{
    //
}

SensorBase* ProcessorBase::getSensorPtr()
{
    return upperNodePtr();
}

