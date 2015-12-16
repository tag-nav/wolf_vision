#include "processor_base.h"

ProcessorBase::ProcessorBase() :
    NodeLinked(MID, "PROCESSOR")
{
    //std::cout << "ProcessorBase::ProcessorBase(): " << __LINE__ << std::endl;
}

ProcessorBase::~ProcessorBase()
{
	//std::cout << "deleting ProcessorBase " << nodeId() << std::endl;
}

SensorBase* ProcessorBase::getSensorPtr()
{
    //            return (SensorBase*)(upperNodePtr());
    return upperNodePtr(); //TODO it seems casting is not needed?
}
