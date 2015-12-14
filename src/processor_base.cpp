#include "processor_base.h"

ProcessorBase::ProcessorBase() :
    NodeLinked(MID, "HARDWARE")
{
    //std::cout << "ProcessorBase::ProcessorBase(): " << __LINE__ << std::endl;
}

ProcessorBase::~ProcessorBase()
{
	//std::cout << "deleting ProcessorBase " << nodeId() << std::endl;
}

