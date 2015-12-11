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

void ProcessorBase::addSensor(SensorBase* _sensor_ptr)
{
    //std::cout << "adding sensor..." << std::endl;
	addDownNode(_sensor_ptr);
    //std::cout << "added!" << std::endl;
}

void ProcessorBase::removeSensor(const SensorBaseIter& _sensor_iter)
{
	removeDownNode(_sensor_iter);
}

void ProcessorBase::removeSensor(SensorBase* _sensor_ptr)
{
	removeDownNode(_sensor_ptr->nodeId());
}

SensorBaseList* ProcessorBase::getSensorListPtr()
{
    return getDownNodeListPtr();
}
