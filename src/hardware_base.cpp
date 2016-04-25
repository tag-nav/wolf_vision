#include "hardware_base.h"


namespace wolf {

HardwareBase::HardwareBase() :
    NodeLinked(MID, "HARDWARE")
{
    //std::cout << "HardwareBase::HardwareBase(): " << __LINE__ << std::endl;
}

HardwareBase::~HardwareBase()
{
	//std::cout << "deleting HardwareBase " << nodeId() << std::endl;
}

SensorBase* HardwareBase::addSensor(SensorBase* _sensor_ptr)
{
    //std::cout << "adding sensor..." << std::endl;
	addDownNode(_sensor_ptr);
    //std::cout << "added!" << std::endl;

    _sensor_ptr->registerNewStateBlocks();

    return _sensor_ptr;

}

void HardwareBase::removeSensor(SensorBase* _sensor_ptr)
{
    removeDownNode(_sensor_ptr->nodeId());
}

} // namespace wolf
