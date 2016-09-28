#include "hardware_base.h"


namespace wolf {

HardwareBase::HardwareBase() //:
        //            NodeLinked(MID, "HARDWARE", "BASE")
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
    sensor_list_.push_back(_sensor_ptr);
//	addDownNode(_sensor_ptr);
    //std::cout << "added!" << std::endl;


    _sensor_ptr->registerNewStateBlocks();

    return _sensor_ptr;

}

void HardwareBase::removeSensor(SensorBase* _sensor_ptr)
{
    sensor_list_.remove(_sensor_ptr);
    delete _sensor_ptr;
//    removeDownNode(_sensor_ptr->nodeId());
}

} // namespace wolf
