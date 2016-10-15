#include "hardware_base.h"
#include "sensor_base.h"


namespace wolf {

HardwareBase::HardwareBase() :
        NodeBase("HARDWARE"),
        problem_ptr_(nullptr)
{
    //std::cout << "HardwareBase::HardwareBase(): " << __LINE__ << std::endl;
}

HardwareBase::~HardwareBase()
{
	std::cout << "destructing H" << nodeId() << std::endl;
}

SensorBasePtr HardwareBase::addSensor(SensorBasePtr _sensor_ptr)
{
    sensor_list_.push_back(_sensor_ptr);
    _sensor_ptr->setProblem(getProblem());

    _sensor_ptr->registerNewStateBlocks();

    return _sensor_ptr;
}

void HardwareBase::removeSensor(SensorBasePtr _sensor_ptr)
{
    sensor_list_.remove(_sensor_ptr);
}

} // namespace wolf
