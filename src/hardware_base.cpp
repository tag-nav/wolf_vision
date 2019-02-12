#include "base/hardware_base.h"
#include "base/sensor/sensor_base.h"

namespace wolf {

HardwareBase::HardwareBase() :
        NodeBase("HARDWARE", "Base")
{
//    std::cout << "constructed H" << std::endl;
}

HardwareBase::~HardwareBase()
{
//	std::cout << "destructed -H" << std::endl;
}

SensorBasePtr HardwareBase::addSensor(SensorBasePtr _sensor_ptr)
{
    sensor_list_.push_back(_sensor_ptr);
    _sensor_ptr->setProblem(getProblem());
    _sensor_ptr->setHardwarePtr(shared_from_this());

    _sensor_ptr->registerNewStateBlocks();

    return _sensor_ptr;
}

} // namespace wolf
