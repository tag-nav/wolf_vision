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
    is_deleting_ = true;
    while (!sensor_list_.empty())
    {
        sensor_list_.front()->destruct();
        sensor_list_.pop_front();
    }
	//std::cout << "deleting HardwareBase " << nodeId() << std::endl;
}

SensorBasePtr HardwareBase::addSensor(SensorBasePtr _sensor_ptr)
{
    //std::cout << "adding sensor..." << std::endl;
    sensor_list_.push_back(_sensor_ptr);
    _sensor_ptr->setProblem(getProblem());
    //std::cout << "added!" << std::endl;

    _sensor_ptr->registerNewStateBlocks();

    return _sensor_ptr;

}

void HardwareBase::removeSensor(SensorBasePtr _sensor_ptr)
{
    sensor_list_.remove(_sensor_ptr);
//    delete _sensor_ptr;
}

} // namespace wolf
