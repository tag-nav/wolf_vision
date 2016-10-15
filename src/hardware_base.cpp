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
    is_removing_ = true;
    while (!sensor_list_.empty())
    {
        //        delete sensor_list_.front(); // TODO remove line
        sensor_list_.pop_front();
    }
	//std::cout << "deleting HardwareBase " << nodeId() << std::endl;
}

SensorBasePtr HardwareBase::addSensor(SensorBasePtr _sensor_ptr)
{
    //std::cout << "adding sensor..." << std::endl;
    sensor_list_.push_back(_sensor_ptr);
    _sensor_ptr->setProblem(getProblem());
//	addDownNode(_sensor_ptr);
    //std::cout << "added!" << std::endl;


    _sensor_ptr->registerNewStateBlocks();

    return _sensor_ptr;

}

void HardwareBase::removeSensor(SensorBasePtr _sensor_ptr)
{
    sensor_list_.remove(_sensor_ptr);
    //    delete _sensor_ptr; // TODO remove line
}

} // namespace wolf
