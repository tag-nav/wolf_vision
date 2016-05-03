/**
 * \file sensor_factory.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "sensor_factory.h"

namespace wolf
{

bool SensorFactory::registerCreator(std::string _sensor_type, CreateSensorCallback createFn)
{
    return callbacks_.insert(CallbackMap::value_type(_sensor_type, createFn)).second;
}

bool SensorFactory::unregisterCreator(std::string _sensor_type)
{
    return callbacks_.erase(_sensor_type) == 1;
}

wolf::SensorBase* SensorFactory::create(std::string _sensor_type, std::string _name, Eigen::VectorXs& _extrinsics, IntrinsicsBase* _intrinsics)
{
    CallbackMap::const_iterator i = callbacks_.find(_sensor_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Sensor type");
    }
    // Invoke the creation function
    return (i->second)(_name, _extrinsics, _intrinsics);
}

// Singleton ---------------------------------------------------
// This class is a singleton. The code below guarantees this.

SensorFactory* SensorFactory::pInstance_ = nullptr;

wolf::SensorFactory* SensorFactory::get() // Unique point of access;
{
    if (pInstance_ == nullptr)
        pInstance_ = new SensorFactory;
    return pInstance_;
}

// singleton: constructor and destructor are private
SensorFactory::SensorFactory(const SensorFactory&){}
SensorFactory::SensorFactory(){}

} /* namespace wolf */
