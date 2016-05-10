/**
 * \file sensor_factory.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "sensor_factory.h"

#include <iostream>

namespace wolf
{

bool SensorFactory::registerCreator(const std::string& _sensor_type, CreateSensorCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_sensor_type, createFn)).second;
    if (reg)
        std::cout << "SeosorFactory     : registered " << _sensor_type << std::endl;
    else
        std::cout << "SeosorFactory     : sensor " << _sensor_type << " already registered. Skipping. " << std::endl;
    return reg;
}

bool SensorFactory::unregisterCreator(const std::string& _sensor_type)
{
    return callbacks_.erase(_sensor_type) == 1;
}

wolf::SensorBase* SensorFactory::create(const std::string& _sensor_type, const std::string& _name, const Eigen::VectorXs& _extrinsics, const IntrinsicsBase* _intrinsics)
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

wolf::SensorFactory* SensorFactory::get() // Unique point of access;
{
    static SensorFactory instance_;
    return &instance_;
}

} /* namespace wolf */
