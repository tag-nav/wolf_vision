/**
 * \file intrinsics_factory.cpp
 *
 *  Created on: May 11, 2016
 *      \author: jsola
 */

#include "intrinsics_factory.h"

#include <iostream>

namespace wolf
{

bool IntrinsicsFactory::registerCreator(const std::string& _sensor_type, CreateIntrinsicsCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_sensor_type, createFn)).second;
    if (reg)
        std::cout << "IntrinsicsFactory : registered " << _sensor_type << std::endl;
    else
        std::cout << "IntrinsicsFactory : intrinsics " << _sensor_type << " already registered. Skipping. " << std::endl;
    return reg;
}

bool IntrinsicsFactory::unregisterCreator(const std::string& _sensor_type)
{
    return callbacks_.erase(_sensor_type) == 1;
}

IntrinsicsBase* IntrinsicsFactory::create(const std::string& _sensor_type, const std::string & _filename)
{
    CallbackMap::const_iterator i = callbacks_.find(_sensor_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Intrinsics type");
    }
    // Invoke the creation function
    std::cout << "Creating intrinsics for sensor " << _sensor_type << "...";
    IntrinsicsBase* p = (i->second)(_filename);
    std::cout << " OK." << std::endl;
    return p;
}

// Singleton ---------------------------------------------------
// This class is a singleton. The code below guarantees this.

IntrinsicsFactory& IntrinsicsFactory::get() // Unique point of access;
{
    static IntrinsicsFactory instance_;
    return instance_;
}

} /* namespace wolf */
