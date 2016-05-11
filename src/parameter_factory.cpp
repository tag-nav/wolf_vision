/**
 * \file parameter_factory.cpp
 *
 *  Created on: May 11, 2016
 *      \author: jsola
 */

#include "parameter_factory.h"

#include <iostream>

namespace wolf
{

bool ParameterFactory::registerCreator(const std::string& _parameter_type, CreateParameterCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_parameter_type, createFn)).second;
    if (reg)
        std::cout << "ParameterFactory  : registered " << _parameter_type << std::endl;
    else
        std::cout << "ParameterFactory  : parameter " << _parameter_type << " already registered. Skipping. " << std::endl;
    return reg;
}

bool ParameterFactory::unregisterCreator(const std::string& _parameter_type)
{
    return callbacks_.erase(_parameter_type) == 1;
}

ParameterBase* ParameterFactory::create(const std::string& _parameter_type, const std::string & _filename_dot_yaml)
{
    CallbackMap::const_iterator i = callbacks_.find(_parameter_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Parameter type");
    }
    // Invoke the creation function
    return (i->second)(_filename_dot_yaml);
}

// Singleton ---------------------------------------------------
// This class is a singleton. The code below guarantees this.

ParameterFactory* ParameterFactory::get() // Unique point of access;
{
    static ParameterFactory instance_;
    return &instance_;
}

} /* namespace wolf */
