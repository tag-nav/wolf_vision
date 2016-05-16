/**
 * \file processor_params_factory.cpp
 *
 *  Created on: May 16, 2016
 *      \author: jsola
 */

#include "processor_params_factory.h"

namespace wolf
{

bool ProcessorParamsFactory::registerCreator(const std::string& _processor_type, CreateProcessorParamsCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_processor_type, createFn)).second;
    if (reg)
        std::cout << "ProcessorParamsFactory : registered " << _processor_type << std::endl;
    else
        std::cout << "ProcessorParamsFactory : params " << _processor_type << " already registered. Skipping. " << std::endl;
    return reg;
}

bool ProcessorParamsFactory::unregisterCreator(const std::string& _processor_type)
{
    return callbacks_.erase(_processor_type) == 1;
}

ProcessorParamsBase* ProcessorParamsFactory::create(const std::string& _processor_type, const std::string & _filename)
{
    CallbackMap::const_iterator i = callbacks_.find(_processor_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown ProcessorParams type");
    }
    // Invoke the creation function
    std::cout << "Creating parameters for processor " << _processor_type << "...";
    ProcessorParamsBase* p = (i->second)(_filename);
    std::cout << " OK." << std::endl;
    return p;
}

// Singleton ---------------------------------------------------
// This class is a singleton. The code below guarantees this.

ProcessorParamsFactory& ProcessorParamsFactory::get() // Unique point of access;
{
    static ProcessorParamsFactory instance_;
    return instance_;
}
} /* namespace wolf */
