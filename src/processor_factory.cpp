/**
 * \file processor_factory.cpp
 *
 *  Created on: May 4, 2016
 *      \author: jsola
 */

#include "processor_factory.h"

#include <iostream>

namespace wolf
{

bool ProcessorFactory::registerCreator(const std::string& _processor_type, CreateProcessorCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_processor_type, createFn)).second;
    if (reg)
        std::cout << "ProcessorFactory  : registered " << _processor_type << std::endl;
    else
        std::cout << "ProcessorFactory  : processor " << _processor_type << " already registered. Skipping. " << std::endl;

    return reg;
}

bool ProcessorFactory::unregisterCreator(const std::string& _processor_type)
{
    return callbacks_.erase(_processor_type) == 1;
}

ProcessorBase* ProcessorFactory::create(const std::string& _processor_type, const std::string& _name, const ProcessorParamsBase* _params)
{
    CallbackMap::const_iterator i = callbacks_.find(_processor_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Processor type");
    }
    // Invoke the creation function
    return (i->second)(_name, _params);
}

ProcessorFactory* ProcessorFactory::get() // Unique point of access;
{
    static ProcessorFactory instance_; // Guaranteed to be destroyed.
                                        // Instantiated on first use.
    return &instance_;
}

} /* namespace wolf */
